#include <AccelStepper.h>
#include "DHT.h"

// --- 핀 정의 ---
#define DHT_PIN     A3
#define DHTTYPE     DHT11
#define TDS_PIN     A2
#define PH_PIN      A1
#define PUMP_PIN    7
#define FLOAT_PIN   6
#define LED_PIN     5
#define PH_PWR_PIN  4

// NEMA17
#define DIR_PIN     8
#define STEP_PIN    9
#define EN_PIN      10

// NEMA23
#define DIR23_PIN   11
#define STEP23_PIN  13
#define EN23_PIN    12

// --- 타이밍 ---
const unsigned long SENSOR_INTERVAL      = 60000UL;
const unsigned long TDS_SAMPLE_INTERVAL  = 200UL;
const unsigned long PUMP_SETTLE          = 1000UL;
const unsigned long FLOAT_CHECK_INTERVAL = 30000UL;

const int TDS_SCOUNT = 15;
const int PH_SCOUNT  = 5;

// --- 시퀀스 상수 ---
const int           MOVES_PER_DIR = 3;
const long          M17_STEP_A    = 150000L;
const long          M17_STEP_B    = 75000L;
const long          M23_STEP      = 17000L;
const unsigned long M17_WAIT      = 17000UL;

const long M17_POS_P1[3] = { 50000L,  75000L, 100000L };
const long M17_POS_P2[3] = { 75000L,  50000L,   5000L };
const long M17_POS_P3[3] = { 50000L,  75000L, 100000L };

// --- 센서 정상 범위 필터 ---
const float DHT_TEMP_MIN = -10.0f;
const float DHT_TEMP_MAX =  60.0f;
const float DHT_HUM_MIN  =   0.0f;
const float DHT_HUM_MAX  = 100.0f;
const float PH_MIN       =   0.0f;
const float PH_MAX       =  14.0f;
const float PH_DELTA     =   0.8f;
const float TDS_MIN      =   0.0f;
const float TDS_MAX      = 1000.0f;
const float TDS_DELTA    =  80.0f;

// --- 이상값 알림 임계값 ---
// TODO: 추후 서버 Species 테이블 기반 동적 임계값으로 교체 예정
const float ALERT_TEMP_MIN =  15.0f;
const float ALERT_TEMP_MAX =  30.0f;
const float ALERT_HUM_MIN  =  30.0f;
const float ALERT_HUM_MAX  =  90.0f;
const float ALERT_PH_MIN   =   5.0f;
const float ALERT_PH_MAX   =   7.5f;
const float ALERT_TDS_MIN  = 200.0f;
const float ALERT_TDS_MAX  = 800.0f;

float lastValidTemp = 25.0f;
float lastValidHum  = 50.0f;
float lastValidPH   =  7.0f;
float lastValidTDS  =  0.0f;

enum SeqState {
  IDLE,
  P1_M17_MOVE, P1_M17_WAIT, P1_M23_MOVE,
  P1_TOP_PHOTO_WAIT,
  P2_M17_MOVE, P2_M17_WAIT, P2_M23_MOVE,
  P3_M17_MOVE, P3_M17_WAIT, P3_M23_HOME,
  P4_M17_DOWN, SEQ_DONE
};

SeqState      seqState      = IDLE;
int           moveCount     = 0;
unsigned long waitStart     = 0;
bool          sensorActive  = true;

DHT dht(DHT_PIN, DHTTYPE);
AccelStepper stepper17(AccelStepper::DRIVER, STEP_PIN,   DIR_PIN);
AccelStepper stepper23(AccelStepper::DRIVER, STEP23_PIN, DIR23_PIN);

int   analogBuffer[TDS_SCOUNT];
int   analogBufferIndex = 0;
float c_temp = 25.0;

float phRunningSum = 0.0;
float phBuffer[PH_SCOUNT];
int   phIndex = 0;
bool  phFull  = false;

bool          pumpRunning     = false;
bool          pumpJustToggled = false;
unsigned long pumpToggleTime  = 0;

int           lastFloatState  = -1;
unsigned long lastFloatCheck  = 0;

unsigned long lastSensor    = 0;
unsigned long tdsLastSample = 0;

bool ledState = false;
bool waterOK  = false;

// -----------------------------------------------
int getMedianNum(int* arr, int len) {
  int buf[TDS_SCOUNT];
  for (int i = 0; i < len; i++) buf[i] = arr[i];
  for (int i = 1; i < len; i++) {
    int key = buf[i], j = i - 1;
    while (j >= 0 && buf[j] > key) { buf[j+1] = buf[j]; j--; }
    buf[j+1] = key;
  }
  return (len & 1) ? buf[(len-1)/2] : (buf[len/2] + buf[len/2-1]) / 2;
}

float getSmoothedPH(float newVal) {
  phRunningSum -= phBuffer[phIndex];
  phBuffer[phIndex] = newVal;
  phRunningSum += newVal;
  phIndex = (phIndex + 1) % PH_SCOUNT;
  if (!phFull && phIndex == 0) phFull = true;
  int len = phFull ? PH_SCOUNT : phIndex;
  return (len > 0) ? phRunningSum / len : newVal;
}

void checkAlert(float temp, float hum, float ph, float tds) {
  if (temp < ALERT_TEMP_MIN || temp > ALERT_TEMP_MAX) {
    Serial.print(F("[ALERT] TEMP:")); Serial.println(temp, 1);
  }
  if (hum < ALERT_HUM_MIN || hum > ALERT_HUM_MAX) {
    Serial.print(F("[ALERT] HUM:")); Serial.println(hum, 1);
  }
  if (ph < ALERT_PH_MIN || ph > ALERT_PH_MAX) {
    Serial.print(F("[ALERT] PH:")); Serial.println(ph, 2);
  }
  if (tds < ALERT_TDS_MIN || tds > ALERT_TDS_MAX) {
    Serial.print(F("[ALERT] TDS:")); Serial.println(tds, 0);
  }
}

void readAndPrintSensors() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  bool tempOK = !isnan(t) && t >= DHT_TEMP_MIN && t <= DHT_TEMP_MAX;
  bool humOK  = !isnan(h) && h >= DHT_HUM_MIN  && h <= DHT_HUM_MAX;
  if (tempOK && humOK) {
    lastValidTemp = t;
    lastValidHum  = h;
    c_temp = t;
  }

  if (!pumpJustToggled) {
    float voltage     = getMedianNum(analogBuffer, TDS_SCOUNT) * 5.0 / 1024.0;
    float compVoltage = voltage / (1.0 + 0.02 * (c_temp - 25.0));
    float tds = (133.42 * pow(compVoltage,3)
               - 255.86 * pow(compVoltage,2)
               + 857.39 * compVoltage) * 0.5;
    if (tds >= TDS_MIN && tds <= TDS_MAX && fabs(tds - lastValidTDS) <= TDS_DELTA) {
      lastValidTDS = tds;
    }
  }

  float voltage = analogRead(PH_PIN) * 5.0 / 1024.0;
  float raw = 2.50 + (7.00 - 2.50) / (0.918 - 0.568) * (voltage - 0.568);
  raw += (c_temp - 25.0f) * (-0.03f);
  if (ledState) raw -= 0.5f;   // LED 켜짐 시 광간섭 보정
  if (raw >= PH_MIN && raw <= PH_MAX && fabs(raw - lastValidPH) <= PH_DELTA) {
    lastValidPH = getSmoothedPH(raw);
  }

  Serial.print(F("[DATA] "));
  Serial.print(F("T:"));      Serial.print(lastValidTemp, 1);
  Serial.print(F(",H:"));     Serial.print(lastValidHum,  1);
  Serial.print(F(",PH:"));    Serial.print(lastValidPH,   2);
  Serial.print(F(",TDS:"));   Serial.print(lastValidTDS,  0);
  Serial.print(F(",LED:"));   Serial.print(ledState ? 1 : 0);
  Serial.print(F(",WATER:")); Serial.println(waterOK ? 1 : 0);

  checkAlert(lastValidTemp, lastValidHum, lastValidPH, lastValidTDS);
}

void startSequence() {
  sensorActive = false;
  moveCount    = 0;
  // m23TotalMoved 제거 — 스윕 방식은 고정 타겟(+M23_STEP, -M23_STEP)만 사용
  stepper17.setCurrentPosition(0);
  stepper23.setCurrentPosition(0);
  digitalWrite(EN_PIN, LOW);
  stepper17.moveTo(M17_POS_P1[0]);
  seqState = P1_M17_MOVE;
  Serial.println(F("[SEQ] START"));
}

// -----------------------------------------------
void setup() {
  Serial.begin(9600);
  delay(2000);
  dht.begin();

  pinMode(PUMP_PIN,  OUTPUT);
  pinMode(FLOAT_PIN, INPUT_PULLUP);
  digitalWrite(PUMP_PIN, LOW);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(EN_PIN,   OUTPUT);
  digitalWrite(EN_PIN, HIGH);

  pinMode(EN23_PIN, OUTPUT);
  digitalWrite(EN23_PIN, HIGH);

  pinMode(PH_PWR_PIN, OUTPUT);
  digitalWrite(PH_PWR_PIN, LOW);

  phFull = false;
  phIndex = 0;
  phRunningSum = 0.0;
  memset(phBuffer, 0, sizeof(phBuffer));

  stepper17.setMaxSpeed(2500);
  stepper17.setAcceleration(1000);
  stepper17.setCurrentPosition(0);

  stepper23.setMaxSpeed(1600);
  stepper23.setAcceleration(800);
  stepper23.setCurrentPosition(0);

  Serial.println(F("=== GrowLab START ==="));
  delay(30000);
  digitalWrite(PH_PWR_PIN, HIGH);
  delay(3000);
  Serial.println(F("[SYSTEM] Ready"));
}

// -----------------------------------------------
void loop() {
  unsigned long now = millis();

  // --- Serial 명령 수신 ---
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'p' && seqState == IDLE) startSequence();
    else if (c == 'O') {
      ledState = true;
      digitalWrite(LED_PIN, HIGH);
      Serial.println(F("[LED] ON"));
    }
    else if (c == 'o') {
      ledState = false;
      digitalWrite(LED_PIN, LOW);
      Serial.println(F("[LED] OFF"));
    }
  }

  // --- 시퀀스 상태머신 ---
  switch (seqState) {
    case IDLE: break;

    // ── P1: 50000 → 75000 → 100000 ──────────────────────────
    case P1_M17_MOVE:
      if (stepper17.distanceToGo() != 0) { stepper17.run(); }
      else {
        waitStart = millis();
        seqState  = P1_M17_WAIT;
        if (moveCount > 0) {
          Serial.println(F("[SEQ] PHOTO"));  // portIndex 0(75000), 1(100000)
        }
      }
      break;

    case P1_M17_WAIT:
      if (millis() - waitStart >= M17_WAIT) {
        moveCount++;
        if (moveCount < MOVES_PER_DIR) {
          stepper17.moveTo(M17_POS_P1[moveCount]);
          seqState = P1_M17_MOVE;
        } else {
          moveCount = 0;
          digitalWrite(EN23_PIN, LOW);
          stepper23.moveTo(M23_STEP);   // ✅ 오른쪽 먼저: 0 → +19000
          seqState = P1_M23_MOVE;
        }
      }
      break;

    case P1_M23_MOVE:
      if (stepper23.distanceToGo() != 0) { stepper23.run(); }
      else {
        digitalWrite(EN23_PIN, HIGH);
        moveCount = 0;
        waitStart = millis();
        Serial.println(F("[SEQ] PHOTO"));  // portIndex 2: M23 오른쪽 회전 후
        seqState = P1_TOP_PHOTO_WAIT;
      }
      break;

    case P1_TOP_PHOTO_WAIT:              // M17 하강 전 대기
      if (millis() - waitStart >= M17_WAIT) {
        stepper17.moveTo(M17_POS_P2[0]);
        seqState = P2_M17_MOVE;
      }
      break;

    // ── P2: 75000 → 50000 → 5000 ────────────────────────────
    case P2_M17_MOVE:
      if (stepper17.distanceToGo() != 0) { stepper17.run(); }
      else {
        waitStart = millis();
        seqState  = P2_M17_WAIT;
        if (moveCount < 2) {
          Serial.println(F("[SEQ] PHOTO"));  // portIndex 3(75000), 4(50000)
        }
      }
      break;

    case P2_M17_WAIT:
      if (millis() - waitStart >= M17_WAIT) {
        moveCount++;
        if (moveCount < MOVES_PER_DIR) {
          stepper17.moveTo(M17_POS_P2[moveCount]);
          seqState = P2_M17_MOVE;
        } else {
          moveCount = 0;
          digitalWrite(EN23_PIN, LOW);
          stepper23.moveTo(-M23_STEP);  // ✅ 왼쪽으로 스윕: +19000 → -19000 (중심 통과)
          seqState = P2_M23_MOVE;
        }
      }
      break;

    case P2_M23_MOVE:
      if (stepper23.distanceToGo() != 0) { stepper23.run(); }
      else {
        digitalWrite(EN23_PIN, HIGH);
        moveCount = 0;
        stepper17.moveTo(M17_POS_P3[0]);
        seqState = P3_M17_MOVE;
      }
      break;

    // ── P3: 50000 → 75000 → 100000 ──────────────────────────
    case P3_M17_MOVE:
      if (stepper17.distanceToGo() != 0) { stepper17.run(); }
      else {
        waitStart = millis();
        seqState  = P3_M17_WAIT;
        Serial.println(F("[SEQ] PHOTO"));  // portIndex 5(50000), 6(75000), 7(100000)
      }
      break;

    case P3_M17_WAIT:
      if (millis() - waitStart >= M17_WAIT) {
        moveCount++;
        if (moveCount < MOVES_PER_DIR) {
          stepper17.moveTo(M17_POS_P3[moveCount]);
          seqState = P3_M17_MOVE;
        } else {
          moveCount = 0;
          digitalWrite(EN23_PIN, LOW);
          stepper23.moveTo(0);           // ✅ 복귀: -19000 → 0
          seqState = P3_M23_HOME;
        }
      }
      break;

    // ── 복귀 ─────────────────────────────────────────────────
    case P3_M23_HOME:
      if (stepper23.distanceToGo() != 0) { stepper23.run(); }
      else {
        digitalWrite(EN23_PIN, HIGH);
        stepper17.moveTo(0);
        seqState = P4_M17_DOWN;
      }
      break;

    case P4_M17_DOWN:
      if (stepper17.distanceToGo() != 0) { stepper17.run(); }
      else {
        seqState = SEQ_DONE;
      }
      break;

    case SEQ_DONE:
      digitalWrite(EN_PIN,   HIGH);
      digitalWrite(EN23_PIN, HIGH);
      sensorActive = true;
      seqState     = IDLE;
      Serial.println(F("[SEQ] DONE"));
      break;
  }

  // --- 펌프 안정화 타이머 ---
  if (pumpJustToggled && (millis() - pumpToggleTime >= PUMP_SETTLE)) {
    pumpJustToggled = false;
  }

  // --- TDS 샘플링 (200ms) ---
  if (sensorActive && !pumpJustToggled && now - tdsLastSample >= TDS_SAMPLE_INTERVAL) {
    tdsLastSample = now;
    analogBuffer[analogBufferIndex] = analogRead(TDS_PIN);
    analogBufferIndex = (analogBufferIndex + 1) % TDS_SCOUNT;
  }

  // --- 통합 센서 읽기 + 출력 (1분마다) ---
  if (sensorActive && now - lastSensor >= SENSOR_INTERVAL) {
    lastSensor = now;
    readAndPrintSensors();
  }

  // --- 플로트 스위치 + 펌프 (30초마다 체크) ---
  if (now - lastFloatCheck >= FLOAT_CHECK_INTERVAL) {
    lastFloatCheck = now;
    int floatVal = digitalRead(FLOAT_PIN);
    if (floatVal != lastFloatState) {
      lastFloatState = floatVal;
      if (floatVal == LOW) {        // HIGH → LOW 로 변경
        waterOK = true;
        digitalWrite(PUMP_PIN, HIGH);
        pumpRunning     = true;
        pumpJustToggled = true;
        pumpToggleTime  = millis();
        Serial.println(F("[FLOAT] OK"));
      } else {
        waterOK = false;
        digitalWrite(PUMP_PIN, LOW);
        pumpRunning     = false;
        pumpJustToggled = true;
        pumpToggleTime  = millis();
        Serial.println(F("[FLOAT] LOW"));
      }
    }
  }
}
