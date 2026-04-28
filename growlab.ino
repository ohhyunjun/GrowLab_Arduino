#include <AccelStepper.h>
#include "DHT.h"

// --- 핀 정의 ---
#define DHT_PIN     A3
#define DHTTYPE     DHT11
#define TDS_PIN     A2
#define PH_PIN      A1
#define PUMP_PIN    7
#define FLOAT_PIN   6

// NEMA17
#define DIR_PIN     8
#define STEP_PIN    9
#define EN_PIN      10

// NEMA23
#define DIR23_PIN   11
#define STEP23_PIN  13
#define EN23_PIN    12

// --- 타이밍 ---
const unsigned long DHT_INTERVAL           = 2000UL;
const unsigned long TDS_SAMPLE_INTERVAL    = 200UL;
const unsigned long TDS_COMPUTE_INTERVAL   = 3000UL;
const unsigned long PH_INTERVAL            = 1000UL;
const unsigned long PUMP_SETTLE            = 1000UL;
const unsigned long FLOAT_CHECK_INTERVAL   = 30000UL;

const int TDS_SCOUNT = 15;
const int PH_SCOUNT  = 5;

// --- 시퀀스 상수 ---
const int           MOVES_PER_DIR = 3;
const long          M17_STEP      = 50000L;
const long          M23_STEP      = 200L;
const unsigned long M17_WAIT      = 10000UL;

// --- 필터 상수 ---
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

bool startupDone = true;

const int SENSOR_WARMUP_COUNT = 5;
int phWarmup  = 0;
int tdsWarmup = 0;

float lastValidTemp = 25.0f;
float lastValidHum  = 50.0f;
float lastValidPH   =  7.0f;
float lastValidTDS  =  0.0f;

enum SeqState {
  IDLE,
  P1_M17_MOVE, P1_M17_WAIT, P1_M23_MOVE,
  P2_M17_MOVE, P2_M17_WAIT, P2_M23_MOVE,
  P3_M17_MOVE, P3_M17_WAIT, P3_M23_HOME,
  P4_M17_DOWN, SEQ_DONE
};

SeqState      seqState      = IDLE;
int           moveCount     = 0;
long          m23TotalMoved = 0;
unsigned long waitStart     = 0;
bool          sensorActive  = true;

DHT dht(DHT_PIN, DHTTYPE);
AccelStepper stepper17(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
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

unsigned long lastDHT        = 0;
unsigned long lastTDSCompute = 0;
unsigned long tdsLastSample  = 0;
unsigned long lastPH         = 0;

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

void startSequence() {
  sensorActive  = false;
  moveCount     = 0;
  m23TotalMoved = 0;
  stepper17.setCurrentPosition(0);
  stepper23.setCurrentPosition(0);
  digitalWrite(EN_PIN, LOW);
  stepper17.moveTo(M17_STEP);
  seqState = P1_M17_MOVE;
  Serial.println(F("\n[SEQ] === 시퀀스 시작 | 센서 중지 ==="));
  Serial.println(F("[Phase1] M17 전진 1회 시작..."));
}

// -----------------------------------------------
void setup() {
  Serial.begin(9600);
  delay(2000);
  dht.begin();

  pinMode(PUMP_PIN,  OUTPUT);
  pinMode(FLOAT_PIN, INPUT_PULLUP);
  digitalWrite(PUMP_PIN, LOW);

  pinMode(EN_PIN,   OUTPUT);
  digitalWrite(EN_PIN, HIGH);

  pinMode(EN23_PIN, OUTPUT);
  digitalWrite(EN23_PIN, HIGH);

  phFull = false;
  phIndex = 0;
  phRunningSum = 0.0;
  memset(phBuffer, 0, sizeof(phBuffer));

  stepper17.setMaxSpeed(2500);
  stepper17.setAcceleration(1000);
  stepper17.setCurrentPosition(0);

  stepper23.setMaxSpeed(800);
  stepper23.setAcceleration(400);
  stepper23.setCurrentPosition(0);

  Serial.println(F("=== GrowLab 시작 ==="));
  Serial.println(F("[SYSTEM] 즉시 시작"));
  Serial.println(F("-------------------"));
}

// -----------------------------------------------
void loop() {
  unsigned long now = millis();

  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'p' && seqState == IDLE) startSequence();
  }

  switch (seqState) {
    case IDLE: break;

    case P1_M17_MOVE:
      if (stepper17.distanceToGo() != 0) { stepper17.run(); }
      else {
        waitStart = millis(); seqState = P1_M17_WAIT;
        Serial.print(F("[Phase1] M17 전진 ")); Serial.print(moveCount+1); Serial.println(F("회 완료. 대기..."));
      }
      break;

    case P1_M17_WAIT:
      if (millis() - waitStart >= M17_WAIT) {
        moveCount++;
        if (moveCount < MOVES_PER_DIR) {
          stepper17.moveTo(M17_STEP * (moveCount+1)); seqState = P1_M17_MOVE;
          Serial.print(F("[Phase1] M17 전진 ")); Serial.print(moveCount+1); Serial.println(F("회 시작..."));
        } else {
          moveCount = 0; digitalWrite(EN23_PIN, LOW);
          stepper23.moveTo(M23_STEP); m23TotalMoved = M23_STEP; seqState = P1_M23_MOVE;
          Serial.println(F("[Phase1] 완료 → M23 1번째 전진..."));
        }
      }
      break;

    case P1_M23_MOVE:
      if (stepper23.distanceToGo() != 0) { stepper23.run(); }
      else {
        digitalWrite(EN23_PIN, HIGH); moveCount = 0;
        stepper17.moveTo(M17_STEP * (MOVES_PER_DIR-1)); seqState = P2_M17_MOVE;
        Serial.println(F("[M23-1] 완료 → [Phase2] M17 역방향 시작..."));
      }
      break;

    case P2_M17_MOVE:
      if (stepper17.distanceToGo() != 0) { stepper17.run(); }
      else {
        waitStart = millis(); seqState = P2_M17_WAIT;
        Serial.print(F("[Phase2] M17 역방향 ")); Serial.print(moveCount+1); Serial.println(F("회 완료. 대기..."));
      }
      break;

    case P2_M17_WAIT:
      if (millis() - waitStart >= M17_WAIT) {
        moveCount++;
        if (moveCount < MOVES_PER_DIR) {
          stepper17.moveTo(M17_STEP * (MOVES_PER_DIR-1-moveCount)); seqState = P2_M17_MOVE;
          Serial.print(F("[Phase2] M17 역방향 ")); Serial.print(moveCount+1); Serial.println(F("회 시작..."));
        } else {
          moveCount = 0; digitalWrite(EN23_PIN, LOW);
          m23TotalMoved += M23_STEP; stepper23.moveTo(m23TotalMoved); seqState = P2_M23_MOVE;
          Serial.print(F("[Phase2] 완료 → M23 2번째 전진 누적 "));
          Serial.print(m23TotalMoved); Serial.println(F("스텝..."));
        }
      }
      break;

    case P2_M23_MOVE:
      if (stepper23.distanceToGo() != 0) { stepper23.run(); }
      else {
        digitalWrite(EN23_PIN, HIGH); moveCount = 0;
        stepper17.moveTo(M17_STEP); seqState = P3_M17_MOVE;
        Serial.println(F("[M23-2] 완료 → [Phase3] M17 전진 시작..."));
      }
      break;

    case P3_M17_MOVE:
      if (stepper17.distanceToGo() != 0) { stepper17.run(); }
      else {
        waitStart = millis(); seqState = P3_M17_WAIT;
        Serial.print(F("[Phase3] M17 전진 ")); Serial.print(moveCount+1); Serial.println(F("회 완료. 대기..."));
      }
      break;

    case P3_M17_WAIT:
      if (millis() - waitStart >= M17_WAIT) {
        moveCount++;
        if (moveCount < MOVES_PER_DIR) {
          stepper17.moveTo(M17_STEP * (moveCount+1)); seqState = P3_M17_MOVE;
          Serial.print(F("[Phase3] M17 전진 ")); Serial.print(moveCount+1); Serial.println(F("회 시작..."));
        } else {
          moveCount = 0; digitalWrite(EN23_PIN, LOW);
          stepper23.moveTo(0); seqState = P3_M23_HOME;
          Serial.print(F("[Phase3] 완료 → M23 원점 복귀 "));
          Serial.print(m23TotalMoved); Serial.println(F("스텝..."));
        }
      }
      break;

    case P3_M23_HOME:
      if (stepper23.distanceToGo() != 0) { stepper23.run(); }
      else {
        digitalWrite(EN23_PIN, HIGH); digitalWrite(EN_PIN, LOW);
        stepper17.moveTo(0); seqState = P4_M17_DOWN;
        Serial.println(F("[M23-HOME] 완료 → [Phase4] M17 하강..."));
      }
      break;

    case P4_M17_DOWN:
      if (stepper17.distanceToGo() != 0) { stepper17.run(); }
      else { seqState = SEQ_DONE; Serial.println(F("[Phase4] M17 하강 완료!")); }
      break;

    case SEQ_DONE:
      digitalWrite(EN_PIN, HIGH); digitalWrite(EN23_PIN, HIGH);
      sensorActive = true; seqState = IDLE;
      Serial.println(F("[SEQ] === 완료! 센서 재개 ===\n"));
      break;
  }

  if (pumpJustToggled && (millis() - pumpToggleTime >= PUMP_SETTLE)) {
    pumpJustToggled = false;
  }

  // --- 온습도 ---
  if (sensorActive && now - lastDHT >= DHT_INTERVAL) {
    lastDHT = now;
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    bool tempOK = !isnan(t) && t >= DHT_TEMP_MIN && t <= DHT_TEMP_MAX;
    bool humOK  = !isnan(h) && h >= DHT_HUM_MIN  && h <= DHT_HUM_MAX;
    if (tempOK && humOK) {
      lastValidTemp = t; lastValidHum = h; c_temp = t;
      Serial.print(F("[DHT11] T:")); Serial.print(t,1);
      Serial.print(F("C H:")); Serial.print(h,1); Serial.println(F("%"));
    } else {
      Serial.println(F("[DHT11] 오류"));
    }
  }

  // --- TDS 샘플링 ---
  if (sensorActive && !pumpJustToggled && now - tdsLastSample >= TDS_SAMPLE_INTERVAL) {
    tdsLastSample = now;
    analogBuffer[analogBufferIndex] = analogRead(TDS_PIN);
    analogBufferIndex = (analogBufferIndex + 1) % TDS_SCOUNT;
  }

  // --- TDS 계산 ---
  if (sensorActive && !pumpJustToggled && now - lastTDSCompute >= TDS_COMPUTE_INTERVAL) {
    lastTDSCompute = now;
    float voltage     = getMedianNum(analogBuffer, TDS_SCOUNT) * 5.0 / 1024.0;
    float compVoltage = voltage / (1.0 + 0.02 * (c_temp - 25.0));
    float tds = (133.42 * pow(compVoltage,3) - 255.86 * pow(compVoltage,2) + 857.39 * compVoltage) * 0.5;
    bool tdsOK = tds >= TDS_MIN && tds <= TDS_MAX
              && (tdsWarmup < SENSOR_WARMUP_COUNT || fabs(tds - lastValidTDS) <= TDS_DELTA);
    if (tdsOK) {
      if (tdsWarmup < SENSOR_WARMUP_COUNT) tdsWarmup++;
      lastValidTDS = tds;
      Serial.print(F("[TDS] ")); Serial.print(tds,0);
      Serial.print(F("ppm ")); Serial.print(voltage,3); Serial.println(F("V"));
    }
  }

  // --- pH ---
  if (sensorActive && now - lastPH >= PH_INTERVAL) {
    lastPH = now;
    if (!pumpJustToggled) {
      float voltage = analogRead(PH_PIN) * 5.0 / 1024.0;
      float raw = 2.50 + (7.00 - 2.50) / (0.918 - 0.568) * (voltage - 0.568);
      raw += (c_temp - 25.0f) * (-0.03f);
      bool phOK = raw >= PH_MIN && raw <= PH_MAX
               && (phWarmup < SENSOR_WARMUP_COUNT || fabs(raw - lastValidPH) <= PH_DELTA);
      if (phOK) {
        if (phWarmup < SENSOR_WARMUP_COUNT) phWarmup++;
        lastValidPH = raw;
        float ph = getSmoothedPH(raw);
        Serial.print(F("[pH] ")); Serial.print(ph,2);
        Serial.print(F(" ")); Serial.print(voltage,3); Serial.println(F("V"));
      }
    }
  }

  // --- 플로트 스위치 + 펌프 ---
  if (now - lastFloatCheck >= FLOAT_CHECK_INTERVAL) {
    lastFloatCheck = now;
    int floatVal = digitalRead(FLOAT_PIN);
    if (floatVal != lastFloatState) {
      lastFloatState = floatVal;
      if (floatVal == HIGH) {
        digitalWrite(PUMP_PIN, HIGH); pumpRunning = true;
        pumpJustToggled = true; pumpToggleTime = millis();
        Serial.println(F("[FLOAT] 정상 → 펌프 ON"));
      } else {
        digitalWrite(PUMP_PIN, LOW); pumpRunning = false;
        pumpJustToggled = true; pumpToggleTime = millis();
        Serial.println(F("[FLOAT] 부족 → 펌프 OFF"));
      }
    }
  }
}