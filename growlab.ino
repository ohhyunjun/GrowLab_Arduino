#include <AccelStepper.h>
#include "DHT.h"
#include <string.h>
#include <stdlib.h>

// 핀 정의
#define DHT_PIN     A3
#define DHTTYPE     DHT11
#define TDS_PIN     A2
#define PH_PIN      A1
#define PUMP_PIN    3
#define FLOAT_PIN   6
#define LED_PIN     5
#define PH_PWR_PIN  4

// NEMA17
#define DIR_PIN     8
#define STEP_PIN    9
#define EN_PIN      10

// NEMA23
// 실제 배선 기준(11=DIR, 12=EN, 13=STEP)으로 맞춤.
// 핀 13은 R4 온보드 "L" LED와 공유되어 스텝 펄스마다 같이 깜빡이지만 기능상 무해함.
#define DIR23_PIN   11
#define STEP23_PIN  13
#define EN23_PIN    12

// 타이밍
const unsigned long SENSOR_INTERVAL      = 60000UL;
const unsigned long TDS_SAMPLE_INTERVAL  = 200UL;
const unsigned long PUMP_SETTLE          = 1000UL;
const unsigned long FLOAT_CHECK_INTERVAL = 30000UL;

// RPi가 응답하지 않는 오류 상황에서만 현재 위치에 정지하기 위한 타임아웃이다.
const unsigned long RPI_RESPONSE_TIMEOUT = 60000UL;

const int TDS_SCOUNT = 15;
const int PH_SCOUNT  = 5;

const long M23_STEP = 17000L;

// --- 포트별 좌표 ---
const long M17_START_POS  = 50000L; // port0 진입 전 시작 위치 (촬영 없음)
const long M17_SAFETY_POS =  5000L; // port4 -> port5, NEMA23 방향 전환 전 안전 위치 (촬영 없음)

struct PortCoord {
  long m17;
  long m23;
};

const PortCoord PORT_COORDS[8] = {
  {  75000L,      0L }, // port 0
  { 100000L,      0L }, // port 1
  { 100000L,  17000L }, // port 2
  {  75000L,  17000L }, // port 3
  {  50000L,  17000L }, // port 4
  {  50000L, -17000L }, // port 5
  {  75000L, -17000L }, // port 6
  { 100000L, -17000L }, // port 7
};

// 센서값 자체의 유효 범위
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

// pH 보정 상수 (기준점 재설정: pH 7.0 = 0.895V 실측)
// ⚠️ 기울기(PH_SLOPE)는 기존 값을 그대로 유지함 — 정확한 재보정을 원하면
//    pH 4.0 또는 pH 10.0 표준액에서 전압을 한 번 더 재서 2점 보정할 것
const float PH_V_AT_7 = 0.895f;
const float PH_SLOPE  = 10.74f;

float alertTempMin = NAN;
float alertTempMax = NAN;
float alertHumMin  = NAN;
float alertHumMax  = NAN;
float alertPhMin   = NAN;
float alertPhMax   = NAN;
float alertTdsMin  = NAN;
float alertTdsMax  = NAN;

bool cropThresholdsConfigured = false;

float lastValidTemp = 25.0f;
float lastValidHum  = 50.0f;
float lastValidPH   =  7.0f;
float lastValidTDS  =  0.0f;

enum SeqState {
  IDLE,
  P1_M17_MOVE,
  P1_M23_MOVE,
  P2_M17_MOVE,
  P2_M23_MOVE,
  P3_M17_MOVE,
  P3_M23_HOME,
  P4_M17_DOWN,
  WAIT_RPI,
  CAMERA_ERROR,
  SEQ_DONE
};

SeqState seqState = IDLE;

int activePort = -1;
int moveCount  = 0;

unsigned long photoRequestStartedAt = 0;

char serialBuffer[96];
byte serialBufferLength = 0;

bool sensorActive = true;

enum WarmupPhase { WARMUP_WAIT, WARMUP_PH_STABILIZE, WARMUP_DONE };
WarmupPhase warmupPhase = WARMUP_WAIT;
unsigned long warmupPhaseStart = 0;
const unsigned long PH_WARMUP_MS    = 30000UL;
const unsigned long PH_STABILIZE_MS =  3000UL;

DHT dht(DHT_PIN, DHTTYPE);
AccelStepper stepper17(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
AccelStepper stepper23(AccelStepper::DRIVER, STEP23_PIN, DIR23_PIN);

int   analogBuffer[TDS_SCOUNT];
int   analogBufferIndex = 0;
float c_temp = 25.0f;

float phRunningSum = 0.0f;
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

int getMedianNum(int* arr, int len) {
  int buf[TDS_SCOUNT];
  for (int i = 0; i < len; i++) buf[i] = arr[i];

  for (int i = 1; i < len; i++) {
    int key = buf[i];
    int j = i - 1;
    while (j >= 0 && buf[j] > key) {
      buf[j + 1] = buf[j];
      j--;
    }
    buf[j + 1] = key;
  }

  return (len & 1)
      ? buf[(len - 1) / 2]
      : (buf[len / 2] + buf[len / 2 - 1]) / 2;
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

bool isOutOfRange(float value, float minValue, float maxValue) {
  if (!isnan(minValue) && value < minValue) return true;
  if (!isnan(maxValue) && value > maxValue) return true;
  return false;
}

void checkAlert(float temp, float hum, float ph, float tds) {
  if (!cropThresholdsConfigured) return;

  if (isOutOfRange(temp, alertTempMin, alertTempMax)) {
    Serial.print(F("[ALERT] TEMP:"));
    Serial.println(temp, 1);
  }
  if (isOutOfRange(hum, alertHumMin, alertHumMax)) {
    Serial.print(F("[ALERT] HUM:"));
    Serial.println(hum, 1);
  }
  if (isOutOfRange(ph, alertPhMin, alertPhMax)) {
    Serial.print(F("[ALERT] PH:"));
    Serial.println(ph, 2);
  }
  if (isOutOfRange(tds, alertTdsMin, alertTdsMax)) {
    Serial.print(F("[ALERT] TDS:"));
    Serial.println(tds, 0);
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
    float voltage = getMedianNum(analogBuffer, TDS_SCOUNT) * 5.0f / 1024.0f;
    float compVoltage = voltage / (1.0f + 0.02f * (c_temp - 25.0f));

    float tds = (
      133.42f * pow(compVoltage, 3)
      - 255.86f * pow(compVoltage, 2)
      + 857.39f * compVoltage
    ) * 0.5f;

    if (
      tds >= TDS_MIN &&
      tds <= TDS_MAX &&
      (
        lastValidTDS == 0.0f ||
        fabs(tds - lastValidTDS) <= TDS_DELTA
      )
    ) {
      lastValidTDS = tds;
    }
  }

  float voltage = analogRead(PH_PIN) * 5.0f / 1024.0f;

  // pH 보정: 기준점(pH 7.0 = 0.895V) 기반 계산
  float raw = 7.00f + PH_SLOPE * (voltage - PH_V_AT_7);

  // 수온 보정
  raw += (c_temp - 25.0f) * (-0.03f);

  // LED 광간섭 보정
  if (ledState) raw -= 0.2f;

  if (
    raw >= PH_MIN &&
    raw <= PH_MAX &&
    (
      lastValidPH == 7.0f ||
      fabs(raw - lastValidPH) <= PH_DELTA
    )
  ) {
    lastValidPH = getSmoothedPH(raw);
  }

  Serial.print(F("[DATA] "));
  Serial.print(F("T:"));      Serial.print(lastValidTemp, 1);
  Serial.print(F(",H:"));     Serial.print(lastValidHum, 1);
  Serial.print(F(",PH:"));    Serial.print(lastValidPH, 2);
  Serial.print(F(",TDS:"));   Serial.print(lastValidTDS, 0);
  Serial.print(F(",LED:"));   Serial.print(ledState ? 1 : 0);
  Serial.print(F(",WATER:")); Serial.println(waterOK ? 1 : 0);

  checkAlert(lastValidTemp, lastValidHum, lastValidPH, lastValidTDS);
}

void startSequence() {
  sensorActive = false;
  activePort = -1;
  moveCount = 0;

  stepper17.setCurrentPosition(0);
  stepper23.setCurrentPosition(0);

  digitalWrite(EN_PIN, LOW);
  digitalWrite(EN23_PIN, HIGH);

  stepper17.moveTo(M17_START_POS);
  seqState = P1_M17_MOVE;

  Serial.println(F("[SEQ] START"));
}

void requestPhoto(int portIndex) {
  activePort = portIndex;
  photoRequestStartedAt = millis();

  Serial.print(F("[PHOTO] PORT:"));
  Serial.println(portIndex);

  seqState = WAIT_RPI;
}

void enterCameraError(const char* reason) {
  Serial.print(F("[ERROR] "));
  Serial.print(reason);
  Serial.print(F(" PORT:"));
  Serial.println(activePort);

  sensorActive = true;
  seqState = CAMERA_ERROR;
}

void continueAfterPhoto() {
  int completedPort = activePort;
  activePort = -1;

  switch (completedPort) {
    case 0:
      moveCount = 2;
      stepper17.moveTo(PORT_COORDS[1].m17);
      seqState = P1_M17_MOVE;
      break;

    case 1:
      moveCount = 0;
      digitalWrite(EN23_PIN, LOW);
      stepper23.moveTo(PORT_COORDS[2].m23);
      seqState = P1_M23_MOVE;
      break;

    case 2:
      moveCount = 0;
      stepper17.moveTo(PORT_COORDS[3].m17);
      seqState = P2_M17_MOVE;
      break;

    case 3:
      moveCount = 1;
      stepper17.moveTo(PORT_COORDS[4].m17);
      seqState = P2_M17_MOVE;
      break;

    case 4:
      moveCount = 2;
      stepper17.moveTo(M17_SAFETY_POS);
      seqState = P2_M17_MOVE;
      break;

    case 5:
      moveCount = 1;
      stepper17.moveTo(PORT_COORDS[6].m17);
      seqState = P3_M17_MOVE;
      break;

    case 6:
      moveCount = 2;
      stepper17.moveTo(PORT_COORDS[7].m17);
      seqState = P3_M17_MOVE;
      break;

    case 7:
      moveCount = 0;
      digitalWrite(EN23_PIN, LOW);
      stepper23.moveTo(0);
      seqState = P3_M23_HOME;
      break;

    default:
      enterCameraError("INVALID_PORT");
      break;
  }
}

bool parseConfigValue(const char* token, float* result) {
  if (strcmp(token, "NA") == 0 ||
      strcmp(token, "null") == 0 ||
      strcmp(token, "NULL") == 0) {
    *result = NAN;
    return true;
  }

  const char* p = token;
  if (*p == '+' || *p == '-') p++;

  bool hasDigit = false;
  bool hasDot = false;

  while (*p != '\0') {
    if (*p >= '0' && *p <= '9') {
      hasDigit = true;
    } else if (*p == '.' && !hasDot) {
      hasDot = true;
    } else {
      return false;
    }
    p++;
  }

  if (!hasDigit) return false;

  *result = atof(token);
  return true;
}

void applyCropThresholdConfig(char* payload) {
  float values[8];
  int valueIndex = 0;

  char* token = strtok(payload, ",");

  while (token != NULL && valueIndex < 8) {
    if (!parseConfigValue(token, &values[valueIndex])) {
      Serial.println(F("[WARN] CFG invalid value"));
      return;
    }
    valueIndex++;
    token = strtok(NULL, ",");
  }

  if (valueIndex != 8 || token != NULL) {
    Serial.println(F("[WARN] CFG requires 8 values"));
    return;
  }

  alertTempMin = values[0];
  alertTempMax = values[1];
  alertHumMin  = values[2];
  alertHumMax  = values[3];
  alertPhMin   = values[4];
  alertPhMax   = values[5];
  alertTdsMin  = values[6];
  alertTdsMax  = values[7];

  cropThresholdsConfigured = true;

  Serial.println(F("[CFG] Crop thresholds updated"));
}

void handleRpiCommand(char* command) {
  if (strncmp(command, "NEXT:", 5) == 0) {
    int portIndex = atoi(command + 5);

    if (seqState != WAIT_RPI) {
      Serial.println(F("[WARN] NEXT ignored: not waiting"));
      return;
    }
    if (portIndex != activePort) {
      Serial.println(F("[WARN] NEXT ignored: port mismatch"));
      return;
    }

    Serial.print(F("[RPi] NEXT PORT:"));
    Serial.println(portIndex);

    continueAfterPhoto();
    return;
  }

  if (strncmp(command, "ERROR:", 6) == 0) {
    int portIndex = atoi(command + 6);

    if (seqState == WAIT_RPI && portIndex == activePort) {
      enterCameraError("RPI_CAMERA_ERROR");
    } else {
      Serial.println(F("[WARN] ERROR ignored: port mismatch"));
    }
    return;
  }

  if (strncmp(command, "CFG:", 4) == 0) {
    applyCropThresholdConfig(command + 4);
    return;
  }

  Serial.print(F("[WARN] Unknown command: "));
  Serial.println(command);
}

void readSerialCommands() {
  while (Serial.available()) {
    char c = (char)Serial.read();

    if (serialBufferLength == 0 && c == 'p' && seqState == IDLE) {
      startSequence();
      continue;
    }

    if (serialBufferLength == 0 && c == 'O') {
      ledState = true;
      digitalWrite(LED_PIN, HIGH);
      Serial.println(F("[LED] ON"));
      continue;
    }

    if (serialBufferLength == 0 && c == 'o') {
      ledState = false;
      digitalWrite(LED_PIN, LOW);
      Serial.println(F("[LED] OFF"));
      continue;
    }

    if (c == '\r') continue;

    if (c == '\n') {
      if (serialBufferLength > 0) {
        serialBuffer[serialBufferLength] = '\0';
        handleRpiCommand(serialBuffer);
        serialBufferLength = 0;
      }
      continue;
    }

    if (serialBufferLength < sizeof(serialBuffer) - 1) {
      serialBuffer[serialBufferLength++] = c;
    } else {
      serialBufferLength = 0;
      Serial.println(F("[WARN] Serial command too long"));
    }
  }
}

void setup() {
  Serial.begin(9600);

  unsigned long usbWaitStart = millis();
  while (!Serial && millis() - usbWaitStart < 3000) {
    ; // wait for USB CDC
  }

  delay(2000);

  dht.begin();

  pinMode(PUMP_PIN, OUTPUT);
  pinMode(FLOAT_PIN, INPUT_PULLUP);
  // 펌프 단순 ON/OFF 제어 (PWM 속도조절 없음). 시작 시 OFF.
  // ⚠️ 배선 극성이 반대(LOW=ON)일 수 있으니 실제 동작 확인 후 필요시 반전할 것.
  digitalWrite(PUMP_PIN, LOW);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH);

  pinMode(EN23_PIN, OUTPUT);
  digitalWrite(EN23_PIN, HIGH);

  pinMode(PH_PWR_PIN, OUTPUT);
  digitalWrite(PH_PWR_PIN, LOW);

  phFull = false;
  phIndex = 0;
  phRunningSum = 0.0f;
  memset(phBuffer, 0, sizeof(phBuffer));

  stepper17.setMaxSpeed(2500);
  stepper17.setAcceleration(1000);
  stepper17.setCurrentPosition(0);

  stepper23.setMaxSpeed(1600);
  stepper23.setAcceleration(800);
  stepper23.setCurrentPosition(0);

  Serial.println(F("=== GrowLab START ==="));

  warmupPhase = WARMUP_WAIT;
  warmupPhaseStart = millis();
}

void loop() {
  unsigned long now = millis();

  if (warmupPhase != WARMUP_DONE) {
    readSerialCommands();

    if (warmupPhase == WARMUP_WAIT) {
      if (now - warmupPhaseStart >= PH_WARMUP_MS) {
        digitalWrite(PH_PWR_PIN, HIGH);
        warmupPhase = WARMUP_PH_STABILIZE;
        warmupPhaseStart = now;
      }
    } else if (warmupPhase == WARMUP_PH_STABILIZE) {
      if (now - warmupPhaseStart >= PH_STABILIZE_MS) {
        warmupPhase = WARMUP_DONE;
        Serial.println(F("[SYSTEM] Ready"));
      }
    }
    return;
  }

  readSerialCommands();

  switch (seqState) {
    case IDLE:
      break;

    case P1_M17_MOVE:
      if (stepper17.distanceToGo() != 0) {
        stepper17.run();
      } else {
        if (moveCount == 0) {
          moveCount = 1;
          stepper17.moveTo(PORT_COORDS[0].m17);
        } else if (moveCount == 1) {
          requestPhoto(0);
        } else {
          requestPhoto(1);
        }
      }
      break;

    case P1_M23_MOVE:
      if (stepper23.distanceToGo() != 0) {
        stepper23.run();
      } else {
        digitalWrite(EN23_PIN, HIGH);
        requestPhoto(2);
      }
      break;

    case P2_M17_MOVE:
      if (stepper17.distanceToGo() != 0) {
        stepper17.run();
      } else {
        if (moveCount == 0) {
          requestPhoto(3);
        } else if (moveCount == 1) {
          requestPhoto(4);
        } else {
          moveCount = 0;
          digitalWrite(EN23_PIN, LOW);
          stepper23.moveTo(PORT_COORDS[5].m23);
          seqState = P2_M23_MOVE;
        }
      }
      break;

    case P2_M23_MOVE:
      if (stepper23.distanceToGo() != 0) {
        stepper23.run();
      } else {
        digitalWrite(EN23_PIN, HIGH);
        moveCount = 0;
        stepper17.moveTo(PORT_COORDS[5].m17);
        seqState = P3_M17_MOVE;
      }
      break;

    case P3_M17_MOVE:
      if (stepper17.distanceToGo() != 0) {
        stepper17.run();
      } else {
        if (moveCount == 0) {
          requestPhoto(5);
        } else if (moveCount == 1) {
          requestPhoto(6);
        } else {
          requestPhoto(7);
        }
      }
      break;

    case P3_M23_HOME:
      if (stepper23.distanceToGo() != 0) {
        stepper23.run();
      } else {
        digitalWrite(EN23_PIN, HIGH);
        stepper17.moveTo(0);
        seqState = P4_M17_DOWN;
      }
      break;

    case P4_M17_DOWN:
      if (stepper17.distanceToGo() != 0) {
        stepper17.run();
      } else {
        seqState = SEQ_DONE;
      }
      break;

    case WAIT_RPI:
      if (millis() - photoRequestStartedAt >= RPI_RESPONSE_TIMEOUT) {
        enterCameraError("RPI_TIMEOUT");
      }
      break;

    case CAMERA_ERROR:
      break;

    case SEQ_DONE:
      digitalWrite(EN_PIN, HIGH);
      digitalWrite(EN23_PIN, HIGH);

      sensorActive = true;
      seqState = IDLE;

      Serial.println(F("[SEQ] DONE"));
      break;
  }

  if (pumpJustToggled &&
      millis() - pumpToggleTime >= PUMP_SETTLE) {
    pumpJustToggled = false;
  }

  if (sensorActive &&
      !pumpJustToggled &&
      now - tdsLastSample >= TDS_SAMPLE_INTERVAL) {
    tdsLastSample = now;

    analogBuffer[analogBufferIndex] = analogRead(TDS_PIN);
    analogBufferIndex = (analogBufferIndex + 1) % TDS_SCOUNT;
  }

  if (sensorActive &&
      now - lastSensor >= SENSOR_INTERVAL) {
    lastSensor = now;
    readAndPrintSensors();
  }

  if (now - lastFloatCheck >= FLOAT_CHECK_INTERVAL) {
    lastFloatCheck = now;

    int floatVal = digitalRead(FLOAT_PIN);

    if (floatVal != lastFloatState) {
      lastFloatState = floatVal;

      if (floatVal == LOW) {
        waterOK = true;                 // 펌프 켜짐 -> WATER: 1

        digitalWrite(PUMP_PIN, HIGH);
        pumpRunning = true;
        pumpJustToggled = true;
        pumpToggleTime = millis();

        Serial.println(F("[FLOAT] LOW -> PUMP ON"));
      } else {
        waterOK = false;                // 펌프 꺼짐 -> WATER: 0

        digitalWrite(PUMP_PIN, LOW);
        pumpRunning = false;
        pumpJustToggled = true;
        pumpToggleTime = millis();

        Serial.println(F("[FLOAT] OK -> PUMP OFF"));
      }
    }
  }
}
