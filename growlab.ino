#include <AccelStepper.h>
#include "DHT.h"
#include <string.h>
#include <stdlib.h>

// 핀 정의
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

// 타이밍
const unsigned long SENSOR_INTERVAL      = 60000UL;
const unsigned long TDS_SAMPLE_INTERVAL  = 200UL;
const unsigned long PUMP_SETTLE          = 1000UL;
const unsigned long FLOAT_CHECK_INTERVAL = 30000UL;

// RPi가 응답하지 않는 오류 상황에서만 현재 위치에 정지하기 위한 타임아웃이다.
const unsigned long RPI_RESPONSE_TIMEOUT = 60000UL;

const int TDS_SCOUNT = 15;
const int PH_SCOUNT  = 5;

// 모터 좌표
const int  MOVES_PER_DIR = 3;
const long M23_STEP      = 17000L;

// 기존의 검증된 안전 이동 좌표
const long M17_POS_P1[3] = { 50000L,  75000L, 100000L };
const long M17_POS_P2[3] = { 75000L,  50000L,   5000L };
const long M17_POS_P3[3] = { 50000L,  75000L, 100000L };

/*
포트 좌표
port 0 = NEMA17  75000, NEMA23      0
port 1 = NEMA17 100000, NEMA23      0
port 2 = NEMA17 100000, NEMA23 +17000
port 3 = NEMA17  75000, NEMA23 +17000
port 4 = NEMA17  50000, NEMA23 +17000
port 5 = NEMA17  50000, NEMA23 -17000
port 6 = NEMA17  75000, NEMA23 -17000
port 7 = NEMA17 100000, NEMA23 -17000
*/

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

// 서버 품종 기준으로 RPi가 내려 주는 런타임 경고 범위
// CFG 명령 전에는 NAN 상태이며, 경고를 보내지 않는다.
float alertTempMin = NAN;
float alertTempMax = NAN;
float alertHumMin  = NAN;
float alertHumMax  = NAN;
float alertPhMin   = NAN;
float alertPhMax   = NAN;
float alertTdsMin  = NAN;
float alertTdsMax  = NAN;

bool cropThresholdsConfigured = false;

// 상태
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

// RPi 명령 문자열 버퍼
// CFG:18,25,40,70,5.5,6.5,400,800 형태를 받을 수 있도록 여유 확보
char serialBuffer[96];
byte serialBufferLength = 0;

bool sensorActive = true;

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

// 센서 보조 함수
int getMedianNum(int* arr, int len) {
  int buf[TDS_SCOUNT];

  for (int i = 0; i < len; i++) {
    buf[i] = arr[i];
  }

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

  if (!phFull && phIndex == 0) {
    phFull = true;
  }

  int len = phFull ? PH_SCOUNT : phIndex;
  return (len > 0) ? phRunningSum / len : newVal;
}

// 품종별 경고
// 최소·최대 중 하나가 설정돼 있지 않으면 설정된 방향만 검사한다.
bool isOutOfRange(float value, float minValue, float maxValue) {
  if (!isnan(minValue) && value < minValue) {
    return true;
  }

  if (!isnan(maxValue) && value > maxValue) {
    return true;
  }

  return false;
}

void checkAlert(float temp, float hum, float ph, float tds) {
  // 서버에서 품종 기준이 아직 내려오지 않았으면 경고하지 않는다.
  if (!cropThresholdsConfigured) {
    return;
  }

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

// 센서 읽기
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

    if (tds >= TDS_MIN &&
        tds <= TDS_MAX &&
        fabs(tds - lastValidTDS) <= TDS_DELTA) {
      lastValidTDS = tds;
    }
  }

  float voltage = analogRead(PH_PIN) * 5.0f / 1024.0f;

  float raw = 2.50f
      + (7.00f - 2.50f) / (0.918f - 0.568f) * (voltage - 0.568f);

  // 수온 보정
  raw += (c_temp - 25.0f) * (-0.03f);

  // LED 광간섭 보정
  if (ledState) {
    raw -= 0.5f;
  }

  if (raw >= PH_MIN &&
      raw <= PH_MAX &&
      fabs(raw - lastValidPH) <= PH_DELTA) {
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

// 촬영 시퀀스 시작
void startSequence() {
  sensorActive = false;
  activePort = -1;
  moveCount = 0;

  // 실제 모터 홈 이동이 아니라 현재 위치를 원점으로 가정하는 코드
  stepper17.setCurrentPosition(0);
  stepper23.setCurrentPosition(0);

  digitalWrite(EN_PIN, LOW);
  digitalWrite(EN23_PIN, HIGH);

  // 50000은 촬영하지 않는 시작용 이동 좌표
  stepper17.moveTo(M17_POS_P1[0]);
  seqState = P1_M17_MOVE;

  Serial.println(F("[SEQ] START"));
}

// Arduino -> RPi 사진 요청
// RPi가 ESP32-CAM 촬영·YOLO·재촬영을 끝내기 전까지 멈춘다.
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

  // 홈 스위치가 없으므로 오류 상태에서 자동 이동·자동 복귀하지 않는다.
  sensorActive = true;
  seqState = CAMERA_ERROR;
}

// RPi가 NEXT:<포트>를 보냈을 때 다음 안전 이동 루트 진행
void continueAfterPhoto() {
  int completedPort = activePort;
  activePort = -1;

  switch (completedPort) {
    case 0:
      moveCount = 2;
      stepper17.moveTo(M17_POS_P1[2]);
      seqState = P1_M17_MOVE;
      break;

    case 1:
      moveCount = 0;
      digitalWrite(EN23_PIN, LOW);
      stepper23.moveTo(M23_STEP);
      seqState = P1_M23_MOVE;
      break;

    case 2:
      moveCount = 0;
      stepper17.moveTo(M17_POS_P2[0]);
      seqState = P2_M17_MOVE;
      break;

    case 3:
      moveCount = 1;
      stepper17.moveTo(M17_POS_P2[1]);
      seqState = P2_M17_MOVE;
      break;

    case 4:
      // 5000은 NEMA23 방향 전환을 위한 안전 위치
      moveCount = 2;
      stepper17.moveTo(M17_POS_P2[2]);
      seqState = P2_M17_MOVE;
      break;

    case 5:
      moveCount = 1;
      stepper17.moveTo(M17_POS_P3[1]);
      seqState = P3_M17_MOVE;
      break;

    case 6:
      moveCount = 2;
      stepper17.moveTo(M17_POS_P3[2]);
      seqState = P3_M17_MOVE;
      break;

    case 7:
      // 모든 포트 촬영 완료: NEMA23 -17000 -> 0
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

// RPi -> Arduino 품종 기준 설정
// CFG:온도최소,온도최대,습도최소,습도최대,pH최소,pH최대,TDS최소,TDS최대
// 예시
// CFG:18,25,40,70,5.5,6.5,400,800
// 품종 기준이 없는 값은 NA 사용 가능
// CFG:18,25,NA,NA,5.5,6.5,400,800
bool parseConfigValue(const char* token, float* result) {
  if (strcmp(token, "NA") == 0 ||
      strcmp(token, "null") == 0 ||
      strcmp(token, "NULL") == 0) {
    *result = NAN;
    return true;
  }

  const char* p = token;

  // 부호 허용
  if (*p == '+' || *p == '-') {
    p++;
  }

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

  if (!hasDigit) {
    return false;
  }

  // AVR에서는 strtof 대신 atof 사용
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

  // 값이 8개가 아니거나 8개를 초과하면 적용하지 않음
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

// RPi -> Arduino 명령
// NEXT:5\n   : 현재 5번 포트의 촬영·검증 종료, 다음 포트 이동
// ERROR:5\n  : RPi/ESP32-CAM 오류, 현재 위치 정지
// CFG:...\n  : 품종별 센서 경고 기준 갱신
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

// 시리얼 수신
// p : 전체 촬영 시작
// O : LED ON
// o : LED OFF
// NEXT:5\n
// ERROR:5\n
// CFG:18,25,40,70,5.5,6.5,400,800\n
void readSerialCommands() {
  while (Serial.available()) {
    char c = (char)Serial.read();

    // 기존 한 글자 제어 명령 유지
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

    if (c == '\r') {
      continue;
    }

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

// 초기화
void setup() {
  Serial.begin(9600);
  delay(2000);

  dht.begin();

  pinMode(PUMP_PIN, OUTPUT);
  pinMode(FLOAT_PIN, INPUT_PULLUP);
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

  // pH 센서 안정화
  delay(30000);
  digitalWrite(PH_PWR_PIN, HIGH);
  delay(3000);

  Serial.println(F("[SYSTEM] Ready"));
}

// 메인 루프
void loop() {
  unsigned long now = millis();

  // RPi 또는 시리얼 모니터 명령 수신
  readSerialCommands();

  // 카메라 이동 상태머신
  switch (seqState) {
    case IDLE:
      break;

    // P1: 50000 -> 75000(port 0) -> 100000(port 1)
    case P1_M17_MOVE:
      if (stepper17.distanceToGo() != 0) {
        stepper17.run();
      } else {
        if (moveCount == 0) {
          // 50000은 촬영하지 않는 시작 위치
          moveCount = 1;
          stepper17.moveTo(M17_POS_P1[1]);
        } else if (moveCount == 1) {
          requestPhoto(0);
        } else {
          requestPhoto(1);
        }
      }
      break;

    // NEMA23: 0 -> +17000, port 2
    case P1_M23_MOVE:
      if (stepper23.distanceToGo() != 0) {
        stepper23.run();
      } else {
        digitalWrite(EN23_PIN, HIGH);
        requestPhoto(2);
      }
      break;

    // P2: 75000(port 3) -> 50000(port 4) -> 5000(안전 위치)
    case P2_M17_MOVE:
      if (stepper17.distanceToGo() != 0) {
        stepper17.run();
      } else {
        if (moveCount == 0) {
          requestPhoto(3);
        } else if (moveCount == 1) {
          requestPhoto(4);
        } else {
          // 5000에서는 촬영하지 않음.
          // 이 위치에서만 NEMA23을 +17000 -> -17000으로 이동.
          moveCount = 0;
          digitalWrite(EN23_PIN, LOW);
          stepper23.moveTo(-M23_STEP);
          seqState = P2_M23_MOVE;
        }
      }
      break;

    // NEMA23: +17000 -> -17000
    case P2_M23_MOVE:
      if (stepper23.distanceToGo() != 0) {
        stepper23.run();
      } else {
        digitalWrite(EN23_PIN, HIGH);

        moveCount = 0;
        stepper17.moveTo(M17_POS_P3[0]);
        seqState = P3_M17_MOVE;
      }
      break;

    // P3: 50000(port 5) -> 75000(port 6) -> 100000(port 7)
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

    // NEMA23: -17000 -> 0
    case P3_M23_HOME:
      if (stepper23.distanceToGo() != 0) {
        stepper23.run();
      } else {
        digitalWrite(EN23_PIN, HIGH);

        stepper17.moveTo(0);
        seqState = P4_M17_DOWN;
      }
      break;

    // NEMA17: 100000 -> 0
    case P4_M17_DOWN:
      if (stepper17.distanceToGo() != 0) {
        stepper17.run();
      } else {
        seqState = SEQ_DONE;
      }
      break;

    // RPi가 NEXT:<현재 포트>를 보낼 때까지 현재 좌표 유지
    case WAIT_RPI:
      if (millis() - photoRequestStartedAt >= RPI_RESPONSE_TIMEOUT) {
        enterCameraError("RPI_TIMEOUT");
      }
      break;

    // 오류 시 자동 복귀·자동 재시작 금지
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

  // 펌프 전환 직후 TDS 안정화
  if (pumpJustToggled &&
      millis() - pumpToggleTime >= PUMP_SETTLE) {
    pumpJustToggled = false;
  }

  // TDS ADC 샘플링
  if (sensorActive &&
      !pumpJustToggled &&
      now - tdsLastSample >= TDS_SAMPLE_INTERVAL) {
    tdsLastSample = now;

    analogBuffer[analogBufferIndex] = analogRead(TDS_PIN);
    analogBufferIndex = (analogBufferIndex + 1) % TDS_SCOUNT;
  }

  // 센서 출력 및 품종별 경고: 1분마다
  if (sensorActive &&
      now - lastSensor >= SENSOR_INTERVAL) {
    lastSensor = now;
    readAndPrintSensors();
  }

  // 플로트 스위치 및 펌프: 30초마다
  if (now - lastFloatCheck >= FLOAT_CHECK_INTERVAL) {
    lastFloatCheck = now;

    int floatVal = digitalRead(FLOAT_PIN);

    if (floatVal != lastFloatState) {
      lastFloatState = floatVal;

      // 현재 배선 기준: LOW를 물 정상 상태로 가정
      if (floatVal == LOW) {
        waterOK = true;

        digitalWrite(PUMP_PIN, HIGH);
        pumpRunning = true;
        pumpJustToggled = true;
        pumpToggleTime = millis();

        Serial.println(F("[FLOAT] OK"));
      } else {
        waterOK = false;

        digitalWrite(PUMP_PIN, LOW);
        pumpRunning = false;
        pumpJustToggled = true;
        pumpToggleTime = millis();

        Serial.println(F("[FLOAT] LOW"));
      }
    }
  }
}
