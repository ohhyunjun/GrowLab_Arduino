#include <DHT.h>

// ── 핀 설정 ──────────────────────────────────
#define DHT_PIN       4
#define DHT_TYPE      DHT11
#define WATER_PIN     A0
#define TDS_PIN       A2

// ── TDS 설정
#define VREF          5.0
#define SCOUNT        30 

// ── 수위 기준값 ───────────────────────────────
#define WATER_LOW     300
#define WATER_MID     600
#define WATER_HIGH    900

// ── 상추 TDS 기준 (ppm) ───────────────────────
#define TDS_TOO_LOW   400     // 부족
#define TDS_OPTIMAL_L 560     // 적정 하한
#define TDS_OPTIMAL_H 840     // 적정 상한
#define TDS_HIGH      1000    // 과영양

// ── TDS 미디언 필터 버퍼 ──────────────────────
int analogBuffer[SCOUNT];
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;

DHT dht(DHT_PIN, DHT_TYPE);

// ── 미디언 필터 함수 (데이터시트 공식 알고리즘) ──
int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++) bTab[i] = bArray[i];

  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp   = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0)
    return bTab[(iFilterLen - 1) / 2];
  else
    return (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
}

// ── TDS 상태 문자열 ───────────────────────────
String getTdsStatus(float tds) {
  if      (tds < TDS_TOO_LOW)   return "양액 부족";
  else if (tds < TDS_OPTIMAL_L) return "양액 보충 필요";
  else if (tds <= TDS_OPTIMAL_H) return "적정 범위";
  else if (tds <= TDS_HIGH)     return "양액 희석 필요";
  else                          return "과영양 위험";
}

void setup() {
  Serial.begin(9600);
  pinMode(TDS_PIN, INPUT);
  dht.begin();
  Serial.println("============================");
  Serial.println("   아두이노 작동    ");
  Serial.println("============================");
}

void loop() {
  // ── TDS 샘플 수집 ────────────────
  static unsigned long tdsSampleTime = millis();
  if (millis() - tdsSampleTime > 40U) {
    tdsSampleTime = millis();
    analogBuffer[analogBufferIndex] = analogRead(TDS_PIN);
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT) analogBufferIndex = 0;
  }

  // ── 출력 (800ms 간격) ────────────────────────
  static unsigned long printTime = millis();
  if (millis() - printTime > 800U) {
    printTime = millis();

    // 온습도
    float humidity    = dht.readHumidity();
    float temperature = dht.readTemperature();
    float tempForComp = 25.0;

    if (isnan(humidity) || isnan(temperature)) {
      Serial.println("[온습도] 센서 오류 - 온도 보정 25°C 기본값 사용");
    } else {
      tempForComp = temperature;
      Serial.print("[온습도] 온도: ");
      Serial.print(temperature, 1);
      Serial.print(" °C  |  습도: ");
      Serial.print(humidity, 1);
      Serial.println(" %");
    }

    // 수위
    int waterRaw = analogRead(WATER_PIN);
    String waterLevel;
    if      (waterRaw < WATER_LOW)  waterLevel = "낮음";
    else if (waterRaw < WATER_MID)  waterLevel = "보통";
    else if (waterRaw < WATER_HIGH) waterLevel = "높음";
    else                            waterLevel = "가득";

    Serial.print("[수위]   Raw: ");
    Serial.print(waterRaw);
    Serial.print("  |  상태: ");
    Serial.println(waterLevel);

    // TDS (미디언 필터 + 온도 보정)
    for (int i = 0; i < SCOUNT; i++)
      analogBufferTemp[i] = analogBuffer[i];

    float avgVoltage   = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 1024.0;
    float compCoeff    = 1.0 + 0.02 * (tempForComp - 25.0);
    float compVoltage  = avgVoltage / compCoeff;
    float tdsValue     = (133.42 * pow(compVoltage, 3)
                        - 255.86 * pow(compVoltage, 2)
                        + 857.39 * compVoltage) * 0.5;

    Serial.print("[TDS]    값: ");
    Serial.print(tdsValue, 0);
    Serial.print(" ppm  |  상태: ");
    Serial.println(getTdsStatus(tdsValue));

    Serial.println("----------------------------");
  }
}