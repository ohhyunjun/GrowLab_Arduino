#include "DHT.h"
#include <SoftwareSerial.h>
#include <AccelStepper.h>

#define DHT_PIN 4       
#define DHTTYPE DHT11
DHT dht(DHT_PIN, DHTTYPE);

#define TDS_PIN A2
#define VREF 5.0
#define SCOUNT 30
unsingned long tdsLastSampleTime = 0;
unsingned long tdsLastComputeTime = 0;

#define PH_PIN A1
#define V_COLA 0.54
#define PH_COLA 2.50
#define V_PH7 0.89
#define PH_PH7 7.00
#define PH_SCOUNT 10
float phBuffer[PH_SCOUNT];
int   phBufferIndex = 0;
bool  phBufferFull = false;
int analogBuffer[SCOUNT];
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
float tdsValue = 0.0;

#define PUMP_PIN 5
bool pumpState = false;

#define FLOAT_PIN 6

#define DIR_PIN 8
#define STEP_PIN 9
#define EN_PIN 10
#define STEPS_PER_REV 200
#define STEP_DELAY    200
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

#define BT_RX 3
#define BT_TX 2
SoftwareSerial bluetooth(BT_RX, BT_TX);

unsigned long lastSensorUpdate = 0;
const unsigned long sensorInterval = 2000;
float c_temp, c_humi, c_tds, c_ph;

void setup() {
  Serial.begin(9600);  
  dht.begin();
  pinMode(TDS_PIN, INPUT);
  pinMode(PH_PIN, INPUT);
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(500);
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(FLOAT_PIN, INPUT_PULLUP);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);
  bluetooth.begin(9600);

}

void loop() {
  float humidity = dht.readHumidity();       
  float temperature = dht.readTemperature(); 
  updateTDS(temperature);
  float phRaw = readPH(temperature);
  float phSmoothed = getSmoothedPH(phRaw);
  int floatState = digitalRead(FLOAT_PIN);

  if (floatState == LOW) {
    Serial.println("수위 : 물 감지");
    digitalWrite(PUMP_PIN, HIGH);
    Serial.println("펌프 작동 중");
  } else {
    Serial.println("수위 : 물 없음");
    digitalWrite(PUMP_PIN, LOW);
  }

  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Failed to read from DHT sensor!"); 
    return;
  }

  bluetooth.println("테스트중....");

  Serial.print("Temperature: ");
  Serial.print((int)temperature);
  Serial.print(" *C, ");
  Serial.print("Humidity: ");
  Serial.print((int)humidity);
  Serial.println(" %");
  Serial.print("TDS: ");
  Serial.print(tdsValue, 0);
  Serial.print("pH: ");
  Serial.print(phSmoothed, 2);
  Serial.println("스탭모터 작동");
  rotateStepper(STEPS_PER_REV * 3, true);

  delay(2000); // 2초 대기 후 다시 측정
}


int getMedianNum(int bArray[], int len) {
  int bTab[len];
  for (int i = 0; i < len; i++) bTab[i] = bArray[i];
  for (int j = 0; j < len - 1; j++) {
    for (int i = 0; i < len - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        int tmp = bTab[i]; bTab[i] = bTab[i+1]; bTab[i+1] = tmp;
      }
    }
  }
  return (len & 1) ? bTab[(len-1)/2] : (bTab[len/2] + bTab[len/2-1]) / 2;
}

void updateTDS(float temperature) {
  unsigned long now = millis();

  if (now - tdsLastSampleTime >= 40UL) {
    tdsLastSampleTime = now;
    analogBuffer[analogBufferIndex++] = analogRead(TDS_PIN);
    if (analogBufferIndex == SCOUNT) analogBufferIndex = 0;
  }

  if (now - tdsLastComputeTime >= 800UL) {
    tdsLastComputeTime = now;

    for (int i = 0; i < SCOUNT; i++) analogBufferTemp[i] = analogBuffer[i];

    float voltage = getMedianNum(analogBufferTemp, SCOUNT) * VREF / 1024.0;

    float compVoltage = voltage / (1.0 + 0.02 * (temperature - 25.0));

    tdsValue = (133.42 * pow(compVoltage, 3)
              - 255.86 * pow(compVoltage, 2)
              + 857.39 * compVoltage) * 0.5;
  }
}

float readPH(float temperature) {
  long sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += analogRead(PH_PIN);
    delay(10);
  }
  float voltage = (sum / 10.0) * VREF / 1024.0;
  float slope   = (PH_PH7 - PH_COLA) / (V_PH7 - V_COLA);
  float ph      = PH_COLA + slope * (voltage - V_COLA);
  ph += (temperature - 25.0f) * (-0.03f);
  return ph;
}

float getSmoothedPH(float newVal) {
  phBuffer[phBufferIndex] = newVal;
  phBufferIndex++;
  if (phBufferIndex >= PH_SCOUNT) { phBufferIndex = 0; phBufferFull = true; }
  int   len = phBufferFull ? PH_SCOUNT : phBufferIndex;
  float sum = 0;
  for (int i = 0; i < len; i++) sum += phBuffer[i];
  return sum / len;
}

void rotateStepper(long steps, bool direction) {
  digitalWrite(DIR_PIN, direction ? HIGH : LOW);
  for (long i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_DELAY);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_DELAY);
  }
}