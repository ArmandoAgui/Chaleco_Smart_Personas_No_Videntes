/************************************************************
 *  CHALECO INTELIGENTE - ENVÍO A ADAFRUIT IO (MQTT)
 *  Envía:
 *    ✔ Inmediato: Caída, UV > 8 durante 60s
 *    ✔ Cada 1 minuto: UV, Dirección, Estado general
 *    ✔ Proximidad: solo uso local (buzzer + Serial)
 ************************************************************/

#include <Wire.h>
#include <QMC5883LCompass.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

// ----------------------------
// Pines
// ----------------------------
#define UV_PIN A0
#define BUZZER_PIN D6
#define TRIG_PIN D8
#define ECHO_PIN D7

#define N 5
int readings[N];
int indexReadings = 0;

// ----------------------------
// Objetos sensores
// ----------------------------
QMC5883LCompass compass;
Adafruit_MPU6050 mpu;

// ----------------------------
// Calibración brújula
// ----------------------------
float offsetX = 0, offsetY = 0;
const float declinacion = 1.9;
const int DISTANCIA_ALERTA = 120; // cm → 1.20 m

// ----------------------------
// ADAFRUIT IO (MQTT)
// ----------------------------
#define WLAN_SSID   "TCL"
#define WLAN_PASS   "987654321"

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "armando04"
#define AIO_KEY         ""

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// FEEDS
Adafruit_MQTT_Publish feedUVIndex(&mqtt, AIO_USERNAME "/feeds/uv_index");
Adafruit_MQTT_Publish feedUVAlert(&mqtt, AIO_USERNAME "/feeds/uv_alert");
Adafruit_MQTT_Publish feedFall(&mqtt, AIO_USERNAME "/feeds/fall_detected");
Adafruit_MQTT_Publish feedHeading(&mqtt, AIO_USERNAME "/feeds/heading_degrees");
Adafruit_MQTT_Publish feedDirection(&mqtt, AIO_USERNAME "/feeds/heading_direction");
Adafruit_MQTT_Publish feedStatus(&mqtt, AIO_USERNAME "/feeds/vest_status");

// Timers
unsigned long lastCompassBeep = 0;
unsigned long compassInterval = 15000;

unsigned long uvStartHigh = 0;
bool uvTimerRunning = false;
unsigned long uvThresholdTime = 60000; // 60 s

bool fallAlertPlayed = false;

unsigned long lastPrint = 0;
unsigned long printInterval = 500;

unsigned long lastSend = 0;
unsigned long sendInterval = 60000; // 1 minuto

// VARIABLES GLOBALES PARA ENVÍO / DEBUG
float uvIndex_global = 0;
int heading_global = 0;
String direction_global = "";
float acceleration_global = 0;
float distancia_global = 0;

// ----------------------------
// WIFI & MQTT
// ----------------------------
void connectWiFi() {
  Serial.print("Conectando a WiFi...");
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado.");
}

void connectMQTT() {
  while (!mqtt.connected()) {
    Serial.print("Conectando a Adafruit IO...");
    if (mqtt.connect()) {
      Serial.println(" Conectado!");
    } else {
      Serial.println(" Error, reintentando en 3s");
      delay(3000);
    }
  }
}

// ----------------------------
// MELODÍAS
// ----------------------------
void melodyCompass(String dir) {
  if (dir == "Norte") {
    digitalWrite(BUZZER_PIN, HIGH); delay(120);
    digitalWrite(BUZZER_PIN, LOW);
  } else if (dir == "Este") {
    for (int i=0;i<2;i++){
      digitalWrite(BUZZER_PIN, HIGH); delay(100);
      digitalWrite(BUZZER_PIN, LOW); delay(120);
    }
  } else if (dir == "Sur") {
    digitalWrite(BUZZER_PIN, HIGH); delay(350);
    digitalWrite(BUZZER_PIN, LOW);
  } else if (dir == "Oeste") {
    for (int i=0;i<3;i++){
      digitalWrite(BUZZER_PIN, HIGH); delay(80);
      digitalWrite(BUZZER_PIN, LOW); delay(100);
    }
  }
}

void melodyUV() {
  for (int i=0; i<3; i++){
    digitalWrite(BUZZER_PIN, HIGH); delay(80);
    digitalWrite(BUZZER_PIN, LOW); delay(80);
  }
}

void melodyFall() {
  for (int i=0; i<5; i++){
    digitalWrite(BUZZER_PIN, HIGH); delay(150);
    digitalWrite(BUZZER_PIN, LOW); delay(100);
  }
}

void melodyDistance() {
  digitalWrite(BUZZER_PIN, HIGH); delay(200);
  digitalWrite(BUZZER_PIN, LOW);  delay(150);
}

// ----------------------------
// Auxiliares brújula
// ----------------------------
void addReading(int value) {
  readings[indexReadings] = value;
  indexReadings = (indexReadings + 1) % N;
}

int getAverage() {
  int sum=0;
  for (int i=0;i<N;i++) sum += readings[i];
  return sum / N;
}

// ----------------------------
// SETUP
// ----------------------------
void setup() {
  Serial.begin(115200);
  Wire.begin(D1, D2);

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // ---- WIFI + MQTT ----
  connectWiFi();
  connectMQTT();

  // ---- BRÚJULA ----
  compass.init();
  float maxX=-10000,minX=10000,maxY=-10000,minY=10000;
  unsigned long t=millis();
  while(millis()-t < 5000){
    compass.read();
    float x=compass.getX();
    float y=compass.getY();
    if(x>maxX)maxX=x;if(x<minX)minX=x;
    if(y>maxY)maxY=y;if(y<minY)minY=y;
    delay(50);
  }
  offsetX=(maxX+minX)/2.0;
  offsetY=(maxY+minY)/2.0;

  // ---- MPU ----
  if (!mpu.begin()) while (1);
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

// ----------------------------
// LOOP
// ----------------------------
void loop() {

  mqtt.processPackets(10);
  if (!mqtt.connected()) connectMQTT();

  unsigned long currentMillis = millis();

  // ========== UV ==========
  int sensorValue = analogRead(UV_PIN);
  float voltage = sensorValue * (1.0 / 1023.0);
  float realVoltage = voltage * 3.3;
  uvIndex_global = realVoltage * 10.0;

  // (umbral lo tenías en 2 para pruebas)
  if (uvIndex_global > 2) {
    if (!uvTimerRunning) {
      uvTimerRunning = true;
      uvStartHigh = currentMillis;
    }
    if (currentMillis - uvStartHigh >= uvThresholdTime) {
      melodyUV();
      feedUVAlert.publish(1);  // envío inmediato cuando se cumple el minuto
    }
  } else {
    uvTimerRunning = false;
  }

  // ========== BRÚJULA ==========
  compass.read();
  float x = compass.getX() - offsetX;
  float y = compass.getY() - offsetY;

  float heading = atan2(y, x) * 180.0 / PI;
  heading -= declinacion;
  if (heading < 0) heading += 360;
  if (heading >= 360) heading -= 360;

  addReading((int)heading);
  heading_global = getAverage();

  if (heading_global >= 315 || heading_global < 45) direction_global = "Norte";
  else if (heading_global < 135) direction_global = "Este";
  else if (heading_global < 225) direction_global = "Sur";
  else direction_global = "Oeste";

  if (currentMillis - lastCompassBeep >= compassInterval) {
    melodyCompass(direction_global);
    lastCompassBeep = currentMillis;
  }

  // ========== CAÍDA ==========
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  acceleration_global = sqrt(
    a.acceleration.x*a.acceleration.x +
    a.acceleration.y*a.acceleration.y +
    a.acceleration.z*a.acceleration.z
  );

  if (acceleration_global < 6.5 && !fallAlertPlayed) {
    melodyFall();
    fallAlertPlayed = true;
    feedFall.publish(1);  // ENVÍO INMEDIATO
  }
  if (acceleration_global > 9.0) fallAlertPlayed = false;

  // ========== PROXIMIDAD (ULTRASONIDO) ==========
  long duracion;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duracion = pulseIn(ECHO_PIN, HIGH, 30000); // timeout para evitar cuelgues
  distancia_global = duracion * 0.0343 / 2.0; // en cm

  if (distancia_global > 0 && distancia_global <= DISTANCIA_ALERTA) {
    melodyDistance();
  }

  // ========== ENVÍO NORMAL (1 MINUTO) ==========
  if (currentMillis - lastSend >= sendInterval) {
    lastSend = currentMillis;

    String status = "OK";
    if (fallAlertPlayed) status = "Caída detectada";
    else if (uvTimerRunning) status = "UV Alto 60s";

    feedUVIndex.publish(uvIndex_global);
    feedHeading.publish(heading_global);
    feedDirection.publish(direction_global.c_str());
    feedStatus.publish(status.c_str());

    feedUVAlert.publish(uvTimerRunning ? 1 : 0);

    Serial.println("📤 Datos enviados a Adafruit IO");
  }

  // ========== IMPRESIÓN SERIE ==========
  if (currentMillis - lastPrint >= printInterval) {
    lastPrint = currentMillis;

    Serial.println("====== DATOS ======");
    Serial.print("UV: "); Serial.println(uvIndex_global);
    Serial.print("Rumbo: "); Serial.print(heading_global);
    Serial.print("° → "); Serial.println(direction_global);
    Serial.print("Acel.: "); Serial.println(acceleration_global);
    Serial.print("Distancia: "); Serial.print(distancia_global);
    Serial.println(" cm");
    Serial.println("===================\n");
  }
}
