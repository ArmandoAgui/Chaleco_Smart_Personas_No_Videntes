#include <Wire.h>
#include <QMC5883LCompass.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

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
QMC5883LCompass compass;
Adafruit_MPU6050 mpu;

// ----------------------------
float offsetX = 0, offsetY = 0;
const float declinacion = 1.9;
const int DISTANCIA_ALERTA = 120;

// ----------------------------
// Timers
// ----------------------------
unsigned long lastCompassBeep = 0;
unsigned long compassInterval = 15000; 

unsigned long uvStartHigh = 0;
bool uvTimerRunning = false;
unsigned long uvThresholdTime = 60000;

bool fallAlertPlayed = false;

// Intervalo de impresión
unsigned long lastPrint = 0;
unsigned long printInterval = 2000;  // 0.5 s

// Variables para imprimir datos
float uvIndex_global = 0;
int heading_global = 0;
String direction_global = "";
float acceleration_global = 0;
float distancia_global = 0;

// ----------------------------
// MELODÍAS
// ----------------------------
void melodyCompass(String dir) {
  Serial.print("🔔 Melodía brújula → ");
  Serial.println(dir);

  if (dir == "Norte") {
    digitalWrite(BUZZER_PIN, HIGH); delay(120);
    digitalWrite(BUZZER_PIN, LOW);
  }
  else if (dir == "Este") {
    for (int i=0; i<2; i++){
      digitalWrite(BUZZER_PIN, HIGH); delay(100);
      digitalWrite(BUZZER_PIN, LOW);  delay(120);
    }
  }
  else if (dir == "Sur") {
    digitalWrite(BUZZER_PIN, HIGH); delay(350);
    digitalWrite(BUZZER_PIN, LOW);
  }
  else if (dir == "Oeste") {
    for (int i=0; i<3; i++){
      digitalWrite(BUZZER_PIN, HIGH); delay(80);
      digitalWrite(BUZZER_PIN, LOW);  delay(100);
    }
  }
}

void melodyUV() {
  Serial.println("🌞🔔 Alerta UV > 8 durante 60s → Melodía UV");
  for (int i=0; i<3; i++){
    digitalWrite(BUZZER_PIN, HIGH); delay(80);
    digitalWrite(BUZZER_PIN, LOW);  delay(80);
  }
}

void melodyFall() {
  Serial.println("🛑🪂 CAÍDA DETECTADA → Melodía caída");
  for (int i=0; i<5; i++){
    digitalWrite(BUZZER_PIN, HIGH); delay(150);
    digitalWrite(BUZZER_PIN, LOW);  delay(100);
  }
}

void melodyDistance() {
  Serial.println("🚧🔔 Obstáculo a menos de 120 cm → Melodía obstáculo");
  digitalWrite(BUZZER_PIN, HIGH); delay(200);
  digitalWrite(BUZZER_PIN, LOW);  delay(150);
}

// ----------------------------
// AUXILIARES
// ----------------------------
void addReading(int value) {
  readings[indexReadings] = value;
  indexReadings = (indexReadings + 1) % N;
}

int getAverage() {
  int sum = 0;
  for (int i = 0; i < N; i++) sum += readings[i];
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

  Serial.println("🔧 Inicializando sensores...");

  // Calibrar brújula
  compass.init();
  float maxX=-10000, minX=10000, maxY=-10000, minY=10000;
  unsigned long t = millis();
  while (millis()-t < 5000){
    compass.read();
    float x = compass.getX();
    float y = compass.getY();
    if (x>maxX) maxX=x; if (x<minX) minX=x;
    if (y>maxY) maxY=y; if (y<minY) minY=y;
    delay(50);
  }
  offsetX = (maxX+minX)/2.0;
  offsetY = (maxY+minY)/2.0;

  // Inicializar MPU
  if (!mpu.begin()) while (1);
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("🚀 Sistema listo.");
}

// ----------------------------
// LOOP
// ----------------------------
void loop() {

  unsigned long currentMillis = millis();

  // ----------- UV SENSOR -----------
  int sensorValue = analogRead(UV_PIN);
  float voltage = sensorValue * (1.0 / 1023.0);
  float realVoltage = voltage * 3.3;
  uvIndex_global = realVoltage * 10.0;

  if (uvIndex_global > 8) {
    if (!uvTimerRunning) {
      uvTimerRunning = true;
      uvStartHigh = currentMillis;
    }
    if (currentMillis - uvStartHigh >= uvThresholdTime) {
      melodyUV();
    }
  } else {
    uvTimerRunning = false;
  }

  // ----------- BRÚJULA -----------
  compass.read();
  float x = compass.getX() - offsetX;
  float y = compass.getY() - offsetY;

  float heading = atan2(y, x) * 180.0 / PI;
  heading -= declinacion;
  if (heading < 0) heading += 360;
  if (heading >= 360) heading -= 360;

  addReading((int)heading);
  heading_global = getAverage();

  if ((heading_global >= 315) || (heading_global < 45)) direction_global = "Norte";
  else if (heading_global >= 45 && heading_global < 135) direction_global = "Este";
  else if (heading_global >= 135 && heading_global < 225) direction_global = "Sur";
  else direction_global = "Oeste";

  if (currentMillis - lastCompassBeep >= compassInterval) {
    melodyCompass(direction_global);
    lastCompassBeep = currentMillis;
  }

  // ----------- CAÍDA -----------
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
  }
  if (acceleration_global > 9.0) fallAlertPlayed = false;

  // ----------- ULTRASONIDO -----------
  long duracion;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duracion = pulseIn(ECHO_PIN, HIGH);
  distancia_global = duracion * 0.0343 / 2;

  if (distancia_global > 0 && distancia_global <= DISTANCIA_ALERTA) {
    melodyDistance();
  }

  // ----------- IMPRESIÓN CADA 500 ms -----------
  if (currentMillis - lastPrint >= printInterval) {
    lastPrint = currentMillis;

    Serial.println("===== DATOS =====");
    Serial.print("UV Index: "); Serial.println(uvIndex_global);
    Serial.print("Brújula: "); Serial.print(heading_global);
    Serial.print("° → "); Serial.println(direction_global);
    Serial.print("Aceleración: "); Serial.println(acceleration_global);
    Serial.print("Distancia: "); Serial.print(distancia_global); Serial.println(" cm");
    Serial.println("=================\n");
  }
}

