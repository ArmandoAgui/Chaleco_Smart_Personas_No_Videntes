#include <Wire.h>
#include <QMC5883LCompass.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ----------------------------
// Pines y configuraciones
// ----------------------------
#define UV_PIN A0
#define BUZZER_PIN D6  // mismo buzzer para dirección y caída
#define TRIG_PIN D8
#define ECHO_PIN D7

#define N 5  // Número de lecturas para promedio
int readings[N];
int indexReadings = 0;

// ----------------------------
// Objetos de sensores
// ----------------------------
QMC5883LCompass compass;
Adafruit_MPU6050 mpu;

// ----------------------------
// Variables de calibración
// ----------------------------
float offsetX = 0, offsetY = 0;
const float declinacion = 1.9; // declinación magnética de El Salvador
const int DISTANCIA_ALERTA = 120; // cm

// ----------------------------
// Funciones auxiliares
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

void beepDirection(String direction) {
  if (direction == "Norte") {
    tone(BUZZER_PIN, 1000, 100);
  } else if (direction == "Noreste") {
    tone(BUZZER_PIN, 1200, 150);
  } else if (direction == "Este") {
    tone(BUZZER_PIN, 1500, 200);
  } else if (direction == "Sureste") {
    tone(BUZZER_PIN, 1800, 250);
  } else if (direction == "Sur") {
    tone(BUZZER_PIN, 900, 300);
  } else if (direction == "Suroeste") {
    tone(BUZZER_PIN, 800, 350);
  } else if (direction == "Oeste") {
    tone(BUZZER_PIN, 700, 400);
  } else if (direction == "Noroeste") {
    tone(BUZZER_PIN, 600, 450);
  }
}

// ----------------------------
// SETUP
// ----------------------------
void setup() {
  Serial.begin(115200);
  Wire.begin(D2, D1);  // SDA, SCL (ajustado a QMC y MPU)

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // --- Inicialización brújula ---
  compass.init();
  Serial.println("Gira el sensor lentamente 360° para calibrar...");
  float maxX = -10000, minX = 10000, maxY = -10000, minY = 10000;
  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {
    compass.read();
    float x = compass.getX();
    float y = compass.getY();
    if (x > maxX) maxX = x;
    if (x < minX) minX = x;
    if (y > maxY) maxY = y;
    if (y < minY) minY = y;
    delay(50);
  }
  offsetX = (maxX + minX) / 2.0;
  offsetY = (maxY + minY) / 2.0;
  Serial.println("✅ Calibración brújula completa!");

  // --- Inicialización MPU6050 ---
  if (!mpu.begin()) {
    Serial.println("❌ No se detectó el MPU6050.");
    while (1);
  }
  Serial.println("✅ MPU6050 listo.");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(100);
}

// ----------------------------
// LOOP PRINCIPAL
// ----------------------------
void loop() {
  // --- Sensor UV ---
  int sensorValue = analogRead(UV_PIN);
  float voltage = sensorValue * (1.0 / 1023.0);
  float realVoltage = voltage * 3.3;
  float uvIndex = realVoltage * 10.0;
  Serial.print("UV ADC: "); Serial.print(sensorValue);
  Serial.print("\tVoltaje: "); Serial.print(realVoltage, 2);
  Serial.print(" V\tÍndice UV: "); Serial.println(uvIndex, 1);

  // --- Brújula ---
  compass.read();
  float x = compass.getX() - offsetX;
  float y = compass.getY() - offsetY;
  float heading = atan2(y, x) * 180.0 / PI;
  heading -= declinacion;
  if (heading < 0) heading += 360.0;
  if (heading >= 360.0) heading -= 360.0;
  addReading((int)heading);
  int avgHeading = getAverage();

  String direction;
  if ((avgHeading >= 315 && avgHeading <= 360) || (avgHeading >= 0 && avgHeading < 45))
    direction = "Norte";
  else if (avgHeading >= 45 && avgHeading < 135)
    direction = "Este";
  else if (avgHeading >= 135 && avgHeading < 225)
    direction = "Sur";
  else if (avgHeading >= 225 && avgHeading < 315)
    direction = "Oeste";

  Serial.print("Grados geográficos: ");
  Serial.print(avgHeading);
  Serial.print("°  Dirección: ");
  Serial.println(direction);

  beepDirection(direction);

  // --- MPU6050 (detección de caída) ---
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float Atotal = sqrt(
    a.acceleration.x * a.acceleration.x +
    a.acceleration.y * a.acceleration.y +
    a.acceleration.z * a.acceleration.z
  );

  Serial.print("A_total: ");
  Serial.print(Atotal);
  Serial.println(" m/s²");

  if (Atotal < 6.5) {
    Serial.println("⚠️ ¡Caída detectada! Activando buzzer...");
    digitalWrite(BUZZER_PIN, HIGH);
    delay(2000);
    digitalWrite(BUZZER_PIN, LOW);
  }

  // --- Sensor ultrasónico HC-SR04 ---
  long duracion;
  float distancia;

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duracion = pulseIn(ECHO_PIN, HIGH);
  distancia = duracion * 0.0343 / 2;

  Serial.print("Distancia: ");
  Serial.print(distancia);
  Serial.println(" cm");

  if (distancia <= DISTANCIA_ALERTA && distancia > 0) {
    Serial.println("🚧 Obstáculo detectado a menos de 1.20 m");
    tone(BUZZER_PIN, 1000);  // tono de advertencia
  } else {
    noTone(BUZZER_PIN);
  }

  delay(1000);
}
