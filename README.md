# Chaleco Inteligente con ESP8266 – Fusión de Sensores MPU6050, QMC5883L y UV

Este proyecto integra tres sensores principales sobre un ESP8266 (NodeMCU/Wemos D1 Mini) para crear un **sistema de asistencia multisensorial**, ideal para chalecos inteligentes o dispositivos de navegación.

---

## 🚀 Funcionalidad

- **QMC5883L (brújula digital)**  
  Calcula la orientación geográfica (N, S, E, O) con corrección de declinación magnética local (+1.9° para El Salvador).  
  Cada dirección emite un tono característico por el buzzer (D6).

- **MPU6050 (acelerómetro y giroscopio)**  
  Mide la aceleración total y detecta caídas (Atotal < 5 m/s²).  
  En caso de caída, activa el buzzer durante 2 segundos.

- **Sensor UV GUVAS-S12SD**  
  Conectado al pin analógico A0, mide el índice UV aproximado en tiempo real.  
  Los valores se imprimen en el monitor serial con conversión de voltaje a índice.

---

## 🧠 Conexiones (NodeMCU / ESP8266)

| Componente | SDA | SCL | VCC | GND | Otro pin |
|-------------|-----|-----|-----|-----|-----------|
| **MPU6050** | D2 | D1 | 3.3V | GND | — |
| **QMC5883L** | D2 | D1 | 3.3V | GND | — |
| **Sensor UV** | — | — | 3.3V | GND | A0 |
| **Buzzer activo** | — | — | — | GND | D6 |

> ⚠️ Ambos sensores I²C (MPU6050 y QMC5883L) comparten las líneas D1–D2 sin conflicto, ya que usan direcciones distintas (`0x68` y `0x0D`).

---

## 🔍 Calibración
Durante los primeros 5 segundos de encendido, el sistema calibra automáticamente los offsets del magnetómetro QMC5883L.  
Gira el sensor lentamente en 360° antes de que se muestre “✅ Calibración completa!”.

---

## 📡 Requisitos

- ESP8266 (NodeMCU o Wemos D1 Mini)
- Librerías Arduino:
  - `Wire.h`
  - `Adafruit_MPU6050`
  - `Adafruit_Sensor`
  - `QMC5883LCompass`
- Arduino IDE configurado para “NodeMCU 1.0 (ESP-12E Module)”

---

## 🧩 Autoría

Proyecto desarrollado por **Lili & Armand**  
Curso: *Programación de Artefactos – UCA (El Salvador)*  
Año: 2025  
