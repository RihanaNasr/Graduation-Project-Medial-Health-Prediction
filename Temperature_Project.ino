#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#define Err 0
// ---------- MAX30102 (Heart Rate) ----------
MAX30105 maxSensor;

const byte RATE_SIZE = 4;      // how many last beats we average
byte rates[RATE_SIZE];         // array of last BPM values
byte rateIndex = 0;
long lastBeat = 0;             // time (ms) of the last beat
float beatAvg = 0;             // averaged BPM
float tempC =0;
// ---------- MAX30205 (Temperature) ----------
const uint8_t MAX30205_ADDR = 0x48;    // default I2C address

// ---------- 1. Init functions ----------

// Heart-rate sensor (MAX30102)
bool initMAX30102() {
  Serial.println("Initializing MAX30102...");

  // I2C already started in setup() with Wire.begin()

  if (!maxSensor.begin(Wire, I2C_SPEED_FAST)) {   // check sensor presence
    Serial.println("MAX30102 not found. Check wiring & power!");
    return false;
  }

  maxSensor.setup();                  // library default config
  maxSensor.setPulseAmplitudeRed(0x0A);
  maxSensor.setPulseAmplitudeIR(0x0A);

  Serial.println("MAX30102 ready ✅");
  return true;
}

// Temperature sensor (MAX30205)
bool initMAX30205() {
  Serial.println("Initializing MAX30205...");

  // Just try to contact the device
  Wire.beginTransmission(MAX30205_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("MAX30205 not found. Check wiring & power!");
    return false;
  }

  Serial.println("MAX30205 ready ✅");
  return true;
}

// ---------- 2. Read functions ----------

// Heart rate: returns average BPM (0 if no valid reading)
float readMAX30102_BPM() {
  long irValue = maxSensor.getIR();   // infrared value (blood flow)

  if (irValue < 50000) {              // likely no finger
    return 0.0;
  }

  if (checkForBeat(irValue)) {
    long now = millis();
    long delta = now - lastBeat;
    lastBeat = now;

    float bpm = 60.0 / (delta / 1000.0);

    if (bpm > 20 && bpm < 255) {
      rates[rateIndex] = (byte)bpm;
      rateIndex = (rateIndex + 1) % RATE_SIZE;

      int sum = 0;
      for (byte i = 0; i < RATE_SIZE; i++) sum += rates[i];
      beatAvg = (float)sum / RATE_SIZE;
    }
  }

  return beatAvg;   // may be 0 at the beginning
}

// Temperature: returns °C (NAN if read failed)
float readMAX30205_TempC() {
  // Tell sensor we want the temperature register (0x00)
  Wire.beginTransmission(MAX30205_ADDR);
  Wire.write(0x00);
  if (Wire.endTransmission(false) != 0) {
    return tempC;
  }

  // Read 2 bytes
  Wire.requestFrom(MAX30205_ADDR, (uint8_t)2);
  if (Wire.available() < 2) {
    return tempC;
  }

  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  int16_t raw = (msb << 8) | lsb;

  // Each LSB = 1/256 °C (from MAX30205 datasheet)
  float tempC = (raw / 256.0f)+Err;
  return tempC;
}

// ---------- 3. Main Arduino structure ----------

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin();   // start I2C once for all sensors

  bool okHR  = initMAX30102();
  bool okTmp = initMAX30205();

  if (!okHR || !okTmp) {
    Serial.println("One or more sensors failed to init.");
    // you can choose to block here or continue
  }
}

void loop() {
  tempC = readMAX30205_TempC();

  Serial.print(" | Temp (°C): ");
  Serial.println(tempC);

}
