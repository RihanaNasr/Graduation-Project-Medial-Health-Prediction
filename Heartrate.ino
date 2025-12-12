#include <Wire.h>
#include <MAX30105.h>
#include "heartRate.h"  // Include the heartRate library

MAX30105 particleSensor;

const byte RATE_SIZE = 4;  // Array size for averaging the BPM readings
byte rates[RATE_SIZE]; // Array to store BPM readings
byte rateSpot = 0; // Array index for the next BPM value
long lastBeat = 0; // Timestamp of the last detected beat

float beatsPerMinute;
int beatAvg; // Average BPM from the last few readings

unsigned long lastUpdate = 0;  // Timestamp for non-blocking updates
unsigned long updateInterval = 1000;  // Update every 1 second

void setup() {
  Serial.begin(115200); // Start Serial Monitor at 115200 baud rate
  Serial.println("Initializing...");

  // Initialize MAX30105 sensor on ESP32 using I2C communication
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30105 was not found. Please check wiring/power.");
    while (1); // Stop execution if sensor isn't detected
  }

  // Inform the user to place finger on the sensor
  Serial.println("Place your index finger on the sensor with steady pressure.");
  
  // Configure the sensor with default settings
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0F);  // Increase red LED pulse amplitude for better detection
  particleSensor.setPulseAmplitudeGreen(0);  // Turn off green LED (not needed for heart rate)

  // Initial message to display once sensor is initialized
  Serial.println("Sensor Initialized. Ready to read heart rate data.");
}

void loop() {
  long irValue = particleSensor.getIR(); // Get infrared value from the sensor

  // Only process the sensor readings if enough time has passed
  if (millis() - lastUpdate >= updateInterval) {
    //lastUpdate = millis();  // Update the timestamp

    // Check if a beat is detected based on IR signal changes
    if (checkForBeat(irValue)) {
      long delta = millis() - lastBeat; // Time between the current and previous beat
      lastBeat = millis(); // Update the lastBeat timestamp

      // Calculate the current beats per minute (BPM)
      beatsPerMinute = 60 / (delta / 1000.0);

      // If BPM is in a valid range, store it for averaging
      if (beatsPerMinute < 255 && beatsPerMinute > 20) { // Valid BPM range
        rates[rateSpot++] = (byte)beatsPerMinute;
        rateSpot %= RATE_SIZE;  // Wrap the index back to the beginning if it exceeds RATE_SIZE

        // Calculate the average BPM from the last few readings
        beatAvg = 0;
        for (byte x = 0; x < RATE_SIZE; x++) {
          beatAvg += rates[x];
        }
        beatAvg /= RATE_SIZE; // Compute the average BPM
      }
    }

    // Print the IR value, current BPM, and average BPM to the Serial Monitor
    unsigned long currentMillis = millis();  // Get current time in milliseconds
    Serial.print("Time: ");
    Serial.print(currentMillis);  // Print the timestamp
    Serial.print(", IR=");
    Serial.print(irValue); // Print the raw IR value
    Serial.print(", BPM=");
    Serial.print(beatsPerMinute); // Print the current BPM
    Serial.print(", Avg BPM=");
    Serial.println(beatAvg); // Print the average BPM

    // Check if the IR value is too low (weak signal) or no finger detected
    if (irValue < 50000) {
      Serial.println("Signal too weak - Check finger placement!");
    }
  }
}
