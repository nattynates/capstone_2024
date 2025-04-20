//// Pin defines ////
const int adcPin = 4;  // GPIO4 = ADC1_CH3

//// Constant definitions ////

// Power meter calibration and conversion
const float slope = 0.018; // From datasheet
const float v_at_0dbm = 1.424; // From testing and calibration

// Sampling control
const int bufferSize = 30;  // Sample size for stability detection - change for smaller or larger sample size
float buffer[bufferSize];
int bufferIndex = 0;
const float stabilityThreshold = 1; // dBm threshold for stable power out
unsigned long lastSampleTime = 0;
const unsigned long sampleInterval = 100; // ms - change if buffer size is changed
bool bufferFull = false;
bool isStable = false;
unsigned long lastStablePrint = 0;
const unsigned long stablePrintInterval = 2000; // ms - Update time

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  Serial.println("RF Power Monitor with Stability Check");
}

void loop() {

  unsigned long currentTime = millis();
  // Sample every 100 ms
  if (currentTime - lastSampleTime >= sampleInterval) {
    lastSampleTime = currentTime;

    // Convert power meter Vout to dBm
    int raw = analogRead(adcPin);
    float voltage = (3.3 / 4095.0) * raw;
    float dBm = (voltage - v_at_0dbm) / slope;

    // Sampling function
    buffer[bufferIndex] = dBm;
    bufferIndex = (bufferIndex + 1) % bufferSize;

    if (!bufferFull && bufferIndex == 0) bufferFull = true;

    if (bufferFull) {
      float minVal = buffer[0];
      float maxVal = buffer[0];
      for (int i = 1; i < bufferSize; i++) {
        if (buffer[i] < minVal) minVal = buffer[i];
        if (buffer[i] > maxVal) maxVal = buffer[i];
      }

      float delta = maxVal - minVal;
      isStable = (delta <= stabilityThreshold);

      Serial.print("dBm: ");
      Serial.print(dBm, 2);
      Serial.print(" | Δ: ");
      Serial.print(delta, 2);
      Serial.print(isStable ? "Stable" : "Unstable");
      Serial.println();

      // If stable, print power
      if (isStable && (currentTime - lastStablePrint >= stablePrintInterval)) {
        Serial.print("\n Stable Power: ");
        Serial.print(dBm, 2);
        Serial.println(" dBm\n");
        lastStablePrint = currentTime;
      }

      // Reset stable print timer if unstable
      if (!isStable) {
        lastStablePrint = currentTime;
      }
    }
  }
}
