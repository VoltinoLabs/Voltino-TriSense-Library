/*
 * Example: AdvancedFusion.ino (Updated for v1.2.0)
 * Advanced sensor fusion (Complementary Filter with Gaussian confidence)
 * Features dynamic FIFO draining and ODR drift tracking.
 * * HW: Raspberry Pi Pico 2 / ESP32 + Voltino TriSense
 */

#include <TriSense.h>

TriSense sensor;
AdvancedTriFusion fusion(&sensor.imu, &sensor.mag);

unsigned long lastPrint = 0;
const unsigned long printInterval = 20000; // 50Hz Serial output

void setup() {
  Serial.begin(115200);
  delay(500);

  // 1. Sensor Initialization (Hybrid Mode: AK/BMP on I2C, ICM on SPI CS17)
  if (!sensor.beginAll(MODE_HYBRID, 17, 10000000)) {
    Serial.println("Sensor init failed!");
    while (1);
  }

  // [NEW in 1.2.0] Optional: Switch IMU to 20-bit High-Resolution FIFO mode!
  // The fusion algorithm will automatically adapt to the higher precision.
  sensor.imu.setFIFOMode(FIFO_20BIT_HIRES);

  // =============================================================
  // 2. CALIBRATION (Insert your values here)
  // =============================================================
  sensor.imu.setAccelSoftwareOffset(0.00, 0.00, 0.00);
  sensor.imu.setAccelSoftwareScale(1.00, 1.00, 1.00);
  
  Serial.println("Calibrating Gyro... Keep still!");
  sensor.autoCalibrateGyro(1000); 

  fusion.setMagHardIron(-46.02, -0.85, -46.00);
  float softIron[3][3] = {
    {0.965, 0.008, -0.002},
    {0.008, 0.981, 0.139},
    {-0.002, 0.139, 1.077}
  };
  fusion.setMagSoftIron(softIron);
  fusion.setDeclination(5.6); 

  Serial.println("Calibrating initial orientation...");
  fusion.initOrientation();
  Serial.println("Calibration Done, System Running.");
}

void loop() {
  // Fusion Update: Internally drains FIFO and applies real-time dt correction
  if (fusion.update()) {
    
    unsigned long now = micros();
    if (now - lastPrint >= printInterval) {
      lastPrint = now;
      
      float roll, pitch, yaw;
      fusion.getOrientationDegrees(roll, pitch, yaw);
      
      Serial.print("Roll: "); Serial.print(roll, 1);
      Serial.print(" | Pitch: "); Serial.print(pitch, 1);
      Serial.print(" | Yaw: "); Serial.println(yaw, 1);
    }
  }
}