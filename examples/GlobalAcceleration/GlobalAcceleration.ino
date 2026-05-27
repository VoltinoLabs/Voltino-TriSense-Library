/*
 * Example: GlobalAcceleration.ino (Updated for v1.2.0)
 *
 * Description:
 * Extracts pure Global Acceleration (World Frame Acceleration)
 * compensated for sensor tilt using AdvancedTriFusion.
 */

#include <TriSense.h>

TriSense sensor;
AdvancedTriFusion fusion(&sensor.imu, &sensor.mag);

unsigned long lastPrint = 0;
const unsigned long printInterval = 40000; // 25Hz output

void setup() {
  Serial.begin(115200);
  delay(1000); 

  if (!sensor.beginAll(MODE_HYBRID, 17)) {
    Serial.println("Sensor init failed!");
    while (1) delay(100);
  }

  // Quick calibration for demonstration
  sensor.autoCalibrateGyro(500);

  Serial.println("Calibrating initial orientation...");
  fusion.initOrientation();
  Serial.println("System Running. Format: R, P, Y | Ax_G, Ay_G, Az_G");
}

void loop() {
  if (fusion.update()) {
    
    unsigned long now = micros();
    if (now - lastPrint >= printInterval) {
      lastPrint = now;
      
      float roll, pitch, yaw;
      fusion.getOrientationDegrees(roll, pitch, yaw);

      // Get GLOBAL ACCELERATION [G]
      float ax_g, ay_g, az_g;
      fusion.getGlobalAcceleration(ax_g, ay_g, az_g);

      // Remove Earth's 1G gravity from Z axis to get pure linear movement:
      az_g -= 1.0f; 

      Serial.print("R:"); Serial.print(roll, 1);
      Serial.print("\tP:"); Serial.print(pitch, 1);
      Serial.print("\tY:"); Serial.print(yaw, 1);
      
      Serial.print("  |  LinAcc(G) -> X:"); Serial.print(ax_g, 3);
      Serial.print(" Y:"); Serial.print(ay_g, 3);
      Serial.print(" Z:"); Serial.println(az_g, 3);
    }
  }
}