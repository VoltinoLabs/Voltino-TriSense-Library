#include "TriSense.h"

TriSense::TriSense() {}

bool TriSense::beginAll(TriSenseMode mode, uint8_t spiCsPin, uint32_t spiFreq) {
  _mode = mode;
  Wire.begin();
  if (!bmp.begin()) return false;
  if (!mag.begin()) return false;

  if (_mode == MODE_I2C) {
    if (!imu.begin(BUS_I2C)) return false;
    imu.setODR(DEFAULT_IMU_ODR); 
  } else {
    if (!imu.begin(BUS_SPI, spiCsPin, spiFreq)) return false;
    imu.setODR(DEFAULT_IMU_SPI_ODR); 
  }

  imu.setFIFOMode(FIFO_16BIT);

  bmp.setOversampling(BMP580_OSR_x2, BMP580_OSR_x2);
  bmp.setODR(BMP580_ODR_240Hz);
  bmp.setPowerMode(BMP580_MODE_NORMAL);
  mag.setODR(AK_ODR_100HZ); 
  
  return true;
}

bool TriSense::beginBMP(uint8_t addr) { return bmp.begin(addr); }
bool TriSense::beginMAG() { return mag.begin(); }
bool TriSense::beginIMU(ICM_BUS busType, uint8_t csPin, uint32_t freq) { return imu.begin(busType, csPin, freq); }

void TriSense::resetHardwareOffsets() { imu.resetHardwareOffsets(); }
void TriSense::autoCalibrateGyro(uint16_t samples) { imu.autoCalibrateGyro(samples); }
void TriSense::autoCalibrateAccel() { imu.autoCalibrateAccel(); }

bool TriSense::getSnapshot(TriSenseDataSnapshot &data) {
  float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
  bool imuOk = imu.readFIFO(ax, ay, az, gx, gy, gz);
  data.accelX = ax; data.accelY = ay; data.accelZ = az;
  data.gyroX = gx;  data.gyroY = gy;  data.gyroZ = gz;

  bool magOk = mag.readData();
  data.magX = mag.x; data.magY = mag.y; data.magZ = mag.z;

  data.pressure = bmp.readPressure();
  data.temperature = bmp.readTemperature();

  return imuOk && magOk;
}

float TriSense::readPressure() { return bmp.readPressure(); }
float TriSense::readTemperature() { return bmp.readTemperature(); }
float TriSense::readAltitude(float seaLevelPressure) { return bmp.readAltitude(seaLevelPressure); }

TriSenseFusion::TriSenseFusion(ICM42688P* imu, AK09918C* mag) : _imu(imu), _mag(mag) {}

FUSION_MATH_TYPE TriSenseFusion::gaussianGain(FUSION_MATH_TYPE x, FUSION_MATH_TYPE mu, FUSION_MATH_TYPE sigma) {
  if (sigma == 0.0) return 0.0;
  FUSION_MATH_TYPE diff = x - mu;
  return exp(-(diff * diff) / ((FUSION_MATH_TYPE)2.0 * sigma * sigma));
}

void TriSenseFusion::setAccelGaussian(float ref, float sigma) { accRef = ref; accSigma = sigma; }
void TriSenseFusion::setMagGaussian(float ref, float sigma, float tiltSigma) { magRef = ref; magSigma = sigma; magTiltSigmaDeg = tiltSigma; }
void TriSenseFusion::setMagGaussian(float ref, float sigma) { magRef = ref; magSigma = sigma; }
void TriSenseFusion::setMagTiltSigma(float sigmaDeg) { magTiltSigmaDeg = sigmaDeg; }
void TriSenseFusion::setMagCalibration(float hard[3], float soft[3][3]) {
  for(int i=0; i<3; i++) magHardIron[i] = hard[i];
  for(int i=0; i<3; i++) for(int j=0; j<3; j++) magSoftIron[i][j] = soft[i][j];
}
void TriSenseFusion::setDeclination(float deg) { magneticDeclination = deg; }
void TriSenseFusion::setGyroOffsets(float x, float y, float z) { gyroOffset[0] = x; gyroOffset[1] = y; gyroOffset[2] = z; }
void TriSenseFusion::setMagHardIron(float x, float y, float z) { magHardIron[0] = x; magHardIron[1] = y; magHardIron[2] = z; }
void TriSenseFusion::setMagSoftIron(float matrix[3][3]) { for(int i=0; i<3; i++) for(int j=0; j<3; j++) magSoftIron[i][j] = matrix[i][j]; }
void TriSenseFusion::setYawKi(float ki) { yawKi = ki; }
void TriSenseFusion::setMaxGains(float accelGain, float magGain) { maxAccelGain = accelGain; maxMagGain = magGain; }
void TriSenseFusion::setMagCheckInterval(float intervalMs) { magCheckIntervalUs = (unsigned long)(intervalMs * 1000.0f); }
void TriSenseFusion::setLocalGravity(float g) { _localGravity = g; }

void TriSenseFusion::calibrateAccelStatic(int samples) {
  double sumX=0, sumY=0, sumZ=0; 
  int count = 0;
  
  if (_imu->getFIFOMode() != FIFO_NONE) {
      float ax, ay, az, gx, gy, gz;
      while(_imu->readFIFO(ax, ay, az, gx, gy, gz));
  }

  while(count < samples) { 
    float ax, ay, az, gx, gy, gz;
    if (_imu->readFIFO(ax, ay, az, gx, gy, gz)) {
      sumX += ax; sumY += ay; sumZ += az; 
      count++;
    } else {
      delay(1); 
    }
  }
  
  accelOffset[0] = (float)(sumX / samples) - 0.0f; 
  accelOffset[1] = (float)(sumY / samples) - 0.0f; 
  accelOffset[2] = (float)(sumZ / samples) - 1.0f; 
}

void TriSenseFusion::initOrientation(int samples) {
  FUSION_MATH_TYPE axSum=0, aySum=0, azSum=0, mxSum=0, mySum=0, mzSum=0; 
  int count = 0;
  
  // Pročištění starého zásobníku před startem, aby se to nezaseklo hned na začátku
  if (_imu->getFIFOMode() != FIFO_NONE) {
      float ax, ay, az, gx, gy, gz;
      while(_imu->readFIFO(ax, ay, az, gx, gy, gz));
  }

  while(count < samples) {
     float ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
     
     bool imuReady = _imu->readFIFO(ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw);
     _mag->readData(); 

     if(imuReady) {
         FUSION_MATH_TYPE ax = ax_raw - accelOffset[0]; 
         FUSION_MATH_TYPE ay = ay_raw - accelOffset[1]; 
         FUSION_MATH_TYPE az = az_raw - accelOffset[2]; 
         axSum+=ax; aySum+=ay; azSum+=az;
         
         FUSION_MATH_TYPE mx_raw = _mag->x - magHardIron[0]; 
         FUSION_MATH_TYPE my_raw = _mag->y - magHardIron[1]; 
         FUSION_MATH_TYPE mz_raw = _mag->z - magHardIron[2];
         
         FUSION_MATH_TYPE mx = magSoftIron[0][0]*mx_raw + magSoftIron[0][1]*my_raw + magSoftIron[0][2]*mz_raw;
         FUSION_MATH_TYPE my = magSoftIron[1][0]*mx_raw + magSoftIron[1][1]*my_raw + magSoftIron[1][2]*mz_raw;
         FUSION_MATH_TYPE mz = magSoftIron[2][0]*mx_raw + magSoftIron[2][1]*my_raw + magSoftIron[2][2]*mz_raw;
         
         mxSum+=mx; mySum+=my; mzSum+=mz; 
         count++; 
     } else {
         delay(1); // Uvolnění procesoru (nikoliv tvrdý delay po dobu sběru jako předtím)
     }
  }
  
  FUSION_MATH_TYPE r, p, y; 
  getCorrectionAngles(axSum/samples, aySum/samples, azSum/samples, mxSum/samples, mySum/samples, mzSum/samples, r, p, y);
  
  FUSION_MATH_TYPE c1 = cos(y * (FUSION_MATH_TYPE)PI / 360.0); 
  FUSION_MATH_TYPE s1 = sin(y * (FUSION_MATH_TYPE)PI / 360.0); 
  FUSION_MATH_TYPE c2 = cos(p * (FUSION_MATH_TYPE)PI / 360.0); 
  FUSION_MATH_TYPE s2 = sin(p * (FUSION_MATH_TYPE)PI / 360.0); 
  FUSION_MATH_TYPE c3 = cos(r * (FUSION_MATH_TYPE)PI / 360.0); 
  FUSION_MATH_TYPE s3 = sin(r * (FUSION_MATH_TYPE)PI / 360.0);
  
  q[0] = c1*c2*c3 + s1*s2*s3; 
  q[1] = c1*c2*s3 - s1*s2*c3; 
  q[2] = c1*s2*c3 + s1*c2*s3; 
  q[3] = s1*c2*c3 - c1*s2*s3;

  int nominalHz = _imu->getODRHz();
  _realDt = (nominalHz > 0) ? (1.0 / (FUSION_MATH_TYPE)nominalHz) : 0.001;
  
  _lastOdrCheckTime = micros();
  _lastIntegrationTime = micros(); 
  _sampleCount = 0;
}

void TriSenseFusion::quaternionToEuler(FUSION_MATH_TYPE& roll, FUSION_MATH_TYPE& pitch, FUSION_MATH_TYPE& yaw) {
  FUSION_MATH_TYPE sinr_cosp = 2.0 * (q[0] * q[1] + q[2] * q[3]); 
  FUSION_MATH_TYPE cosr_cosp = 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]); 
  roll = atan2(sinr_cosp, cosr_cosp);
  
  FUSION_MATH_TYPE sinp = 2.0 * (q[0] * q[2] - q[3] * q[1]); 
  if (abs(sinp) >= 1.0) pitch = copysign((FUSION_MATH_TYPE)PI / 2.0, sinp); else pitch = asin(sinp);
  
  FUSION_MATH_TYPE siny_cosp = 2.0 * (q[0] * q[3] + q[1] * q[2]); 
  FUSION_MATH_TYPE cosy_cosp = 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]); 
  yaw = atan2(siny_cosp, cosy_cosp);
}

void TriSenseFusion::getOrientationDegrees(float& roll, float& pitch, float& yaw) {
  FUSION_MATH_TYPE r, p, y; quaternionToEuler(r, p, y); 
  roll = (float)(r * 180.0 / PI); pitch = (float)(p * 180.0 / PI); yaw = (float)(y * 180.0 / PI); 
  if (yaw < 0) yaw += 360.0f; if (yaw >= 360.0f) yaw -= 360.0f;
}

void TriSenseFusion::getCorrectionAngles(FUSION_MATH_TYPE ax, FUSION_MATH_TYPE ay, FUSION_MATH_TYPE az, 
                                         FUSION_MATH_TYPE mx, FUSION_MATH_TYPE my, FUSION_MATH_TYPE mz, 
                                         FUSION_MATH_TYPE& roll, FUSION_MATH_TYPE& pitch, FUSION_MATH_TYPE& yaw) {
  roll  = atan2(ay, az) * 180.0 / PI; pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  FUSION_MATH_TYPE phi = roll * (FUSION_MATH_TYPE)PI / 180.0; 
  FUSION_MATH_TYPE theta = pitch * (FUSION_MATH_TYPE)PI / 180.0;
  FUSION_MATH_TYPE by = my * cos(phi) - mz * sin(phi); 
  FUSION_MATH_TYPE bx = mx * cos(theta) + my * sin(theta) * sin(phi) + mz * sin(theta) * cos(phi);
  yaw = atan2(-by, bx) * 180.0 / PI + (FUSION_MATH_TYPE)magneticDeclination; 
  if (yaw < 0) yaw += 360.0; if (yaw >= 360.0) yaw -= 360.0;
}

void TriSenseFusion::getGlobalAcceleration(float& ax, float& ay, float& az, AccelUnit unit) {
  FUSION_MATH_TYPE qw = q[0], qx = q[1], qy = q[2], qz = q[3]; 
  FUSION_MATH_TYPE x2 = qx + qx, y2 = qy + qy, z2 = qz + qz;
  FUSION_MATH_TYPE xx = qx * x2, xy = qx * y2, xz = qx * z2; 
  FUSION_MATH_TYPE yy = qy * y2, yz = qy * z2, zz = qz * z2; 
  FUSION_MATH_TYPE wx = qw * x2, wy = qw * y2, wz = qw * z2;
  
  FUSION_MATH_TYPE ax_g = (1.0 - (yy + zz)) * lastAx + (xy - wz) * lastAy + (xz + wy) * lastAz;
  FUSION_MATH_TYPE ay_g = (xy + wz) * lastAx + (1.0 - (xx + zz)) * lastAy + (yz - wx) * lastAz;
  FUSION_MATH_TYPE az_g = (xz - wy) * lastAx + (yz + wx) * lastAy + (1.0 - (xx + yy)) * lastAz;

  if (unit == ACCEL_UNIT_MS2) {
     ax = (float)(ax_g * _localGravity);
     ay = (float)(ay_g * _localGravity);
     az = (float)(az_g * _localGravity);
  } else {
     ax = (float)ax_g;
     ay = (float)ay_g;
     az = (float)az_g;
  }
}

void TriSenseFusion::getLinearAcceleration(float& ax, float& ay, float& az, AccelUnit unit) {
  getGlobalAcceleration(ax, ay, az, unit);
  if (unit == ACCEL_UNIT_MS2) az -= _localGravity; else az -= 1.0f;           
}

void TriSenseFusion::gyroIntegration(FUSION_MATH_TYPE gx, FUSION_MATH_TYPE gy, FUSION_MATH_TYPE gz, FUSION_MATH_TYPE dt) {
  FUSION_MATH_TYPE qDot1 = 0.5 * (-q[1] * gx - q[2] * gy - q[3] * gz); 
  FUSION_MATH_TYPE qDot2 = 0.5 * (q[0] * gx + q[2] * gz - q[3] * gy);
  FUSION_MATH_TYPE qDot3 = 0.5 * (q[0] * gy - q[1] * gz + q[3] * gx); 
  FUSION_MATH_TYPE qDot4 = 0.5 * (q[0] * gz + q[1] * gy - q[2] * gx);
  q[0] += qDot1 * dt; q[1] += qDot2 * dt; q[2] += qDot3 * dt; q[3] += qDot4 * dt;
  FUSION_MATH_TYPE recipNorm = 1.0 / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]); 
  q[0] *= recipNorm; q[1] *= recipNorm; q[2] *= recipNorm; q[3] *= recipNorm;
}

SimpleTriFusion::SimpleTriFusion(ICM42688P* imu, AK09918C* mag) : TriSenseFusion(imu, mag) {}

bool SimpleTriFusion::update() {
  float ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw; 
  bool dataProcessed = false;
  
  if (_imu->readFIFO(ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw)) {
      dataProcessed = true;
      _sampleCount++;
      
      lastAx = ax_raw - accelOffset[0]; 
      lastAy = ay_raw - accelOffset[1]; 
      lastAz = az_raw - accelOffset[2]; 
      lastGx = gx_raw - gyroOffset[0];  
      lastGy = gy_raw - gyroOffset[1];  
      lastGz = gz_raw - gyroOffset[2];
      
      FUSION_MATH_TYPE dt;
      unsigned long nowMicros = micros();

      if (_imu->getFIFOMode() == FIFO_NONE) {
          if (_lastIntegrationTime == 0) _lastIntegrationTime = nowMicros;
          dt = (nowMicros - _lastIntegrationTime) / 1000000.0;
          _lastIntegrationTime = nowMicros;
          if (dt <= 0.0) dt = 0.00001;
          if (dt > 0.1) dt = 1.0 / (FUSION_MATH_TYPE)(_imu->getODRHz() > 0 ? _imu->getODRHz() : 1000);
      } else {
          dt = _realDt;
      }

      gyroIntegration(lastGx * (FUSION_MATH_TYPE)PI/180.0, 
                      lastGy * (FUSION_MATH_TYPE)PI/180.0, 
                      lastGz * (FUSION_MATH_TYPE)PI/180.0, dt);
  }
  
  if (!dataProcessed) return false;
  
  unsigned long now = micros(); 
  
  if (now - _lastOdrCheckTime >= 1000000UL) { 
      if (_lastOdrCheckTime != 0 && _sampleCount > 0) {
          FUSION_MATH_TYPE measuredDt = (FUSION_MATH_TYPE)(now - _lastOdrCheckTime) / 1000000.0 / (FUSION_MATH_TYPE)_sampleCount;
          int nominalHz = _imu->getODRHz();
          FUSION_MATH_TYPE nominalDt = 1.0 / (FUSION_MATH_TYPE)(nominalHz > 0 ? nominalHz : 1000);
          
          if (measuredDt > nominalDt * 0.4 && measuredDt < nominalDt * 1.6) {
              _realDt = _realDt * 0.7 + measuredDt * 0.3; 
          }
      }
      _lastOdrCheckTime = now;
      _sampleCount = 0;
  }
  return true;
}

AdvancedTriFusion::AdvancedTriFusion(ICM42688P* imu, AK09918C* mag) : TriSenseFusion(imu, mag) {}

void AdvancedTriFusion::complementaryCorrection(FUSION_MATH_TYPE ax, FUSION_MATH_TYPE ay, FUSION_MATH_TYPE az, 
                                                FUSION_MATH_TYPE mx, FUSION_MATH_TYPE my, FUSION_MATH_TYPE mz, 
                                                FUSION_MATH_TYPE correction_dt) {
  FUSION_MATH_TYPE totalAccelG = sqrt(ax * ax + ay * ay + az * az); 
  FUSION_MATH_TYPE recipNorm = 1.0 / totalAccelG; 
  ax *= recipNorm; ay *= recipNorm; az *= recipNorm;
  
  FUSION_MATH_TYPE magStrength = sqrt(mx * mx + my * my + mz * mz);
  
  FUSION_MATH_TYPE vx = 2.0 * (q[1] * q[3] - q[0] * q[2]); 
  FUSION_MATH_TYPE vy = 2.0 * (q[0] * q[1] + q[2] * q[3]); 
  FUSION_MATH_TYPE vz = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
  FUSION_MATH_TYPE ex = (ay * vz - az * vy); 
  FUSION_MATH_TYPE ey = (az * vx - ax * vz);
  
  FUSION_MATH_TYPE g_gain_accel_raw = gaussianGain(totalAccelG, (FUSION_MATH_TYPE)accRef, (FUSION_MATH_TYPE)accSigma); 
  FUSION_MATH_TYPE g_gain_mag_raw = gaussianGain(magStrength, (FUSION_MATH_TYPE)magRef, (FUSION_MATH_TYPE)magSigma);
  FUSION_MATH_TYPE final_accel_gain = g_gain_accel_raw * (FUSION_MATH_TYPE)maxAccelGain; 
  FUSION_MATH_TYPE final_mag_gain = g_gain_mag_raw * (FUSION_MATH_TYPE)maxMagGain;
  
  FUSION_MATH_TYPE roll_rad, pitch_rad, yaw_rad; quaternionToEuler(roll_rad, pitch_rad, yaw_rad);
  FUSION_MATH_TYPE tilt_gain_roll = gaussianGain(fabs(roll_rad * 180.0 / (FUSION_MATH_TYPE)PI), 0.0, (FUSION_MATH_TYPE)magTiltSigmaDeg); 
  FUSION_MATH_TYPE tilt_gain_pitch = gaussianGain(fabs(pitch_rad * 180.0 / (FUSION_MATH_TYPE)PI), 0.0, (FUSION_MATH_TYPE)magTiltSigmaDeg);
  final_mag_gain *= tilt_gain_roll * tilt_gain_pitch;
  
  FUSION_MATH_TYPE roll_corr, pitch_corr, yaw_corr; 
  getCorrectionAngles(ax * totalAccelG, ay * totalAccelG, az * totalAccelG, mx, my, mz, roll_corr, pitch_corr, yaw_corr);
  FUSION_MATH_TYPE yaw_deg = yaw_rad * 180.0 / (FUSION_MATH_TYPE)PI; 
  if (yaw_deg < 0) yaw_deg += 360.0; if (yaw_deg >= 360.0) yaw_deg -= 360.0;
  
  FUSION_MATH_TYPE delta_yaw_deg = yaw_corr - yaw_deg; 
  if (delta_yaw_deg > 180.0) delta_yaw_deg -= 360.0; if (delta_yaw_deg < -180.0) delta_yaw_deg += 360.0;
  FUSION_MATH_TYPE delta_yaw_rad = delta_yaw_deg * (FUSION_MATH_TYPE)PI / 180.0;
  
  if (lastDeltaYawRad * delta_yaw_rad < 0.0) gyroBiasZ = 0.0; 
  lastDeltaYawRad = delta_yaw_rad;
  gyroBiasZ -= (FUSION_MATH_TYPE)yawKi * delta_yaw_rad * final_mag_gain * correction_dt;
  
  FUSION_MATH_TYPE w_x = final_accel_gain * ex * correction_dt; 
  FUSION_MATH_TYPE w_y = final_accel_gain * ey * correction_dt; 
  FUSION_MATH_TYPE w_z = final_mag_gain * delta_yaw_rad * correction_dt;
  
  FUSION_MATH_TYPE q0_old = q[0], q1_old = q[1], q2_old = q[2], q3_old = q[3];
  q[0] += 0.5 * (-q1_old * w_x - q2_old * w_y - q3_old * w_z); 
  q[1] += 0.5 * (q0_old * w_x + q2_old * w_z - q3_old * w_y);
  q[2] += 0.5 * (q0_old * w_y - q1_old * w_z + q3_old * w_x); 
  q[3] += 0.5 * (q0_old * w_z + q1_old * w_y - q2_old * w_x);
  
  recipNorm = 1.0 / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]); 
  q[0] *= recipNorm; q[1] *= recipNorm; q[2] *= recipNorm; q[3] *= recipNorm;
}

bool AdvancedTriFusion::update() {
  float ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw; 
  bool dataProcessed = false;
  
  if (_imu->readFIFO(ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw)) {
      dataProcessed = true;
      _sampleCount++;
      
      lastAx = ax_raw - accelOffset[0]; 
      lastAy = ay_raw - accelOffset[1]; 
      lastAz = az_raw - accelOffset[2];
      lastGx = gx_raw - gyroOffset[0]; 
      lastGy = gy_raw - gyroOffset[1]; 
      lastGz = gz_raw - gyroOffset[2];
      
      FUSION_MATH_TYPE dt;
      unsigned long nowMicros = micros();

      if (_imu->getFIFOMode() == FIFO_NONE) {
          if (_lastIntegrationTime == 0) _lastIntegrationTime = nowMicros;
          dt = (nowMicros - _lastIntegrationTime) / 1000000.0;
          _lastIntegrationTime = nowMicros;
          if (dt <= 0.0) dt = 0.00001;
          if (dt > 0.1) dt = 1.0 / (FUSION_MATH_TYPE)(_imu->getODRHz() > 0 ? _imu->getODRHz() : 1000);
      } else {
          dt = _realDt;
      }

      gyroIntegration(lastGx * (FUSION_MATH_TYPE)PI/180.0, 
                      lastGy * (FUSION_MATH_TYPE)PI/180.0, 
                      (lastGz - gyroBiasZ) * (FUSION_MATH_TYPE)PI/180.0, dt);
  }
  
  if (!dataProcessed) return false;
  
  unsigned long now = micros(); 
  
  if (now - _lastOdrCheckTime >= 1000000UL) { 
      if (_lastOdrCheckTime != 0 && _sampleCount > 0) {
          FUSION_MATH_TYPE measuredDt = (FUSION_MATH_TYPE)(now - _lastOdrCheckTime) / 1000000.0 / (FUSION_MATH_TYPE)_sampleCount;
          int nominalHz = _imu->getODRHz();
          FUSION_MATH_TYPE nominalDt = 1.0 / (FUSION_MATH_TYPE)(nominalHz > 0 ? nominalHz : 1000);
          if (measuredDt > nominalDt * 0.4 && measuredDt < nominalDt * 1.6) {
              _realDt = _realDt * 0.7 + measuredDt * 0.3;
          }
      }
      _lastOdrCheckTime = now;
      _sampleCount = 0;
  }
  
  if (now - lastMagCheckTime >= magCheckIntervalUs) {
     lastMagCheckTime = now;
     if (_mag->readData()) {
        FUSION_MATH_TYPE mx_raw = _mag->x - magHardIron[0]; 
        FUSION_MATH_TYPE my_raw = _mag->y - magHardIron[1]; 
        FUSION_MATH_TYPE mz_raw = _mag->z - magHardIron[2];
        lastMx = magSoftIron[0][0]*mx_raw + magSoftIron[0][1]*my_raw + magSoftIron[0][2]*mz_raw;
        lastMy = magSoftIron[1][0]*mx_raw + magSoftIron[1][1]*my_raw + magSoftIron[1][2]*mz_raw;
        lastMz = magSoftIron[2][0]*mx_raw + magSoftIron[2][1]*my_raw + magSoftIron[2][2]*mz_raw;
     }
  }

  unsigned long correctionDeltaUs = now - lastSuccessfulCorrectionTime;
  FUSION_MATH_TYPE correction_dt = (FUSION_MATH_TYPE)correctionDeltaUs / 1000000.0;
  if (correction_dt > 0.1 || lastSuccessfulCorrectionTime == 0) correction_dt = 0.01;
  
  complementaryCorrection(lastAx, lastAy, lastAz, lastMx, lastMy, lastMz, correction_dt);
  lastSuccessfulCorrectionTime = now;

  return true;
}