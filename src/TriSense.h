#ifndef TRISENSE_H
#define TRISENSE_H

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "BMP580.h"
#include "AK09918C.h"
#include "ICM42688P_voltino.h"

// ---------------------------------------------------------
// DYNAMIC ARCHITECTURE DETECTION & OPTIMIZATION
// ---------------------------------------------------------
// Odkomentováním jedné z těchto direktiv můžeš manuálně vynutit přesnost
// #define FORCE_FUSION_FLOAT
// #define FORCE_FUSION_DOUBLE

#if defined(FORCE_FUSION_DOUBLE)
  #define FUSION_MATH_TYPE double
  #define DEFAULT_IMU_ODR ODR_1KHZ
  #define DEFAULT_IMU_SPI_ODR ODR_8KHZ
  #define DEFAULT_CALIBRATION_SAMPLES 1000
#elif defined(FORCE_FUSION_FLOAT)
  #define FUSION_MATH_TYPE float
  #define DEFAULT_IMU_ODR ODR_1KHZ
  #define DEFAULT_IMU_SPI_ODR ODR_4KHZ
  #define DEFAULT_CALIBRATION_SAMPLES 1000
#elif defined(__AVR__)
  // Arduino Uno/Nano: Low memory, slow CPU. Use float, lower ODR, fewer samples.
  #define FUSION_MATH_TYPE float
  #define DEFAULT_IMU_ODR ODR_100HZ
  #define DEFAULT_IMU_SPI_ODR ODR_500HZ
  #define DEFAULT_CALIBRATION_SAMPLES 200
#elif defined(ESP32)
  // ESP32: Fast CPU, handles double precision efficiently. High ODR.
  #define FUSION_MATH_TYPE double
  #define DEFAULT_IMU_ODR ODR_1KHZ
  #define DEFAULT_IMU_SPI_ODR ODR_8KHZ
  #define DEFAULT_CALIBRATION_SAMPLES 1000
#elif defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_RP2350)
  // RP2040/RP2350: Extremely fast DSP with hardware FPU.
  // Using float here is optimal to leverage single-cycle FPU instructions.
  #define FUSION_MATH_TYPE float
  #define DEFAULT_IMU_ODR ODR_1KHZ
  #define DEFAULT_IMU_SPI_ODR ODR_8KHZ
  #define DEFAULT_CALIBRATION_SAMPLES 1000
#else
  // Fallback for any other/unrecognized architectures
  #define FUSION_MATH_TYPE float
  #define DEFAULT_IMU_ODR ODR_200HZ
  #define DEFAULT_IMU_SPI_ODR ODR_1KHZ
  #define DEFAULT_CALIBRATION_SAMPLES 500
#endif

// Unified structural container for all synchronous raw sensor metrics
struct TriSenseDataSnapshot {
  float accelX;
  float accelY;
  float accelZ;
  float gyroX;
  float gyroY;
  float gyroZ;
  float magX;
  float magY;
  float magZ;
  float pressure;
  float temperature;
};

enum TriSenseMode {
  MODE_I2C,
  MODE_HYBRID   // AK and BMP on I2C, ICM on SPI
};

class TriSense {
public:
  BMP580 bmp;
  AK09918C mag;
  ICM42688P imu;

  TriSense();
  
  bool beginAll(TriSenseMode mode, uint8_t spiCsPin = 17, uint32_t spiFreq = 4000000);
  bool beginBMP(uint8_t addr = BMP580_DEFAULT_I2C_ADDR);
  bool beginMAG();
  bool beginIMU(ICM_BUS busType = BUS_I2C, uint8_t csPin = 17, uint32_t freq = 4000000);

  void resetHardwareOffsets();
  void autoCalibrateGyro(uint16_t samples = DEFAULT_CALIBRATION_SAMPLES);
  void autoCalibrateAccel(); 

  // Unified single-call hardware snapshot data layer
  bool getSnapshot(TriSenseDataSnapshot &data);

  // Direct BMP580 abstraction wrapper layer
  float readPressure();
  float readTemperature();
  float readAltitude(float seaLevelPressure = 1013.25f);

private:
  TriSenseMode _mode;
};

class TriSenseFusion {
public: 
  ICM42688P* _imu;
  AK09918C* _mag;
  
  // Dynamic Math Types for Core Fusion calculations
  FUSION_MATH_TYPE q[4] = {1.0, 0.0, 0.0, 0.0};
  FUSION_MATH_TYPE lastAx = 0, lastAy = 0, lastAz = 0;
  FUSION_MATH_TYPE lastGx = 0, lastGy = 0, lastGz = 0;
  FUSION_MATH_TYPE lastMx = 0, lastMy = 0, lastMz = 0;
  
  // Configuration arrays kept as standard float (memory efficient, no DSP loss)
  float accelOffset[3] = {0.0f, 0.0f, 0.0f};      
  float gyroOffset[3] = {0.0f, 0.0f, 0.0f};       
  float magHardIron[3] = {0.0f, 0.0f, 0.0f};      
  float magSoftIron[3][3] = {{1,0,0},{0,1,0},{0,0,1}}; 
  
  float accRef = 1.0f;          
  float accSigma = 0.05f;       
  float magRef = 50.88f;         
  float magSigma = 3.5f;       
  float magTiltSigmaDeg = 15.0f;
  float magneticDeclination = 0.0f;
  float yawKi = 0.005f;           
  float maxAccelGain = 0.1f;    
  float maxMagGain = 0.1f;      
  
  unsigned long magCheckIntervalUs = 5000; // Zvýšeno pro Advanced fusion

  // --- Real-time ODR Drift Tracker Variables ---
  uint32_t _sampleCount = 0;
  unsigned long _lastOdrCheckTime = 0;
  FUSION_MATH_TYPE _realDt = 0.001; 

  // Internal high-precision math
  FUSION_MATH_TYPE gaussianGain(FUSION_MATH_TYPE x, FUSION_MATH_TYPE mu, FUSION_MATH_TYPE sigma);
  void gyroIntegration(FUSION_MATH_TYPE gx, FUSION_MATH_TYPE gy, FUSION_MATH_TYPE gz, FUSION_MATH_TYPE dt);
  void getCorrectionAngles(FUSION_MATH_TYPE ax, FUSION_MATH_TYPE ay, FUSION_MATH_TYPE az, 
                           FUSION_MATH_TYPE mx, FUSION_MATH_TYPE my, FUSION_MATH_TYPE mz, 
                           FUSION_MATH_TYPE& roll, FUSION_MATH_TYPE& pitch, FUSION_MATH_TYPE& yaw);
  void quaternionToEuler(FUSION_MATH_TYPE& roll, FUSION_MATH_TYPE& pitch, FUSION_MATH_TYPE& yaw);

public:
  TriSenseFusion(ICM42688P* imu, AK09918C* mag);
  virtual bool update() = 0;
  
  void calibrateAccelStatic(int samples = DEFAULT_CALIBRATION_SAMPLES);
  void initOrientation(int samples = DEFAULT_CALIBRATION_SAMPLES);
  
  // Public user API retained as float for backwards sketch compatibility
  void getOrientationDegrees(float& roll, float& pitch, float& yaw);
  void getGlobalAcceleration(float& ax_g, float& ay_g, float& az_g);
  
  void setAccelGaussian(float ref, float sigma);
  void setMagGaussian(float ref, float sigma, float tiltSigma); 
  void setMagGaussian(float ref, float sigma);                  
  void setMagTiltSigma(float sigmaDeg);                         
  void setMagCalibration(float hardIron[3], float softIron[3][3]);
  void setDeclination(float deg);
  void setGyroOffsets(float x, float y, float z);
  void setMagHardIron(float x, float y, float z);
  void setMagSoftIron(float matrix[3][3]);
  void setYawKi(float ki);
  void setMaxGains(float accelGain, float magGain);
  void setMagCheckInterval(float intervalMs);
};

class SimpleTriFusion : public TriSenseFusion {
public:
  SimpleTriFusion(ICM42688P* imu, AK09918C* mag);
  bool update() override;
};

class AdvancedTriFusion : public TriSenseFusion {
private:
  unsigned long lastMagCheckTime = 0; 
  unsigned long lastSuccessfulCorrectionTime = 0; 
  FUSION_MATH_TYPE gyroBiasZ = 0.0;
  FUSION_MATH_TYPE lastDeltaYawRad = 0.0;
  
  void complementaryCorrection(FUSION_MATH_TYPE ax, FUSION_MATH_TYPE ay, FUSION_MATH_TYPE az, 
                               FUSION_MATH_TYPE mx, FUSION_MATH_TYPE my, FUSION_MATH_TYPE mz, 
                               FUSION_MATH_TYPE correction_dt);
public:
  AdvancedTriFusion(ICM42688P* imu, AK09918C* mag);
  bool update() override;
};

#endif