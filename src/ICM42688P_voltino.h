#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#define ICM42688_REG_DEVICE_CONFIG 0x11
#define ICM42688_REG_DRIVE_CONFIG  0x13
#define ICM42688_REG_INT_CONFIG    0x14
#define ICM42688_REG_FIFO_CONFIG   0x16
#define ICM42688_REG_TEMP_DATA1    0x1D
#define ICM42688_REG_ACCEL_DATA_X1 0x1F
#define ICM42688_REG_GYRO_DATA_X1  0x25
#define ICM42688_REG_FIFO_COUNTH   0x2E 
#define ICM42688_REG_FIFO_COUNTL   0x2F 
#define ICM42688_REG_FIFO_DATA     0x30 
#define ICM42688_REG_SIGNAL_PATH_RESET 0x4B 
#define ICM42688_REG_TMST_CONFIG   0x54
#define ICM42688_REG_APEX_CONFIG0  0x56
#define ICM42688_REG_SMD_CONFIG    0x57
#define ICM42688_REG_FIFO_CONFIG1  0x5F
#define ICM42688_REG_FIFO_CONFIG2  0x60
#define ICM42688_REG_FIFO_CONFIG3  0x61
#define ICM42688_REG_INT_SOURCE0   0x65
#define ICM42688_REG_WHO_AM_I      0x75
#define ICM42688_REG_BANK_SEL      0x76
#define ICM42688_REG_PWR_MGMT0     0x4E
#define ICM42688_REG_GYRO_CONFIG0  0x4F
#define ICM42688_REG_ACCEL_CONFIG0 0x50

#define ICM_ADDR_PRIMARY 0x68
#define ICM_ADDR_SECONDARY 0x69
#define WHO_AM_I_EXPECTED 0x47

enum ICM_BUS { BUS_I2C, BUS_SPI };
enum ICM_ACCEL_FS { AFS_16G = 0, AFS_8G = 1, AFS_4G = 2, AFS_2G = 3 };
enum ICM_GYRO_FS { GFS_2000DPS = 0, GFS_1000DPS = 1, GFS_500DPS = 2, GFS_250DPS = 3, GFS_125DPS = 4 };
enum ICM_ODR {
  ODR_32KHZ = 1, ODR_16KHZ = 2, ODR_8KHZ = 3, ODR_4KHZ = 4,
  ODR_2KHZ = 5, ODR_1KHZ = 6, ODR_200HZ = 7, ODR_100HZ = 8,
  ODR_50HZ = 9, ODR_25HZ = 10, ODR_12_5HZ = 11, ODR_500HZ = 15
};

enum ICM_FIFO_MODE {
  FIFO_NONE = 0,        
  FIFO_16BIT = 1,       
  FIFO_20BIT_HIRES = 2  
};

class ICM42688P {
public:
  ICM42688P();
  
  bool beginI2C(uint32_t freq = 0, uint8_t i2cAddr = ICM_ADDR_PRIMARY, int8_t sdaPin = -1, int8_t sclPin = -1);
  bool beginSPI(int8_t csPin, uint32_t freq = 0, int8_t sckPin = -1, int8_t misoPin = -1, int8_t mosiPin = -1);
  bool begin(ICM_BUS busType, int8_t csPin = -1, uint32_t freq = 0, uint8_t i2cAddr = ICM_ADDR_PRIMARY, int8_t sckSclPin = -1, int8_t misoSdaPin = -1, int8_t mosiPin = -1);
  
  void setDebug(bool enable);
  void setODR(ICM_ODR odr);
  void setAccelFS(ICM_ACCEL_FS fs);
  void setGyroFS(ICM_GYRO_FS fs);
  void setFIFOMode(ICM_FIFO_MODE mode);
  
  int getODRHz(); 
  ICM_FIFO_MODE getFIFOMode() { return _fifoMode; } 
  
  bool readIMU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz);
  float readTemperature();

  bool readSensorData(float &ax, float &ay, float &az, float &gx, float &gy, float &gz);
  bool readHardwareFIFO(float &ax, float &ay, float &az, float &gx, float &gy, float &gz);
  bool readHardwareFIFOHires(float &ax, float &ay, float &az, float &gx, float &gy, float &gz);
  bool readFIFO(float &ax, float &ay, float &az, float &gx, float &gy, float &gz);

  void resetHardwareOffsets();
  void autoCalibrateGyro(uint16_t samples = 750);
  void autoCalibrateAccel(); 
  
  void setGyroSoftwareOffset(float ox, float oy, float oz);
  void setAccelSoftwareOffset(float ox, float oy, float oz);
  void setAccelSoftwareScale(float sx, float sy, float sz);

  void getGyroSoftwareOffset(float &ox, float &oy, float &oz);
  void getAccelSoftwareOffset(float &ox, float &oy, float &oz);
  void getAccelSoftwareScale(float &sx, float &sy, float &sz);

  void setGyroOffset(float ox, float oy, float oz);
  void setAccelOffset(float ox, float oy, float oz);
  void setAccelScale(float sx, float sy, float sz);
  void getGyroOffset(float &ox, float &oy, float &oz);
  void getAccelOffset(float &ox, float &oy, float &oz);
  void getAccelScale(float &sx, float &sy, float &sz);

  float getAccelOffsetX();
  float getAccelOffsetY();
  float getAccelOffsetZ();
  float getAccelScaleX();
  float getAccelScaleY();
  float getAccelScaleZ();
  float getGyroOffsetX();
  float getGyroOffsetY();
  float getGyroOffsetZ();

private:
  ICM_BUS _bus;
  int8_t _csPin;
  uint8_t _i2cAddr;
  uint32_t _spiFreq;
  ICM_ODR _odr;
  ICM_FIFO_MODE _fifoMode;
  bool _debug;

  float _accelScaleFactor;
  float _gyroScaleFactor;

  float gyrOffset[3] = {0,0,0};
  float accOffset[3] = {0,0,0};
  float accScale[3]  = {1,1,1};

  void writeRegister(uint8_t reg, uint8_t val);
  uint8_t readRegister(uint8_t reg);
  void readRegisters(uint8_t reg, uint8_t *buf, size_t len);
  void setBank(uint8_t bank);

  void enforceBandwidthLimit();
  int _getHzFromODR(ICM_ODR odr);
};