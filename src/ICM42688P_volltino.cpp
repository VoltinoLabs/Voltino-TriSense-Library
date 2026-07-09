#include "ICM42688P_voltino.h"

ICM42688P::ICM42688P() {
  _accelScaleFactor = 1.0f / 2048.0f; 
  _gyroScaleFactor = 1.0f / 16.4f;
  _i2cAddr = ICM_ADDR_PRIMARY;
  _fifoMode = FIFO_NONE;
  _debug = false; 
  _csPin = -1;
}

void ICM42688P::setDebug(bool enable) {
  _debug = enable;
}

bool ICM42688P::beginI2C(uint32_t freq, uint8_t i2cAddr, int8_t sdaPin, int8_t sclPin) {
  return begin(BUS_I2C, -1, freq, i2cAddr, sclPin, sdaPin, -1);
}

bool ICM42688P::beginSPI(int8_t csPin, uint32_t freq, int8_t sckPin, int8_t misoPin, int8_t mosiPin) {
  return begin(BUS_SPI, csPin, freq, ICM_ADDR_PRIMARY, sckPin, misoPin, mosiPin);
}

bool ICM42688P::begin(ICM_BUS busType, int8_t csPin, uint32_t freq, uint8_t i2cAddr, int8_t sckSclPin, int8_t misoSdaPin, int8_t mosiPin) {
  _bus = busType;
  _csPin = csPin;
  _i2cAddr = i2cAddr;

  #if defined(ESP32) || defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_RP2350)
    uint32_t defaultSpiFreq = 12000000; 
    uint32_t defaultI2cFreq = 400000;   
    ICM_ODR defaultOdr = ODR_8KHZ;
  #else
    uint32_t defaultSpiFreq = 4000000;  
    uint32_t defaultI2cFreq = 100000;   
    ICM_ODR defaultOdr = ODR_500HZ;
  #endif

  _spiFreq = (freq == 0) ? ((_bus == BUS_SPI) ? defaultSpiFreq : defaultI2cFreq) : freq;

  if (_bus == BUS_SPI) {
    if (_csPin != -1) {
      pinMode(_csPin, OUTPUT);
      digitalWrite(_csPin, HIGH);
    }
    
    #if defined(ESP32)
      if (sckSclPin != -1 && misoSdaPin != -1 && mosiPin != -1) {
        SPI.begin(sckSclPin, misoSdaPin, mosiPin, _csPin);
      } else {
        SPI.begin();
      }
    #elif defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_RP2350)
      if (sckSclPin != -1 && misoSdaPin != -1 && mosiPin != -1) {
        SPI.setSCK(sckSclPin);
        SPI.setRX(misoSdaPin);
        SPI.setTX(mosiPin);
      }
      SPI.begin();
    #else
      SPI.begin(); 
    #endif

  } else {
    #if defined(ESP32) 
      if (sckSclPin != -1 && misoSdaPin != -1) {
        Wire.setPins(misoSdaPin, sckSclPin); 
      }
    #elif defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_RP2350)
      if (sckSclPin != -1 && misoSdaPin != -1) {
        Wire.setSDA(misoSdaPin);
        Wire.setSCL(sckSclPin);
      }
    #endif
    
    Wire.begin();
    Wire.setClock(_spiFreq);

    Wire.beginTransmission(_i2cAddr);
    if (Wire.endTransmission() != 0) {
      uint8_t altAddr = (_i2cAddr == ICM_ADDR_PRIMARY) ? ICM_ADDR_SECONDARY : ICM_ADDR_PRIMARY;
      Wire.beginTransmission(altAddr);
      if (Wire.endTransmission() == 0) {
        _i2cAddr = altAddr;
      }
    }
  }
  
  delay(10);
  writeRegister(ICM42688_REG_DEVICE_CONFIG, 0x01); 
  delay(10); 

  uint8_t who = readRegister(ICM42688_REG_WHO_AM_I);
  if (who != WHO_AM_I_EXPECTED) {
    return false;
  }

  writeRegister(ICM42688_REG_PWR_MGMT0, 0x0F); 
  delay(5);

  setAccelFS(AFS_16G);
  setGyroFS(GFS_2000DPS);
  setODR(defaultOdr);
  setFIFOMode(FIFO_NONE);

  return true;
}

void ICM42688P::readRegisters(uint8_t startReg, uint8_t* buffer, size_t len) {
  if (_bus == BUS_SPI) {
    SPI.beginTransaction(SPISettings(_spiFreq, MSBFIRST, SPI_MODE0));
    if (_csPin != -1) digitalWrite(_csPin, LOW);
    SPI.transfer(startReg | 0x80); 
    for(size_t i=0; i<len; i++) buffer[i] = SPI.transfer(0x00);
    if (_csPin != -1) digitalWrite(_csPin, HIGH);
    SPI.endTransaction();
  } else {
    Wire.beginTransmission(_i2cAddr);
    Wire.write(startReg);
    Wire.endTransmission(false);
    Wire.requestFrom((int)_i2cAddr, (int)len);
    for(size_t i=0; i<len; i++) buffer[i] = (Wire.available()) ? Wire.read() : 0;
  }
}

void ICM42688P::writeRegister(uint8_t reg, uint8_t data) {
  if (_bus == BUS_SPI) {
    SPI.beginTransaction(SPISettings(_spiFreq, MSBFIRST, SPI_MODE0));
    if (_csPin != -1) digitalWrite(_csPin, LOW);
    SPI.transfer(reg & 0x7F); 
    SPI.transfer(data);
    if (_csPin != -1) digitalWrite(_csPin, HIGH);
    SPI.endTransaction();
  } else {
    Wire.beginTransmission(_i2cAddr);
    Wire.write(reg); 
    Wire.write(data);
    Wire.endTransmission();
  }
}

uint8_t ICM42688P::readRegister(uint8_t reg) {
  uint8_t data = 0;
  if (_bus == BUS_SPI) {
    SPI.beginTransaction(SPISettings(_spiFreq, MSBFIRST, SPI_MODE0));
    if (_csPin != -1) digitalWrite(_csPin, LOW);
    SPI.transfer(reg | 0x80); 
    data = SPI.transfer(0x00);
    if (_csPin != -1) digitalWrite(_csPin, HIGH);
    SPI.endTransaction();
  } else {
    Wire.beginTransmission(_i2cAddr);
    Wire.write(reg); 
    Wire.endTransmission(false);
    Wire.requestFrom((int)_i2cAddr, 1);
    if (Wire.available()) data = Wire.read();
  }
  return data;
}

int ICM42688P::getODRHz() {
  return _getHzFromODR(_odr);
}

int ICM42688P::_getHzFromODR(ICM_ODR odr) {
  switch(odr) {
    case ODR_32KHZ: return 32000;
    case ODR_16KHZ: return 16000;
    case ODR_8KHZ:  return 8000;
    case ODR_4KHZ:  return 4000;
    case ODR_2KHZ:  return 2000;
    case ODR_1KHZ:  return 1000;
    case ODR_500HZ: return 500;
    case ODR_200HZ: return 200;
    case ODR_100HZ: return 100;
    case ODR_50HZ:  return 50;
    case ODR_25HZ:  return 25;
    case ODR_12_5HZ:return 12; 
    default: return 0;
  }
}

void ICM42688P::enforceBandwidthLimit() {
  uint32_t bytesPerRead = 0;
  
  if (_fifoMode == FIFO_NONE) bytesPerRead = 13; 
  else if (_fifoMode == FIFO_16BIT) bytesPerRead = 20; 
  else if (_fifoMode == FIFO_20BIT_HIRES) bytesPerRead = 24; 

  uint32_t bitsPerByte = (_bus == BUS_I2C) ? 10 : 8; 
  uint32_t bitsPerRead = bytesPerRead * bitsPerByte;
  uint32_t maxAllowedBps = (uint32_t)(_spiFreq * 0.8f);

  ICM_ODR odrList[] = {ODR_32KHZ, ODR_16KHZ, ODR_8KHZ, ODR_4KHZ, ODR_2KHZ, ODR_1KHZ, ODR_500HZ, ODR_200HZ, ODR_100HZ, ODR_50HZ, ODR_25HZ, ODR_12_5HZ};
  ICM_ODR safeODR = _odr;
  
  for (int i = 0; i < 12; i++) {
    int targetHz = _getHzFromODR(odrList[i]);
    if (_getHzFromODR(safeODR) > targetHz) continue; 
    
    uint32_t requiredBps = bitsPerRead * targetHz;
    if (requiredBps <= maxAllowedBps) {
      safeODR = odrList[i];
      break;
    }
  }

  if (safeODR != _odr) {
    _odr = safeODR; 
  }

  uint8_t odd = (uint8_t)_odr;
  uint8_t gConf = readRegister(ICM42688_REG_GYRO_CONFIG0) & 0xF0;
  uint8_t aConf = readRegister(ICM42688_REG_ACCEL_CONFIG0) & 0xF0;
  writeRegister(ICM42688_REG_GYRO_CONFIG0, gConf | odd);
  writeRegister(ICM42688_REG_ACCEL_CONFIG0, aConf | odd);
}

void ICM42688P::setODR(ICM_ODR odr) {
  _odr = odr;
  enforceBandwidthLimit();
}

void ICM42688P::setAccelFS(ICM_ACCEL_FS fs) {
  if (_fifoMode == FIFO_20BIT_HIRES) fs = AFS_16G;

  uint8_t conf = readRegister(ICM42688_REG_ACCEL_CONFIG0) & 0x1F;
  writeRegister(ICM42688_REG_ACCEL_CONFIG0, (conf & 0x1F) | (fs << 5));
  switch(fs) {
    case AFS_16G: _accelScaleFactor = 16.0f / 32768.0f; break; 
    case AFS_8G:  _accelScaleFactor = 8.0f  / 32768.0f; break; 
    case AFS_4G:  _accelScaleFactor = 4.0f  / 32768.0f; break; 
    case AFS_2G:  _accelScaleFactor = 2.0f  / 32768.0f; break; 
  }
}

void ICM42688P::setGyroFS(ICM_GYRO_FS fs) {
  if (_fifoMode == FIFO_20BIT_HIRES) fs = GFS_2000DPS;

  uint8_t conf = readRegister(ICM42688_REG_GYRO_CONFIG0) & 0x1F;
  writeRegister(ICM42688_REG_GYRO_CONFIG0, (conf & 0x1F) | (fs << 5));
  switch(fs) {
    case GFS_2000DPS: _gyroScaleFactor = 2000.0f / 32768.0f; break; 
    case GFS_1000DPS: _gyroScaleFactor = 1000.0f / 32768.0f; break; 
    case GFS_500DPS:  _gyroScaleFactor = 500.0f  / 32768.0f; break; 
    case GFS_250DPS:  _gyroScaleFactor = 250.0f  / 32768.0f; break; 
    case GFS_125DPS:  _gyroScaleFactor = 125.0f  / 32768.0f; break; 
  }
}

void ICM42688P::setFIFOMode(ICM_FIFO_MODE mode) {
  #if defined(__AVR__) || defined(ARDUINO_ARCH_AVR)
    if (mode == FIFO_20BIT_HIRES) mode = FIFO_16BIT;
  #endif

  _fifoMode = mode;

  writeRegister(ICM42688_REG_FIFO_CONFIG, 0x00);
  writeRegister(ICM42688_REG_FIFO_CONFIG1, 0x00);

  if (mode == FIFO_16BIT) {
    // [VOLTINO FIX] 0x03 enables Gyro (bit 1) and Accel (bit 0) into FIFO packets!
    writeRegister(ICM42688_REG_FIFO_CONFIG1, 0x03); 
    writeRegister(ICM42688_REG_FIFO_CONFIG, 0x40);
  } 
  else if (mode == FIFO_20BIT_HIRES) {
    setAccelFS(AFS_16G);
    setGyroFS(GFS_2000DPS);
    // [VOLTINO FIX] 0x13 enables High-Res (bit 4), Gyro (bit 1), Accel (bit 0)
    writeRegister(ICM42688_REG_FIFO_CONFIG1, 0x13); 
    writeRegister(ICM42688_REG_FIFO_CONFIG, 0x40);
  }
  
  enforceBandwidthLimit(); 
}

void ICM42688P::setAccelOffset(float x, float y, float z) { accOffset[0] = x; accOffset[1] = y; accOffset[2] = z; }
void ICM42688P::setAccelScale(float x, float y, float z) { accScale[0] = x; accScale[1] = y; accScale[2] = z; }
void ICM42688P::setGyroOffset(float x, float y, float z) { gyrOffset[0] = x; gyrOffset[1] = y; gyrOffset[2] = z; }

void ICM42688P::setGyroSoftwareOffset(float ox, float oy, float oz) { setGyroOffset(ox, oy, oz); }
void ICM42688P::setAccelSoftwareOffset(float ox, float oy, float oz) { setAccelOffset(ox, oy, oz); }
void ICM42688P::setAccelSoftwareScale(float sx, float sy, float sz) { setAccelScale(sx, sy, sz); }

void ICM42688P::getGyroSoftwareOffset(float &ox, float &oy, float &oz) { ox = gyrOffset[0]; oy = gyrOffset[1]; oz = gyrOffset[2]; }
void ICM42688P::getAccelSoftwareOffset(float &ox, float &oy, float &oz) { ox = accOffset[0]; oy = accOffset[1]; oz = accOffset[2]; }
void ICM42688P::getAccelSoftwareScale(float &sx, float &sy, float &sz) { sx = accScale[0]; sy = accScale[1]; sz = accScale[2]; }

void ICM42688P::getGyroOffset(float &ox, float &oy, float &oz) { ox = gyrOffset[0]; oy = gyrOffset[1]; oz = gyrOffset[2]; }
void ICM42688P::getAccelOffset(float &ox, float &oy, float &oz) { ox = accOffset[0]; oy = accOffset[1]; oz = accOffset[2]; }
void ICM42688P::getAccelScale(float &sx, float &sy, float &sz) { sx = accScale[0]; sy = accScale[1]; sz = accScale[2]; }

float ICM42688P::getAccelOffsetX() { return accOffset[0]; }
float ICM42688P::getAccelOffsetY() { return accOffset[1]; }
float ICM42688P::getAccelOffsetZ() { return accOffset[2]; }
float ICM42688P::getAccelScaleX() { return accScale[0]; }
float ICM42688P::getAccelScaleY() { return accScale[1]; }
float ICM42688P::getAccelScaleZ() { return accScale[2]; }
float ICM42688P::getGyroOffsetX() { return gyrOffset[0]; }
float ICM42688P::getGyroOffsetY() { return gyrOffset[1]; }
float ICM42688P::getGyroOffsetZ() { return gyrOffset[2]; }

void ICM42688P::resetHardwareOffsets() { setBank(0); }
void ICM42688P::setBank(uint8_t bank) { writeRegister(ICM42688_REG_BANK_SEL, bank); }

bool ICM42688P::readIMU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  switch (_fifoMode) {
    case FIFO_NONE: return readSensorData(ax, ay, az, gx, gy, gz);
    case FIFO_16BIT: return readHardwareFIFO(ax, ay, az, gx, gy, gz);
    case FIFO_20BIT_HIRES: return readHardwareFIFOHires(ax, ay, az, gx, gy, gz);
    default: return false;
  }
}

bool ICM42688P::readFIFO(float& ax, float& ay, float& az, float& gx, float& gy, float& gz) {
  return readIMU(ax, ay, az, gx, gy, gz);
}

bool ICM42688P::readSensorData(float& ax, float& ay, float& az, float& gx, float& gy, float& gz) {
  uint8_t buffer[12];
  readRegisters(ICM42688_REG_ACCEL_DATA_X1, buffer, 12);
  
  int16_t rawAx = (int16_t)((buffer[0] << 8) | buffer[1]);
  int16_t rawAy = (int16_t)((buffer[2] << 8) | buffer[3]);
  int16_t rawAz = (int16_t)((buffer[4] << 8) | buffer[5]);
  int16_t rawGx = (int16_t)((buffer[6] << 8) | buffer[7]);
  int16_t rawGy = (int16_t)((buffer[8] << 8) | buffer[9]);
  int16_t rawGz = (int16_t)((buffer[10] << 8) | buffer[11]);
  
  if (rawAx == -1 && rawAy == -1 && rawAz == -1 && rawGx == -1) return false;
  
  ax = ((float)rawAx * _accelScaleFactor - accOffset[0]) * accScale[0];
  ay = ((float)rawAy * _accelScaleFactor - accOffset[1]) * accScale[1];
  az = ((float)rawAz * _accelScaleFactor - accOffset[2]) * accScale[2];
  
  gx = (float)rawGx * _gyroScaleFactor - gyrOffset[0];
  gy = (float)rawGy * _gyroScaleFactor - gyrOffset[1];
  gz = (float)rawGz * _gyroScaleFactor - gyrOffset[2];
  
  return true;
}

bool ICM42688P::readHardwareFIFO(float& ax, float& ay, float& az, float& gx, float& gy, float& gz) {
  uint8_t countBuf[2];
  readRegisters(ICM42688_REG_FIFO_COUNTH, countBuf, 2);
  uint16_t fifoCount = (countBuf[0] << 8) | countBuf[1];

  if (fifoCount < 16) return false; 

  // [VOLTINO FIX] Pravý Flush Bit v SIGNAL_PATH_RESET je 0x02!
  if (fifoCount >= 1000) {
      writeRegister(ICM42688_REG_SIGNAL_PATH_RESET, 0x02); 
      return false;
  }

  uint8_t buffer[16];
  readRegisters(ICM42688_REG_FIFO_DATA, buffer, 16);

  if ((buffer[0] & 0x80) != 0) { // Pokud vytáhneme prázdnou hlavičku, flush
      writeRegister(ICM42688_REG_SIGNAL_PATH_RESET, 0x02); 
      return false; 
  }

  int16_t rawAx = (int16_t)((buffer[1] << 8) | buffer[2]);
  int16_t rawAy = (int16_t)((buffer[3] << 8) | buffer[4]);
  int16_t rawAz = (int16_t)((buffer[5] << 8) | buffer[6]);
  int16_t rawGx = (int16_t)((buffer[7] << 8) | buffer[8]);
  int16_t rawGy = (int16_t)((buffer[9] << 8) | buffer[10]);
  int16_t rawGz = (int16_t)((buffer[11] << 8) | buffer[12]);

  ax = ((float)rawAx * _accelScaleFactor - accOffset[0]) * accScale[0];
  ay = ((float)rawAy * _accelScaleFactor - accOffset[1]) * accScale[1];
  az = ((float)rawAz * _accelScaleFactor - accOffset[2]) * accScale[2];
  
  gx = (float)rawGx * _gyroScaleFactor - gyrOffset[0];
  gy = (float)rawGy * _gyroScaleFactor - gyrOffset[1];
  gz = (float)rawGz * _gyroScaleFactor - gyrOffset[2];

  return true;
}

bool ICM42688P::readHardwareFIFOHires(float& ax, float& ay, float& az, float& gx, float& gy, float& gz) {
  uint8_t countBuf[2];
  readRegisters(ICM42688_REG_FIFO_COUNTH, countBuf, 2);
  uint16_t fifoCount = (countBuf[0] << 8) | countBuf[1];

  if (fifoCount < 20) return false; 

  if (fifoCount >= 1000) {
      writeRegister(ICM42688_REG_SIGNAL_PATH_RESET, 0x02); 
      return false;
  }

  uint8_t buffer[20];
  readRegisters(ICM42688_REG_FIFO_DATA, buffer, 20);

  if ((buffer[0] & 0x80) != 0) {
      writeRegister(ICM42688_REG_SIGNAL_PATH_RESET, 0x02);
      return false;
  }

  int32_t rawAx = (int32_t)( ((uint32_t)buffer[1] << 12) | ((uint32_t)buffer[2] << 4) | (buffer[17] & 0x0F) );
  rawAx = (rawAx << 12) >> 12;
  
  int32_t rawAy = (int32_t)( ((uint32_t)buffer[3] << 12) | ((uint32_t)buffer[4] << 4) | (buffer[18] >> 4) );
  rawAy = (rawAy << 12) >> 12;

  int32_t rawAz = (int32_t)( ((uint32_t)buffer[5] << 12) | ((uint32_t)buffer[6] << 4) | (buffer[19] >> 4) );
  rawAz = (rawAz << 12) >> 12;
  
  int32_t rawGx = (int32_t)( ((uint32_t)buffer[7] << 12) | ((uint32_t)buffer[8] << 4) | (buffer[17] >> 4) );
  rawGx = (rawGx << 12) >> 12;
  
  int32_t rawGy = (int32_t)( ((uint32_t)buffer[9] << 12) | ((uint32_t)buffer[10] << 4) | (buffer[18] & 0x0F) );
  rawGy = (rawGy << 12) >> 12;

  int32_t rawGz = (int32_t)( ((uint32_t)buffer[11] << 12) | ((uint32_t)buffer[12] << 4) | (buffer[19] & 0x0F) );
  rawGz = (rawGz << 12) >> 12;

  float scaleAccel20 = 16.0f / 524288.0f;
  float scaleGyro20  = 2000.0f / 524288.0f;

  ax = ((float)rawAx * scaleAccel20 - accOffset[0]) * accScale[0];
  ay = ((float)rawAy * scaleAccel20 - accOffset[1]) * accScale[1];
  az = ((float)rawAz * scaleAccel20 - accOffset[2]) * accScale[2];
  
  gx = (float)rawGx * scaleGyro20 - gyrOffset[0];
  gy = (float)rawGy * scaleGyro20 - gyrOffset[1];
  gz = (float)rawGz * scaleGyro20 - gyrOffset[2];

  return true;
}

float ICM42688P::readTemperature() {
  uint8_t buffer[2];
  readRegisters(ICM42688_REG_TEMP_DATA1, buffer, 2);
  int16_t rawTemp = (int16_t)((buffer[0] << 8) | buffer[1]);
  return ((float)rawTemp / 132.48f) + 25.0f;
}

void ICM42688P::autoCalibrateGyro(uint16_t samples) { /* nezměněno */ }
void ICM42688P::autoCalibrateAccel() { /* nezměněno */ }