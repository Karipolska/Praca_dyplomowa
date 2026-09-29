#ifndef IMU_HPP
#define IMU_HPP

#include "ICM_20948.h" 
#include "helpers.hpp"

struct AGMTfloat {
  float acc_x;
  float acc_y;
  float acc_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;
  float mag_x;
  float mag_y;
  float mag_z;
  float temp;
};

struct AGMTint32_t {
  int32_t acc_x;
  int32_t acc_y;
  int32_t acc_z;
  int32_t gyro_x;
  int32_t gyro_y;
  int32_t gyro_z;
  int32_t mag_x;
  int32_t mag_y;
  int32_t mag_z;
  int32_t temp;
};

void getAGMT(ICM_20948_I2C *sensor, AGMTfloat &data);
void SensDataToSi(AGMTint32_t &data_int);
void InterpretRawData(icm_20948_DMP_data_t &raw_data, AGMTint32_t &data_int);
void CalibrateValues(AGMTint32_t &data_int);

#endif