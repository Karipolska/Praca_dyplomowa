#include "imu.hpp"

void getAGMT(ICM_20948_I2C *sensor, AGMTfloat &data){
    
    data.acc_x = sensor->accX(); // mili g
    data.acc_y = sensor->accY();
    data.acc_z = sensor->accZ();
    data.gyro_x = sensor->gyrX(); // dps
    data.gyro_y = sensor->gyrY();
    data.gyro_z = sensor->gyrZ();
    data.mag_x = sensor->magX(); // uT
    data.mag_y = sensor->magY();
    data.mag_z = sensor->magZ();
    data.temp = sensor->temp(); // C
}

void SensDataToSi(AGMTint32_t &data_int){
    
    data_int.acc_x = milgToSi(data_int.acc_x); // m/s2
    data_int.acc_y = milgToSi(data_int.acc_y);
    data_int.acc_z = milgToSi(data_int.acc_z);
    /* Don't change rest of data */
}

void InterpretRawData(icm_20948_DMP_data_t &raw_data, AGMTint32_t &data_int){

    auto normalizeData = [&](int16_t data) -> double{
        double data_d = static_cast<double>(data);
        /* Value range is +-scale saved in int16_t. */
        return (data_d / INT16_MAX); 
    };

    auto getFloat = [&](double data_d, uint16_t scale){
        return static_cast<float>(data_d * scale);
    };

    data_int.acc_x = floatToInt32Prec(gToSi(getFloat(normalizeData(raw_data.Raw_Accel.Data.X), 4)));
    data_int.acc_y = floatToInt32Prec(gToSi(getFloat(normalizeData(raw_data.Raw_Accel.Data.Y), 4)));
    data_int.acc_z = floatToInt32Prec(gToSi(getFloat(normalizeData(raw_data.Raw_Accel.Data.Z), 4)));
    data_int.gyro_x = floatToInt32Prec(getFloat(normalizeData(raw_data.Raw_Gyro.Data.X), 500)); 
    data_int.gyro_y = floatToInt32Prec(getFloat(normalizeData(raw_data.Raw_Gyro.Data.Y), 500));
    data_int.gyro_z = floatToInt32Prec(getFloat(normalizeData(raw_data.Raw_Gyro.Data.Z), 500));
    /* Magnetometer and temperature are not used so nothing to be interpreted here */
}
/**
 * Data addded in these functions was obtained by 30s data read in stable position.
 */
void CalibrateValues(AGMTint32_t &data_int){
    data_int.acc_z = data_int.acc_z - 10000;
    data_int.acc_y = data_int.acc_y + 14500;
    data_int.acc_x = data_int.acc_x + 800;
    data_int.gyro_x = data_int.gyro_x - 11500;
    data_int.gyro_y = data_int.gyro_y - 58140;
    data_int.gyro_z = data_int.gyro_z + 31750;
}
/*
stare dane, przed 29.05
    data_int.acc_z = data_int.acc_z + 13550;
    data_int.acc_y = data_int.acc_y + 41760;
    data_int.acc_x = data_int.acc_x + 6000;
    data_int.gyro_x = data_int.gyro_x - 3680;
    data_int.gyro_y = data_int.gyro_y - 76710;
    data_int.gyro_z = data_int.gyro_z - 36700;

    30.05
    data_int.acc_z = data_int.acc_z - 12000;
    data_int.acc_y = data_int.acc_y + 18300;
    data_int.acc_x = data_int.acc_x - 3300;
    data_int.gyro_x = data_int.gyro_x - 11500;
    data_int.gyro_y = data_int.gyro_y - 58140;
    data_int.gyro_z = data_int.gyro_z + 31750;


*/