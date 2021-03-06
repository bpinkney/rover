#ifndef IMU_H
#define IMU_H

#include "mbed.h"
#include "k64f_common.h"

// definitions for the gyro sensor
#define L3GD20_REGISTER_WHO_AM_I	0x0F
#define L3GD20_ADDR           0xD6 //0x6B //
#define L3GD20_CTRL_REG1      0x20
#define L3GD20_CTRL_REG4      0x23
#define L3GD20_STATUS_REG     0x27
#define L3GD20_OUT_X_L        0x28

#define L3GD20_OUT_TEMP       0x26

#define L3GD20_RANGE_250DPS   0x00                // Measurement range selected by CTRL_REG4
#define L3GD20_RANGE_500DPS   0x01                // Default range = 250 Degree-per-Second = 0.7 rev/second
#define L3GD20_RANGE_2000DPS  0x02                // Range determines Sensitivity
#define L3GD20_SENSITIVITY_250DPS   0.00875       // Roughly 22/256 for fixed point match
#define L3GD20_SENSITIVITY_500DPS   0.0175        // Roughly 45/256
#define L3GD20_SENSITIVITY_2000DPS  0.070         // Roughly 18/256
#define L3GD20_DPS_TO_RADS          0.017453293   // = pi/180 degrees/s to rad/s multiplier


// definitions for the accelerometer
#define LSM303_A_ADDR            0x32      // address for writing. +1 for reading, see manual p. 20/42.
#define LSM303_A_CTRL_REG1       0x20
#define LSM303_A_CTRL_REG2       0x21
#define LSM303_A_CTRL_REG4       0x23
#define LSM303_A_OUT_X_L         0x28
#define LSM303_A_FS_2G           0x00      // Full Scale measurement range - selected by CTRL_REG4
#define LSM303_A_Sensitivity     0.001     // Linear acceleration sensitivity
#define LSM303_A_GRAVITY_EARTH   9.80665f   // Earth's gravity in m/s^2 upon calibration of sensor

// definitions for the magnetic sensor
#define LSM303_M_ADDR        0x3C // address for writing. +1 for reading, see datasheet p. 21/42.
#define LSM303_M_CRA_REG     0x00 
#define LSM303_M_CRB_REG     0x01 
#define LSM303_M_MR_REG      0x02
#define LSM303_M_OUT_X_H     0x03        // Watch out: order of H and L reversed 
#define LSM303_M_FS_13G      0x01        // Full Scale measuremetn range - selected by CRB_REG
#define LSM303_M_Sensitivity_XY  1100       // Corresponding sensitivity = 1100 Bits/Gauss
#define LSM303_M_Sensitivity_Z  980       // Corresponding sensitivity = 980 Bits/Gauss


//determined by bpinkney, matlab trials


class IMU
{
  public:
/* constructor */
	IMU();
    IMU(PinName sda, PinName scl);

/* accessible functions */
    char init(void);
    //char readData(float *);
    char readGyroTemp();
    char readGyros();
    char readAccs();
    char readMags();
    void filterData(float *, double *);
    uint32_t whoAmI();
    
  private:
      I2C _i2c;
      char address;
      int16_t L3GD20_biasX;  /* digit counts */
      int16_t L3GD20_biasY;
      int16_t L3GD20_biasZ;
      float last_gyro_x;
      float last_gyro_y;
      float last_gyro_z;

      double  g_0;
      double  FF[3];
      double  FD[6][9]; 
};

#endif
