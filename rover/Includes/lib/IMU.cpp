#include <stdio.h>
#include "mbed.h"
#include "IMU.h"
#include "k64f_common.h"

IMU::IMU(PinName sda, PinName scl) : _i2c(sda, scl) {
    _i2c.frequency(400000);  /* 400kHz, fast mode. */
}

char IMU::init(void)  /* returns error upon a non-zero return */
{
   char ack, rx, tx[2];
   double pi, a, A;

// 1. Initialize selected registers: 2c.read and i2c.write return 0 on success (ack)
// --------------------------------
//
// 1.a Enable L3DG20 gyrosensor and set operational mode:
// CTRL_REG1: set to 0x1F = 0001-1111 --> enable sensor, DR= 95Hz, LPF-Cut-off-freq=25Hz.
// CTRL_REG1: set to 0x5F = 0101-1111 --> enable sensor, DR=190Hz, LPF-Cut-off-freq=25Hz.
// CTRL_REG4: left at default = 0x00 --> Full Scale = 250 degrees/second --> Sensitivity = 0.00875 dps/digit.
    address = L3GD20_ADDR;
    tx[0] = L3GD20_CTRL_REG1; // address contrl_register 1
    tx[1] = 0x5F; // 01-01-1-111 enable sensor and set operational mode.
    ack = _i2c.write(address, tx, 2);
    ack |= _i2c.write(address, tx, 1);
    ack |= _i2c.read(address+1, &rx, 1); if (rx != 0x5F) ack |= 1;

    address = L3GD20_ADDR;
	tx[0] = L3GD20_CTRL_REG4; // address contrl_register 4
	tx[1] = 0x10; // 00-01-0-000 set Full Scale = 500 deg/s for testing.
	ack = _i2c.write(address, tx, 2);
	ack |= _i2c.write(address, tx, 1);
	ack |= _i2c.read(address+1, &rx, 1); if (rx != 0x10) ack |= 1;
//
// 1.b Enable LSM303 accelerometer and set operational mode:
// CTRL_REG1: set to 0x37 = 0011 1111 --> DR =  25Hz & enable sensor in low power mode (normal mode zmienic ostatnie 4 na 0111)
// CTRL_REG1: set to 0x47 = 0100 1111 --> DR =  50Hz & enable sensor -//
// CTRL_REG1: set to 0x57 = 0101 1111 --> DR = 100Hz & enable sensor
// CTRL_REG1: set to 0x67 = 0110 1111 --> DR = 200Hz & enable sensor 
// CTRL_REG1: set to 0x77 = 0111 0111 --> DR = 400Hz & enable sensor (0x7F low power mode - b duze oscylacje)
// CTRL_REG4: set to 0x08 = 0000 1000 --> Full Scale = +/- 2G & high resolution --> Sensitivity = 0.001G/digit.

    //data rate
    address = LSM303_A_ADDR; 
    tx[0] = LSM303_A_CTRL_REG1;
    tx[1] = 0x77; //                    --> 400 Hz Data rate speed - p.24/42 of datasheet
    ack |= _i2c.write(address, tx, 2);
    ack |= _i2c.write(address, tx, 1);
    ack |= _i2c.read(address+1, &rx, 1); if (rx != 0x77) ack |= 1;

    //highpass filter
   /* tx[0] = LSM303_A_CTRL_REG2;
	tx[1] = 0x30; //
	ack |= _i2c.write(address, tx, 2);
	ack |= _i2c.write(address, tx, 1);
	ack |= _i2c.read(address+1, &rx, 1); if (rx != 0x30) ack |= 1;*/

	//scale range
    tx[0] = LSM303_A_CTRL_REG4;
    //tx[1] = 0x08; // 0000 1000 enable high resolution mode + selects default 2G scale. p.26/42
    tx[1] = 0x18; // 0001 1000 enable high resolution mode + selects 4G scale.
    ack |= _i2c.write(address, tx ,2);
    ack |= _i2c.write(address, tx, 1);
    ack |= _i2c.read(address+1, &rx, 1); if (rx != 0x18) ack |= 1;
//
// 1.c enable LSM303 magnetometer and set operational mode:
// CRA_REG is reset from 0x10 to 0x14 = 00010100 -->  30 Hz data output rate.
// CRA_REG is reset from 0x10 to 0x18 = 00011000 -->  75 Hz data output rate.
// CRA_REG is reset from 0x10 to 0x1C = 00011100 --> 220 Hz data output rate.
// CRB_REG is kept at default = 00100000 = 0x20 --> range +/- 1.3 Gauss, Gain = 1100/980(Z) LSB/Gauss.
// MR_REG is reset from 0x03 to 0x00 -> continuos conversion mode in stead of sleep mode.
    address = LSM303_M_ADDR; 
    tx[0] = LSM303_M_CRA_REG;
    tx[1] = 0x18;                       //  -->  75 Hz minimum output rate - p.36/42 of datasheet
    ack |= _i2c.write(address, tx, 2);
    ack |= _i2c.write(address, tx, 1);
    ack |= _i2c.read(address+1, &rx, 1); if (rx != 0x18) ack |= 1;    
    tx[0] = LSM303_M_MR_REG;
    tx[1] = 0x00; // 0000 0000 --> continuous-conversion mode 25 Hz Data rate speed - p.24/42 of datasheet
    ack |= _i2c.write(address, tx, 2);
    ack |= _i2c.write(address, tx, 1);
    ack |= _i2c.read(address+1, &rx, 1); if (rx != 0x00) ack |= 1;    
 
// 2. Initialize calibration constants with predetermined values.
// acceleration:
// My calibration values, vs. the website http://rwsarduino.blogspot.be/2013/01/inertial-orientation-sensing.html

/* my predetermined static bias counts */
    L3GD20_biasX = (int16_t)  97; // BTLO 90 /* digit counts */
    L3GD20_biasY = (int16_t)  5; //BYLO 180
    L3GD20_biasZ = (int16_t) 140; //BYLO -10

/* reference gravity acceleration */
    //g_0 = 9.815; //przy ustawieniu zakresu na +/- 2g
    g_0 = 19.63; //przy ustawienu zakresu na +/- 4g  // CTR_REG 4 = 0001 1000 0x18

/* filter parameters: assume 400 Hz sampling rare and 2nd order Butterworth filter with fc = 5Hz. */
    pi = 3.1415926536;
    A = tan(pi*5/400); a = 1 + sqrt(2.0)*A + A*A;
    FF[1] = 2*(A*A-1)/a;
    FF[2] = (1-sqrt(2.0)*A+A*A)/a;
    FF[0] = (1+FF[1]+FF[2])/4;

    return ack;
}

uint32_t IMU::whoAmI() {
    char who_am_i = 0;
    _i2c.read(L3GD20_REGISTER_WHO_AM_I, &who_am_i, 1);
    return (uint32_t) who_am_i;
}

char IMU::readGyros(){
	char ack, reg, D[6];
	int16_t W[3];
	float x, y, z;
	//ext_gyro_data_t oegd = {0,0,0};
	//int16_t temp;

	// report the data in rad/s
	// gyro data are 16 bit readings per axis, stored: X_l, X_h, Y_l, Y_h, Z_l, Z_h
	// #define L3GD20_SENSITIVITY_250DPS 0.00875  ---  #define L3GD20_DPS_TO_RADS  0.017453293
	address = L3GD20_ADDR;
	reg = L3GD20_OUT_X_L | 0x80; // set address auto-increment bit
	ack = _i2c.write(address,&reg,1);  ack |= _i2c.read(address+1,D,6);
	W[0] = (int16_t) (D[1] << 8 | D[0]);
	W[1] = (int16_t) (D[3] << 8 | D[2]);
	W[2] = (int16_t) (D[5] << 8 | D[4]);

	//get_ext_gyro_temp(&temp, sizeof(temp));

	x = -((float)W[0]*L3GD20_SENSITIVITY_500DPS*L3GD20_DPS_TO_RADS);// - (float)(0.002171*((float)temp) - 0.044515);//(float) 0.971*(W[0]-L3GD20_biasX)*L3GD20_SENSITIVITY_250DPS*L3GD20_DPS_TO_RADS;
	y = ((float)W[1]*L3GD20_SENSITIVITY_500DPS*L3GD20_DPS_TO_RADS - 0.009710);// - (float)(-0.000237791*((float)temp) + 0.006740119);//(float) 0.998*(W[1]-L3GD20_biasY)*L3GD20_SENSITIVITY_250DPS*L3GD20_DPS_TO_RADS;
	z = -((float)W[2]*L3GD20_SENSITIVITY_500DPS*L3GD20_DPS_TO_RADS);// - (float)(-0.001434761*((float)temp) + 0.032907619);//(float) 1.002*(W[2]-L3GD20_biasZ)*L3GD20_SENSITIVITY_250DPS*L3GD20_DPS_TO_RADS;

	//get_ext_gyro_data(&oegd, sizeof(oegd));
	//last_gyro_x = x;
	//last_gyro_y = y;
	//last_gyro_z = z;
	//filter (iir lowpass, N = 10) //replace later
	LP_FILT(last_gyro_x, x, 3);
	LP_FILT(last_gyro_y, y, 3);
	LP_FILT(last_gyro_z, z, 3);

	set_ext_gyro_data({last_gyro_x, last_gyro_y, last_gyro_z});

	return ack;
}

char IMU::readGyroTemp(){
	char ack, reg, D[2];
	int16_t W;

	// +1 increment in output corresponds to -1C in temp
	// reading (T) of approx 13.5 corresponds to 22.5 degrees ambient
	// resulting calulation is thus: 36 - T = degrees centigrade

	address = L3GD20_ADDR;
	reg = L3GD20_OUT_TEMP | 0x80; // set address auto-increment bit
	ack = _i2c.write(address,&reg,1);  ack |= _i2c.read(address+1,D,2);
	W = (int16_t) (36 - (int16_t)(D[1] << 8 | D[0]));
	if(W < 55 && W > -20){ //filter crazy temp readings
		set_ext_gyro_temp({W});
	}

	return ack;
}

char IMU::readAccs(){
	char ack, reg, D[6];
	float x, y, z;
	int16_t W[3];
	// Accelerometer data are stored as 12 bit readings, left justified per axis.
	// The data needs to be shifted 4 digits to the right! This is not general, only for the A measurement.
	address = LSM303_A_ADDR;
	reg = LSM303_A_OUT_X_L | 0x80; // set address auto-increment bit
	ack |= _i2c.write(address,&reg,1);  ack |= _i2c.read(address+1,D,6);
	W[0] = ((int16_t) (D[1] << 8 | D[0])) >> 4;
	W[1] = ((int16_t) (D[3] << 8 | D[2])) >> 4;
	W[2] = ((int16_t) (D[5] << 8 | D[4])) >> 4;
	x = (float)W[0]*LSM303_A_GRAVITY_EARTH*2/1000;//(float) g_0*0.986*(W[0]+18)/1000;  // kalibracja - zmiana z 0.991 na 0.986 i z +34 na +18
	//*(d+3)
	y = (float)W[1]*LSM303_A_GRAVITY_EARTH*2/1000;//(float) g_0*0.99*(W[1]-4)/1000;   // kalibracja - zmiana z 0.970 na 0.99 i z +2 na -4
	z = (float)W[2]*LSM303_A_GRAVITY_EARTH*2/1000;//(float) g_0*0.983*(W[2]+9)/1000;  // kalibracja - zmiana +28 na +9

	set_ext_acc_data({x, y, z});

	return ack;
}
char IMU::readMags(){
	char ack, reg, D[6];
	int16_t W[3];
	float x, y, z;
	// GN = 001
	// Magnetometer; are stored as 12 bit readings, right justified per axis.
	address = LSM303_M_ADDR;
	reg = LSM303_M_OUT_X_H | 0x80; // set address auto-increment bit
	ack |= _i2c.write(address,&reg,1);  ack |= _i2c.read(address+1,D,6);
	W[0] = ((int16_t) (D[0] << 8 | D[1]));
	W[1] = ((int16_t) (D[4] << 8 | D[5]));
	W[2] = ((int16_t) (D[2] << 8 | D[3]));
	x = (float)W[0];//*LSM303_M_Sensitivity_XY;//(float) 2.813*(W[0]-220)/1100; //Z bylo 264
	y = (float)W[1];//*LSM303_M_Sensitivity_XY;//(float) 2.822*(W[1]+ 230)/1100; //X bylo
	z = (float)W[2];//*LSM303_M_Sensitivity_Z;//(float) 2.880*(W[2]- 380)/980;  //Y

	set_ext_mag_data({x, y, z});

	return ack;
}

/*char IMU::readData(float *d)
{
      char ack, reg, D[6];
      int16_t W[3];
}*/

void IMU::filterData(float *d, double *D) 
// 2nd order Butterworth filter. Filter coefficients FF computed in function init.
{
    for (int i=0; i<9; ++i) {
//        *(FD+9*i+2) = *(FD+9*i+1); *(FD+9*i+1) = *(FD+9*i); *(FD+9*i) = (double) d[i];
         FD[2][i] = FD[1][i]; FD[1][i] = FD[0][i]; FD[0][i] = (double) d[i];
         FD[5][i] = FD[4][i]; FD[4][i] = FD[3][i]; 
         FD[3][i] = FF[0]*(FD[0][i] + 2*FD[1][i] + FD[2][i]) - FF[1]*FD[4][i] - FF[2]*FD[5][i];
         D[i] = FD[3][i]; 
    }
//         D[0] = FD[0][2]; D[1] = FD[1][2]; D[2] = FD[2][2];
}
