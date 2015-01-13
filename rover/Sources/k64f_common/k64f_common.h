#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#include <ctime>
#include <iostream>
#include <fstream>
#include <sstream> // for ostringstream
#include <string>

#include "mbed.h"
#include "rtos.h"
#include "FXOS8700Q.h"
#include "SDFileSystem.h"
//#include "Adafruit_L3GD20.h"

#include "data_structs.h"

#ifndef K64F_COMMON_H_
#define K64F_COMMON_H_

//usb serial out (9600 baud)
extern Serial pc;

//internal i2c bus for acc and mag
extern FXOS8700Q_acc k64f_acc; // Proper Ports and I2C Address for K64F Freedom board
extern FXOS8700Q_mag k64f_mag; // Proper Ports and I2C Address for K64F Freedom board

//sd card
//extern SDFileSystem sd;

//external (adafruit) accel and mag, gyro, baro and temp
//extern IMU ext_imu;

//data struct mutexes (why not have a crapload? a hit of RAM now for free threads later)
/*extern Mutex k64f_acc_mutex;
extern Mutex k64f_mag_mutex;

//data struct global iterations
extern k64f_acc_data_t k64f_acc_data;
extern k64f_mag_data_t k64f_mag_data;
*/

//data struct getters and setters
void get_k64f_acc_data(void* buffer, int size);
void set_k64f_acc_data(k64f_acc_data_t value);

void get_k64f_mag_data(void* buffer, int size);
void set_k64f_mag_data(k64f_mag_data_t value);

void get_ext_gyro_data(void* buffer, int size);
void set_ext_gyro_data(ext_gyro_data_t value);

void get_ext_acc_data(void* buffer, int size);
void set_ext_acc_data(ext_acc_data_t value);

void get_ext_mag_data(void* buffer, int size);
void set_ext_mag_data(ext_mag_data_t value);

//macros

#endif

