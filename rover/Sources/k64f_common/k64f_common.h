#include <math.h>
#include <stdarg.h>
#include <stdio.h>
//#include <unistd.h>
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
#include <math.h>
//#include "Adafruit_L3GD20.h"

#include "data_structs.h"

#ifndef K64F_COMMON_H_
#define K64F_COMMON_H_

//usb serial out (9600 baud)
//extern Serial pc;

//radio
extern Serial rs;

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

void get_ext_gyro_temp(void* buffer, int size);
void set_ext_gyro_temp(ext_gyro_temp_t value);

void get_ext_acc_data(void* buffer, int size);
void set_ext_acc_data(ext_acc_data_t value);

void get_ext_mag_data(void* buffer, int size);
void set_ext_mag_data(ext_mag_data_t value);

void get_craft_orientation_est(void* buffer, int size);
void set_craft_orientation_est(craft_orientation_est_t value);

void get_craft_orientation_des(void* buffer, int size);
void set_craft_orientation_des(craft_orientation_des_t value);

void get_craft_rates_des(void* buffer, int size);
void set_craft_rates_des(craft_rates_t value);

void get_craft_accs_est(void* buffer, int size);//angular accs
void set_craft_accs_est(craft_accs_t value);

void get_motor_thrust_des(void* buffer, int size);
void set_motor_thrust_des(motor_thrust_des_t value);

//ir sensors
void get_lf_ir_pair(void* buffer, int size);
void set_lf_ir_pair(side_vector_t value);
void get_rf_ir_pair(void* buffer, int size);
void set_rf_ir_pair(side_vector_t value);
void get_rb_ir_pair(void* buffer, int size);
void set_rb_ir_pair(side_vector_t value);
void get_lb_ir_pair(void* buffer, int size);
void set_lb_ir_pair(side_vector_t value);

//macros
#define LP_FILT(var, new_val, N)  \
  do { var = ((float)var * ((float)N) + ((float)new_val)) / ((float)N+1); } while(false)

#define WRAP_2PI(rad)		\
  while ((rad) > M_PI)		\
  {							\
    rad -= 2*M_PI;			\
  }							\
  while ((rad) < -M_PI)		\
  {							\
    rad += 2*M_PI;			\
  }							\

//#define WRAP_TO_PI(x) (x < -M_PI ? x+M_PI*2 : (x > M_PI ? x - M_PI*2: x))

#define BOUND_VARIABLE(VAR,LOW,HIGH) \
  do \
  { \
    if ((VAR) < (LOW)) \
    { \
      (VAR) = (LOW); \
    } \
    else if ((VAR) > (HIGH)) \
    { \
      (VAR) = (HIGH); \
    } \
  } \
  while (0)

#endif



