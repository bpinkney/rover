
#include <stdint.h>

#ifndef DATA_STRUCTS_H_
#define DATA_STRUCTS_H_

//internal accelerometer
struct k64f_acc_data_t{//__attribute__ ((__packed__)) {
    float x;
    float y;
    float z;
};

//internal mag
struct k64f_mag_data_t{//__attribute__ ((__packed__)) {
    float x;
    float y;
    float z;
};

//external gyro
struct ext_gyro_data_t{//__attribute__ ((__packed__)) {
    float x;
    float y;
    float z;
};

struct ext_gyro_temp_t{//__attribute__ ((__packed__)) {
    int16_t temp_c;
};

//external acc
struct ext_acc_data_t{//__attribute__ ((__packed__)) {
    float x;
    float y;
    float z;
};

//external mag
struct ext_mag_data_t{//__attribute__ ((__packed__)) {
    float x;
    float y;
    float z;
};

//orientation estimation in radians
struct craft_orientation_est_t{//__attribute__ ((__packed__)) {
    float roll;//y
    float pitch;//x
    float yaw;
};

//desired ang orientation in radians
struct craft_orientation_des_t{//__attribute__ ((__packed__)) {
    float roll;//y
    float pitch;//x
    float yaw;
};

//desired or est ang rates in radians/s
struct craft_rates_t{//__attribute__ ((__packed__)) {
    float roll;//y
    float pitch;//x
    float yaw;
};

//desired or est ang acceleration in radians/s^2
struct craft_accs_t{//__attribute__ ((__packed__)) {
    float roll;//y
    float pitch;//x
    float yaw;
};

//desired motor thrusts, range[0:1] (0 - 100% thrust) ~ thrust * 10000 RPM w/turnigy 935s and 3S battery
struct motor_thrust_des_t{//__attribute__ ((__packed__)) {
    float fl;//
    float fr;
    float rr;
    float rl;
};

//IR sensor stuff
struct top_vector_t{//__attribute__ ((__packed__)) {
    float magnitude;
    float trust;
};

struct side_vector_t{//__attribute__ ((__packed__)) {
    float magnitude;
    float trust;
    float a;
    float b;
    float a_trust;
    float b_trust;
};

struct bottom_vector_t{//__attribute__ ((__packed__)) {
    float magnitude;
    float trust;
    float a;
	float b;
	float c;
	float a_trust;
	float b_trust;
	float c_trust;
};




#endif
