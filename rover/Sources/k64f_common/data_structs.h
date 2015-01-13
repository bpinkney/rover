
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
    int16_t x;
    int16_t y;
    int16_t z;
};

//external gyro
struct ext_gyro_data_t{//__attribute__ ((__packed__)) {
    float x;
    float y;
    float z;
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

#endif
