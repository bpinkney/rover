#include "k64f_common.h"

//usb serial out (9600 baud)
Serial pc(USBTX, USBRX);


//internal i2c bus for acc and mag
FXOS8700Q_acc k64f_acc(PTE25, PTE24, FXOS8700CQ_SLAVE_ADDR1); // Proper Ports and I2C Address for K64F Freedom board
FXOS8700Q_mag k64f_mag(PTE25, PTE24, FXOS8700CQ_SLAVE_ADDR1); // Proper Ports and I2C Address for K64F Freedom board

//sd card
//SDFileSystem sd(PTE3, PTE1, PTE2, PTE4, "sdcard"); // MOSI, MISO, SCK, CS

//external (adafruit) accel and mag, gyro, baro and temp
//IMU ext_imu(PTE25,PTE24);



//data struct mutexes (why not have a crapload? a hit of RAM now for free threads later)
Mutex k64f_acc_mutex;
Mutex k64f_mag_mutex;
Mutex ext_gyro_mutex;
Mutex ext_acc_mutex;
Mutex ext_mag_mutex;
Mutex orient_est_mutex;
Mutex orient_des_mutex;
Mutex motor_thrust_des_mutex;

//data struct global iterations
k64f_acc_data_t k64f_acc_data = {0,0,0};
k64f_mag_data_t k64f_mag_data = {0,0,0};
ext_gyro_data_t ext_gyro_data = {0,0,0};
ext_gyro_temp_t ext_gyro_temp = {0};
ext_acc_data_t ext_acc_data = {0,0,0};
ext_mag_data_t ext_mag_data = {0,0,0};
craft_orientation_est_t craft_orientation_est = {0,0,0};
craft_orientation_des_t craft_orientation_des = {0,0,0};
motor_thrust_des_t motor_thrust_des = {0,0,0,0};

//data struct getters and setters
void get_k64f_acc_data(void* buffer, int size){
    k64f_acc_mutex.lock();
    memcpy(buffer, &k64f_acc_data, size);
    k64f_acc_mutex.unlock();
}
void set_k64f_acc_data(k64f_acc_data_t value){
	k64f_acc_mutex.lock();
	k64f_acc_data = value;
    k64f_acc_mutex.unlock();
}

void get_k64f_mag_data(void* buffer, int size){

    k64f_mag_mutex.lock();
    memcpy(buffer, &k64f_mag_data, size);
    k64f_mag_mutex.unlock();

}
void set_k64f_mag_data(k64f_mag_data_t value){
	k64f_mag_mutex.lock();
	k64f_mag_data = value;
    k64f_mag_mutex.unlock();
}

void get_ext_gyro_data(void* buffer, int size){

    ext_gyro_mutex.lock();
    memcpy(buffer, &ext_gyro_data, size);
    ext_gyro_mutex.unlock();

}
void set_ext_gyro_data(ext_gyro_data_t value){
	ext_gyro_mutex.lock();
	ext_gyro_data = value;
    ext_gyro_mutex.unlock();
}

void get_ext_gyro_temp(void* buffer, int size){
    ext_gyro_mutex.lock();
    memcpy(buffer, &ext_gyro_temp, size);
    ext_gyro_mutex.unlock();
}
void set_ext_gyro_temp(ext_gyro_temp_t value){
	ext_gyro_mutex.lock();
	ext_gyro_temp = value;
    ext_gyro_mutex.unlock();
}

void get_ext_acc_data(void* buffer, int size){

    ext_acc_mutex.lock();
    memcpy(buffer, &ext_acc_data, size);
    ext_acc_mutex.unlock();

}
void set_ext_acc_data(ext_acc_data_t value){
	ext_acc_mutex.lock();
	ext_acc_data = value;
    ext_acc_mutex.unlock();
}

void get_ext_mag_data(void* buffer, int size){

    ext_mag_mutex.lock();
    memcpy(buffer, &ext_mag_data, size);
    ext_mag_mutex.unlock();

}
void set_ext_mag_data(ext_mag_data_t value){
	ext_mag_mutex.lock();
	ext_mag_data = value;
    ext_mag_mutex.unlock();
}

void get_craft_orientation_est(void* buffer, int size){
	orient_est_mutex.lock();
    memcpy(buffer, &craft_orientation_est, size);
    orient_est_mutex.unlock();
}
void set_craft_orientation_est(craft_orientation_est_t value){
	orient_est_mutex.lock();
	craft_orientation_est = value;
	orient_est_mutex.unlock();
}

void get_craft_orientation_des(void* buffer, int size){
	orient_des_mutex.lock();
    memcpy(buffer, &craft_orientation_des, size);
    orient_des_mutex.unlock();
}
void set_craft_orientation_des(craft_orientation_des_t value){
	orient_des_mutex.lock();
	craft_orientation_des = value;
	orient_des_mutex.unlock();
}

void get_motor_thrust_des(void* buffer, int size){
	motor_thrust_des_mutex.lock();
    memcpy(buffer, &motor_thrust_des, size);
    motor_thrust_des_mutex.unlock();
}
void set_motor_thrust_des(motor_thrust_des_t value){
	motor_thrust_des_mutex.lock();
	motor_thrust_des = value;
	motor_thrust_des_mutex.unlock();
}



//macros


