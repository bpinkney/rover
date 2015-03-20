#include "k64f_common.h"

//usb serial out (9600 baud)
//Serial pc(USBTX, USBRX);

//telemetry and general logging
Serial rs(PTC17, PTC16); // tx, rx (baud rate set in thread)

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
Mutex rates_des_mutex;
Mutex accs_est_mutex;
Mutex motor_thrust_des_mutex;

//ir mutxes
Mutex lf_ir_pair_mutex;
Mutex rf_ir_pair_mutex;
Mutex rb_ir_pair_mutex;
Mutex lb_ir_pair_mutex;

//data struct global iterations
k64f_acc_data_t k64f_acc_data = {0,0,0};
k64f_mag_data_t k64f_mag_data = {0,0,0};
ext_gyro_data_t ext_gyro_data = {0,0,0};
ext_gyro_temp_t ext_gyro_temp = {0};
ext_acc_data_t ext_acc_data = {0,0,0};
ext_mag_data_t ext_mag_data = {0,0,0};
craft_orientation_est_t craft_orientation_est = {0,0,0};
craft_orientation_des_t craft_orientation_des = {0,0,0};
craft_rates_t craft_rates_des = {0,0,0};
craft_accs_t craft_accs_est = {0,0,0};
motor_thrust_des_t motor_thrust_des = {0,0,0,0};

//ir sensors
side_vector_t lf_ir_pair = {0,0,0,0};
side_vector_t rf_ir_pair = {0,0,0,0};
side_vector_t rb_ir_pair = {0,0,0,0};
side_vector_t lb_ir_pair = {0,0,0,0};

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

void get_craft_rates_des(void* buffer, int size){
	rates_des_mutex.lock();
    memcpy(buffer, &craft_rates_des, size);
    rates_des_mutex.unlock();
}
void set_craft_rates_des(craft_rates_t value){
	rates_des_mutex.lock();
	craft_rates_des = value;
	rates_des_mutex.unlock();
}

void get_craft_accs_est(void* buffer, int size){
	accs_est_mutex.lock();
    memcpy(buffer, &craft_accs_est, size);
    accs_est_mutex.unlock();
}
void set_craft_accs_est(craft_accs_t value){
	accs_est_mutex.lock();
	craft_accs_est = value;
	accs_est_mutex.unlock();
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


//ir getters and setters
void get_lf_ir_pair(void* buffer, int size){
	lf_ir_pair_mutex.lock();
    memcpy(buffer, &lf_ir_pair, size);
    lf_ir_pair_mutex.unlock();
}
void set_lf_ir_pair(side_vector_t value){
	lf_ir_pair_mutex.lock();
	lf_ir_pair = value;
	lf_ir_pair_mutex.unlock();
}

void get_rf_ir_pair(void* buffer, int size){
	rf_ir_pair_mutex.lock();
    memcpy(buffer, &rf_ir_pair, size);
    rf_ir_pair_mutex.unlock();
}
void set_rf_ir_pair(side_vector_t value){
	rf_ir_pair_mutex.lock();
	rf_ir_pair = value;
	rf_ir_pair_mutex.unlock();
}

void get_rb_ir_pair(void* buffer, int size){
	rb_ir_pair_mutex.lock();
    memcpy(buffer, &rb_ir_pair, size);
    rb_ir_pair_mutex.unlock();
}
void set_rb_ir_pair(side_vector_t value){
	rb_ir_pair_mutex.lock();
	rb_ir_pair = value;
	rb_ir_pair_mutex.unlock();
}

void get_lb_ir_pair(void* buffer, int size){
	lb_ir_pair_mutex.lock();
    memcpy(buffer, &lb_ir_pair, size);
    lb_ir_pair_mutex.unlock();
}
void set_lb_ir_pair(side_vector_t value){
	lb_ir_pair_mutex.lock();
	lb_ir_pair = value;
	lb_ir_pair_mutex.unlock();
}


//macros


