#include "mbed.h"
#include "rtos.h"
#include "k64f_common.h"

#ifndef ORIENTATION_ESTIMATOR_H_
#define ORIENTATION_ESTIMATOR_H_

class orientation_estimator_t {

private:
	//orientation_estimator_t() {} (need if overloading public constructor)

	//estimator trusts
	float gyro_trust;
	float acc_trust;

	float pitch, roll, yaw; //(x, y, z), check the flyer and imu axes if you need to verify
	float pitch_rot, roll_rot, yaw_rot; //rotations away from base frame if required

	float dt; //make sure to change this period if the calling frequency changes (could probably link it up)

	ext_gyro_data_t egd;
	ext_gyro_data_t last_egd;
	k64f_acc_data_t iad;
	k64f_mag_data_t imd;
	craft_orientation_est_t o_orient;
	craft_accs_t acc_est;
	craft_accs_t last_acc_est;

	//private methods


public:
	uint16_t imu_read_period;

	orientation_estimator_t();
	void init();
	void run_estimate_loop();

};

#endif

