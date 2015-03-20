#include "mbed.h"
#include "rtos.h"
#include "k64f_common.h"

#ifndef FLIGHT_CONTROLLER_H_
#define FLIGHT_CONTROLLER_H_

class flight_controller_t {

private:
	//flight_controller_t() {} (need if overloading public constructor)
	/*float pitch_p;// if pitch_p = 6, a 0.5 radian offset will attempt to be corrected with a 3 rad/s movement speed
	float pitch_i;
	float pitch_d;

	float roll_p;
	float roll_i;
	float roll_d;

	float yaw_p;
	float yaw_i;
	float yaw_d;

	float pitch_rate_p;
	float pitch_rate_i;
	float pitch_rate_d;

	float roll_rate_p;
	float roll_rate_i;
	float roll_rate_d;

	float yaw_rate_p;
	float yaw_rate_i;
	float yaw_rate_d;

	float pitch_err_previous;
	float roll_err_previous;
	float yaw_err_previous;

	float pitch_rate_err_previous;
	float roll_rate_err_previous;
	float yaw_rate_err_previous;

	float pitch_err_integral;
	float roll_err_integral;
	float yaw_err_integral;

	float pitch_rate_err_integral;
	float roll_rate_err_integral;
	float yaw_rate_err_integral;

	float integral_limit;
	float integral_rate_limit;

	craft_orientation_des_t orient_des;
	craft_orientation_est_t orient_est;
	motor_thrust_des_t curr_motor_thrust;
	ext_gyro_data_t egd;
	ext_gyro_data_t last_egd;

	float pitch_err, roll_err, yaw_err;
	float pitch_rate_err, roll_rate_err, yaw_rate_err;
	float pitch_rate_des, roll_rate_des, yaw_rate_des;
	float pitch_thrust_delta, roll_thrust_delta, yaw_thrust_delta;

	uint32_t timestamp;
	float dt_inner;
	float dt_outer;
	float base_thrust;

	uint8_t outer_loop_activate_count;*/

	//pddd new stuff
	float pitch_p;
	float roll_p;
	float yaw_p;

	float roll_i;
	float pitch_i;
	float yaw_i;

	float pitch_d;
	float roll_d;
	float yaw_d;

	float pitch_dd;
	float roll_dd;
	float yaw_dd;

	float roll_integral;
	float pitch_integral;
	float yaw_integral;

	float pitch_pos_err, roll_pos_err, yaw_pos_err;
	float pitch_vel_err, roll_vel_err, yaw_vel_err;
	float pitch_acc_err, roll_acc_err, yaw_acc_err;
	float pitch_thrust_delta, roll_thrust_delta, yaw_thrust_delta;

	float dt;
	float base_thrust;
	float min_thrust;
	float max_thrust;
	float front_thrust, right_thrust, rear_thrust, left_thrust;

	float max_vel_err;
	float max_pos_err;

	//position
	craft_orientation_des_t orient_des;
	craft_orientation_des_t last_orient_des;
	craft_orientation_est_t orient_est;
	craft_orientation_est_t last_orient_est;

	motor_thrust_des_t curr_motor_thrust;

	//speed
	craft_rates_t des_rates;
	//craft_rates_t last_des_rates; (if using non-zero des_acc)
	ext_gyro_data_t egd;
	ext_gyro_data_t last_egd;

	//acceleration
	craft_accs_t des_ang_acc;
	craft_accs_t est_ang_acc;

	//private methods


public:
	flight_controller_t();
	void init();
	void update_pitch_pddd(float p, float d, float dd);
	void update_roll_pddd(float p, float d, float dd);
	void update_yaw_pddd(float p, float d, float dd);

	void update_yaw_i(float i);
	/*void update_roll_pids(float p, float i, float d);
	void update_pitch_pids(float p, float i, float d);
	void update_yaw_pids(float p, float i, float d);
	void update_roll_rate_pids(float p, float i, float d);
	void update_pitch_rate_pids(float p, float i, float d);
	void update_yaw_rate_pids(float p, float i, float d);*/
	void set_base_thrust(float thrust);

	void set_test_vars(float a, float b, float c);

	void run_control_loop(); //p-d-dd loop for rate and angle combined

	/*void run_outer_control_loop(); //orientation to rate
	void run_inner_control_loop(); //rate to thrusts*/



};

#endif

