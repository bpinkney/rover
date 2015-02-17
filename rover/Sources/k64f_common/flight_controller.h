#include "mbed.h"
#include "rtos.h"
#include "k64f_common.h"

#ifndef FLIGHT_CONTROLLER_H_
#define FLIGHT_CONTROLLER_H_

class flight_controller_t {

private:
	//flight_controller_t() {} (need if overloading public constructor)
	float pitch_p;// if pitch_p = 6, a 0.5 radian offset will attempt to be corrected with a 3 rad/s movement speed
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

	float pitch_err, roll_err, yaw_err;
	float pitch_rate_err, roll_rate_err, yaw_rate_err;
	float pitch_rate_delta, roll_rate_delta, yaw_rate_delta;
	float pitch_thrust_delta, roll_thrust_delta, yaw_thrust_delta;

	uint32_t timestamp;
	float dt_inner;
	float dt_outer;
	float base_thrust;

	uint8_t outer_loop_activate_count;

	//private methods


public:
	flight_controller_t();
	void init();
	void update_roll_pids(float p, float i, float d);
	void update_pitch_pids(float p, float i, float d);
	void update_yaw_pids(float p, float i, float d);
	void update_roll_rate_pids(float p, float i, float d);
	void update_pitch_rate_pids(float p, float i, float d);
	void update_yaw_rate_pids(float p, float i, float d);
	void set_base_thrust(float thrust);

	void run_outer_control_loop(); //orientation to rate
	void run_inner_control_loop(); //rate to thrusts



};

#endif

