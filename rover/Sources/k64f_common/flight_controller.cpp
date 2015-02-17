#include "flight_controller.h"

flight_controller_t::flight_controller_t(){
	init();
}

void flight_controller_t::init(){
	pitch_p = 0.00;
	pitch_i = 0.0;
	pitch_d = 0;

	roll_p = 0.00;
	roll_i = 0.0;
	roll_d = 0;

	yaw_p = 0;
	yaw_i = 0;
	yaw_d = 0;

	pitch_rate_p = 0.0;
	pitch_rate_i = 0;
	pitch_rate_d = 0;

	roll_rate_p = 0.0;
	roll_rate_i = 0;
	roll_rate_d = 0;

	yaw_rate_p = 0;
	yaw_rate_i = 0;
	yaw_rate_d = 0;

	pitch_err_previous = 0;
	roll_err_previous = 0;
	yaw_err_previous = 0;

	pitch_rate_err_previous = 0;
	roll_rate_err_previous = 0;
	yaw_rate_err_previous = 0;

	pitch_err_integral = 0;
	roll_err_integral = 0;
	yaw_err_integral = 0;

	pitch_rate_err_integral = 0;
	roll_rate_err_integral = 0;
	yaw_rate_err_integral = 0;

	integral_limit = 0.05; // m/s angular rate
	integral_rate_limit = 0.05; //thurst /1

	orient_des = {0,0,0};
	orient_est = {0,0,0};
	curr_motor_thrust = {0,0,0,0};
	egd = {0,0,0};

	timestamp = 0; //used when dt is not constant
	dt_outer = 100;
	dt_inner = 10;
	base_thrust = 0;

	outer_loop_activate_count = 0;
}

void flight_controller_t::update_roll_pids(float p, float i, float d){
	if(p != -1){
		roll_p = p;
	}
	if(i != -1){
		roll_i = i;
	}
	if(d != -1){
		roll_d = d;
	}
}
void flight_controller_t::update_pitch_pids(float p, float i, float d){
	if(p != -1){
		pitch_p = p;
	}
	if(i != -1){
		pitch_i = i;
	}
	if(d != -1){
		pitch_d = d;
	}
}
void flight_controller_t::update_yaw_pids(float p, float i, float d){
	if(p != -1){
		yaw_p = p;
	}
	if(i != -1){
		yaw_i = i;
	}
	if(d != -1){
		yaw_d = d;
	}
}
void flight_controller_t::update_roll_rate_pids(float p, float i, float d){
	if(p != -1){
		roll_rate_p = p;
	}
	if(i != -1){
		roll_rate_i = i;
	}
	if(d != -1){
		roll_rate_d = d;
	}
}
void flight_controller_t::update_pitch_rate_pids(float p, float i, float d){
	if(p != -1){
		pitch_rate_p = p;
	}
	if(i != -1){
		pitch_rate_i = i;
	}
	if(d != -1){
		pitch_rate_d = d;
	}
}
void flight_controller_t::update_yaw_rate_pids(float p, float i, float d){
	if(p != -1){
		yaw_rate_p = p;
	}
	if(i != -1){
		yaw_rate_i = i;
	}
	if(d != -1){
		yaw_rate_d = d;
	}
}

void flight_controller_t::set_base_thrust(float thrust){
	base_thrust = thrust;
}

void flight_controller_t::run_outer_control_loop(){

	//get current desired and estimate orientations
	get_craft_orientation_des(&orient_des, sizeof(orient_des));
	get_craft_orientation_est(&orient_est, sizeof(orient_est));

	pitch_err = (orient_des.pitch - orient_est.pitch);
	roll_err = orient_des.roll - orient_est.roll;
	yaw_err = orient_des.yaw - orient_est.yaw;

	//account for yaw crossover
	WRAP_2PI(yaw_err);

	//for 'I' term
	pitch_err_integral = min(max(pitch_err_integral + pitch_err*dt_outer, -integral_limit), integral_limit);
	roll_err_integral = min(max(roll_err_integral + roll_err*dt_outer, -integral_limit), integral_limit);
	yaw_err_integral = min(max(yaw_err_integral + yaw_err*dt_outer, -integral_limit), integral_limit);

	//PIDs
	pitch_rate_delta =
		pitch_p*pitch_err +
		pitch_d*(pitch_err - pitch_err_previous)/dt_outer +
		pitch_i*pitch_err_integral;

	roll_rate_delta =
		roll_p*roll_err +
		roll_d*(roll_err - roll_err_previous)/dt_outer +
		roll_i*roll_err_integral;

	yaw_rate_delta =
		yaw_p*yaw_err +
		yaw_d*(yaw_err - yaw_err_previous)/dt_outer +
		yaw_i*yaw_err_integral;

	//for i+1 'D' term
	pitch_err_previous = pitch_err;
	roll_err_previous = roll_err;
	yaw_err_previous = yaw_err;

}

void flight_controller_t::run_inner_control_loop(){

	if(outer_loop_activate_count == 0){
		run_outer_control_loop();
	}

	//get current angular velocity from gyros
	get_ext_gyro_data(&egd, sizeof(egd));

	//determine rate errors
	pitch_rate_err = egd.x - pitch_rate_delta; // m/s // x gyro = -x craft
	roll_rate_err = -egd.y - roll_rate_delta; // m/s (negative due to stuff and such)
	yaw_rate_err = egd.z - yaw_rate_delta; //verification required // z gyro = -z craft

	//for 'I' term
	pitch_rate_err_integral = min(max(pitch_rate_err_integral + pitch_rate_err*dt_inner, -integral_rate_limit), integral_rate_limit);
	roll_rate_err_integral = min(max(roll_rate_err_integral + roll_rate_err*dt_inner, -integral_rate_limit), integral_rate_limit);
	yaw_rate_err_integral = min(max(yaw_rate_err_integral + yaw_rate_err*dt_inner, -integral_rate_limit), integral_rate_limit);

	//PIDs
	pitch_thrust_delta =
		pitch_rate_p*pitch_rate_err +
		pitch_rate_d*(pitch_rate_err - pitch_rate_err_previous)/dt_inner +
		pitch_rate_i*pitch_rate_err_integral;

	roll_thrust_delta =
		roll_rate_p*roll_rate_err +
		roll_rate_d*(roll_rate_err - roll_rate_err_previous)/dt_inner +
		roll_rate_i*roll_rate_err_integral;

	yaw_thrust_delta =
		yaw_rate_p*yaw_rate_err +
		yaw_rate_d*(yaw_rate_err - yaw_rate_err_previous)/dt_inner +
		yaw_rate_i*yaw_rate_err_integral;

	set_motor_thrust_des({
		base_thrust + pitch_thrust_delta + roll_thrust_delta + yaw_thrust_delta,//FL
		base_thrust + pitch_thrust_delta - roll_thrust_delta - yaw_thrust_delta,//FR
		base_thrust - pitch_thrust_delta - roll_thrust_delta + yaw_thrust_delta,//RR
		base_thrust - pitch_thrust_delta + roll_thrust_delta - yaw_thrust_delta//RL
	});

	//for i+1 'D' term
	pitch_rate_err_previous = pitch_rate_err;
	roll_rate_err_previous = roll_rate_err;
	yaw_rate_err_previous = yaw_rate_err;

	//increment outer loop flag
	if(outer_loop_activate_count > 8){//every 1/10 inner loops run the outer loop
		outer_loop_activate_count = 0;
	}else{
		outer_loop_activate_count++;
	}
}






