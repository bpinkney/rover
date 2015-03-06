#include "flight_controller.h"

flight_controller_t::flight_controller_t(){
	init();
}

void flight_controller_t::init(){
	pitch_p = 0.45;//35 normally
	pitch_i =  0.01;
	pitch_d =  0.003;

	roll_p = 0.35;
	roll_i =  0.01;
	roll_d =  0.002;

	yaw_p = 0.9;
	yaw_i = 0.00;
	yaw_d = 0;

	pitch_rate_p = 0.04;
	pitch_rate_i = 0.003;
	pitch_rate_d = 0;

	roll_rate_p = 0.025;
	roll_rate_i = 0.001;
	roll_rate_d = 0.0;

	yaw_rate_p = 0.025;
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

	integral_limit = 0.5; // m/s angular rate
	integral_rate_limit = 0.5; //thrust /1

	orient_des = {0,0,0};
	orient_est = {0,0,0};
	curr_motor_thrust = {0,0,0,0};
	egd = {0,0,0};

	timestamp = 0; //used when dt is not constant
	dt_outer = 0.04;
	dt_inner = 0.01;
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
void flight_controller_t::set_test_vars(float a, float b, float c){
	set_craft_orientation_des({b,a,c});//rpy
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
	pitch_rate_des =
		pitch_p*pitch_err +
		pitch_d*(pitch_err - pitch_err_previous)/dt_outer +
		pitch_i*pitch_err_integral;

	roll_rate_des =
		roll_p*roll_err +
		roll_d*(roll_err - roll_err_previous)/dt_outer +
		roll_i*roll_err_integral;

	yaw_rate_des =
		yaw_p*yaw_err +
		yaw_d*(yaw_err - yaw_err_previous)/dt_outer +
		yaw_i*yaw_err_integral;

	set_craft_rates_des({roll_rate_des, pitch_rate_des, yaw_rate_des});

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
	pitch_rate_err = pitch_rate_des - egd.x; // m/s // orientation diffs between accels(craft orient) and gyros are accounted for in sensor read
	roll_rate_err = roll_rate_des - egd.y; // m/s
	yaw_rate_err = yaw_rate_des - egd.z; //verification required // z gyro = -z craft

	//feb.27
	//pitch_rate_err = -egd.x + pitch_rate_des; // m/s // x gyro = -x craft
	//	roll_rate_err = -egd.y - roll_rate_des; // m/s (negative due to stuff and such)
	//	yaw_rate_err = -egd.z + yaw_rate_des; //verification required // z gyro = -z craft

	//pc.printf("Desired Angular Rate Change [RP] [%f, %f]     Gyro Rates [YXZ] [%f, %f, %f]\n\r", roll_rate_des, pitch_rate_des, egd.y, egd.x, egd.z);

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

	//feb27
	/*set_motor_thrust_des({
			base_thrust*(1 + pitch_thrust_delta + roll_thrust_delta + yaw_thrust_delta),//FL
			base_thrust*(1 + pitch_thrust_delta - roll_thrust_delta - yaw_thrust_delta),//FR
			base_thrust*(1 - pitch_thrust_delta - roll_thrust_delta + yaw_thrust_delta),//RR
			base_thrust*(1 - pitch_thrust_delta + roll_thrust_delta - yaw_thrust_delta)//RL
		});*/

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
	if(outer_loop_activate_count > 2){//every 1/4 inner loops run the outer loop
		outer_loop_activate_count = 0;
	}else{
		outer_loop_activate_count++;
	}
}






