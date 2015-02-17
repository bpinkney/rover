
#include "mbed.h"
#include "rtos.h"
#include "esc.h"

#include <time.h>

//k64f custom classes/resources
#include "k64f_common.h"
//#include "k64f_io.h"
#include "k64f_acc_mag.h"
#include "flight_controller.h"
#include "orientation_estimator.h"
#include "IMU.h"

//define thread pointers
Thread *core_sensors_thread;
Thread *estimator_thread;
Thread *motor_control_thread;
Thread *flight_control_thread;
Thread *nav_sensor_thread;
Thread *remote_control_thread;
Thread *position_control_thread;
Thread *flight_logger_thread;
Thread *remote_logger_thread;

//imu access wrappers
k64f_sensor_interface int_imu;
IMU ext_imu(PTE25,PTE24);

//sd communications wrapper
//k64f_io_t k64f_io;

SDFileSystem sd(PTE3, PTE1, PTE2, PTE4, "sdcard");
//logfile filedescriptor

//orient estimator
orientation_estimator_t orientation_estimator;

//flight_controller
flight_controller_t flight_controller;

//ofstream log_fp;

//uptime
const uint16_t imu_read_period = 10;
const uint16_t estimator_period = 10;
const uint16_t flight_control_inner_period = 10;
const uint16_t flight_control_outer_period = 100;
uint32_t sw_uptime = 0;

uint8_t status = 0;

//temp
float ext_throttle = 0.0;
int8_t ext_yaw = 0;
int8_t ext_pitch = 0;
int8_t ext_roll = 0;

float ext_rp = 0.00;
float ext_pp = 0.00;

//buffers
//char log_buffer[1024];

//PwmOut out(PTA0);
//PwmOut led(LED3);

//generic tool fuctions

void get_formatted_date_time(char * buffer){
	std::time_t result = std::time(nullptr);
	sprintf(buffer, std::asctime(std::localtime(&result)));
}

void blinkLED1(void const *args) {
	DigitalOut led1(LED1);
	while(true){
		led1 = !led1;
		Thread::wait(300);
	}
}

void blinkLED2(void const *args) {
	DigitalOut led2(LED2);
	while(true){
		led2 = !led2;
		Thread::wait(500);
	}
}

void blinkLED3(void const *args) {
	DigitalOut led3(LED3);
	while(true){
		led3 = !led3;
		Thread::wait(800);
	}
}

void show_status_led_thread(void const *args) {
	uint8_t step_count = 0; //8 steps
	DigitalOut led1(LED1);//red
	DigitalOut led2(LED2);//green
	DigitalOut led3(LED3);//blue

	uint8_t current_status = 99;
	//LEDs guide:
	// 1&2&3 light blue w/red dot...
	// 1&2: yellowy green
	// 1&3: purply
	// 2&3 light blue

	//status is currently according to following enums:
	//	0 - Boot/Init Phase					-> blue slow flash
	//	1 - Armed Status (flight-ready) 	-> green triple flash
	//	2 - Takeoff Commencing				-> green triple flash w/ backlight
	//	33 - Error State					-> red fast flash

	while(true){
		if (current_status != status && current_status != 33){
			//change this later, but for now error state cannot be undone, LED-wise at least
			led1 = 1;
			led2 = 1;
			led3 = 1;
			step_count = 0;
			current_status = status;
		}

		if(current_status == 0){
			if(step_count < 4){
				led3 = 0;
			}else{
				led3 = 1;
			}
		}else if(current_status == 1){
			if(step_count > 5){
				led2 = 1;
			}else{
				led2 = !led2;
			}
		}else if(current_status == 2){
			led3 = 0;
			if(step_count > 5){
				led2 = 1;
			}else{
				led2 = !led2;
			}
		}
		else if(current_status == 33){
			led1 = !led1;
			led2 = !led1;
		}

		if (step_count >= 7){
			step_count = 0;
		}else{
			step_count++;
		}

		Thread::wait(200);
	}
}

//RTOS timer class access functions
void tick_clock(void const *n){
	sw_uptime++;
}

void fetch_sensor_data(void const *n){
	sw_uptime+=imu_read_period;
	int_imu.fetch_sensor_data();
	ext_imu.readGyros();
	ext_imu.readGyroTemp();
	ext_imu.readAccs();
	ext_imu.readMags();
}

//thread run functions (see thread declarations for descriptions)
void run_core_sensors_thread(void const *args){
	char ext_sensor_ack;

	pc.printf("Start Sensors...\n");
	int_imu.init();
	ext_sensor_ack = ext_imu.init();
	pc.printf("\r\n\nIMUS: External IMU Init Ack = %d\r\n", ext_sensor_ack);
	RtosTimer imu_timer(fetch_sensor_data, osTimerPeriodic, (void *)0);
	//RtosTimer print_timer(print_sensor_data, osTimerPeriodic, (void *)1);

	if(ext_sensor_ack == 0){
		imu_timer.start(imu_read_period);
	}else{
		status = 33;
		pc.printf("External IMU could not be started!!\n\r");
	}
	//print_timer.start(250);

	//uint32_t count = 0;
	while(true){
		Thread::wait(osWaitForever);
	}
	pc.printf("Thread Ended Prematurely!");
}

void update_estimator(void const *n){

	orientation_estimator.run_estimate_loop();

	/*
	 * gyro_est(i, 1) = (gyro_x(i, 1))*time_d(i)/1000;
  gyro_est(i, 2) = (gyro_y(i, 1))*time_d(i)/1000;

  acc_est(i, 1) = atan2(acc_x(i, 1),sqrt(acc_y(i, 1)^2 + acc_z(i, 1)^2));
  acc_est(i, 2) = atan2(acc_y(i, 1),sqrt(acc_x(i, 1)^2 + acc_z(i, 1)^2));

  overall_est(i, :) = gyro_trust*(overall_est(i-1, :) + gyro_est(i, :)) + acc_est(i, :)*acc_trust;



  and for mag
  Rx = [1 0 0; 0 cos(pitch) sin(pitch); 0 -sin(pitch) -cos(pitch)];
Ry = [cos(roll) 0 -sin(roll); 0 1 0; sin(roll) 0 cos(roll)];
corrected_mag = Rx*Ry*flight_log_data.rover_int_mag(i, :)';
yaw_est(i,1) = atan2(corrected_mag(2,1),corrected_mag(1,1));

results in:
x =    magx*cos(rol) - magz*sin(rol)
y =    magy*cos(pit) + magz*cos(rol)*sin(pit) + magx*sin(pit)*sin(rol)
z =  - magy*sin(pit) - magz*cos(pit)*cos(rol) - magx*cos(pit)*sin(rol)

yaw = atan2(y,x)

	 */
	/*float pitch, roll, yaw; //(x, y, z), check the flyer and imu axes if you need to verify

	ext_gyro_data_t egd = {0,0,0};
	k64f_acc_data_t iad = {0,0,0};
	k64f_mag_data_t imd = {0,0,0};
	craft_orientation_est_t o_orient = {0,0,0};
	//craft_orientation_est_t n_orient = {0,0,0};

	get_ext_gyro_data(&egd, sizeof(egd));
	get_k64f_acc_data(&iad, sizeof(iad));
	get_k64f_mag_data(&imd, sizeof(imd));
	get_craft_orientation_est(&o_orient, sizeof(o_orient));

	roll = gyro_trust*(o_orient.roll + (egd.x*((float)imu_read_period)/1000)) + acc_trust*(float)atan2(iad.x,sqrt(pow(iad.y,2) + pow(iad.z,2)));
	pitch = gyro_trust*(o_orient.pitch + (egd.y*((float)imu_read_period)/1000)) + acc_trust*(float)atan2(iad.y,sqrt(pow(iad.x,2) + pow(iad.z,2)));

	yaw = atan2(imd.z*sin(pitch) - cos(pitch)*imd.y, cos(roll)*imd.x - imd.y*sin(pitch)*sin(roll) - cos(pitch)*imd.z*sin(roll));
			//atan2(cos(pitch)*imd.y - imd.x*sin(pitch)*sin(roll) + cos(roll)*imd.z*sin(pitch), -imd.z*sin(roll) + cos(roll)*imd.x);
	//atan2(imd.z*sin(pitch) - cos(pitch)*imd.y, cos(roll)*imd.x - imd.y*sin(pitch)*sin(roll) - cos(pitch)*imd.z*sin(roll))

	//atan2(cos(pitch)*conj(y) - conj(x)*sin(pitch)*sin(roll) + cos(roll)*conj(z)*sin(pitch), cos(roll)*conj(x) - conj(z)*sin(roll))
	//old busted method (wrong axes) //= atan2((imd.y*cos(roll) + imd.z*cos(pitch)*sin(roll) + imd.x*sin(roll)*sin(pitch)),(imd.x*cos(pitch) - imd.x*sin(roll)));
	//atan2(-cos(pitch)*imd.y - imd.x*sin(pitch)*sin(roll) - cos(roll)*imd.z*sin(pitch), imd.z*sin(roll) - cos(roll)*imd.x)
	set_craft_orientation_est({roll, pitch, yaw}); //radians*/

	//pc.printf("Orient Est [RPY]: %f, %f, %f\n\r", roll, pitch, yaw);
}

void run_estimator_thread(void const *args){

	orientation_estimator.init();
	orientation_estimator.imu_read_period = imu_read_period;

	RtosTimer estimator_timer(update_estimator, osTimerPeriodic, (void *)0);

	pc.printf("Starting Estimator\n\r");
	estimator_timer.start(estimator_period);

	while(true){Thread::wait(osWaitForever);}
}

void run_motor_control_thread(void const *args){

	pc.printf("MOTORS: initilaize ESCs\n\r");

	uint8_t calibrate_motor_range = 1;
	ESC m1(PTA1);//FL
	ESC m2(PTA2);//FR
	ESC m3(PTC2);//RR
	ESC m4(PTC3);//RL

	uint8_t m1_on = 1;
	uint8_t m2_on = 1;
	uint8_t m3_on = 1;
	uint8_t m4_on = 1;

	float m1_throttle = 0.0;
	float m2_throttle = 0.0;
	float m3_throttle = 0.0;
	float m4_throttle = 0.0;

	motor_thrust_des_t des_motor_thrust = {0,0,0,0};

	pc.printf("ESC intialized\n\r");
	//pc.printf("set init throttle to : %f\n\r", throttle_var);
	float init_time = 0;

	while(1)
		{
		if(init_time < 5000){//wait for init of ESCs
			m1_throttle = 1;
			m2_throttle = 1;
			m3_throttle = 1;
			m4_throttle = 1;
			//pc.printf("set throttle to : %f\n\r", throttle_var);
		}else if(init_time < 10000){
			m1_throttle = 0;
			m2_throttle = 0;
			m3_throttle = 0;
			m4_throttle = 0;
			//pc.printf("set throttle to : %f\n\r", throttle_var);
		}else{

			//these are all correct relative to the controller
			/*m1_throttle = ext_throttle - 0.05*ext_yaw + 0.02*ext_pitch - 0.02*ext_roll;	//FL
			m2_throttle = ext_throttle + 0.05*ext_yaw + 0.02*ext_pitch + 0.02*ext_roll; //FR
			m3_throttle = ext_throttle - 0.05*ext_yaw - 0.02*ext_pitch + 0.02*ext_roll; //RR
			m4_throttle = ext_throttle + 0.05*ext_yaw - 0.02*ext_pitch - 0.02*ext_roll; //RL*/

			get_motor_thrust_des(&des_motor_thrust, sizeof(des_motor_thrust));
			if(ext_throttle>0){
				m1_throttle = des_motor_thrust.fl;
				m2_throttle = des_motor_thrust.fr;
				m3_throttle = des_motor_thrust.rr;
				m4_throttle = des_motor_thrust.rl;
			}else{
				m1_throttle = 0;
				m2_throttle = 0;
				m3_throttle = 0;
				m4_throttle = 0;
			}

		}

			//... update throttle_var ...
			//memorize the throttle value (it doesn't send it to the ESC).
			/*if(m1_on){m1 = m1_throttle;}else{m1 = 0;}
			if(m2_on){m2 = m2_throttle;}else{m2 = 0;}
			if(m3_on){m3 = m3_throttle;}else{m3 = 0;}
			if(m4_on){m4 = m4_throttle;}else{m4 = 0;}*/

			//... do whatever you want - e.g. call esc1.setThrottle(throttle_var) again ...

			m1 = m1_throttle;
			m2 = m2_throttle;
			m3 = m3_throttle;
			m4 = m4_throttle;

			m1(); //actually sets the throttle to the ESC.
			m2();
			m3();
			m4();

			Thread::wait(20);  //20ms is the default period of the ESC pwm; the ESC may not run faster.

			init_time+=20;

		}


	while(true){Thread::wait(osWaitForever);}
}

void flight_control_inner_loop(void const *n){
	flight_controller.run_inner_control_loop();
}

void run_flight_control_thread(void const *args){
	flight_controller.init();

	RtosTimer flight_control_timer(flight_control_inner_loop, osTimerPeriodic, (void *)0);

	pc.printf("Starting Flight Controller\n\r");
	flight_control_timer.start(flight_control_inner_period);


	/*float pitch_p = 0.05;
	float pitch_i = 0.001;
	float pitch_d = 0;

	float roll_p = 0.05;
	float roll_i = 0.001;
	float roll_d = 0;

	float yaw_p = 0;
	float yaw_i = 0;
	float yaw_d = 0;

	float pitch_err_previous = 0;
	float roll_err_previous = 0;
	float yaw_err_previous = 0;

	float pitch_err_integral = 0;
	float roll_err_integral = 0;
	float yaw_err_integral = 0;

	craft_orientation_des_t orient_des = {0,0,0};
	craft_orientation_est_t orient_est = {0,0,0};
	motor_thrust_des_t curr_motor_thrust = {0,0,0,0};
	float pitch_err, roll_err;
	float pitch_thrust_delta, roll_thrust_delta;

	uint32_t timestamp = sw_uptime-10;
	float dt = 10;
	//temp
	float base_thrust = 0;*/

	//while(true){
		//loop

		/*pitch_p = ext_pp;
		roll_p = ext_rp;//ext_rp;

		get_craft_orientation_des(&orient_des, sizeof(orient_des));
		get_craft_orientation_est(&orient_est, sizeof(orient_est));
		//get_motor_thrust_des(&curr_motor_thrust, sizeof(curr_motor_thrust));

		dt = (float)(sw_uptime-timestamp);//ms
		if(dt!=0){
			timestamp = sw_uptime;

			pitch_err = orient_des.pitch - orient_est.pitch;
			roll_err = orient_des.roll - orient_est.roll;
			//yaw_err = orient_des.yaw - orient_est.yaw; //account for crossover required

			//for 'I' term
			pitch_err_integral = pitch_err_integral + pitch_err*dt;
			roll_err_integral = roll_err_integral + roll_err*dt;
			//yaw_err_integral = yaw_err_integral + yaw_err*dt;

			//PIDs
			pitch_thrust_delta =
			pitch_p*pitch_err +
			pitch_d*(pitch_err - pitch_err_previous)/dt +
			pitch_i*pitch_err_integral;

			roll_thrust_delta =
			roll_p*roll_err +
			roll_d*(roll_err - roll_err_previous)/dt +
			roll_i*roll_err_integral;

			yaw_thrust_delta =
			yaw_p*yaw_err +
			yaw_d*(yaw_err - yaw_err_previous)/dt +
			yaw_i*yaw_err_integral;


			set_motor_thrust_des({
				ext_throttle + pitch_thrust_delta - roll_thrust_delta,// + yaw_thrust_delta,//FL
				ext_throttle + pitch_thrust_delta + roll_thrust_delta,// - yaw_thrust_delta,//FR
				ext_throttle - pitch_thrust_delta + roll_thrust_delta,// + yaw_thrust_delta,//RR
				ext_throttle - pitch_thrust_delta - roll_thrust_delta,// - yaw_thrust_delta//RL
			});

			//for i+1 'D' term
			pitch_err_previous = pitch_err;
			roll_err_previous = roll_err;
		}*/
		//yaw_err_previous = yaw_err;





		/*ext_throttle + pitch_thrust_delta - roll_thrust_delta,
					ext_throttle + pitch_thrust_delta + roll_thrust_delta,
					ext_throttle - pitch_thrust_delta + roll_thrust_delta,
					ext_throttle - pitch_thrust_delta - roll_thrust_delta*/

		/*m1_throttle = ext_throttle - 0.05*ext_yaw + 0.02*ext_pitch - 0.02*ext_roll;	//FL
		m2_throttle = ext_throttle + 0.05*ext_yaw + 0.02*ext_pitch + 0.02*ext_roll; //FR
		m3_throttle = ext_throttle - 0.05*ext_yaw - 0.02*ext_pitch + 0.02*ext_roll; //RR
		m4_throttle = ext_throttle + 0.05*ext_yaw - 0.02*ext_pitch - 0.02*ext_roll; //RL*/

		//pc.printf("FLIGHT_CONTROL: Pitch thrust delta: %f, Roll thrust delta: %f\n\r", pitch_thrust_delta, roll_thrust_delta);

		//Thread::wait(osWaitForever);
	//}







	while(true){Thread::wait(osWaitForever);}
}

void run_nav_sensor_thread(void const *args){
	while(true){Thread::wait(osWaitForever);}
}

void run_remote_control_thread(void const *args){
	pc.printf("RADIO: Start Radio Serial\r\n");
	//telemetry serial in/out (57000 baud, set in remote thread)
	Serial pc_rad(PTC17, PTC16); // tx, rx
	pc_rad.baud(57000); //only baud rate accepted by radios
	pc_rad.printf("Welcome to ROVER Remote Logging and Control\n\r");
	pc.printf("RADIO: Polling for remote chars...\r\n");
	uint8_t c;

	//snes controller config
	//dpad: wasd
	//buttons: tfgh
	//LR triggers: zx
	//Select Start: kl
	while(1) {
		if(pc_rad.readable()) {
			c = pc_rad.getc();
			pc.printf("Radio chars recieved: '%d'\n\r", c);
			if(c == 'w' && ext_throttle < 0.25){
				ext_throttle = 0.25;
				flight_controller.set_base_thrust(ext_throttle);
				pc.printf("Throttle start to: '%f' percent.\n\r", ext_throttle);
			}else if(c == 'w' && ext_throttle <= 0.9){
				ext_throttle = ext_throttle + 0.05;
				flight_controller.set_base_thrust(ext_throttle);
				pc.printf("Throttle set to: '%f' percent.\n\r", ext_throttle);
			}
			else if(c == 's' && ext_throttle >= 0.3){
				ext_throttle = ext_throttle - 0.05;
				flight_controller.set_base_thrust(ext_throttle);
				pc.printf("Throttle set to: '%f' percent.\n\r", ext_throttle);
			}else if(c == 's' && ext_throttle >= 0.1){
				ext_throttle = 0;
				flight_controller.set_base_thrust(ext_throttle);
				pc.printf("Throttle Off.\n\r");
			}else if(c == 'x'){
				ext_yaw = 0;
				ext_pitch = 0;
				ext_roll = 0;
				pc.printf("Yaw pitch roll reset\n\r", ext_throttle);
			}else if(c == 'f'){
				ext_rp -= 0.001;
				flight_controller.update_roll_rate_pids(ext_rp, -1, -1);
				flight_controller.update_pitch_rate_pids(ext_rp, -1, -1);
				pc.printf("PitcHRate/RollRate P is now: %f\n\r", ext_rp);
				//ext_yaw = -1;
				//pc.printf("Yaw left.\n\r");
			}
			else if(c == 'h'){
				ext_rp += 0.001;
				flight_controller.update_pitch_rate_pids(ext_rp, -1, -1);
				flight_controller.update_roll_rate_pids(ext_rp, -1, -1);
				pc.printf("PitchRate/RollRate P is now: %f\n\r", ext_rp);
				//ext_yaw = 1;
				//pc.printf("Yaw right.\n\r");
			}
			else if(c == 't'){
				ext_pp += 0.005;
				flight_controller.update_pitch_pids(ext_pp, -1, -1);
				flight_controller.update_roll_pids(ext_pp, -1, -1);
				pc.printf("Pitch/Roll P is now: %f\n\r", ext_pp);
				//ext_pitch = 1;
				//c.printf("Pitch forward.\n\r");
			}
			else if(c == 'g'){
				ext_pp -= 0.005;
				flight_controller.update_pitch_pids(ext_pp, -1, -1);
				flight_controller.update_roll_pids(ext_pp, -1, -1);
				pc.printf("Pitch/Roll P is now: %f\n\r", ext_pp);
				//ext_pitch = -1;
				//pc.printf("Pitch backward.\n\r");
			}
			else if(c == 'a'){
				ext_roll = 1;
				pc.printf("Roll left.\n\r");
			}
			else if(c == 'd'){
				ext_roll = -1;
				pc.printf("Roll right.\n\r");
			}
			else if(c == 'k'){
				ext_throttle = 0;
				ext_yaw = 0;
				ext_pitch = 0;
				ext_roll = 0;
				pc.printf("Throttle Killed\n\r");
			}
		}else{
			Thread::wait(200);
		}
	}

	while(true){Thread::wait(osWaitForever);}
}

void run_position_control_thread(void const *args){
	while(true){Thread::wait(osWaitForever);}
}

void run_flight_logger_thread(void const *args){

	//TODO: move this all to an RTOS timer handler so that logging is actually equidistant (for now printing timestamps only is fine)

	//LOGGING (fprintf with actual variables for example) needs a larger stack size, check the thread instantiation for more info

	//logging data structures (methods are all in main due to sdfilesystem pointer issues, TODO resolve and wrap in class?)
	FILE *log_fp;
	char log_buffer[1024];
	char filename[100];
	char date_stamp[100];

	uint8_t log_number = 1;
	uint8_t log_chosen = 0;

	ext_mag_data_t emd = {0,0,0};
	ext_acc_data_t ead = {0,0,0};
	ext_gyro_data_t egd = {0,0,0};
	ext_gyro_temp_t egt = {0};
	k64f_acc_data_t iad = {0,0,0};
	k64f_mag_data_t imd = {0,0,0};
	craft_orientation_est_t orient_est = {0,0,0};
	craft_orientation_des_t orient_des = {0,0,0};
	motor_thrust_des_t motor_thrust = {0,0,0,0};

	pc.printf("Flight Log Starting...\n\r");
	wait(2);

	while(!log_chosen){
		sprintf(filename, "/sdcard/flight_log_%d.txt", log_number);
		log_fp = fopen(filename, "r");
		if (log_fp != NULL) {
			fclose(log_fp);
			log_number++;
			//remove(filename);
			//pc.printf("Remove an existing flight log with the same name \n\r");
		}else{
			log_chosen = 1;
		}
	}

	get_formatted_date_time(date_stamp);

	sprintf(log_buffer,
			"\n%s\n%s%d\n%s\n%s\n%s\n",
			date_stamp,
			"Flight Log #", log_number,
			"ROVER: 'Rover Observation Vehicle for Enclosed Regions'",
			"Flight Log - Format Version '0.4'",
			"LEGEND:[rover_t, rover_status, [rover_orient], [rover_ex_gyro], rover_ex_gyro_temp, [rover_ex_acc], [rover_int_acc], [rover_ex_mag], [rover_int_mag]]");

	log_fp = fopen(filename, "w");
	if (log_fp == NULL) {
		pc.printf("Unable to write flightlog header!! \n");
		fclose(log_fp);
		status = 33;
		Thread::wait(osWaitForever);
	} else {
		fprintf(log_fp, log_buffer);
		fclose(log_fp);
		pc.printf("Flight Log Started as '%s' on %s\n\r", filename, date_stamp);

		//RtosTimer flight_log_timer(write_flight_log_entry, osTimerPeriodic, log_fp, log_buffer);
		//flight_log_timer.start(1000);

		status = 1;

		while(true){
			if (log_fp == NULL) {
				pc.printf("Unable to write flightlog!! \n");
			} else {
				get_ext_mag_data(&emd, sizeof(emd));
				get_ext_acc_data(&ead, sizeof(ead));
				get_ext_gyro_data(&egd, sizeof(egd));
				get_ext_gyro_temp(&egt, sizeof(egt));
				get_k64f_acc_data(&iad, sizeof(iad));
				get_k64f_mag_data(&imd, sizeof(imd));
				get_craft_orientation_est(&orient_est, sizeof(orient_est));
				get_craft_orientation_des(&orient_des, sizeof(orient_des));
				get_motor_thrust_des(&motor_thrust, sizeof(motor_thrust));

				sprintf(log_buffer,
						"DATA[%d, 0, [%f,%f,%f],[%f,%f,%f], [%f, %f, %f], %d, [%f, %f, %f], [%f, %f, %f], [%f, %f, %f], [%f, %f, %f],[%f,%f,%f,%f]]\n",
						sw_uptime,
						orient_est.roll, orient_est.pitch, orient_est.yaw,
						orient_des.roll, orient_des.pitch, orient_des.yaw,
						egd.x, egd.y, egd.z,
						egt.temp_c,
						ead.x, ead.y, ead.z,
						iad.x, iad.y, iad.z,
						emd.x, emd.y, emd.z,
						imd.x, imd.y, imd.z,
						motor_thrust.fl, motor_thrust.fr, motor_thrust.rr, motor_thrust.rl);
				//pc.printf(log_buffer);pc.printf("\r");
				log_fp = fopen(filename, "a");
				fprintf(log_fp, log_buffer);
				fclose(log_fp);
			}
			Thread::wait(50); //(reduce to <10Hz later)(usually 50)

		}
	}
}


void run_remote_logger_thread(void const *args){

	/*craft_orientation_des_t orient_des = {0,0,0};

	RtosTimer test_timer(test_periodic, osTimerPeriodic, &orient_des);
		//RtosTimer print_timer(print_sensor_data, osTimerPeriodic, (void *)1);

	test_timer.start(1000);
*/
	while(true){Thread::wait(osWaitForever);}
}

/* thread priorities for reference
  osPriorityIdle          = -3,          ///< priority: idle (lowest)
  osPriorityLow           = -2,          ///< priority: low
  osPriorityBelowNormal   = -1,          ///< priority: below normal
  osPriorityNormal        =  0,          ///< priority: normal (default)
  osPriorityAboveNormal   = +1,          ///< priority: above normal
  osPriorityHigh          = +2,          ///< priority: high
  osPriorityRealtime      = +3,          ///< priority: realtime (highest)
  osPriorityError         =  0x84        ///< system cannot determine priority or thread has illegal priority
 */

// start ROVER Threads (make sure to set priority)
int start_operating_threads(){

	//// flight critical threads:  = new Thread(priority 1)

	// collects sensor data from the main IMU sensors  = new Thread(flight essential)
	// and stores them to their respective global instances  = new Thread(behind mutexes)
	core_sensors_thread = new Thread(run_core_sensors_thread, NULL, osPriorityRealtime);

	// runs the estimator using the latest
	// data from the core_sensor_thread  = new Thread(mutexed consumption/production)
	estimator_thread = new Thread(run_estimator_thread, NULL, osPriorityRealtime);

	// drives the motors based on input from the flight controller
	motor_control_thread = new Thread(run_motor_control_thread, NULL, osPriorityRealtime);

	// flight controller which handles basic movement commands
	// and updates motor_control speeds according to flight settings and estimator
	flight_control_thread = new Thread(run_flight_control_thread, NULL, osPriorityRealtime);


	//// navigation critical threads:  = new Thread(priority 2)

	// collects data from navigation sensors and stores
	// them to global instances  = new Thread(mutexed)
	nav_sensor_thread = new Thread(run_nav_sensor_thread, NULL, osPriorityAboveNormal);

	// receives data from the telemetry radio  = new Thread(mutexed serial)
	// and sends it to the position controller  = new Thread(includes kill commands)
	remote_control_thread = new Thread(run_remote_control_thread, NULL, osPriorityRealtime);

	// controls the position of the flyer and perturbates the
	// flight controller to allow for movement  = new Thread(based on nav sensors and radio_rx)
	position_control_thread = new Thread(run_position_control_thread, NULL, osPriorityAboveNormal);


	//// non-critical threads:  = new Thread(priority 3)

	// logs data at a quick rate to the onboard SD card (needs a high thread priority to obtain sdcard pin write lock
	//  = new Thread(mutexed consumer to most producer threads)
	flight_logger_thread = new Thread(run_flight_logger_thread, NULL, osPriorityRealtime, (DEFAULT_STACK_SIZE * 2.25));

	// grabs preformatted log data as produced by flight_logger_thread
	// and sends it over telemetry radio  = new Thread(tx, mutexed serial)
	remote_logger_thread = new Thread(run_remote_logger_thread, NULL, osPriorityNormal);
	return 0;
}

int main() {

	//INIT
	pc.printf("\r\n\nSTART\r\n");
	status = 0;
	Thread status_thread(show_status_led_thread, NULL, osPriorityRealtime);

	start_operating_threads();

	while (true) {
		Thread::wait(250);
	}

}
