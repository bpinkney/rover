
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
//const uint16_t flight_control_outer_period = 20;
uint32_t sw_uptime = 0;



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

//status stuff (does this need to be mutexed?)(yeah, i'm pretty sure it does since interthread uint8s aren't even atomic!)
Mutex status_mutex;
uint8_t status = 0;

uint8_t get_status(void* buffer){
    status_mutex.lock();
    memcpy(buffer, &status, 1);
    status_mutex.unlock();
}
void set_status(uint8_t value){
	status_mutex.lock();
	status = value;
	status_mutex.unlock();
}

void show_status_led_thread(void const *args) {
	uint8_t step_count = 0; //8 steps
	DigitalOut led1(LED1);//red
	DigitalOut led2(LED2);//green
	DigitalOut led3(LED3);//blue

	uint8_t current_status = 99;
	uint8_t new_status = 0;
	//LEDs guide:
	// 1&2&3 light blue w/red dot...
	// 1&2: yellowy green
	// 1&3: purply
	// 2&3 light blue

	//status is currently according to following enums:
	//	10 - connect to radios				-> yellow_green_solid w/ blip
	//	0 - Boot/Init Phase					-> blue slow flash
	//	1 - Armed Status (flight-ready) 	-> green triple flash
	//	2 - Takeoff Commencing				-> green triple flash w/ backlight
	//	33 - Error State					-> red/green fast flash

	while(true){

		get_status(&new_status);

		if (current_status != new_status && current_status != 33){
			//change this later, but for now error state cannot be undone, LED-wise at least
			led1 = 1;
			led2 = 1;
			led3 = 1;
			step_count = 0;
			current_status = new_status;
		}

		if(current_status == 0){
			if(step_count < 4){
				led3 = 0;
			}else{
				led3 = 1;
			}
		}else if(current_status == 10){
			if(step_count < 7){
				led1 = 0;
				led2 = 0;
			}else{
				led1 = 1;
				led2 = 1;
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

	rs.printf("Start Sensors...\n");
	int_imu.init();
	ext_sensor_ack = ext_imu.init();
	rs.printf("\r\n\nIMUS: External IMU Init Ack = %d\r\n", ext_sensor_ack);
	RtosTimer imu_timer(fetch_sensor_data, osTimerPeriodic, (void *)0);
	//RtosTimer print_timer(print_sensor_data, osTimerPeriodic, (void *)1);

	if(ext_sensor_ack == 0){
		imu_timer.start(imu_read_period);
	}else{
		set_status(33);
		rs.printf("External IMU could not be started!!\n\r");
	}
	//print_timer.start(250);

	//uint32_t count = 0;
	while(true){
		Thread::wait(osWaitForever);
	}
	rs.printf("Thread Ended Prematurely!");
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

	//rs.printf("Orient Est [RPY]: %f, %f, %f\n\r", roll, pitch, yaw);
}

void run_estimator_thread(void const *args){

	orientation_estimator.init();
	orientation_estimator.imu_read_period = imu_read_period;

	RtosTimer estimator_timer(update_estimator, osTimerPeriodic, (void *)0);

	rs.printf("Starting Estimator\n\r");
	estimator_timer.start(estimator_period);

	while(true){Thread::wait(osWaitForever);}
}

void run_motor_control_thread(void const *args){

	rs.printf("MOTORS: initialize ESCs\n\r");

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

	rs.printf("ESC initialized\n\r");
	//rs.printf("set init throttle to : %f\n\r", throttle_var);
	uint32_t init_time = 0;
	uint8_t calibrate = 0;

	while(1)
		{
		if(init_time < 5000 && calibrate){//wait for init of ESCs
			m1_throttle = 1;
			m2_throttle = 1;
			m3_throttle = 1;
			m4_throttle = 1;
			//rs.printf("set throttle to : %f\n\r", throttle_var);
		}else if(init_time < 10000){
			m1_throttle = 0;
			m2_throttle = 0;
			m3_throttle = 0;
			m4_throttle = 0;
			//rs.printf("set throttle to : %f\n\r", throttle_var);
		}else{

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

			Thread::wait(10);  //20ms is the default period of the ESC pwm; the ESC may not run faster.

			init_time+=10;

			/*if(init_time % 2000 == 0){
				rs.printf("Throttle: %f\n\r", m1_throttle);
			}*/

		}


	while(true){Thread::wait(osWaitForever);}
}

void flight_control_inner_loop(void const *n){
	flight_controller.run_control_loop();
}

void run_flight_control_thread(void const *args){
	flight_controller.init();

	RtosTimer flight_control_timer(flight_control_inner_loop, osTimerPeriodic, (void *)0);

	rs.printf("Starting Flight Controller\n\r");
	flight_control_timer.start(flight_control_inner_period);


	while(true){Thread::wait(osWaitForever);}
}

void run_nav_sensor_thread(void const *args){
	while(true){Thread::wait(osWaitForever);}
}

void run_remote_control_thread(void const *args){
	//rs.printf("RADIO: Start Radio Serial\r\n");
	//telemetry serial in/out (57000 baud, set in remote thread)
	rs.baud(57000); //only baud rate accepted by radios
	rs.printf("Welcome to ROVER Remote Logging and Control\n\r");
	rs.printf("RADIO: Polling for remote chars...\r\n");
	uint8_t c;
	set_status(0);
	//snes controller config
	//dpad: wasd
	//buttons: tfgh
	//LR triggers: zx
	//Select Start: kl

	float pitch_p = 0.08;//0.18;
	float pitch_d = 0.0095;//0.025;

	float roll_p = 0.09;//0.13;
	float roll_d = 0.0095;//0.027;

	float pitch_dd = 0;
	float roll_dd = 0;

	float des_pitch;
	float des_roll;

	while(1) {
		if(rs.readable()) {
			c = rs.getc();
			//rs.printf("Radio chars received: '%d'\n\r", c);
			if(c == 'w' && ext_throttle < 0.25){
				ext_throttle = 0.25;
				flight_controller.set_base_thrust(ext_throttle);
				rs.printf("Throttle start to: '%f' percent.\n\r", ext_throttle);
			}else if(c == 'w' && ext_throttle <= 0.9){
				ext_throttle = ext_throttle + 0.05;
				flight_controller.set_base_thrust(ext_throttle);
				rs.printf("Throttle set to: '%f' percent.\n\r", ext_throttle);
			}
			else if(c == 's' && ext_throttle >= 0.3){
				ext_throttle = ext_throttle - 0.05;
				flight_controller.set_base_thrust(ext_throttle);
				rs.printf("Throttle set to: '%f' percent.\n\r", ext_throttle);
			}else if(c == 's' && ext_throttle >= 0.1){
				ext_throttle = 0;
				flight_controller.set_base_thrust(ext_throttle);
				rs.printf("Throttle Off.\n\r");
			}else if(c == 'x'){
				ext_yaw = 0;
				ext_pitch = 0;
				ext_roll = 0;
				rs.printf("Yaw pitch roll reset\n\r", ext_throttle);

				//d
			}else if(c == 'h'){//A (Y)
				pitch_d += 0.0005;
				flight_controller.update_pitch_pddd(-1, pitch_d, -1);
				rs.printf("%d: pitch D is now: %f\n\r", sw_uptime,pitch_d);

			}
			else if(c == 'f'){//(A) Y
				pitch_d -= 0.0005;
				flight_controller.update_pitch_pddd(-1, pitch_d, -1);
				rs.printf("%d: pitch D is now: %f\n\r", sw_uptime,pitch_d);
			}
			else if(c == 't'){//X (B)
				roll_d += 0.0005;
				flight_controller.update_roll_pddd(-1, roll_d,-1);
				rs.printf("%d: roll D is now: %f\n\r", sw_uptime,roll_d);
			}
			else if(c == 'g'){//(X) B
				roll_d -= 0.0005;
				flight_controller.update_roll_pddd(-1, roll_d,-1);
				rs.printf("%d: roll D is now: %f\n\r", sw_uptime,roll_d);
			}

				//p
		/*}else if(c == 'h'){//A (Y)
			pitch_p += 0.002;
			flight_controller.update_pitch_pddd(pitch_p,-1, -1);
			rs.printf("%d: pitch P is now: %f\n\r", sw_uptime,pitch_p);

		}
		else if(c == 'f'){//(A) Y
			pitch_p -= 0.002;
			flight_controller.update_pitch_pddd(pitch_p,-1, -1);
			rs.printf("%d: pitch P is now: %f\n\r", sw_uptime,pitch_p);
		}
		else if(c == 't'){//X (B)
			roll_p += 0.002;
			flight_controller.update_roll_pddd(roll_p,-1, -1);
			rs.printf("%d: roll P is now: %f\n\r", sw_uptime,roll_p);
		}
		else if(c == 'g'){//(X) B
			roll_p -= 0.002;
			flight_controller.update_roll_pddd(roll_p,-1, -1);
			rs.printf("%d: roll P is now: %f\n\r", sw_uptime,roll_p);
		}*/

				//dd
			/*}else if(c == 'h'){//A (Y)
				pitch_dd += 0.00005;
				flight_controller.update_pitch_pddd(-1, -1, pitch_dd);
				rs.printf("%d: pitch DD is now: %f\n\r", sw_uptime,pitch_dd);

			}
			else if(c == 'f'){//(A) Y
				pitch_dd -= 0.00005;
				flight_controller.update_pitch_pddd(-1, -1, pitch_dd);
				rs.printf("%d: pitch DD is now: %f\n\r", sw_uptime,pitch_dd);
			}
			else if(c == 't'){//X (B)
				roll_dd += 0.00005;
				flight_controller.update_roll_pddd(-1, -1, roll_dd);
				rs.printf("%d: roll DD is now: %f\n\r", sw_uptime,roll_dd);
			}
			else if(c == 'g'){//(X) B
				roll_dd -= 0.00005;
				flight_controller.update_roll_pddd(-1, -1, roll_dd);
				rs.printf("%d: roll DD is now: %f\n\r", sw_uptime,roll_dd);
			}*/
			else if(c == 'a'){
				//ext_roll = 1;
				rs.printf("Roll left.\n\r");
			}
			else if(c == 'd'){
				//ext_roll = -1;
				rs.printf("Roll right.\n\r");
			}
			else if(c == 'k'){
				ext_throttle = 0;
				ext_yaw = 0;
				ext_pitch = 0;
				ext_roll = 0;
				rs.printf("Throttle Killed\n\r");
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
	craft_rates_t rates_des = {0,0,0};
	craft_accs_t accs_est = {0,0,0};
	motor_thrust_des_t motor_thrust = {0,0,0,0};
	Thread::wait(2000);
	rs.printf("Flight Log Starting...\n\r");

	uint8_t sd_ready = sd.disk_initialize();

	while(!log_chosen){
		sprintf(filename, "/sdcard/flight_log_%d.txt", log_number);
		log_fp = fopen(filename, "r");
		if (log_fp != NULL) {
			fclose(log_fp);
			log_number++;
			//remove(filename);
			//rs.printf("Remove an existing flight log with the same name \n\r");
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
			"Flight Log - Format Version '0.6'",
			"LEGEND:[rover_t, rover_status, [rover_orient_est], [rover_orient_des],[rover_rates_des],[rover_accs_est], [rover_ex_gyro], rover_ex_gyro_temp, [rover_ex_acc], [rover_int_acc], [rover_ex_mag], [rover_int_mag], [rover_thrusts_des]");

	log_fp = fopen(filename, "w");
	if (log_fp == NULL || sd_ready!=0) {
		rs.printf("Unable to write flight log header!! \n");
		fclose(log_fp);
		set_status(33);
		Thread::wait(osWaitForever);
	} else {
		fprintf(log_fp, log_buffer);
		fclose(log_fp);
		rs.printf("Flight Log Started as '%s' on %s\n\r", filename, date_stamp);

		//RtosTimer flight_log_timer(write_flight_log_entry, osTimerPeriodic, log_fp, log_buffer);
		//flight_log_timer.start(1000);

		set_status(1);

		while(true){
			if (log_fp == NULL) {
				rs.printf("Unable to write flight log!! \n");
			} else {
				get_ext_mag_data(&emd, sizeof(emd));
				get_ext_acc_data(&ead, sizeof(ead));
				get_ext_gyro_data(&egd, sizeof(egd));
				get_ext_gyro_temp(&egt, sizeof(egt));
				get_k64f_acc_data(&iad, sizeof(iad));
				get_k64f_mag_data(&imd, sizeof(imd));
				get_craft_orientation_est(&orient_est, sizeof(orient_est));
				get_craft_orientation_des(&orient_des, sizeof(orient_des));
				get_craft_rates_des(&rates_des, sizeof(rates_des));
				get_craft_accs_est(&accs_est, sizeof(accs_est));
				get_motor_thrust_des(&motor_thrust, sizeof(motor_thrust));

				sprintf(log_buffer,
						"DATA[%d, 0, [%f,%f,%f],[%f,%f,%f],[%f,%f,%f],[%f,%f,%f], [%f, %f, %f], %d, [%f, %f, %f], [%f, %f, %f], [%f, %f, %f], [%f, %f, %f],[%f,%f,%f,%f]]\n",
						sw_uptime,
						orient_est.roll, orient_est.pitch, orient_est.yaw,
						orient_des.roll, orient_des.pitch, orient_des.yaw,
						rates_des.roll, rates_des.pitch, rates_des.yaw,
						accs_est.roll, accs_est.pitch, accs_est.yaw,
						egd.x, egd.y, egd.z,
						egt.temp_c,
						ead.x, ead.y, ead.z,
						iad.x, iad.y, iad.z,
						emd.x, emd.y, emd.z,
						imd.x, imd.y, imd.z,
						motor_thrust.fl, motor_thrust.fr, motor_thrust.rr, motor_thrust.rl);
				//rs.printf(log_buffer);rs.printf("\r");
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

	//status indicator
	set_status(10);
	Thread status_thread(show_status_led_thread, NULL, osPriorityRealtime);

	//INIT
	//handles terminal logging, so it goes first...
	// receives data from the telemetry radio  = new Thread(mutexed serial)
	// and sends it to the position controller  = new Thread(includes kill commands)
	Thread::wait(10000);//race the radios...
	remote_control_thread = new Thread(run_remote_control_thread, NULL, osPriorityRealtime);
	Thread::wait(10000);
	rs.printf("\r\n\nSTART\r\n");


	start_operating_threads();

	while (true) {
		Thread::wait(250);
	}

}
