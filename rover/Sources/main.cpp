
#include "mbed.h"
#include "rtos.h"

#include <time.h>

//k64f custom classes/resources
#include "k64f_common.h"
//#include "k64f_io.h"
#include "k64f_acc_mag.h"
#include "IMU.h"

//turnigy multistar operates at 8kHz PWM
float pwm_period = 1/10;

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

//ofstream log_fp;

//uptime
uint32_t sw_uptime = 0;

uint8_t status = 0;

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

void driveMotor(void const *args) {//not working yet
	PwmOut motor1(PTA0);
	motor1.pulsewidth(pwm_period);
	motor1.period(pwm_period);


	float pe;
	while(true){
		pe= motor1.read();
		//pc.printf("A read= %f\n\r", pe);
		Thread::wait(1);
	}
}


//RTOS timer class access functions
void tick_clock(void const *n){
	sw_uptime++;
}


void fetch_sensor_data(void const *n){
	sw_uptime+=10;
	int_imu.fetch_sensor_data();
	ext_imu.readGyros();
	ext_imu.readAccs();
	ext_imu.readMags();
}

//thread run functions (see thread declarations for descriptions)
void run_core_sensors_thread(void const *args){
	char ext_sensor_ack;

	pc.printf("Start Sensors...\n");
	int_imu.init();
	ext_sensor_ack = ext_imu.init();
	pc.printf("\r\n\nIMU Init Ack= %d\r\n", ext_sensor_ack);
	RtosTimer imu_timer(fetch_sensor_data, osTimerPeriodic, (void *)0);
	//RtosTimer print_timer(print_sensor_data, osTimerPeriodic, (void *)1);

	if(ext_sensor_ack == 0){
		imu_timer.start(10);
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

void run_estimator_thread(void const *args){
	while(true){Thread::wait(osWaitForever);}
}

void run_motor_control_thread(void const *args){
	while(true){Thread::wait(osWaitForever);}
}

void run_flight_control_thread(void const *args){
	while(true){Thread::wait(osWaitForever);}
}

void run_nav_sensor_thread(void const *args){
	while(true){Thread::wait(osWaitForever);}
}

void run_remote_control_thread(void const *args){
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
	k64f_acc_data_t iad = {0,0,0};
	k64f_mag_data_t imd = {0,0,0};

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
			"Flight Log - Format Version '0.1'",
			"LEGEND:[rover_t, rover_status, [rover_orient], [rover_ex_gyro], [rover_ex_acc], [rover_int_acc], [rover_ex_mag], [rover_int_mag]]");

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
				get_k64f_acc_data(&iad, sizeof(iad));
				get_k64f_mag_data(&imd, sizeof(imd));

				sprintf(log_buffer,
						"DATA[%d, 0, [0,0,0], [%f, %f, %f], [%f, %f, %f], [%f, %f, %f], [%f, %f, %f], [%d, %d, %d]]\n",
						sw_uptime,
						egd.x, egd.y, egd.z,
						ead.x, ead.y, ead.z,
						iad.x, iad.y, iad.z,
						emd.x, emd.y, emd.z,
						imd.x, imd.y, imd.z);
				//pc.printf(log_buffer);pc.printf("\r");
				log_fp = fopen(filename, "a");
				fprintf(log_fp, log_buffer);
				fclose(log_fp);
			}
			Thread::wait(15); //(reduce to <10Hz later)

		}
	}
}

void run_remote_logger_thread(void const *args){
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
	remote_control_thread = new Thread(run_nav_sensor_thread, NULL, osPriorityAboveNormal);

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
