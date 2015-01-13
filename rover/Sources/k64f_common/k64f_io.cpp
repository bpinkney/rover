/*
#include "k64f_io.h"


k64f_io_t::k64f_io_t(){

}

void k64f_io_t::init() {
	//wait(2);
}

void k64f_io_t::init_flight_log(FILE * log_fp){

	//sprintf(filename, "%s", "/sd/test_log2.txt");


	wait(2);
	log_fp = fopen("/sdcard/test_log2.txt", "r");
	if (log_fp != NULL) {
		fclose(log_fp);
		remove("/sdcard/test_log2.txt");
		pc.printf("Remove an existing flight log with the same name \n\r");
	}

	sprintf(log_buffer,
			"\n%s\n%s\n%s\n",
			"Thu Aug 23 14:55:02 2001",
			"ROVER: 'Rover Observation Vehicle for Enclosed Regions'",
			"Flight Log - Format Version '0.1'",
			"LEGEND:[rover_t, rover_status, [rover_orient], [rover_ex_gyro], [rover_ex_acc], [rover_int_acc], [rover_ex_mag], [rover_int_mag]]");
	log_fp = fopen("/sdcard/test_log2.txt", "w");
	if (log_fp == NULL) {
		pc.printf("Unable to write flightlog header!! \n");
	} else {
		fprintf(log_fp, log_buffer);
	}
}

void k64f_io_t::write_flight_log_entry(FILE * log_fp){
	/*if (log_fp == NULL) {
		pc.printf("Unable to write flightlog!! \n");
	} else {
		ext_mag_data_t emd = get_ext_mag_data();
		ext_acc_data_t ead = get_ext_acc_data();
		ext_gyro_data_t egd = get_ext_gyro_data();
		k64f_acc_data_t iad = get_k64f_acc_data();
		k64f_mag_data_t imd = get_k64f_mag_data();

		sprintf(log_buffer,
				"DATA[0, 0, [0,0,0], [%f, %f, %f], [%f, %f, %f], [%f, %f, %f], [%f, %f, %f], [%d, %d, %d]]\n",
				egd.x, egd.y, egd.z,
				ead.x, ead.y, ead.z,
				iad.x, iad.y, iad.z,
				emd.x, emd.y, emd.z,
				imd.x, imd.y, imd.z);

		fprintf(log_fp, log_buffer);
	}
}

char* k64f_io_t::get_formatted_date_time(){
	return 0;
}

int64_t k64f_io_t::get_epoch_timestamp(){
	return 0;
}*/
