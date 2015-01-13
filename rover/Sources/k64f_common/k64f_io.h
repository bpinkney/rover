/*#include "mbed.h"
#include "k64f_common.h"

#ifndef K64F_IO_H_
#define K64F_IO_H_


class k64f_io_t {

private:
	//FILE *log_fp;
	//char filename[1024];
	char log_buffer[1024];
	char* get_formatted_date_time();
	int64_t get_epoch_timestamp();

public:
	k64f_io_t();
	void init();
	void init_flight_log(FILE * log_fp);
	void write_flight_log_entry(FILE * log_fp);

};

#endif*/


