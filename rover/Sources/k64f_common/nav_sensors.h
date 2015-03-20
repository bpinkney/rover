#include "mbed.h"
#include "rtos.h"
#include "k64f_common.h"

#ifndef NAV_SENSORS_H_
#define NAV_SENSORS_H_

class nav_sensors_t {

private:
	//magnitudes
	//let ir0..7 refer to side sensors, ir8..10 bottom sensor, ir11 top sensor
	float ir_sensors[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
	float ir_sensors_trust[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
	side_vector_t lf;
	side_vector_t rf;
	side_vector_t rb;
	side_vector_t lb;


public:
	nav_sensors_t();
	void init();
	void test_proc();
	float read_input(uint8_t cs, uint8_t channel);

	float derive_d(float da, float db);
	float derive_u(float da, float db);
	float derive_v(float da, float db);
	float combine_trusts(float da_trust, float db_trust);

	void get_side_geometries();

};

#endif

