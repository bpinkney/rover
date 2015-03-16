/*
 * k64f_acc_mag.h
 *
 *  Created on: Dec 22, 2014
 *      Author: James T. Kirk
 */

//#include "mbed.h"
#include "k64f_common.h"

#ifndef K64F_ACC_MAG_H_
#define K64F_ACC_MAG_H_

#define FXOS8700Q_00_LSB 0.0023928226f //in m/s^2
#define FXOS8700Q_01_LSB 0.0047856452f //in m/s^2
#define FXOS8700Q_10_LSB 0.0095712904f //in m/s^2


class k64f_sensor_interface {

private:
	MotionSensorDataUnits mag_data;
	MotionSensorDataUnits acc_data;

	MotionSensorDataCounts mag_raw;
	MotionSensorDataCounts acc_raw;

	int16_t raX, raY, raZ;
	int16_t rmX, rmY, rmZ;

	k64f_acc_data_t oiad;
	k64f_mag_data_t oimd;

	float acc_x;
	float acc_y;
	float acc_z;

	float mag_x;
	float mag_y;
	float mag_z;

	//k64f_sensor_interface() {} (need if overloading public constructor)



public:
	k64f_sensor_interface();
	void init();
	void print_sensor_data();
	int fetch_sensor_data();

};

#endif /* K64F_ACC_MAG_H_ */
