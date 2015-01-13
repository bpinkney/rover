/*
 * k64f_acc_mag.cpp
 *
 *  Created on: Dec 22, 2014
 *      Author: James T. Kirk
 */

#include "k64f_acc_mag.h"

k64f_sensor_interface::k64f_sensor_interface(){
	//init();
}

void k64f_sensor_interface::init(){
	k64f_acc.enable();
	//acc.writeRegs()
}

int k64f_sensor_interface::fetch_sensor_data(){
	k64f_acc.getAxis(acc_raw);
	k64f_mag.getAxis(mag_raw);
	set_k64f_acc_data({
		(float)acc_raw.x*FXOS8700Q_00_LSB,
		(float)acc_raw.y*FXOS8700Q_00_LSB,
		(float)acc_raw.z*FXOS8700Q_00_LSB});
	set_k64f_mag_data({mag_raw.x, mag_raw.y, mag_raw.z});
	return 0;
}

void k64f_sensor_interface::print_sensor_data(){
	//pc.printf("\r\n\nFXOS8700Q Who Am I= %X\r\n", acc.whoAmI());
	/*k64f_acc_data_t data_acc = get_k64f_acc_data();
	k64f_mag_data_t data_mag = get_k64f_mag_data();


	//pc.printf("FXOS8700Q ACC: X=%d Y=%d Z=%d  ", acc_raw.x, acc_raw.y, acc_raw.z);
	//pc.printf("    MAG: X=%d Y=%d Z=%d\r\n", mag_raw.x, mag_raw.y, mag_raw.z);
	pc.printf("FXOS8700Q ACC: X=%f Y=%f Z=%f  ", data_acc.x, data_acc.y, data_acc.z);
	pc.printf("    MAG: X=%d Y=%d Z=%d\r\n", data_mag.x, data_mag.y, data_mag.z);
	/*
	acc.getX(&raX);
	acc.getY(&raY);
	acc.getZ(&raZ);
	mag.getX(&rmX);
	mag.getY(&rmY);
	mag.getZ(&rmZ);
	pc.printf("FXOS8700Q ACC: X=%d Y=%d Z=%d  ", raX, raY, raZ);
	pc.printf("    MAG: X=%d Y=%d Z=%d\r\n\n", rmX, rmY, rmZ);
	*/

}



