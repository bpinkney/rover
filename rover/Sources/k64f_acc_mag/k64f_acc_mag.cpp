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

	//pull old data
	k64f_acc_data_t oiad = {0,0,0};
	get_k64f_acc_data(&oiad, sizeof(oiad));

	//poll sensors
	k64f_acc.getAxis(acc_raw);
	k64f_mag.getAxis(mag_raw);


	//do maths
	float acc_x = (float)((float)acc_raw.x*FXOS8700Q_00_LSB*1.003748557 - 0.0465);
	float acc_y = (float)((float)acc_raw.y*FXOS8700Q_00_LSB*0.984794786 - 0.0720);
	float acc_z = (float)((float)acc_raw.z*FXOS8700Q_00_LSB*0.996061268 - 0.0543);

	//low pass filter (IIR)
	LP_FILT(oiad.x, acc_x, 15);
	LP_FILT(oiad.y, acc_y, 15);
	LP_FILT(oiad.z, acc_z, 15);

	set_k64f_acc_data({
		oiad.x,
		oiad.y,
		oiad.z});
	set_k64f_mag_data({
		(float)(((float)mag_raw.x - 53.0)*0.9791),
		(float)(((float)mag_raw.y - 17.0)*1.0006),
		(float)(((float)mag_raw.z - 806.0)*1.0212)
	});
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



