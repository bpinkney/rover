#include "orientation_estimator.h"

orientation_estimator_t::orientation_estimator_t(){
	init();
}

void orientation_estimator_t::init(){
	gyro_trust = 0.9;
	acc_trust = 1-gyro_trust;

	egd = {0,0,0};
	iad = {0,0,0};
	imd = {0,0,0};
	o_orient = {0,0,0};
}

void orientation_estimator_t::run_estimate_loop(){

	//pc.printf("gogog");
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

	get_ext_gyro_data(&egd, sizeof(egd));
	get_k64f_acc_data(&iad, sizeof(iad));
	get_k64f_mag_data(&imd, sizeof(imd));
	get_craft_orientation_est(&o_orient, sizeof(o_orient));

	//pc.printf("Gyro Data [XYZ] [%f, %f, %f]\n\r", egd.x, egd.y, egd.z);

	roll = gyro_trust*(o_orient.roll + (egd.x*((float)imu_read_period)/1000)) + acc_trust*(float)atan2(iad.x,sqrt(pow(iad.y,2) + pow(iad.z,2)));
	pitch = gyro_trust*(o_orient.pitch + (egd.y*((float)imu_read_period)/1000)) + acc_trust*(float)atan2(iad.y,sqrt(pow(iad.x,2) + pow(iad.z,2)));

	yaw = atan2(imd.z*sin(pitch) - cos(pitch)*imd.y, cos(roll)*imd.x - imd.y*sin(pitch)*sin(roll) - cos(pitch)*imd.z*sin(roll));

	set_craft_orientation_est({roll, pitch, yaw}); //radians


}






