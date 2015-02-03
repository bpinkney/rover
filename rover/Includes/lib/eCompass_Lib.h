/************************************************************************************************
* File       : eCompass.h
* Date       : 18/07/2013 
* Purpose    : Wi-Go eCompass interface and hardware peripherals definition 
* Author     : Frank Vannieuwkerke
* Description: ported from Avnet Wi-Go code
*              For additional information on Wi-Go please visit
*                    www.em.avnet.com/wi-go
*              Additional info on the Freescale eCompass software is available at
*                    http://www.freescale.com/webapp/sps/site/prod_summary.jsp?code=E-Compass
************************************************************************************************/
#ifndef ECOMPASS_H__
#define ECOMPASS_H__

#include "mbed.h"
#include "MotionSensor.h"


#ifdef  __cplusplus
extern "C" {
#endif

typedef short int16;
typedef unsigned short uint16;
typedef unsigned int uint32;
typedef unsigned char byte;

extern void run_eCompass(void);
extern void init_eCompass(void);
extern void calibrate_eCompass(void);

#define NED_COMPASS     1
#define ANDROID_COMPASS 2
#define WINDOWS_COMPASS 3

//  Macros for constants used in fractional decimal conversion routine.
//  FRAC_2d1 = 2^-1 = 0.5
//  FRAC_2d2 = 2^-2 = 0.25 etc...
#define FRAC_2d1              5000
#define FRAC_2d2              2500
#define FRAC_2d3              1250
#define FRAC_2d4               625  

// sensor scaling constants
#define FCOUNTSPERUT 10.0F              // sensor specific: MAG3110 provide 10 counts / uT 
#define FCOUNTSPERG 4096.0F             // sensor specific: MMA8451 provide 4096 counts / g in 2g mode


 
typedef struct  {
    // Always 1
       uint16              packet_id;
    // Light sensor
       int16               light;
    // Counts at a 0.020MS interval when sensor data is processed
       uint32              timestamp;
    // Ineeger data from accelerometer
       int16               acc_x, acc_y, acc_z;
    // Integer data from Magnetometer
       int16               mag_x, mag_y, mag_z;
     // Roll, Pitch, Yaw and Compass from the eCompass algorithm
       int16               roll, pitch, yaw, compass;
    // Altitude and temperature
       int16               alt, temp;
    // Data from Accelerometer converted to floating point
       float               fax, fay, faz;
    // Accelerometer data converted to G's
       float               fGax, fGay, fGaz;
    // Data from Magnetometer converted to floating point
       float               fmx, fmy, fmz;
    // Magnetometer data converted to UT's
       float               fUTmx, fUTmy, fUTmz;
       float               q0, q1, q2, q3;
    
} axis6_t;

class eCompass
{
public:
    eCompass();
    void run(MotionSensorDataCounts &acc_raw, MotionSensorDataCounts &mag_raw);
    void calibrate(void);
    void init(void);
    

private:
};

#ifdef  __cplusplus
}
#endif

#endif