#include "eCompass_Lib.h"


extern void run_eCompass(void);
extern void init_eCompass(void);
extern void calibrate_eCompass(void);

extern void hal_map(MotionSensorDataCounts * acc_raw, MotionSensorDataCounts * mag_raw);

axis6_t axis6;
uint32_t seconds;
uint32_t compass_type;
int32_t tcount;
uint8_t cdebug;    

/*
typedef struct MotionSensorDataCounts
{
    int16_t x, y, z;
} MotionSensorDataCounts;
*/
 
eCompass::eCompass()
{
init_eCompass();
// default compass is NED
// Can be changed by user code to:
// compass_type = ANDROID_COMPASS;
// compass_type = WINDOWS_COMPASS;
compass_type = NED_COMPASS; 
cdebug = 0;
}


//eCompass::~eCompass() { }

void eCompass::run(MotionSensorDataCounts &acc_raw, MotionSensorDataCounts &mag_raw)
{
    axis6.timestamp = tcount;
    hal_map(&acc_raw, &mag_raw);
    //
    // raw data
    axis6.acc_x = acc_raw.x;
    axis6.acc_y = acc_raw.y;
    axis6.acc_z = acc_raw.z;
    axis6.mag_x = mag_raw.x;
    axis6.mag_y = mag_raw.y;
    axis6.mag_z = mag_raw.z;
    //
    // raw data converted to floating ouing
    axis6.fax = (float) acc_raw.x;
    axis6.fay = (float) acc_raw.y;
    axis6.faz = (float) acc_raw.z;
    axis6.fmx = (float) mag_raw.x;
    axis6.fmy = (float) mag_raw.y;
    axis6.fmz = (float) mag_raw.z;
    //
    // Accelerometer data converted to Gs
    axis6.fGax = ((float) acc_raw.x) / 4096.0f;
    axis6.fGay = ((float) acc_raw.y) / 4096.0f;
    axis6.fGaz = ((float) acc_raw.z) / 4096.0f;
    //
    // Magnetometer data converted to microteslas
    axis6.fUTmx = ((float) mag_raw.x) / 10.0f;
    axis6.fUTmy = ((float) mag_raw.y) / 10.0f;
    axis6.fUTmz = ((float) mag_raw.z) / 10.0f; 
    //printf("r");
    if(!cdebug) run_eCompass();
    }
   
void eCompass::calibrate(void) {
    calibrate_eCompass();
}
   
void eCompass::init(void) {
    init_eCompass();
}
