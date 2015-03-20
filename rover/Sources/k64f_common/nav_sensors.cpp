#include "nav_sensors.h"

#define START_BIT   0x04
#define MODE_SINGLE 0x02    // Single-ended mode
#define MODE_DIFF   0x00    // Differential mode

#define PAIR_ANGLE 0.13439f; //(radians of 7.7 degrees)

SPI spi(PTD2, PTD3, PTD1); // mosi, miso, sclk
DigitalOut cs0(PTC4);
DigitalOut cs1(PTD0);

nav_sensors_t::nav_sensors_t(){
	init();

	lf = {0,0,0,0};
	rf = {0,0,0,0};
	rb = {0,0,0,0};
	lb = {0,0,0,0};

}

void nav_sensors_t::init(){
	cs1 = 1;
	cs0 = 1;
	wait_us(1);
}

float nav_sensors_t::read_input(uint8_t cs, uint8_t channel)
{
    int command_high = START_BIT | MODE_SINGLE | ((channel & 0x04) >> 2);
    int command_low = (channel & 0x03) << 6;

    cs0 = cs;
    cs1 = !cs;

    // Odd writing requirements, see the datasheet for details
    spi.write(command_high);
    int high_byte = spi.write(command_low) & 0x0F;
    int low_byte = spi.write(0);

    //get sensor reference
    int index = (cs)*8 + channel;
    ir_sensors[index];

    //deselect
    cs0 = 1;
    cs1 = 1;
    wait_us(1);

    int conv_result = (high_byte << 8) | low_byte;
    float output = 55.65 * pow(((((float)conv_result) / 4096)*3.3),-1.037);

    BOUND_VARIABLE(output, (float)20, (float)150);
    LP_FILT(ir_sensors_trust[index], 1- fabs(ir_sensors[index] - output)/150, 10);
    LP_FILT(ir_sensors[index], output, 25);
    //ir_sensors[index] = ; //derived from excel trendline

    return ir_sensors[index];
}

float nav_sensors_t::derive_d(float da, float db){
	return (1.98197*da*db)/(da + db);
}

float nav_sensors_t::derive_u(float da, float db){
	return (float)((0.13399*db)/pow((0.017952*pow(db,2) + (0.017952*pow(db,2)*pow((0.99098*da - 0.990983*db),2))/pow((0.13399*da + 0.13399*db),2)), 0.5));
}

float nav_sensors_t::derive_v(float da, float db){
	return (float)(-(0.13399*db*(0.99098*da - 0.99098*db))/(sqrt((0.017952*pow(db,2) + (0.017952*pow(db,2)*pow((0.99098*da - 0.99098*db),2))/pow((0.13399*da + 0.13399*db),2)))*(0.13399*da + 0.13399*db)));
}

float nav_sensors_t::combine_trusts(float da_trust, float db_trust){
	return da_trust*db_trust;
}

void nav_sensors_t::get_side_geometries(){
	//get sensor data
	for(int i = 0; i < 8; i++){
		read_input(0, i);
	}

	//geometry
	lf = {
			derive_d(ir_sensors[0], ir_sensors[1]),
			combine_trusts(ir_sensors_trust[0], ir_sensors_trust[1]),
			derive_u(ir_sensors[0], ir_sensors[1]),
			derive_v(ir_sensors[0], ir_sensors[1])
	};
	rb = {
			derive_d(ir_sensors[2], ir_sensors[3]),
			combine_trusts(ir_sensors_trust[2], ir_sensors_trust[3]),
			derive_u(ir_sensors[2], ir_sensors[3]),
			derive_v(ir_sensors[2], ir_sensors[3])
		};
	rf = {
			derive_d(ir_sensors[4], ir_sensors[5]),
			combine_trusts(ir_sensors_trust[4], ir_sensors_trust[5]),
			derive_u(ir_sensors[4], ir_sensors[5]),
			derive_v(ir_sensors[4], ir_sensors[5])
		};
	lb = {
			derive_d(ir_sensors[6], ir_sensors[7]),
			combine_trusts(ir_sensors_trust[6], ir_sensors_trust[7]),
			derive_u(ir_sensors[6], ir_sensors[7]),
			derive_v(ir_sensors[6], ir_sensors[7])
		};

	set_lf_ir_pair({lf.magnitude, lf.trust, lf.u, lf.v});
	set_rf_ir_pair({rf.magnitude, rf.trust, rf.u, rf.v});
	set_rb_ir_pair({rb.magnitude, rb.trust, rb.u, rb.v});
	set_lb_ir_pair({lb.magnitude, lb.trust, lb.u, lb.v});
}

void nav_sensors_t::test_proc(){

	// Select the device by seting chip select low
	cs1 = 0;

	// Send 0x8f, the command to read the WHOAMI register
	spi.write(0x8F);

	// Send a dummy byte to receive the contents of the WHOAMI register
	int whoami = spi.write(0x00);
	printf("WHOAMI register = 0x%X\n", whoami);

	// Deselect the device
	cs1 = 1;
}



