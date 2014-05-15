/*

	Test code for using ufo daughter board as slave. Master is initialized but unused.
	Collect sensor data and sid's are mockups.

	Extends slave functionality to include ADC call

	LED+ sent to ADCA[0..3] in PORT A

*/
#include "avr_compiler.h"
#include "twi_master_driver.h"
#include "twi_slave_driver.h"
#include <string.h>
#include <util/delay.h>	
#include "adc.h"

/*! Defining an example slave address. */
// this is the slave address of the device itself, not a slave it talks to
#define SLAVE_ADDRESS    0x55

// #define ARDUSLAVE        0x4F

/*! CPU speed 2MHz, BAUDRATE 100kHz and Baudrate Register Settings */
#define CPU_SPEED   2000000
#define BAUDRATE	 100000
#define TWI_BAUDSETTING TWI_BAUD(CPU_SPEED, BAUDRATE)

/*! Defining number of bytes in local arrays. */
#define SENSOR_NVAL     1
#define DEBUG_NVAL	    7  // debug >= data
#define DATA_BYTES      4  // 4 = float, 2 = int16, 1 = byte

#define FIRMWARE_VERSION 0
#define HARDWARE_VERSION 0

// used in sys clock definition. temporarily allows config change (only 4 clock cycles)
#define CONFIGURATION_CHANGE_PROTECTION  do { CCP = 0xd8; } while (0)

/* TWI Globals */
TWI_Master_t twicMaster;    /*!< TWI master module. */
TWI_Slave_t twicSlave;      /*!< TWI slave module. */

/* ADC Globals */
uint16_t adca_result;
uint8_t adca_result_flag = 0;
uint8_t ndvi_flag = 0;

uint8_t cal_flag = 4;

float gain = 1; 
float offset = 0;
int16_t ifactor;
int32_t icorrection;

uint8_t pin = 0;

//float alpha = 0.99; //defines averaging time 

/* ADC ISRs */
ISR(ADCA_CH0_vect) {
	adca_result = ADCA.CH0.RES;
	adca_result_flag = 1;  // Interrupt flag is cleared upon return from ISR
}


/* PULSE Data */

float data_values[DEBUG_NVAL] = {-99.0, -99.0, -99.0, -99.0, -99.0, -99.0, -99.0};
float null_value = -99.0;
uint8_t sensor_sid[SENSOR_NVAL] = {0x05};
uint8_t debug_sid[DEBUG_NVAL] = {0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B};

float * return_datum;

uint8_t data_ready = 0;

uint8_t command = 0;
uint8_t command_argument = 0;
uint8_t iSID = 0;

uint8_t matchSID(uint8_t SID){
	for (uint8_t iSID = 0; iSID<DEBUG_NVAL; iSID++){
		if(debug_sid[iSID] == SID) return iSID;
	}
	return -1;
}

void triggerADC(void){

	if (cal_flag==4){
		pin = 0;
		ADCA.CH0.MUXCTRL = (pin<<3) | 0x05; // pin[0..3] in positions 3..4 & Pad GND (0x05)
	} else {
		// Set initial channel for measurement / calibration
	    ADCA.CH0.MUXCTRL = 0x00 | 0x01; // measures pin0 as + pin1 as -		
	}

	adca_result = 0;
	adca_result_flag = 0;
	ndvi_flag = 0;
	data_ready = 0; 

	data_values[0] = null_value; 
	data_values[1] = null_value;
	data_values[2] = null_value;
	data_values[3] = null_value;
	data_values[4] = null_value;
	data_values[5] = null_value;
	data_values[6] = null_value;

	// Start ADC conversion
    ADCA.CH0.CTRL |= 0x80;
}

// Our slave is always on port C
void TWIC_SlaveProcessData(void)
{

	uint8_t bufIndex = twicSlave.bytesReceived;
	command = twicSlave.receivedData[0];
	if (bufIndex>0) command_argument = twicSlave.receivedData[1];

	switch (command){
		case 0x10:
			// data ready query
			twicSlave.sendData[0] = data_ready;
			break;
		case 0x20:
			// send sensor value, regular or debug
			iSID = matchSID(command_argument);
			if (iSID>=0){
				return_datum = &data_values[iSID];			
			} else {
				return_datum = &null_value;
			}
			memcpy((void *)twicSlave.sendData, return_datum, sizeof(float));			
			break;
		case 0x30:
			// send sensor nval
			twicSlave.sendData[0] = SENSOR_NVAL;
			break;
		case 0x40:
			// send debug nval
			twicSlave.sendData[0] = DEBUG_NVAL;
			break;	
		case 0x50:
			// send sensor sid array
			memcpy((void *)twicSlave.sendData, sensor_sid, SENSOR_NVAL);
			break;	
		case 0x60:
			// send debug sid array
			memcpy((void *)twicSlave.sendData, debug_sid, DEBUG_NVAL);
			break;	
		case 0x70:
			// send data type (== number of bytes)
			twicSlave.sendData[0] = DATA_BYTES;
			break;
		case 0x80:
			// trigger new set of measurements
			triggerADC();
			break;

	}
}


int main(void)
{

	ifactor = (int16_t)(gain*16384);
	icorrection = (int32_t)((0.5 - offset*gain)*16384);
	
	// When daughter talks to chips on board, it is port E
	// When daughter talks to arduino as slave, it is port C
	/* Initialize TWI master. */
	TWI_MasterInit(&twicMaster,
	              &TWIC,
	              TWI_MASTER_INTLVL_LO_gc,
	              TWI_BAUDSETTING);

	/* Initialize TWI slave. */
	TWI_SlaveInitializeDriver(&twicSlave, &TWIC, TWIC_SlaveProcessData);
	TWI_SlaveInitializeModule(&twicSlave,
	                          SLAVE_ADDRESS,
	                          TWI_SLAVE_INTLVL_LO_gc);

	/* Enable all interrupt levels. */
	/* TWI is lo ADC is hi */
	PMIC.CTRL |= PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;

	// Initialize the ADC
	adc_init();

	//adc_set_gain_calibration_channels();

	sei();

	triggerADC();

	// the loop
	while (1) {

		if(adca_result_flag) { // If a conversion has completed
			cli(); // disable interrupts
			if (cal_flag == 4){
				//adca_result = adc_compensate(adca_result, ifactor, icorrection);
				uint8_t sign = (adca_result>>11) & 0x01;
				uint16_t value = adca_result & 0x7FF;
				float voltage = 0;
				float adcgain = 1;
				float span = 3.3/1.6/adcgain;
				// equations assume gain 1, INTVCC aref, differential meas
				if (sign){
					voltage = -(float)((~value & 0x7FF) + 1)/(1<<11)*span;
				} else {
					voltage = (float)value/(1<<11)*span;
				}

				data_values[pin + 3] = voltage;

				adca_result_flag = 0;
				ndvi_flag |= (1<<pin); // ndvi is complete at 1111b
				pin = (pin+1) & 0x03; // bitmask ensures this cycles over 4 pins
				ADCA.CH0.MUXCTRL = (pin<<3) | 0x05; // pin[0..3] in positions 3..4 & Pad GND (0x05)
			} else {
				data_values[pin] = adca_result; // these config readings will be overwritten later
				cal_flag |= (1<<pin);
				pin = (pin+1) & 0x01; // bitmask ensures this cycles over 2 pins
				ADCA.CH0.MUXCTRL ^= 0x09; // toggle bits [0,4] to measures pin1 as + pin0 as -  
			}
		    ADCA.CH0.CTRL |= 0x80; // start new conversion
	        sei();
        }
        if(ndvi_flag == 0x0F){
        	cli();
        	data_values[1] = data_values[5]/data_values[3]; //Red bottom/top
        	data_values[2] = data_values[6]/data_values[4]; //NIR bottom/top
        	data_values[0] = (data_values[2]-data_values[1])/(data_values[2]+data_values[1]); //NDVI
        	data_ready = 1;
        	ndvi_flag = 0;
        	ADCA.CH0.CTRL &= (~0x80); // stop measurements
        	sei();
        }
        if(cal_flag == 3){
        	cli();
    		offset = (data_values[0] - data_values[1])/2;
    		data_values[0] = null_value; data_values[1] = null_value;
			ifactor = (int16_t)(gain*16384); // 16384 = 2^14
			icorrection = (int32_t)((0.5 - offset*gain)*16384); 
        	cal_flag = 4;
        	sei();
        }

	}
}

/*! TWIC Master Interrupt vector. */
ISR(TWIC_TWIM_vect){
	TWI_MasterInterruptHandler(&twicMaster);
}

/*! TWIC Slave Interrupt vector. */
ISR(TWIC_TWIS_vect){
	TWI_SlaveInterruptHandler(&twicSlave);
}

void sysclk_init(void) {
	
	// Assume operation from internal RC oscillator (2 MHz)
	
	// Assign pointer to Power Reduction Register
	uint8_t *reg = (uint8_t *)&PR.PRGEN;
	uint8_t i;
	
	// Turn off all peripheral clocks (initially) to conserve power
	for (i = 0; i <= 6; i++) {
		*(reg++) = 0xFF;
	}
	
	//////////////////////////////////////////////////////////////////////
	//OSC.PLLCTRL
	//     7       6       5       4       3        2        1        0
	// |  PLLSRC[1:0]  | PLLDIV |              PLLFAC[4:0]                 |
	//     0       0       0       0       0        0        0        0
	// Set 2MHz Internal RC Osc as PLL source
	// Set PLL to 16MHz
	OSC.PLLCTRL = OSC_PLLSRC_RC2M_gc | OSC_PLLFAC3_bm; // 0x08
	//////////////////////////////////////////////////////////////////////
	
	//////////////////////////////////////////////////////////////////////
	//OSC.CTRL
	//     7       6       5       4        3        2         1        0
	// |   -   |   -   |   -   | PLLEN | XOSCEN | RC32KEN | RC32MEN | RC2MEN |
	//     0       0       0       0        0        0         0        0
	// Enable PLL
	// Clock Source and Multiplication Factor must be selected in PLLCTRL
	// before setting this bit.
	OSC.CTRL |= OSC_PLLEN_bm; // 0x10
	//////////////////////////////////////////////////////////////////////
	
	//////////////////////////////////////////////////////////////////////
	//OSC.STATUS
	//     7       6       5        4        3         2          1          0
	// |   -   |   -   |   -   | PLLRDY | XOSCRDY | RC32KRDY | RC32MRDY | RC2MRDY |
	//     0       0       0        0        0         0          0          0
	// Wait for PLL to stabilize
	while (0 == (OSC.STATUS & OSC_PLLRDY_bm)); // wait for STATUS reg to read 0x10
	//////////////////////////////////////////////////////////////////////
	
	//////////////////////////////////////////////////////////////////////
	//CLK.PSCTRL
	//     7       6       5       4       3       2       1       0
	// |   -   |               PSADIV[4:0]             |    PSBCDIV    |
	//     0       0       0       0       0       0       0       0
	// Enable change on protected I/O reg
	CONFIGURATION_CHANGE_PROTECTION;
	// Set Clk_per4 equal to 8 MHz
	CLK.PSCTRL = CLK_PSADIV0_bm; // 0x04
	// Leaving CLK_PSBCDIV[1:0] with default value (b'00') sets
	// Clk_per2, Clk_per, and Clk_cpu equal to Clk_per4 (8 MHz)
	// Note: ADC is clocked from Clk_per
	//////////////////////////////////////////////////////////////////////
	
	//////////////////////////////////////////////////////////////////////
	//CLK.CTRL
	//     7       6       5       4       3       2       1       0
	// |   -   |   -   |   -   |   -   |   -   |      SCLKSEL[2:0]    |
	//     0       0       0       0       0       0       0       0
	// Enable change on protected I/O reg
	CONFIGURATION_CHANGE_PROTECTION;
	// Set PLL as sys clk source
	CLK.CTRL = CLK_SCLKSEL2_bm; // 0x04
	//////////////////////////////////////////////////////////////////////
	
	//////////////////////////////////////////////////////////////////////
	//CLK.RTCCTRL
	//     7       6       5       4       3       2       1       0
	// |   -   |   -   |   -   |   -   |      RTCSRC[2:0]     |  RTCEN  |
	//     0       0       0       0       0       0       0       0
	// LCD Runs off the RTC
	// Enable RTC, source from 32kHz internal ULP osc
	CLK.RTCCTRL = CLK_RTCEN_bm; // 0x01
	//////////////////////////////////////////////////////////////////////
}

