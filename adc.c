#include <avr/io.h>
#include "adc.h"

// These 2 files need to be included in order to read
// the production calibration values from EEPROM
#include <avr/pgmspace.h>
#include <stddef.h>

int16_t adc_compensate( int16_t adcvalue, int16_t factor, int32_t correction ){ 
 return (((((int32_t)adcvalue*factor)+correction)<<2)>>16); 
} //See AVR130: Characterization and Calibration of the ADC on an AVR


void adc_init(void) {
	
	// ADC Clock was disabled initially in sysclk_init()
	// Must re-activate the ADC clock before configuring its registers (we're using ADCA)
	PR.PRPA &= ~0x02; // Clear ADC bit in Power Reduction Port B Register

	// Calibration values are stored at production time
	// Load stored bytes into the calibration registers
	// First NVM read is junk and must be thrown away
	ADCA.CALL = ReadCalibrationByte( offsetof(NVM_PROD_SIGNATURES_t, ADCACAL0) );
	ADCA.CALH = ReadCalibrationByte( offsetof(NVM_PROD_SIGNATURES_t, ADCACAL1) );
	ADCA.CALL = ReadCalibrationByte( offsetof(NVM_PROD_SIGNATURES_t, ADCACAL0) );
	ADCA.CALH = ReadCalibrationByte( offsetof(NVM_PROD_SIGNATURES_t, ADCACAL1) );
	
	//////////////////////////////////////////////////////////////////////
	//ADCA.CH0.CTRL
	//     7        6       5        4        3        2       1       0
	// | START  |   -   |   -   |         GAIN[2:0]        | INPUTMODE[1:0] |
	//     0        0       0        0        0        0       0       0
	// Place ADC channel in single-ended mode
	// Default internal => Put into single-ended
	// Gain set to 1
	//ADCA.CH0.CTRL = ADC_CH_INPUTMODE0_bm; // 0x01 Single ended, no gain
	ADCA.CH0.CTRL = 0x02; // 0x02 Double ended, no gain
	//ADCA.CH0.CTRL = 0x04 | 0x03; // 0x03 Double ended+gain, 8x gain
	//////////////////////////////////////////////////////////////////////
	
	//////////////////////////////////////////////////////////////////////
	//ADCA.CH0.MUXCTRL 
	//     7        6       5        4        3        2       1       0
	// |   -    |           MUXPOS[3:0]             |     MUXNEG[2:0]     |
	//     0        0       0        0        0        0       0       0
	// Connect potentiometer (PB1) to positive input
	// MUXNEG bits are ignored in single-ended mode
	// ADCA.CH0.MUXCTRL = ADC_CH_MUXINT0_bm; // 0x08
	// NB: if we use gain, the Pad GND code becomes 0x04. Internal GND 0x07 for both
	//////////////////////////////////////////////////////////////////////
	
	//////////////////////////////////////////////////////////////////////
	//ADCA.CTRLB
	//     7        6       5         4         3         2       1        0
	// |   -   | CURRLIMIT[1:0] | CONVMODE | FREERUN | RESOLUTION[1:0] |   -   |
	//     0        0       0         0         0         0       0        0
	// Apply no limit to ADC sample rate
	// Put ADC in signed mode
	// Disable Free-run mode (single conversion upon trigger)
	// Resolution set to 12-bit, right justified (11-bit effective in signed mode)
	ADCA.CTRLB = ADC_CONMODE_bm ; // change from default to signed mode
	//ADCA.CTRLB &= ~ADC_CONMODE_bm; // ensure unsigned 
	//////////////////////////////////////////////////////////////////////
	
	//////////////////////////////////////////////////////////////////////
	//ADCA.PRESCALER
	//     7       6       5       4       3       2       1       0
	// |   -   |   -   |   -   |   -   |       |     PRESCALER[2:0]    |
	//     0       0       0       0       0       0       0       0
	// The ADC runs off of the CPU_per clock
	// In sys_clk_init() the internal 2MHz RC osc was used to source a 16 MHz PLL
	// The PLL is then divided using Prescalers A, B, and C setting CPU_per to 8 MHz
	// According to AVR1300, the ADC clock should run in the range 100 kHz ~ approx 1.4 MHz
	// Set ADC clock to 125kHz:  CPU_per/64    =>    8MHz/64 = 125kHz
	//ADCA.PRESCALER = ADC_PRESCALER2_bm; // 0x04
	ADCA.PRESCALER = 0x02; // 2MHz/32 = 125kHz
	//////////////////////////////////////////////////////////////////////
	
	//////////////////////////////////////////////////////////////////////
	//ADCA.REFCTRL
	//     7       6       5       4       3       2        1         0
	// |   -   |      REFSEL[2:0]      |   -   |   -   | BANDGAP | TEMPREF |
	//     0       0       0       0       0       0        0         0
	// Set Vref to Vcc/1.6.  This gives 3.3/1.6 = approx 2.06V
	// With effectively 11-bit resolution, this means each LSB 
	// will represent approximately 1 mV.
	//NB: we may want to switch to bandgap reference as most accurate,
	//    but this may require gain to be 1/2 to ensure no clipping
	ADCA.REFCTRL = ADC_REFSEL0_bm; // 0x10
	//////////////////////////////////////////////////////////////////////
	
	//////////////////////////////////////////////////////////////////////
	//ADCA.EVCTRL
	//     7       6       5       4       3       2       1       0
	// |   -   |   -   |   -   |   EVSEL[1:0]  |      EVACT[2:0]       |
	//     0       0       0       0       0       0       0       0
	// Not implementing Event System so ensure EVCTRL is reading zeros
	ADCA.EVCTRL = 0x00;
	//////////////////////////////////////////////////////////////////////
	
	//////////////////////////////////////////////////////////////////////
	//ADCA.INTFLAGS
	//     7       6       5       4       3       2       1       0
	// |   -   |   -   |   -   |   -   |   -   |   -   |   -   | CH0IF |
	//     0       0       0       0       0       0       0       0
	// Ensure the ADC complete flag is cleared (by writing a '1' to it)
	ADCA.INTFLAGS = ADC_CH0IF_bm; // 0x01
	//////////////////////////////////////////////////////////////////////
	
	//////////////////////////////////////////////////////////////////////
	//ADCA.CH0.INTCTRL
	//     7       6       5       4        3       2       1       0
	// |   -   |   -   |   -   |   -   |  INTMODE[1:0]  |  INTLVL[1:0]  |
	//     0       0       0       0        0       0       0       0
	// Configure interrupt on conversion complete with high priority
	ADCA.CH0.INTCTRL = ADC_CH_INTLVL1_bm | ADC_CH_INTLVL0_bm; // 0x03
	//////////////////////////////////////////////////////////////////////
	
	//////////////////////////////////////////////////////////////////////
	//ADCA.CTRLA
	//     7       6       5       4       3         2        1        0
	// |   -   |   -   |   -   |   -   |   -   | CH0START | FLUSH | ENABLE |
	//     0       0       0       0       0         0        0        0
	// Enable ADC, module B
	ADCA.CTRLA = ADC_ENABLE_bm; // 0x01
	//////////////////////////////////////////////////////////////////////
}

uint8_t ReadCalibrationByte(uint8_t index){
	uint8_t result;
	NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
	result = pgm_read_byte(index);
	NVM_CMD = NVM_CMD_NO_OPERATION_gc;
	return(result);
}