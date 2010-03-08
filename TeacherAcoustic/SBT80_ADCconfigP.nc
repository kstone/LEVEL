/*****************************************************************************************/
/* Filename : SBT80_ADCconfigP.nc                                                        */
/*****************************************************************************************/
/*                                                                                       */
/* This code is intended for the EasySen SBT80 Sensor board along with TelosB            */
/* family of wireless motes.                                                             */
/*                                                                                       */
/* Code samples ADC channel for acoustic sensor.                                         */ 
/*                                                                                       */
/*****************************************************************************************/


#include "Msp430Adc12.h"


module SBT80_ADCconfigP {
    provides interface AdcConfigure<const msp430adc12_channel_config_t*> as ADC1;
}

implementation { 
    const msp430adc12_channel_config_t config_ADC1 = {
            inch:		INPUT_CHANNEL_A1,
            sref: 		REFERENCE_VREFplus_AVss,
            ref2_5v: 	REFVOLT_LEVEL_2_5,
            adc12ssel: 	SHT_SOURCE_ACLK,
            adc12div:	SHT_CLOCK_DIV_1,
            sht: 		SAMPLE_HOLD_4_CYCLES,
            sampcon_ssel: 	SAMPCON_SOURCE_SMCLK,
            sampcon_id: 	SAMPCON_CLOCK_DIV_1
    };

    async command const msp430adc12_channel_config_t* ADC1.getConfiguration() {
        return &config_ADC1;
    }

}
