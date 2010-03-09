/*****************************************************************************************/
/* Filename : SBT80_ADCconfigC.nc                                                        */
/*****************************************************************************************/
/*                                                                                       */
/* This code is intended for the EasySen SBT80 Sensor board along with TelosB            */
/* family of wireless motes.                                                             */
/*                                                                                       */
/* Code samples ADC channel for acoustic sensor.                                         */ 
/*                                                                                       */
/*****************************************************************************************/




generic configuration SBT80_ADCconfigC() {
    provides {
        interface Read<uint16_t> as ReadADC1; //Acoustic 
    }
}
implementation {
    components new AdcReadClientC() as AdcRC1;
    ReadADC1 = AdcRC1;
    AdcRC1.AdcConfigure -> SBT80_ADCconfigP.ADC1;
    components SBT80_ADCconfigP;

}
