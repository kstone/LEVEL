/*****************************************************************************************/
/* Filename : SenseAcousticAppC.n                                                         */
/*****************************************************************************************/
/*                                                                                       */
/* This code is intended for the EasySen SBT80 Sensor board along with TelosB            */
/* family of wireless motes.                                                             */
/*                                                                                       */
/* Code samples ADC channel for acoustic sensor.                                         */ 
/*                                                                                       */
/*****************************************************************************************/



#include "Timer.h"
#include "SenseAcoustic.h"
#include "printf.h"

configuration SenseAcousticAppC 
{ 
} 

implementation { 

    #define CONFIG_AVCC 

    components SenseAcousticC;
    components MainC;
    components LedsC;
    components new TimerMilliC() as WakeupTimer;
    components new TimerMilliC() as CalibrationTimer;
    components new TimerMilliC() as SpeechTimer;

    SenseAcousticC.Boot -> MainC;
    SenseAcousticC.Leds -> LedsC;
    SenseAcousticC.WakeupTimer -> WakeupTimer;
    SenseAcousticC.CalibrationTimer -> CalibrationTimer;
    SenseAcousticC.SpeechTimer -> SpeechTimer;

    components HplMsp430GeneralIOC;
    SenseAcousticC.SBcontrol -> HplMsp430GeneralIOC.Port23;


    // --------- ADC related ---------
    components new SBT80_ADCconfigC() as MIC;

    SenseAcousticC.ReadMIC		-> MIC.ReadADC1;


    // --------- Message related ---------
    components ActiveMessageC;
    components new AMSenderC(AM_SENSETORADIOMSG);

    SenseAcousticC.Packet -> AMSenderC;
    SenseAcousticC.AMPacket -> AMSenderC;
    SenseAcousticC.AMSend -> AMSenderC;
    SenseAcousticC.AMControl -> ActiveMessageC;


}
