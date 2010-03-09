/*****************************************************************************************/
/* Filename : StudentAcousticAppC.n                                                         */
/*****************************************************************************************/
/*                                                                                       */
/* This code is intended for the EasySen SBT80 Sensor board along with TelosB            */
/* family of wireless motes.                                                             */
/*                                                                                       */
/* Code samples ADC channel for acoustic sensor.                                         */ 
/*                                                                                       */
/*****************************************************************************************/



#include "Timer.h"
#include "LEVEL.h"
#include "printf.h"

configuration StudentAcousticAppC 
{ 
} 

implementation { 

    #define CONFIG_AVCC 

    components StudentAcousticC;
    components MainC;
    components LedsC;
    components new TimerMilliC() as WakeupTimer;
    components new TimerMilliC() as CalibrationTimer;
    components new TimerMilliC() as SpeechTimer;
    components new TimerMilliC() as GreenLightTimer;

    StudentAcousticC.Boot -> MainC;
    StudentAcousticC.Leds -> LedsC;
    StudentAcousticC.WakeupTimer -> WakeupTimer;
    StudentAcousticC.CalibrationTimer -> CalibrationTimer;
    StudentAcousticC.SpeechTimer -> SpeechTimer;
    StudentAcousticC.GreenLightTimer -> GreenLightTimer;

    components HplMsp430GeneralIOC;
    StudentAcousticC.SBcontrol -> HplMsp430GeneralIOC.Port23;


    // --------- ADC related ---------
    components new SBT80_ADCconfigC() as MIC;

    StudentAcousticC.ReadMIC		-> MIC.ReadADC1;


    // --------- Message related ---------
    components ActiveMessageC;
    components new AMSenderC(AM_SENSETORADIOMSG);
    components new AMReceiverC(AM_SENSETORADIOMSG);

    StudentAcousticC.Packet -> AMSenderC;
    StudentAcousticC.AMPacket -> AMSenderC;
    StudentAcousticC.AMSend -> AMSenderC;
    StudentAcousticC.AMControl -> ActiveMessageC;
    StudentAcousticC.Receive -> AMReceiverC;


}
