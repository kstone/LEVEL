/*****************************************************************************************/
/* Filename : TeacherAcousticAppC.nc                                                         */
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

configuration TeacherAcousticAppC 
{ 
} 

implementation { 

    #define CONFIG_AVCC 

    components TeacherAcousticC;
    components MainC;
    components LedsC;
    components new TimerMilliC() as WakeupTimer;
    components new TimerMilliC() as CalibrationTimer;
    components new TimerMilliC() as SpeechTimer;
    components new TimerMilliC() as UniversalTimer;
   

    TeacherAcousticC.Boot -> MainC;
    TeacherAcousticC.Leds -> LedsC;
    TeacherAcousticC.WakeupTimer -> WakeupTimer;
    TeacherAcousticC.CalibrationTimer -> CalibrationTimer;
    TeacherAcousticC.SpeechTimer -> SpeechTimer;
    TeacherAcousticC.UniversalTimer -> UniversalTimer;

    components HplMsp430GeneralIOC;
    TeacherAcousticC.SBcontrol -> HplMsp430GeneralIOC.Port23;


    // --------- ADC related ---------
    components new SBT80_ADCconfigC() as MIC;

    TeacherAcousticC.ReadMIC		-> MIC.ReadADC1;


    // --------- Message related ---------
    components ActiveMessageC;
    components new AMSenderC(AM_SENSETORADIOMSG);

    TeacherAcousticC.Packet -> AMSenderC;
    TeacherAcousticC.AMPacket -> AMSenderC;
    TeacherAcousticC.AMSend -> AMSenderC;
    TeacherAcousticC.AMControl -> ActiveMessageC;

    components UserButtonC;
    TeacherAcousticC.Notify -> UserButtonC;


}
