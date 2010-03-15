/*****************************************************************************************/
/* Filename : TrafficLightAppC.n                                                         */
/*****************************************************************************************/
/*                                                                                       */
/*                                                                                       */
/*  Code is meant to control the traffic light based upon packets received.              */
/*  Red Packet received -> Light turned to yellow, then 1 second later, to Red           */
/*  Green Packet received -> Timer set based on information in packet, at which light    */
/*    turns to green.                                                                    */
/*                                                                                       */
/*****************************************************************************************/



#include "Timer.h"
#include "LEVEL.h"
#include "printf.h"

configuration TrafficLightAppC 
{ 
} 

implementation { 

    #define CONFIG_AVCC 

    components TrafficLightC;
    components MainC;
    components LedsC;
    components new TimerMilliC() as GreenLightTimer;//Shift from Red to Green.
    components new TimerMilliC() as RedLightTimer;//Shift from Yellow to Red
    


// Thanks to Alan Marchiori for help with the GeneralIOC code.
     components HplMsp430GeneralIOC as GeneralIOC;
     components new Msp430GpioC() as Red;
     components new Msp430GpioC() as Green;
     components new Msp430GpioC() as Yellow;


     // this wires to port 6.0, generalIOC has ALL of the telosb pins defined
     Red -> GeneralIOC.Port61;
     TrafficLightC.Red -> Red;

     Green -> GeneralIOC.Port62;
     TrafficLightC.Green -> Green;

     Yellow -> GeneralIOC.Port60;
     TrafficLightC.Yellow -> Yellow;
     //End thanks :)

    TrafficLightC.Boot -> MainC;
    TrafficLightC.Leds -> LedsC;
    TrafficLightC.GreenLightTimer -> GreenLightTimer;
    TrafficLightC.RedLightTimer -> RedLightTimer;

    // --------- Message related ---------
    components ActiveMessageC;
    components new AMSenderC(AM_SENSETORADIOMSG);
    components new AMReceiverC(AM_SENSETORADIOMSG);

    TrafficLightC.Packet -> AMSenderC;
    TrafficLightC.AMPacket -> AMSenderC;
    TrafficLightC.AMSend -> AMSenderC;
    TrafficLightC.AMControl -> ActiveMessageC;
    TrafficLightC.Receive -> AMReceiverC;


}
