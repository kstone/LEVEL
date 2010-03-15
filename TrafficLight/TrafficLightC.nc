/*****************************************************************************************/ 
/* Filename : TrafficLightC.nc                                                         */
/*****************************************************************************************/
/*                                                                                       */
/* Operation: Listens to the radio.  Whenever a green packet is received, starts
*  a timer based on the time specified in that packet.  On timer fire, turn off red light
*   and turn on the green light.
*   Whenever a red packet is received, turn off the green light, turn on the red light.
*/ 
/*****************************************************************************************/


#include "Timer.h"
#include "LEVEL.h"
#include "printf.h"
#include "math.h"

module TrafficLightC
{
    uses {
        interface Boot;
        interface Leds;
        interface Timer<TMilli> as GreenLightTimer;
        interface Timer<TMilli> as RedLightTimer;

        interface Packet;
        interface AMPacket;
        interface AMSend;
        interface Receive;
        interface SplitControl as AMControl;

	//Thanks to Alan Marchiori for help with the GeneralIOC code.
	interface GeneralIO as Red;
	interface GeneralIO as Green;
	interface GeneralIO as Yellow;

    }

}


implementation
{

    uint16_t ChannelNo = 0; 
    #define YELLOW_LENGTH 1000  //How long the yellow should be on before Red Comes
    uint8_t len;
    bool busy = FALSE;  /* used to keep track if radio is busy */
    message_t pkt;

    /* -------- Initializations at powerup --------------- */  
    event void Boot.booted() {
        call AMControl.start();
	call Leds.led0On();

	//Thanks to Alan for help with GeneralIO code.
	call Red.makeOutput();
	call Green.makeOutput();
	call Yellow.makeOutput();
	//End Thanks.

	//Turn off all lights (active low, so clr turns on, set turns off)
	call Red.set();
	call Yellow.set();
	call Green.set();

	//Turn on the Red light 
	call Red.clr();		

    }

    event void GreenLightTimer.fired(){
	//Turn on green LED
	call Leds.led1On();
	//Turn off red LED
	call Leds.led0Off();
	
	call Red.set();//Turn off red.
	call Green.clr();//Turn on green.
	call Yellow.set();//Turn off Yellow
    }	
    event void RedLightTimer.fired(){
	call Leds.led0On();//Turn on Red
	call Leds.led1Off();//Turn off Green
	
	call Red.clr();//Turn on Red
	call Green.set();//Turn off green.
	call Yellow.set();//Turn off yellow
    }	


    event void AMControl.startDone(error_t err) {
    }

    event void AMControl.stopDone(error_t err) {
    }

    /* -------- EVENT: sendDone fired --------------- */
    event void AMSend.sendDone(message_t* msg, error_t error) {
        if (&pkt == msg) {
            busy = FALSE;
            /* Turn off both red and green LED...not really concerned with what packet just sent */
            call Leds.led0Off();
            call Leds.led1Off();
        }
    }

    event  message_t* Receive.receive(message_t* msg, void* payload, uint8_t length){
        SenseToRadioMsg* ptrpkt = (SenseToRadioMsg*)payload;

	/* DEBUG only - flash the blue light to indicate a packet received. */
        call Leds.led2On();
        call Leds.led2Off();
	/* END DEBUG only */
        if (ptrpkt->data[ChannelNo]==RED_PACKET_FLAG){
		call GreenLightTimer.stop();//Stop the green light timer if it is running.
		if (!call RedLightTimer.isRunning())
		{
        		call Leds.led0On();//Turn on the red led
	       		call Leds.led1Off();//Turn off the green led
			call Yellow.clr();//If the red light timer is not running, put yellow on.
			call Green.set();//Turn green off
				
			call RedLightTimer.startOneShot(YELLOW_LENGTH);//Start a timer to turn off the yellow and turn on the red.
		}


        }
	else if (ptrpkt->data[ChannelNo]==GREEN_PACKET_FLAG){
                if (call GreenLightTimer.isRunning())
		{
			/*  Do nothing? */
		}
		else
		{
			call GreenLightTimer.startOneShot((ptrpkt->data[ChannelNo+3])-(ptrpkt->data[ChannelNo+2]));//Start a timer for when the green light should turn on.	
		}
	}
	else
	{
		//What happened?
		call Leds.led2On();//Turn on the blue light to indicate an error.
	}

        return msg;        

    }



}
