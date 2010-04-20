/*****************************************************************************************/ 
/* Filename : StudentAcousticC.nc                                                         */
/*****************************************************************************************/
/*                                                                                       */
/* This code is intended for the EasySen SBT80 Sensor board along with TelosB            */
/* family of wireless motes.                                                             */
/*                                                                                       */
/* Code samples ADC channel for acoustic sensor.                                         */ 
/*                                                                                       */
/* TODO: print statements on lines 114 and 125 are throwing warnings, fix at some point 
 *
 * Printf statements are in the code to assist in debugging.  To see calibration stats:
 * To see the printf statements on the screen, attach the StudentAcoustic mote to comp,
 * Set your MOTECOM variable appropriately for your StudentAcoustic mote,
 * run the Java PrintF application: java net.tinyos.tools.PrintfClient 
 *
 * StudentAcoustic operation:
 *  1.  Calibrates the acoustic sensor for the current environment
 *  2.  Listens for packets from the base station (no relaying)
 *   2a.  If red packet received, turns only red led on.
 *   2b.  If greed packet received, turns only green led on at specified time.
 *
 *  Adding protocol changes as follows
 *    Getting ACKS from BMAC
 *    
 *
 */
/*****************************************************************************************/


#include "Timer.h"
#include "LEVEL.h"
#include "SBT80ADCmap.h"
#include "printf.h"
#include "math.h"

module StudentAcousticC
{
    uses {
        interface Boot;
        interface Leds;
        interface Timer<TMilli> as WakeupTimer;
        interface Timer<TMilli> as CalibrationTimer;
        interface Timer<TMilli> as SpeechTimer;
        interface Timer<TMilli> as GreenLightTimer;

        interface Read<uint16_t> as ReadMIC;

        interface HplMsp430GeneralIO as SBcontrol;

        interface Packet;
        interface AMPacket;
        interface AMSend;
        interface Receive;
        interface SplitControl as AMControl;

    }

}


implementation
{

    /* Define variables and constants */  

    #define SAMPLING_FREQUENCY 100  /* should be higher than 100 ms */
    #define CALIBRATION_TIME 150      /* Times we will sample for calibration period on boot */
    #define SPEECH_TIMER 10000          /* TODO: come up with a better way, or a reason for making this 10 
                                        (this is the delay before assuming teacher is done talking) */


    #define TALKING_DEVIATIONS 1.5 /* Number of standard deviations away from the mean is representative that talking is occuring */

    #define ADDITIVE_DECREASE 5000 /* ms which the student delay should decrease for a round of no talking. */
    #define MULTIPLICATIVE_INCREASE 1.5 /* Rate at which delay should increase when the student does participate */

    #define RED_STATE 0
    #define GREEN_STATE 2

    uint16_t ChannelNo = 0; 

    /* global variable to hold sensor readings */ 
    uint16_t MICdata;

    uint8_t len;
    bool busy = FALSE;  /* used to keep track if radio is busy */
    message_t pkt;
    void task getData();
    
    void task calibrateAcoustic();
    bool calibrating = FALSE; /* used to keep track of what sensor data should be used for calibration */
    uint32_t calibrationData = 0;
    uint32_t calibrationArray[CALIBRATION_TIME];
    uint32_t calibrationAverage = 0;
    uint32_t calibrationIndex = 0;
    uint32_t index = 0;
    uint32_t stddev = 0;
    uint32_t upperBound = 0;
    uint32_t lowerBound = 0;
    uint32_t studentDelay = 0;
    uint8_t state = RED_STATE;
    uint8_t hasParticipatedLastRound = 0;
    uint32_t baseDelayLength=0;
    uint8_t outlierCount = 0;//Outlier Count will make it so that it has to see two "outside" values in a row to trigger.  This is in conjunction with increqasing the sampling frequency.
    /* -------- Initializations at powerup --------------- */  
    event void Boot.booted() {
        call AMControl.start();

        /* Wake up the sensor board */
        call SBcontrol.clr();
        call SBcontrol.makeOutput();
        call SBcontrol.selectIOFunc();
 
        /* Calibrate the acoustic sensor for the current environment */
        post calibrateAcoustic();

    }


    /* -------- EVENT: WakeupTimer fired --------------- */
    event void WakeupTimer.fired(){
        ChannelNo = 0;
        post getData();

    }

    /* -------- EVENT: CalibrationTimer fired --------------- */
    event void CalibrationTimer.fired(){
        if(calibrationIndex < CALIBRATION_TIME){
            post getData();
            calibrationIndex++;
            //printf("Calibration index: %d\n", calibrationIndex);
            call CalibrationTimer.startOneShot(SAMPLING_FREQUENCY);
        }
        else{
            calibrating = FALSE;
            call Leds.led2Off();

            calibrationAverage = calibrationData / calibrationIndex;
            printf("Calibration average: %u\n", calibrationAverage);

            stddev = 0;

            /* Compute the standard deviation of the calibrated data */ 
            while(index < calibrationIndex){
                stddev += (calibrationAverage - calibrationArray[index]) * (calibrationAverage - calibrationArray[index]);
                index++;
            }
            stddev = stddev / (calibrationIndex);
            stddev = sqrtf(stddev);     /** use sqrtf instead of sqrt b/c telosb microcontroller supports single precision variants of math functions **/
            printf("Calibration stddev: %4u\n", stddev);

            upperBound = calibrationAverage + (TALKING_DEVIATIONS * stddev);
            lowerBound = calibrationAverage - (TALKING_DEVIATIONS * stddev);

            /* Start sampling the acoustic sensor at sampling rate */
            call WakeupTimer.startPeriodic(SAMPLING_FREQUENCY);
        }
    }

    /* -------- EVENT: SpeechTimer fired --------------- */
    event void SpeechTimer.fired(){
       //TODO - update the time to delay for next time he talks.
    }

    event void GreenLightTimer.fired(){
	call Leds.led1On();//Turn on the green light
	call Leds.led0Off();//Turn off the red light
	state = GREEN_STATE;
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

        if(calibrating){//Ignore packets while calibrating
	  return msg;
	}
    

        if (ptrpkt->data[ChannelNo]==RED_PACKET_FLAG){
		call GreenLightTimer.stop();//Stop the green light timer if it is running.
        	call Leds.led0On();//Turn on the red light
       		call Leds.led1Off();//Turn off the green light
		if (state == GREEN_STATE){
			state = RED_STATE;
			
			//Now perform actions based upon whether student participated last time.
			if (hasParticipatedLastRound==0){ // Did not participate
				if (studentDelay > ADDITIVE_DECREASE)
					studentDelay-=ADDITIVE_DECREASE;
				else
					studentDelay=0;
			}
			else{ //Did participate
				studentDelay=(studentDelay+baseDelayLength)*MULTIPLICATIVE_INCREASE-baseDelayLength;
			}
			hasParticipatedLastRound = 0;//Reset participation flag for next round.
			call Leds.led2Off();//Turn off blue to indicate round has passed.

		}
        }
	else if (ptrpkt->data[ChannelNo]==GREEN_PACKET_FLAG){
                if (call GreenLightTimer.isRunning())
		{
			/*  Do nothing? */
		}
		else
		{
			baseDelayLength=(ptrpkt->data[ChannelNo+3])-(ptrpkt->data[ChannelNo+2]);//Set the base delay length.
			call GreenLightTimer.startOneShot(baseDelayLength+studentDelay);//Start a timer for when the green light should turn on.	
		}
	}
	else
	{
		//What happened?
		call Leds.led2On();//Turn on the blue light to indicate an error.
	}

        return msg;        

    }


    /* -------- TASK: calibrateAcoustic - calibrate the acoustic sensor --------------- */
    void task calibrateAcoustic(){
        calibrationIndex = 0;
        calibrationData = 0;
        calibrationAverage = 0;
        calibrating = TRUE;
        call Leds.led2On();
        call CalibrationTimer.startOneShot(SAMPLING_FREQUENCY);
    }

    /* -------- TASK: getData - sample the acoustic sensor --------------- */
    void task getData(){
        call ReadMIC.read();
    }


    /* -------- EVENT: MIC channel sampling done - Check against calibrated boundaries  --------------- */
    event void ReadMIC.readDone(error_t result, uint16_t data) {
        if(calibrating){
            calibrationData = calibrationData + data;
            calibrationArray[calibrationIndex] = data;
            printf("%4u\n", data);
        }
        else{
            //MICdata = data;
            if(data < lowerBound || data > upperBound){
		if (state == GREEN_STATE && outlierCount==1)
		{
			hasParticipatedLastRound=1;
			call Leds.led2On();
		}
                outlierCount=1;		

            }
	    else
	    {
		outlierCount=0;
            }

        }

    }

}
