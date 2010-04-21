/*****************************************************************************************/ 
/* Filename : TeacherAcousticC.nc                                                           */
/*****************************************************************************************/
/*                                                                                       */
/* This code is intended for the EasySen SBT80 Sensor board along with TelosB            */
/* family of wireless motes.                                                             */
/*                                                                                       */
/* Code samples ADC channel for acoustic sensor.                                         */ 
/*                                                                                       */
/* TODO: print statements on lines 114 and 125 are throwing warnings, fix at some point 
 * 
 * Doug -- The easiest way to test this in its current form is to install TeacherAcoustic  
 * on 1 mote, install BaseStation on a second and leave BaseStation connected to Comp.   
 * Set your MOTECOM variable appropriately for your base station mote      
 * Run the Java listener tool: java net.tinyos.tools.Listen                             
 *
 * Printf statements are in the code to assist in debugging.  To see calibration stats:
 * To see the printf statements on the screen, attach the TeacherAcoustic mote to comp,
 * Set your MOTECOM variable appropriately for your TeacherAcoustic mote,
 * run the Java PrintF application: java net.tinyos.tools.PrintfClient 
 *
 * TeacherAcoustic operation:
 *  1.  Calibrates the acoustic sensor for the current environment
 *  2.  Sends either red packet 0000dead or green packet 1111dead (broadcast)
 *      packet designates turn on red LED if contents is 0000dead
 *      packet designates turn on green LED if contents is 1111dead
 *      (TODO: determine if we want all motes to default to red, and for acoustic 
 *      packets to only be sent after the user button is pressed.)                       
 *      If sending green packet, TeacherAcoustic motes green LED blinks, 
 *      If sending red packet, TeacherAcoustic motes red LED blinks.                       */
/*****************************************************************************************/


#include "Timer.h"
#include "LEVEL.h"
#include "SBT80ADCmap.h"
#include "printf.h"
#include "math.h"
#include "UserButton.h"

module TeacherAcousticC
{
    uses {
        interface Boot;
        interface Leds;
        interface Timer<TMilli> as WakeupTimer;
        interface Timer<TMilli> as CalibrationTimer;
        interface Timer<TMilli> as SpeechTimer;
        interface Timer<TMilli> as UniversalTimer;

        interface Read<uint16_t> as ReadMIC;

        interface HplMsp430GeneralIO as SBcontrol;

        interface Packet;
        interface AMPacket;
        interface AMSend;
        interface SplitControl as AMControl;

	interface Notify<button_state_t>;
    }

}


implementation
{

    /* Define variables and constants */  

    #define SAMPLING_FREQUENCY 100  /* should be higher than 100 ms */
    #define CALIBRATION_TIME 150      /* Times we will sample for calibration period on boot */
    #define SPEECH_TIMER 5000          /* TODO: come up with a better way, or a reason for making this 10 
                                        (this is the delay before assuming teacher is done talking) */
    #define UNIVERSAL_TIMER 1000000   /* Basically creating a reference frame for events.  Should be large enough that there will be no collisions by wraparound */
    #define HAND_RAISE_DELAY 8000  /* Time (in ms) to delay before giving the kids a green to raise their hands after the teach stops talking. */
    #define SHORT_HAND_RAISE_DELAY 500 /* Time (in ms) to delay before giving kids a green.  Used for the userbutton short circuit. */

    #define TALKING_DEVIATIONS 1.5 /* Number of standard deviations away from the mean is representative that talking is occuring */
    uint16_t ChannelNo = 0; 

    /* global variable to hold sensor readings */ 
    uint16_t MICdata;

    uint8_t len;
    bool busy = FALSE;  /* used to keep track if radio is busy */
    message_t pkt;
    void task getData();
    
    void task sendRedPacket();
    void task sendGreenPacket();
    void task sendDelayGreenPacket();
    uint32_t greenPacketDelay=0;

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
    uint8_t outlierCount = 0;//Outlier Count will make it so that it has to see two "outside" values in a row to trigger.  This is in conjunction with increqasing the sampling frequency.

    /* -------- Initializations at powerup --------------- */  
    event void Boot.booted() {
        call AMControl.start();

        /* Wake up the sensor board */
        call SBcontrol.clr();
        call SBcontrol.makeOutput();
        call SBcontrol.selectIOFunc();
	call Notify.enable();
 
        /* Send a broadcast red packet out */
        post sendRedPacket(); 

        /* Calibrate the acoustic sensor for the current environment */
        post calibrateAcoustic();

        call UniversalTimer.startPeriodic(UNIVERSAL_TIMER);

    }


    /* -------- EVENT: WakeupTimer fired --------------- */
    event void WakeupTimer.fired(){
        ChannelNo = 0;
        post getData();

    }

    /* -------- EVENT: UniversalTimer fired ----------------*/
    event void UniversalTimer.fired(){
      //Do absolutely nothing.  We are simply using this timer for its getNow() commands.
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
        /* Teacher is not talking, turn students lights green...Delay is accounted for on student mote */
        post sendGreenPacket();
	
	/* Turn off debug blue led as well. */
 	call Leds.led2Off();
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

    /* -------- TASK: sendRedPacket - send a broadcast packet out with instructions to turn on red LED --------------- */        
    void task sendRedPacket(){
        /* led0 is red...red packet will have payload of all 0's in ChannelNo 0 */
        SenseToRadioMsg* ptrpkt = (SenseToRadioMsg*)(call Packet.getPayload(&pkt, len));
        ptrpkt->data[ChannelNo] = RED_PACKET_FLAG;
        ptrpkt->data[ChannelNo+1] = DEAD;  /* This is for error checking */

        /* TODO: implement message queue when radio is busy */
        if (!busy) {
            if (call AMSend.send(AM_BROADCAST_ADDR, &pkt, sizeof(SenseToRadioMsg)) == SUCCESS) {
                call Leds.led0On();
                busy = TRUE;
            }
        }
    }

    /* -------- TASK: sendGreenPacket - send a broadcast packet out with universal time and instructions to turn on green LED --------------- */
    void task sendGreenPacket(){
        /* led1 is green...green packet will have payload of all 1's in ChannelNo 0 */
        SenseToRadioMsg* ptrpkt = (SenseToRadioMsg*)(call Packet.getPayload(&pkt, len));
        ptrpkt->data[ChannelNo] = GREEN_PACKET_FLAG;
        ptrpkt->data[ChannelNo +1] = DEAD;  /* This is for error checking */
        ptrpkt->data[ChannelNo +2] = call UniversalTimer.getNow();
        ptrpkt->data[ChannelNo +3] = ptrpkt->data[ChannelNo + 2]+HAND_RAISE_DELAY;
        /* TODO: implement message queue when radio is busy */
        if (!busy) {
            if (call AMSend.send(AM_BROADCAST_ADDR, &pkt, sizeof(SenseToRadioMsg)) == SUCCESS) {
                call Leds.led1On();
                busy = TRUE;
            }
        }
    }
    

   /* -------- TASK: sendDelayGreenPacket - send a broadcast packet out with parameterized delay and instructions to turn on green LED --------------- */
    void task sendDelayGreenPacket(){
        /* led1 is green...green packet will have payload of all 1's in ChannelNo 0 */
        SenseToRadioMsg* ptrpkt = (SenseToRadioMsg*)(call Packet.getPayload(&pkt, len));
        ptrpkt->data[ChannelNo] = GREEN_PACKET_FLAG;
        ptrpkt->data[ChannelNo +1] = DEAD;  /* This is for error checking */
        ptrpkt->data[ChannelNo +2] = call UniversalTimer.getNow();
        ptrpkt->data[ChannelNo +3] = ptrpkt->data[ChannelNo + 2]+greenPacketDelay;
        /* TODO: implement message queue when radio is busy */
        if (!busy) {
            if (call AMSend.send(AM_BROADCAST_ADDR, &pkt, sizeof(SenseToRadioMsg)) == SUCCESS) {
                call Leds.led1On();
                busy = TRUE;
            }
        }
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
		if (outlierCount>=1){
                  /* Students lights should be red, the teacher is talking */
                  post sendRedPacket();  /* we probably don't need to send this packet with every readying...we will for now */

                  /* If the countdown timer to turn students lights green was started, stop it because the teacher is still talking */

		  /* For debug, turn on the blue led when "Talking" is detected" */
		  call Leds.led2On();
		  outlierCount=0;

                  if(call SpeechTimer.isRunning()){
                      printf("SpeechTimer was running, but still talking...turning timer off.\n");
                      call SpeechTimer.stop();
		  }
                }
		else{
		  outlierCount+=1;
		}
            }
            else{
		outlierCount=0;
                /* Need to keep track of how many samples come back in this range...and turn students lights green at the appropriate time.  */
                /* TODO: this is currently dependant on the sampling rate, SAMPLING_FREQUENCY, this is not ideal */
                if(call SpeechTimer.isRunning()){
                    /* The timer is already running...no need to start it...Do we need this? Most likely not //DH */
                }
                else{
                    printf("Sarting the SpeechTimer.\n");
                    call SpeechTimer.startOneShot(SPEECH_TIMER);
                }
            }

        }

    }


    /* -------- EVENT: User Button Pressed or released.  --------------- */
    event void Notify.notify( button_state_t val ) {
	if (val == BUTTON_PRESSED)
	{
        	post sendRedPacket();
	}
	if (val == BUTTON_RELEASED)
	{
		//Send a Green Packet for sometime soon.	
		greenPacketDelay=SHORT_HAND_RAISE_DELAY;
        	post sendDelayGreenPacket();

	}
    }



}
