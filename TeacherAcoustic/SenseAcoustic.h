/*****************************************************************************************/
/* Filename : SenseAcoustic.h                                                             */
/*****************************************************************************************/
/*                                                                                       */
/* This code is intended for the EasySen SBT80 Sensor board along with TelosB            */
/* family of wireless motes.                                                             */
/*                                                                                       */
/* Code samples ADC channel for acoustic sensor.                                         */ 
/*                                                                                       */
/*****************************************************************************************/

#ifndef SENSEACOUSTIC_H
#define SENSEACOUSTIC_H

enum {
    AM_SENSETORADIOMSG = 6,
    BUFFER_SIZE = 8,
};

typedef nx_struct SenseToRadioMsg {
    nx_uint16_t nodeid;
    nx_uint16_t counter;
    nx_uint16_t data[BUFFER_SIZE];
} SenseToRadioMsg;

#endif

