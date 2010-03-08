/*****************************************************************************************/
/* Application : SenseAcoustic                                                        */
/*****************************************************************************************/
/*                                                                                       */
/* This code is intended for the EasySen SBT80 Sensor board along with TelosB            */
/* family of wireless motes.                                                             */
/*                                                                                       */
/* Code samples ADC channel for acoustic sensor.                                         */ 
/*                                                                                       */
/*****************************************************************************************/

TODO: this README is out of date...once we have functionality dialed, we need to update this.

Simple program to sample ADC1 channel (acoustic sensor on SBT80 sensor board) and TX 
packet.  

Includes the printf client which allows user to see output in terminal.

$ make telosb install
$ motelist
  output = <port>
$ java net.tinyos.tools.PrintfClient -comm serial@<port>:telosb
