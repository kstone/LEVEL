/*****************************************************************************************/
/* Application : TeacherAcoustic                                                         */
/*****************************************************************************************/
/*                                                                                       */
/* This application is the teacher component of the LEVEL-ByStudent System.               */
/*                                                                                       */
/*                                                                                       */
/*                                                                                       */
/*                                                                                       */
/*****************************************************************************************/

TODO: this README is out of date...once we have functionality dialed, we need to update this.

The application begins by running a calibration mode for 30 seconds to detect the parameters of the ambient
background.  During this time, there should be no talking being done by the teacher.

After calibration, the LED will no longer show blue, and the teacher may begin teaching.  The mote will detect 
when the teacher is talking.  Once the teacher stops talking, the mote will inform all student motes (StudentAcoustic)
of when the teacher ceased talking and when they should give the student a green light to talk.

Once the teacher begins talking again, the application will inform all student motes that they should switch to Red.


Includes the printf client which allows user to see output in terminal.

$ make telosb install
$ motelist
  output = <port>
$ java net.tinyos.tools.PrintfClient -comm serial@<port>:telosb
