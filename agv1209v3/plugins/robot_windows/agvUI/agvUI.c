#include <webots/camera.h>
#include <webots/motor.h>
#include <webots/plugins/robot_window/robot_wwi.h>
#include <webots/robot.h>
#include <webots/position_sensor.h>
#include <math.h>
#include <stdio.h>

#define TIME_STEP 10

//command check bits
static bool stopAgv = true;
static bool jog = false;
char defaultStation = "A";

//device tags
void wb_robot_window_init() {

  WbDeviceTag Lmotor;
  Lmotor = wb_robot_get_device("Lmotor");
  WbDeviceTag Rmotor;
  Rmotor = wb_robot_get_device("Rmotor");

  WbDeviceTag encoderL;
  encoderL = wb_robot_get_device("sensorL");
  WbDeviceTag encoderR;
  encoderR = wb_robot_get_device("sensorR");
  wb_position_sensor_enable(encoderL, TIME_STEP);
  wb_position_sensor_enable(encoderR, TIME_STEP);
  

}

// A simulation step occurred.
void wb_robot_window_step(int time_step) {
  //receive msg from UI
  const char *message;
  while ((message = wb_robot_wwi_receive_text())) {
  /* MESSAGE FORMAT: msg = "I X J"
  X - destination: A, B, ...
  I - start/stop: 1/0;
  J - jog cmd: Y/N;
  */
  //if press stop button -> stopAGV = true; 
  //if press start button -> stopAGV = false; 
  //if press jog -> stopAGV = false; jog = true;

 if (strcmp(message[1], "1") == 0) {
  stopAGV = false;
   }
 if (strcmp(message[1], "0") == 0) {
   stopAGV = true;
  }
if (strcmp(message[5], "1") == 0) {
  jog = true;
  }
if (strcmp(message[5], "1") == 0) {
  jog = false;
  }
// check destination bit here
desStation = defaultStation;
/*
*/
  }

  //stop button
  if (stopAGV) {
    wb_motor_set_velocity(Lmotor, 0.0);
    wb_motor_set_velocity(Rmotor, 0.0);
    }
 //jog button
  if (jog) {
    wb_motor_set_velocity(Lmotor, 5);
    wb_motor_set_velocity(Rmotor, 5);
    }
//send msg: wR, wL, linePos, v(?)
}

void wb_robot_window_cleanup() {
  // This is called when the robot window is destroyed.
  // There is nothing to do here in this example.
  // This callback can be used to store information.
}


