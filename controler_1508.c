/*
 * File:          controler_1508.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include <webots/robot.h>
#include <webots/camera.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/supervisor.h>


/*
 * You may want to add macros here.
 */
#define TIME_STEP 10

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
// dynamic model of bldcs


const double dt = 0.01;
/*
double x1L = 0;
double x2L = 0;
double x1R = 0;
double x2R = 0;
double LmotorTF(double omega, double v) {
    double x2dot = 512*x1L;
    double x1dot = -514.2857*x1L -930.757*x2L + 16*v;
    x1L = x1L + x1dot*dt;
    x2L = x2L + x2dot*dt;
    omega = 25.9157*x2L;
  return omega;
  }
  
double RmotorTF(double omega, double v) {
    //double omegaDot = (-497.1976)*omega + (241.8188)*v;
    double x2dot = 512*x1R;
    double x1dot = -514.2857*x1R -930.757*x2R + 16*v;
    x1R = x1R + x1dot*dt;
    x2R = x2R + x2dot*dt;
    omega = 25.9157*x2L;
  return omega; //gear ratio
  }
  */
  // DISCRETE TRANSFER FUNCTION
  double DZmotorTF(double omegaPre1,double omegaPre2, double v, double vPre) {
   
    double omega = 0.1517*omegaPre1 - 0.005841*omegaPre2+ 0.41*v -0.02951*vPre;
   
  return omega;
  }

int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();

  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */

   
    WbDeviceTag magSensor = wb_robot_get_device("mgs1600");
    wb_camera_enable(magSensor, TIME_STEP);
    
    int image_width = wb_camera_get_width(magSensor);
    int image_height = wb_camera_get_height(magSensor);
    //printf("%d; %d; ", image_width, image_height);
     
     WbDeviceTag encoderL;
     encoderL = wb_robot_get_device("sensorL");
     WbDeviceTag encoderR;
     encoderR = wb_robot_get_device("sensorR");
     wb_position_sensor_enable(encoderL, TIME_STEP);
     wb_position_sensor_enable(encoderR, TIME_STEP);
     double encL0 = 0;
     double encR0 = 0;   
     
    WbDeviceTag Lmotor;
    Lmotor = wb_robot_get_device("Lmotor");
    WbDeviceTag Rmotor;
    Rmotor = wb_robot_get_device("Rmotor");
    
    wb_motor_enable_torque_feedback(Lmotor, TIME_STEP);
    wb_motor_enable_torque_feedback(Rmotor, TIME_STEP);
    
    //Errors init
    double eApre = 0;
    double eVpre = 0;
    double eWpre = 0;
    double linePre = 0;
    
    double preAvgVL = 0;
    double preAvgVR = 0;
    double preErrVL = 0;
    double preErrVR = 0;
    
    double ErrVL = 0;
    double ErrVR = 0;
    double wLz1 = 0;
    double wLz2 = 0;
    double vLz1 = 0;
    double vLz2 = 0;
    
    double wRz1 =0;
    double wRz2 = 0;
    double vRz1 = 0;
    double vRz2 = 0;
    double uLpre = 0;
    double uRpre = 0;
    double uLa = 0;
    double uRa = 0;
    double iL = 0;
    double iR = 0;
    double uPre = 0;
    
    double IuW = 0;
    double IuV = 0;
  // Test set:
  wb_motor_set_position(Lmotor, INFINITY);
  wb_motor_set_position(Rmotor, INFINITY);
  const int agvSpeed = 1;
  double wheelSpeed = agvSpeed/0.08;
  
  //wb_motor_set_acceleration(Lmotor, 6.25);
  //wb_motor_set_acceleration(Rmotor, 6.25);
  
  //wb_motor_set_available_torque(Lmotor, 18);
  //wb_motor_set_available_torque(Rmotor, 18);
  //wb_motor_set_velocity(Lmotor,5);
 // wb_motor_set_velocity(Rmotor,5);
//---------------------------------------
  
FILE *fptr;

// Open a file in writing mode
//fptr = fopen(""D:\hoc-tap\LVTN\CTRLSIM\agv1209\simData.csv"", "w");
fptr = fopen("D:\\hoc-tap\\LVTN\\CTRLSIM\\agv1209v3\\simData.csv", "w");

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
   int t = 0;
   double uA = 0;
   double uApre = 0;
   
   double tAl = 0;
   double tDl = 0;
   double tAr = 0;
   double tDr= 0;
   
   double aL = 0;
   double aR = 0;
   
   double aLpre = 0;
   double aRpre = 0;
   
   double rLpre = 0;
   double rRpre = 0;
   
   double w_refL = 0;   // persistent state
   double w_refR = 0;
   
   double smooth_stepL(double target_w) {
    double max_delta = 6.25* dt;
    double err = target_w - w_refL;

    if (err > max_delta)       w_refL += max_delta;
    else if (err < -max_delta) w_refL -= max_delta;
    else                        w_refL = target_w;

    return w_refL;
}

 double smooth_stepR(double target_w) {
    double max_delta = 6.25* dt;
    double err = target_w - w_refR;

    if (err > max_delta)       w_refR += max_delta;
    else if (err < -max_delta) w_refR -= max_delta;
    else                        w_refR = target_w;

    return w_refR;
}

double uVpre = 0;
double uWpre = 0;

    double rWheel = 0.08;
    double lWheel = 0.2;
    double wL;
    double wR;
   
    double vAGV;
    double eV;
    double eA;
    double eW;
    double vPID[3] = {10, .25,0};
    double wPID[3] = {8, 5, 5}; // current optimal 1, 1, 1
   
   
  while (wb_robot_step(TIME_STEP) != -1) {
    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */
    const unsigned char *image = wb_camera_get_image(magSensor);
    
    
    //int wb_camera_get_width(magSensor);
    //int wb_camera_get_height(WbDeviceTag tag);


    /* Process sensor data here */
    
    double encL = wb_position_sensor_get_value(encoderL);
    double encR = wb_position_sensor_get_value(encoderR);
    double oL = (encL - encL0)/dt;// m/s
    double oR = (encR - encR0)/dt;
    
    double vL = oL*0.08;
    double vR = oR*0.08;
   
    encL0 = encL;
    encR0 = encR;
    
    double avgVLin = 0;
    double avgVRin = 0;

    double avgVLout = 0;
    double avgVRout = 0;
    
    int pixelCnt = 0;
    double linePos = 0;
    int stationMarker = 0;
    double magValue = 0;
    for (int x = 0; x < image_width; x++) {
      for (int y = 0; y < image_height; y++) {
      int pixelGray = wb_camera_image_get_gray(image,image_width,x, y );
      //int pixelBlue = wb_camera_image_get_blue(image,image_width,x, y );
    
        
        //printf("%d; ",pixelGray);
        
        if (pixelGray <= 60) {
          magValue = 1;
          pixelCnt += 1;
        }
        else if (pixelGray >= 100) {
          magValue = 0;
        }
        else {
        magValue = 0;
        //pixelCnt += 1;
        }
        double mmX = 2*x - 74;
        
        linePos += mmX*magValue; //mm
                
     }
     }
     
     // linePos = linePos/pixelCnt;
     // printf("%d; %f; %f \n", pixelCnt, linePos, (vL + vR)/2);
     // if (pixelCnt ==0) {
      // fclose(fptr);  
      // printf("%d", t);
      // wb_motor_set_velocity(Lmotor, 0);
      // wb_motor_set_velocity(Rmotor, 0);
      // wb_robot_cleanup();
      // exit(0);
      
     // }
     if (pixelCnt == 0) {
          fclose(fptr);
          printf("%d", t);
      
          wb_motor_set_velocity(Lmotor, 0);
          wb_motor_set_velocity(Rmotor, 0);
      
          wb_robot_cleanup();
          exit(0);
      }
      
      linePos = linePos / pixelCnt;
     
    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
  
    double rpsLeft = wheelSpeed;
    double rpsRight = wheelSpeed;
    
    double omega = linePos;
    if (omega < 0) {
    rpsLeft += 0.3*omega;
    rpsRight += 0.1*omega;
    
    }
    if (omega > 0) {
    rpsRight -= 0.3*omega;
     rpsLeft -= 0.1*omega;
    }
      */
    
    // PID controller

    
    
    //double dPID[3] = {1, 0, 0};
    
    vAGV = (vL + vR)/2;
     double targetAngle = atan(linePos/337.5);
    double headAngle = atan((vR - vL)*dt/0.4);
    //headAngle = 0;   
    eA = - targetAngle;
    //eW = (eA - eApre)/dt;
    eW = linePos;
    
    //eW = eA; 
    
    eV = agvSpeed - vAGV;
    
    IuW = IuW + eW;
    IuV = IuV + eV;
    if (fabs(IuV) > 200) {IuV = copysign(200, IuV); }
    
    if (fabs(IuW) > 18)  {IuW = copysign(18, IuW); }
    double ffW = eW - eWpre;
    //eW = eW + ffW*dt;
    
    
    double uV = vPID[0]*eV + vPID[1]*IuV*dt + vPID[2]*(eV - eVpre)/dt;
    uV = 1;
    //ffW = uV/0.9;
    //eW = eW + ffW;
    
    double uW = .2*eW + .2175*IuW*dt + .016*(eW - eWpre)/dt;
    //double uW = 0.03 * eW;
    //if (uW > 2) uW = 2;
    //if (uW < -2) uW = -2 ;
    //uW = uW*cos(eA);
    uV = uV*cos(fmin(fabs(uW), 3.141/2));
    //uV = uV*cos((eA+ eApre));
    
    /* double curveDecel ;
    
    double c1 = abs(linePos)/75;
    double c2 = abs(linePre)/75;
    
    curveDecel = dPID[0]*c1 + dPID[1]*(c1 + c2) + dPID[2]*(c1-c2);
    curveDecel  = (c1 + c2)/2;
    
    uV = uV*(1 - curveDecel); 
    
        if (abs(linePos) >= 40) {
    uV = -uV*0.1;
    }    
    
            if (fabs(linePos) >= 20) {
    uV = 0.25;
    } else  if (fabs(linePos) >= 15) {
      uV = uV*0.5;
    }  else if (fabs(linePos) >= 10) {
    uV = uV*0.625;
    } else if (fabs(linePos) >= 5) {
    uV = uV*0.75;
    } 
      
      
    uV = uV*(-fabs(linePos)/45+ 1);
    if (fabs(linePos) >= 45) {uV = 0;}      
    */  
    

 
   //if (uV - uVpre > 1*dt) {uV = uVpre + 1*dt; }
   //if (uVpre - uV > 1*dt) {uV = uVpre -1*dt; }
    uVpre = uV;  
    
    uA = uApre + copysign(1*dt, 1 - fabs(uV));
    //uV = fmin(uV, uVpre + uA*dt);
    uApre = uA;
    uWpre = uW;
    
    //double uWmax = fmax(uV, 0.1)/lWheel;
    //if (uW >  uWmax) uW =  uWmax;
    //if (uW < -uWmax) uW = -uWmax;
       
    wR = (uV - uW*lWheel)/rWheel;
    wL = (uV + uW*lWheel)/rWheel;
  
    eVpre = eV;
    eApre = eA;
    eWpre = eW;
    uWpre = uW;

/*if (t == 0) {
w_refL = wL;
w_refR = wR;
}*/

//wL = smooth_stepL(wL);
//wR = smooth_stepR(wR);

//if (wL - uLpre > 6.25*dt) {wL = uLpre + 6.25*dt; }
//if (wR - uRpre > 6.25*dt) {wR = uRpre + 6.25*dt; }
  /*double jerk = 12.5;
  
  if (wL > uLpre) { 
  if (uLpre >0) {aL = aLpre + copysign(jerk*dt, 0.5/0.08 - fabs(uLpre) );}
  if (uLpre < 0) { aL = aLpre + copysign(jerk*dt, -0.5/0.08 + fabs(uLpre) );  }
  wL = fmin(wL, uLpre + fabs(aL*dt)); }
  
  aLpre = aL;
  
  if (wR > uRpre) { 
  if (uRpre > 0) {aR = aRpre + copysign(jerk*dt, 0.5/0.08 - fabs(uRpre) );}
  if (uRpre < 0) { aR = aRpre + copysign(jerk*dt, -0.5/0.08 + fabs(uRpre) );  }
  wR = fmin(wR, uRpre + fabs(aR*dt)); }
  aRpre = aR;*/
  
 double jerk  = 12.5;   // rad/s^3
//double a_max = 6.25;   // rad/s^2

// wL_target = the desired/kinematic target speed for this tick (before shaping)
// uLpre     = shaper's previous output speed (persists between ticks)
// aLpre     = shaper's previous acceleration (persists between ticks)

double a_max_accel = 12.5/2;
double a_max_decel = 12.5/2;    // try larger, e.g. 12-15, tune from here

double errL   = wL - uLpre;
double dstopL = aLpre * fabs(aLpre) / (2.0 * jerk);

double jcmdL;
if (errL > dstopL) {
    jcmdL = jerk;
} else if (errL < -dstopL) {
    jcmdL = -jerk;
} else {
    jcmdL = copysign(jerk, -aLpre);
}

aL = aLpre + jcmdL * dt;

// asymmetric clamp: different bounds depending on direction of accel
if (aL > a_max_accel) aL = a_max_accel;
if (aL < -a_max_decel) aL = -a_max_decel;

wL = uLpre + aL * dt;

uLpre = wL;
aLpre = aL;

double errR   = wR- uRpre;
double dstopR = aRpre * fabs(aRpre) / (2.0 * jerk);

double jcmdR;
if (errR > dstopR) {
    jcmdR = jerk;
} else if (errR < -dstopR) {
    jcmdR = -jerk;
} else {
    jcmdR = copysign(jerk, -aRpre);
}

aR = aRpre + jcmdR * dt;

// asymmetric clamp: different bounds depending on direction of accel
if (aR > a_max_accel) aR = a_max_accel;
if (aR < -a_max_decel) aR = -a_max_decel;

wR = uRpre + aR * dt;

uRpre = wR;
aRpre = aR;
 
 //double aLz0 = (wL - uLpre)/dt;
 //double aRz0 = (wR - uRpre)/dt;
 
 //if (wL > uLpre ) { wL = fmin(wL, uLpre + fmin(12.5, fabs(aLz0) + jerk*dt)*dt); }
 //else if (wL < uLpre) { wL = fmax(wL, uLpre - fmin(12.5, fabs(aLz0) + jerk*dt)*dt); }
 
 //if (wR > uRpre) { wR = fmin(wR, uRpre + fmin(12.5, fabs(aRz0) + jerk*dt)*dt); }
 //else if (wR < uRpre) {  wR = fmax(wR, uRpre - fmin(12.5, fabs(aRz0) + jerk*dt)*dt); }
 //wb_motor_set_acceleration(Lmotor, fmin(abs(aLz0) + jerk*dt, 12.5));
 //wb_motor_set_acceleration(Rmotor, fmin(abs(aRz0) + jerk*dt, 12.5));
 
 if (fabs(wR) > 1.25/rWheel) {
     wR = copysign(1.25/rWheel, wR);
    }
    
if (fabs(wL) > 1.25/rWheel) {   
      wL = copysign(1.25/rWheel, wL);
 }
 
/*
double vamax = 0.5/0.08;
double lz1 = fmin(fabs(wLz1),1/0.08);
double rz1 = fmin(fabs(wRz1),1/0.08);
aL = fabs(-fabs(lz1) + vamax*2);
aR = fabs(-fabs(rz1) + vamax*2);
aL = fmin(aL, aLpre + jerk*dt);
aL = fmin(aL, 6.25);
aR = fmin(aR, aRpre + jerk*dt);
aR = fmin(aR, 6.25);
wb_motor_set_acceleration(Lmotor, aL);
wb_motor_set_acceleration(Rmotor, aR);
aLpre = aL;
aRpre = aR;

*/

/*
aLpre = aL;
aRpre = aR;
uLpre = wL;
uRpre = wL;
uLa = aLnew;
uRa = aRnew;
*/
//rLpre = vL/0.08;
//rRpre = vR/0.08;

uLpre = wL;
uRpre = wR;

ErrVL = wL - oL;
ErrVR = wR - oR;

//if (abs(ErrVL)/dt > 6.25) {ErrVL = copysign(6.25*dt,wL - vL/0.08);}
//if (abs(ErrVR)/dt > 6.25) {ErrVR = copysign(6.25*dt,wR - vR/0.08);}

iL = iL + ErrVL;
iR = iR + ErrVR;
if (fabs(iL) > 500) {iL = copysign(500, iL); }
if (fabs(iR) > 500) {iR = copysign(500, iR); }

double ffL = 0.41*oL - 0.02951*oL + 0.1517*vLz1 -0.005841*vLz2;
double ffR = 0.41*oR - 0.02951*oR + 0.1517*vRz1 -0.005841*vRz2;

avgVLout =  .0224*ErrVL + 2.24*iL*dt + (5.61*pow(10, -5))*(ErrVL - preErrVL)/dt;
avgVRout =  .0224*ErrVR + 2.24*iR*dt + (5.61*pow(10, -5))*(ErrVR - preErrVR)/dt;
avgVLout = fmin(fabs(avgVLout), 48 )*copysign(1, avgVLout);
avgVRout = fmin(fabs(avgVRout), 48 )*copysign(1, avgVRout);

//avgVLout = 10*t*dt*t*dt;
//avgVRout = 10*t*dt*t*dt;

//avgVLout = 48;
//avgVRout = 48;

 preAvgVL = avgVLout;
 preAvgVR = avgVRout;
 preErrVL = ErrVL;
 preErrVR = ErrVR; 

double omegaL = 0;
double omegaR = 0;

//omegaL = LmotorTF(vL/0.08 ,avgVLout);
//omegaR = RmotorTF(vR/0.08 ,avgVRout);
// DZmotorTF(double omegaPre1,double omegaPre2, double v, double vPre)
omegaL = DZmotorTF(wLz1, wLz2, vLz1, vLz2);
omegaR = DZmotorTF(wRz1, wRz2, vRz1, vRz2);
//omegaL = DZmotorTF(wLz1, wLz2, avgVLout, vLz1);
//omegaR = DZmotorTF(wRz1, wRz2, avgVRout, vRz1);
//var updates:
wLz2 = wLz1;
wLz1 = omegaL;
vLz2 = vLz1;
vLz1 = avgVLout;

wRz2 = wRz1;
wRz1 = omegaR;
vRz2 = vRz1;
vRz1 = avgVRout; //time delay


   //wb_motor_set_available_torque(Lmotor, 18);
   //wb_motor_set_available_torque(Rmotor, 18);
   // wb_motor_set_velocity(Lmotor, omegaL);
   // wb_motor_set_velocity(Rmotor, omegaR);
   
   wb_motor_set_velocity(Lmotor, wL);
   wb_motor_set_velocity(Rmotor, wR);
    
    if(t < 50) {
      wb_motor_set_velocity(Lmotor, 0);
      wb_motor_set_velocity(Rmotor, 0);
      
     }

   
    //* torque feedback
    
    double tL = wb_motor_get_torque_feedback(Lmotor);
    double tR = wb_motor_get_torque_feedback(Rmotor);
     //*/   
 
   if (t % 10 == 0) {   
    //printf(" heading error: %f; \n ", eA);
    printf(" Left speed: %f; Right speed: %f; \n ", vL, vR);
    printf("line position: %f mm; station: %d \n", linePos, stationMarker);
    printf("left torque: %f; right torque: %f;\n \n ", tL, tR);
    
    
    }
    //printf("%.4f, %.4f, %.4f, %.4f \n ", x1L, x2L, x1R, x2R);
    printf("left: %f; right: %f;\n \n ", avgVLout, avgVRout);
    double vecV = sqrt(pow((vL + vR)/2, 2) + pow((vL - vR)/lWheel, 2));
      fprintf(fptr, "%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f \n", (omegaL + omegaR)*0.08/2, omegaL,  omegaR, aL, aR, linePos, linePos, uW);
    t+=1;
    

      
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}


