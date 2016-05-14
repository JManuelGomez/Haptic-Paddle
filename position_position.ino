/*
 *  This program is based on the program developed by Stanford University for the Hapkit
 *  to teach the course Introduction to Haptics.
 *  (https://lagunita.stanford.edu/courses/SelfPaced/Haptics/2014/about)
 *  We want to thank Stanford University for this program and the course.
 *
 * ---------------------------------------------------------------------------------------
 * POSITION-POSITION SCHEMED WITH A HAPTIC PADDLE DEVICE
 * ---------------------------------------------------------------------------------------
 * Juan Manuel Gandarias Palacios 
 * 29/04/2016
 * Master in Mechatronics Engineering
 * University of Malaga
 * --------------------------------------------------------------------------------------
 * 
 * Implementation of a position-position bilateral teleoperation scheme with two haptic
 * paddle devices developed at the TaISLab of the University of Malaga
 * 
 * --------------------------------------------------------------------------------------
 */

//---------------------------------------------------------------------------------------
//           LIBRARIES
//---------------------------------------------------------------------------------------
#include <math.h>


//---------------------------------------------------------------------------------------
//          DEFINITION OF VARIABLES
//---------------------------------------------------------------------------------------

// Master pin declares
int pwmPinM = 5;           // PWM output pin for motor master
int dirPinM = 8;           // direction output pin for motor master
int sensorPosPinM = A2;    // input pin for MR sensor master

// Slave pin declares
int pwmPinS = 6;           // PWM output pin for motor slave
int dirPinS = 7;           // direction output pin for motor slave
int sensorPosPinS = A0;    // input pin for MR sensor slave

// Master's position tracking variables
int updatedPosM = 0;         // keeps track of the latest updated value of the MR sensor reading
int rawPosM = 0;             // current raw reading from MR sensor
int lastRawPosM = 0;         // last raw reading from MR sensor
int lastLastRawPosM = 0;     // last last raw reading from MR sensor
int flipNumberM = 0;         // keeps track of the number of flips over the 180deg mark
int tempOffsetM = 0;   
int rawDiffM = 0;
int lastRawDiffM = 0;
int rawOffsetM = 0;
int lastRawOffsetM = 0;
const int flipThreshM = 700;  // threshold to determine whether or not a flip over the 180 degree mark occurred
boolean flippedM = false;

// Slave's position tracking variables
int updatedPosS = 0;         // keeps track of the latest updated value of the MR sensor reading
int rawPosS = 0;             // current raw reading from MR sensor
int lastRawPosS = 0;         // last raw reading from MR sensor
int lastLastRawPosS = 0;     // last last raw reading from MR sensor
int flipNumberS = 0;         // keeps track of the number of flips over the 180deg mark
int tempOffsetS = 0;   
int rawDiffS = 0;
int lastRawDiffS = 0;
int rawOffsetS = 0;
int lastRawOffsetS = 0;
const int flipThreshS = 700;  // threshold to determine whether or not a flip over the 180 degree mark occurred
boolean flippedS = false;

// Master kinematics variables
double xhM = 0;              // position of the handle [m]
double lastXhM = 0;          //last x position of the handle
double vhM = 0;              //velocity of the handle
double lastVhM = 0;          //last velocity of the handle
double lastLastVhM = 0;      //last last velocity of the handle
double tsM = 0;              //sector rotation
double rhM = 0.075;          //[m] 
double rpM = 0.005;          //[m]
double rsM = 0.06;           //[m]  
double km = 100;               //Master's Gain      

// Slave kinematics variables
double xhS = 0;              // position of the handle [m]
double lastXhS = 0;          //last x position of the handle
double vhS = 0;              //velocity of the handle
double lastVhS = 0;          //last velocity of the handle
double lastLastVhS = 0;      //last last velocity of the handle
double tsS = 0;              //sector rotation
double rhS = 0.075;          //[m] 
double rpS = 0.005;          //[m]
double rsS = 0.06;           //[m] 
double ks = 100;               //Slave's Gain

// Master force output variables
double forceM = 0;           // force at the handle
double TpM = 0;              // torque of the motor pulley
double dutyM = 0;            // duty cylce (between 0 and 255)
unsigned int outputM = 0;    // output command to the motor

// Slave force output variables
double forceS = 0;           // force at the handle
double TpS = 0;              // torque of the motor pulley
double dutyS = 0;            // duty cylce (between 0 and 255)
unsigned int outputS = 0;    // output command to the motor

//---------------------------------------------------------------------------------------
//            SETUP FUNCTION
//---------------------------------------------------------------------------------------

void setup() {

    
  // Set up serial communication
  Serial.begin(57600);
  
  // Set PWM frequency for Master and Slave
  setPwmFrequency(pwmPinM,1);        
  setPwmFrequency(pwmPinS,1);
  
  // Input pins
  pinMode(sensorPosPinM, INPUT);     // set Master's MR sensor pin to be an input
  pinMode(sensorPosPinS, INPUT);     // set Slaves's MR sensor pin to be an input
  
  // Output pins
  pinMode(pwmPinM, OUTPUT);          // PWM pin for Master's motor 
  pinMode(dirPinM, OUTPUT);          // dir pin for Master's motor   
  pinMode(pwmPinS, OUTPUT);          // PWM pin for Slave's motor 
  pinMode(dirPinS, OUTPUT);          // dir pin for Slave's motor 
  
  // Initialize motor 
  analogWrite(pwmPinM, 0);           // set Master's motor to not be spinning (0/255)
  digitalWrite(dirPinM, LOW);        // set direction of the Master's motor
  analogWrite(pwmPinS, 0);           // set Slave's to not be spinning (0/255)
  digitalWrite(dirPinS, LOW);        // set direction of the Slave's motor
  
  // Initialize position valiables of Master and Slave
  lastLastRawPosM = analogRead(sensorPosPinM);
  lastRawPosM = analogRead(sensorPosPinM);
  lastLastRawPosS = analogRead(sensorPosPinS);
  lastRawPosS = analogRead(sensorPosPinS);

}


//---------------------------------------------------------------------------------------
//      MAIN LOOP
//---------------------------------------------------------------------------------------
void loop() {
  
  
  //*************************************************************************
  //        Lecture Master's angle sensor
  //*************************************************************************
  
  // Get voltage output by MR sensor
  rawPosM = analogRead(sensorPosPinM);  //current raw position from MR sensor

  // Calculate differences between subsequent MR sensor readings
  rawDiffM = rawPosM - lastRawPosM;          //difference btwn current raw position and last raw position
  lastRawDiffM = rawPosM - lastLastRawPosM;  //difference btwn current raw position and last last raw position
  rawOffsetM = abs(rawDiffM);
  lastRawOffsetM = abs(lastRawDiffM);
  
  // Update position record-keeping vairables
  lastLastRawPosM = lastRawPosM;
  lastRawPosM = rawPosM;
  
  // Keep track of flips over 180 degrees
  if((lastRawOffsetM > flipThreshM) && (!flippedM)) { // enter this anytime the last offset is greater than the flip threshold AND it has not just flipped
    if(lastRawDiffM > 0) {        // check to see which direction the drive wheel was turning
      flipNumberM--;              // cw rotation 
    } else {                     // if(rawDiff < 0)
      flipNumberM++;              // ccw rotation
    }
    if(rawOffsetM > flipThreshM) { // check to see if the data was good and the most current offset is above the threshold
      updatedPosM = rawPosM + flipNumberM*rawOffsetM; // update the pos value to account for flips over 180deg using the most current offset 
      tempOffsetM = rawOffsetM;
    } else {                     // in this case there was a blip in the data and we want to use lastactualOffset instead
      updatedPosM = rawPosM + flipNumberM*lastRawOffsetM;  // update the pos value to account for any flips over 180deg using the LAST offset
      tempOffsetM = lastRawOffsetM;
    }
    flippedM = true;            // set boolean so that the next time through the loop won't trigger a flip
  } else {                        // anytime no flip has occurred
    updatedPosM = rawPosM + flipNumberM*tempOffsetM; // need to update pos based on what most recent offset is 
    flippedM = false;
  }

  tsM = -.0107*updatedPosM + 4.9513; // Compute the angle of the sector pulley (ts) in degrees based on updatedPos
  

  //*************************************************************************
  //        Lecture Slave's angle sensor
  //*************************************************************************
  
    // Get voltage output by MR sensor
  rawPosS= analogRead(sensorPosPinS);  //current raw position from MR sensor

  // Calculate differences between subsequent MR sensor readings
  rawDiffS = rawPosS - lastRawPosS;          //difference btwn current raw position and last raw position
  lastRawDiffS = rawPosS - lastLastRawPosS;  //difference btwn current raw position and last last raw position
  rawOffsetS = abs(rawDiffS);
  lastRawOffsetS = abs(lastRawDiffS);
  
  // Update position record-keeping vairables
  lastLastRawPosS = lastRawPosS;
  lastRawPosS = rawPosS;
  
  // Keep track of flips over 180 degrees
  if((lastRawOffsetS > flipThreshS) && (!flippedS)) { // enter this anytime the last offset is greater than the flip threshold AND it has not just flipped
    if(lastRawDiffS > 0) {        // check to see which direction the drive wheel was turning
      flipNumberS--;              // cw rotation 
    } else {                     // if(rawDiff < 0)
      flipNumberS++;              // ccw rotation
    }
    if(rawOffsetS > flipThreshS) { // check to see if the data was good and the most current offset is above the threshold
      updatedPosS = rawPosS + flipNumberS*rawOffsetS; // update the pos value to account for flips over 180deg using the most current offset 
      tempOffsetS = rawOffsetS;
    } else {                     // in this case there was a blip in the data and we want to use lastactualOffset instead
      updatedPosS = rawPosS + flipNumberS*lastRawOffsetS;  // update the pos value to account for any flips over 180deg using the LAST offset
      tempOffsetS = lastRawOffsetS;
    }
    flippedS = true;            // set boolean so that the next time through the loop won't trigger a flip
  } else {                        // anytime no flip has occurred
    updatedPosS = rawPosS + flipNumberS*tempOffsetS; // need to update pos based on what most recent offset is 
    flippedS = false;
  }
  
  tsS = -.0107*updatedPosS + 4.9513; // Compute the angle of the sector pulley (ts) in degrees based on updatedPos
  

  //*************************************************************************************
  //            Compute position in meters and estimate Master's velocity
  //*************************************************************************************
  // Compute the position of the handle based on handle rotation(ts)
     xhM = rhM*(tsM*3.14159/180);       
     
  // filtered velocity (2nd-order filter)
  //   vhM = -(.95*.95)*lastLastVhM + 2*.95*lastVhM + (1-.95)*(1-.95)*(xhM-lastXhM)/.0001;  
  //   lastXhM = xhM;
  //   lastLastVhM = lastVhM;
  //   lastVhM = vhM;
     
  //*************************************************************************************
  //            Compute position in meters and estimate Slave's velocity
  //*************************************************************************************
  // Compute the position of the handle based on handle rotation(ts)
     xhS = rhS*(tsS*3.14159/180);       
     
  // filtered velocity (2nd-order filter)
  //   vhS = -(.95*.95)*lastLastVhS + 2*.95*lastVhS + (1-.95)*(1-.95)*(xhS-lastXhS)/.0001;  
  //   lastXhS = xhS;
  //   lastLastVhS = lastVhS;
  //   lastVhS = vhS;

  //*************************************************************************************
  //            Assign a motor output force in Newtons
  //************************************************************************************* 
  
  // Pose-Pose Scheme
  forceM = km*(xhS-xhM);
  forceS = ks*(xhM-xhS);

  //------------------------------------------------------------------------------------
  // Compute the require motor pulley torque (Tp) to generate that force 
     TpM = (rpM/rsM) * rhM * forceM; 
     TpS = (rpS/rsS) * rhS * forceS; 
  
  //*************************************************************************************
  //            Set output 
  //*************************************************************************************
  // Determine correct direction for motor torque
  if(forceM < 0) { 
    digitalWrite(dirPinM, HIGH);
  } else {
    digitalWrite(dirPinM, LOW);
  }
  
  if(forceS < 0) { 
    digitalWrite(dirPinS, HIGH);
  } else {
    digitalWrite(dirPinS, LOW);
  }

  // Compute the duty cycle required to generate Tp (torque at the Master's motor pulley)
  dutyM = sqrt(abs(TpM)/0.03);

  // Compute the duty cycle required to generate Tp (torque at the Slave's motor pulley)
  dutyS = sqrt(abs(TpS)/0.03);
  
  // Make sure the duty cycle is between 0 and 100%
  if (dutyM > 1) {            
    dutyM = 1;
  } else if (dutyM < 0) { 
    dutyM = 0;
  }  
  outputM = (int)(dutyM* 255);   // convert duty cycle to output signal
  analogWrite(pwmPinM,outputM);  // output the signal
  
    // Make sure the duty cycle is between 0 and 100%
  if (dutyS > 1) {            
    dutyS = 1;
  } else if (dutyS < 0) { 
    dutyS = 0;
  }  
  outputS = (int)(dutyS* 255);   // convert duty cycle to output signal
  analogWrite(pwmPinS,outputS);  // output the signal
}



//---------------------------------------------------------------------------------------
//      FUNCTION TO SET PWM
//---------------------------------------------------------------------------------------

void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}
