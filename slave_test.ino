/*
 * ---------------------------------------------------------------------------------------
 * CONTROL TESTING OF THE SLAVE HAPTIC PADDLE
 * ---------------------------------------------------------------------------------------
 * Juan Manuel Gandarias Palacios 
 * 28/04/2016
 * Master in Mechatronics Engineering
 * University of Malaga
 * --------------------------------------------------------------------------------------
 * 
 * Program for testing the second haptic paddle connected to the Hapkit Board for being
 * programmed as a slave in a Bilateral teleoperated system  
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

// Slave pin declares
int pwmPin = 6;           // PWM output pin for motor slave
int dirPin = 7;           // direction output pin for motor slave
int sensorPosPin = A0;    // input pin for MR sensor slave


// Slave position tracking variables
int updatedPos = 0;         // keeps track of the latest updated value of the MR sensor reading
int rawPos = 0;             // current raw reading from MR sensor
int lastRawPos = 0;         // last raw reading from MR sensor
int lastLastRawPos = 0;     // last last raw reading from MR sensor
int flipNumber = 0;         // keeps track of the number of flips over the 180deg mark
int tempOffset = 0;   
int rawDiff = 0;
int lastRawDiff = 0;
int rawOffset = 0;
int lastRawOffset = 0;
const int flipThresh = 700;  // threshold to determine whether or not a flip over the 180 degree mark occurred
boolean flipped = false;

// Slave kinematics variables
double xh = 0;              // position of the handle [m]
double lastXh = 0;          //last x position of the handle
double vh = 0;              //velocity of the handle
double lastVh = 0;          //last velocity of the handle
double lastLastVh = 0;      //last last velocity of the handle
double ts = 0;              //sector rotation
double rh = 0.065659;       //[m] 
double rp = 0.004191;       //[m]
double rs = 0.073152;       //[m] 
     
// Slave force output variables
double force = 0;           // force at the handle
double Tp = 0;              // torque of the motor pulley
double duty = 0;            // duty cylce (between 0 and 255)
unsigned int output = 0;    // output command to the motor

//---------------------------------------------------------------------------------------
//            SETUP FUNCTION
//---------------------------------------------------------------------------------------

void setup() {
 
  // Set up serial communication
  Serial.begin(57600);
  
  // Set PWM frequency 
  setPwmFrequency(pwmPin,1);
  
  // Input pins
  pinMode(sensorPosPin, INPUT);     // set MR sensor pin to be an input

  // Output pins
  pinMode(pwmPin, OUTPUT);          // PWM pin for motor A
  pinMode(dirPin, OUTPUT);          // dir pin for motor A
  
  // Initialize motor 
  analogWrite(pwmPin, 0);           // set to not be spinning (0/255)
  digitalWrite(dirPin, LOW);        // set direction
  
  // Initialize position valiables
  lastLastRawPos = analogRead(sensorPosPin);
  lastRawPos = analogRead(sensorPosPin);
}

//---------------------------------------------------------------------------------------
//      MAIN LOOP
//---------------------------------------------------------------------------------------
void loop() {
  
  //*************************************************************************************
  //            MR sensor lecture
  //*************************************************************************************
  // Give the value of the pulley's angle

  // Get voltage output by MR sensor
  rawPos = analogRead(sensorPosPin);  //current raw position from MR sensor

  // Calculate differences between subsequent MR sensor readings
  rawDiff = rawPos - lastRawPos;          //difference btwn current raw position and last raw position
  lastRawDiff = rawPos - lastLastRawPos;  //difference btwn current raw position and last last raw position
  rawOffset = abs(rawDiff);
  lastRawOffset = abs(lastRawDiff);
  
  // Update position record-keeping vairables
  lastLastRawPos = lastRawPos;
  lastRawPos = rawPos;
  
  // Keep track of flips over 180 degrees
  if((lastRawOffset > flipThresh) && (!flipped)) { // enter this anytime the last offset is greater than the flip threshold AND it has not just flipped
    if(lastRawDiff > 0) {        // check to see which direction the drive wheel was turning
      flipNumber--;              // cw rotation 
    } else {                     // if(rawDiff < 0)
      flipNumber++;              // ccw rotation
    }
    if(rawOffset > flipThresh) { // check to see if the data was good and the most current offset is above the threshold
      updatedPos = rawPos + flipNumber*rawOffset; // update the pos value to account for flips over 180deg using the most current offset 
      tempOffset = rawOffset;
    } else {                     // in this case there was a blip in the data and we want to use lastactualOffset instead
      updatedPos = rawPos + flipNumber*lastRawOffset;  // update the pos value to account for any flips over 180deg using the LAST offset
      tempOffset = lastRawOffset;
    }
    flipped = true;            // set boolean so that the next time through the loop won't trigger a flip
  } else {                        // anytime no flip has occurred
    updatedPos = rawPos + flipNumber*tempOffset; // need to update pos based on what most recent offset is 
    flipped = false;
  }

  ts = -.0107*updatedPos + 4.9513; // Compute the angle of the sector pulley (ts) in degrees based on updatedPos
  
  //*************************************************************************************
  //            Compute position in meters 
  //*************************************************************************************
  // Compute the position of the handle based on handle rotation(ts)

     xh = rh*(ts*3.14159/180);       
     
  // filtered velocity (2nd-order filter)
     vh = -(.95*.95)*lastLastVh + 2*.95*lastVh + (1-.95)*(1-.95)*(xh-lastXh)/.0001;  
     lastXh = xh;
     lastLastVh = lastVh;
     lastVh = vh;

  //*************************************************************************************
  //            Assign a motor output force in Newtons
  //************************************************************************************* 
  // Generate a force by simply assigning this to a constant number (in Newtons)
  //force = 0.5;
  //------------------------------------------------------------------------------------
  // Render a VIRTUAL SPRING
//  double k_spring = 20; //[N/m]
//  force = -k_spring*xh;
  //------------------------------------------------------------------------------------
  // Render a VIRTUAL DAMPER
  double b_damper = 1.2; 
  force = -b_damper*vh;
  //------------------------------------------------------------------------------------
  
  //------------------------------------------------------------------------------------
  // Compute the require motor pulley torque (Tp) to generate that force 
     Tp = (rp/rs) * rh * force; 
 
  //*************************************************************************************
  //            Set output 
  //*************************************************************************************
  // Determine correct direction for motor torque
  if(force < 0) { 
    digitalWrite(dirPin, HIGH);
  } else {
    digitalWrite(dirPin, LOW);
  }

  // Compute the duty cycle required to generate Tp (torque at the motor pulley)
  duty = sqrt(abs(Tp)/0.03);

  // Make sure the duty cycle is between 0 and 100%
  if (duty > 1) {            
    duty = 1;
  } else if (duty < 0) { 
    duty = 0;
  }  
  output = (int)(duty* 255);   // convert duty cycle to output signal
  analogWrite(pwmPin,output);  // output the signal
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
