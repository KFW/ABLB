/* ABLB
 * ActoBitty Line Bot 
 * 
 * DEV branch
 *
 * base robot Actobotics ActoBitty:
 * https://www.servocity.com/html/actobitty_2_wheel_robot_kit.html#.VthCuvkrI-U
 * 
 * microcontroller board DFRobot Romeo 2 (has built in motor controller):
 * http://www.dfrobot.com/wiki/index.php/Romeo_V2-All_in_one_Controller_(R3)_(SKU:DFR0225) 
 *
 * mount panel for board:
 * http://www.thingiverse.com/thing:1377159
 * 
 * line follower array from Sparkfun:
 * https://github.com/sparkfun/Line_Follower_Array
 * https://learn.sparkfun.com/tutorials/sparkfun-line-follower-array-hookup-guide
 * position provided ranges from -127 (far L) to 127 (far R)
 * 
 * PID quick tutorial
 * http://letsmakerobots.com/node/39972
 *  
 */


#include "Wire.h"              // for I2C
#include "sensorbar.h"         // needs SparkFun library

// Uncomment one of the four lines to match your SX1509's address
//  pin selects. SX1509 breakout defaults to [0:0] (0x3E).
const uint8_t SX1509_ADDRESS = 0x3E;  // SX1509 I2C address (00)
//const byte SX1509_ADDRESS = 0x3F;  // SX1509 I2C address (01)
//const byte SX1509_ADDRESS = 0x70;  // SX1509 I2C address (10)
//const byte SX1509_ADDRESS = 0x71;  // SX1509 I2C address (11)

SensorBar mySensorBar(SX1509_ADDRESS);

// will try to avoid floating point math
const byte Kp = 1;
const byte Kd = 4;

const byte MAXSPEED = 255; // Max
int Lspeed = MAXSPEED;     // int not byte since may exceed 255 in calculations, but will ultimately be constrained
int Rspeed = MAXSPEED;



const int ButtonPin = 0;
int buttonVal = 0;
boolean goFlag = false;
int error = 0;
int lastError = 0;


// Romeo standard pins
int Lmotor = 5;                // M1 Speed Control
int Rmotor = 6;                // M2 Speed Control
int Ldir = 4;                  // M1 Direction Control
int Rdir = 7;                  // M1 Direction Control

// make it easier to change directions when changing motors
const boolean LFWD = HIGH;
const boolean LREV = LOW;
const boolean RFWD = LOW;
const boolean RREV = HIGH;

void setup() {
 //Default: the IR will only be turned on during reads.
  mySensorBar.setBarStrobe();
  //Other option: Command to run all the time
  //mySensorBar.clearBarStrobe();

  //Default: dark on light
  mySensorBar.clearInvertBits();
  //Other option: light line on dark
  //mySensorBar.setInvertBits();
  
  //Don't forget to call .begin() to get the bar ready.  This configures HW.
  uint8_t returnStatus = mySensorBar.begin();
 /*
  if(returnStatus)
  {
	  Serial.println("sx1509 IC communication OK");
  }
  else
  {
	  Serial.println("sx1509 IC communication FAILED!");
  }
  Serial.println();
  */
  
} // end setup()

void loop() {

  buttonVal = analogRead(ButtonPin);
  
  if (buttonVal < 30){       // button 1
    halt();
    goFlag = false;
  }
  else if (buttonVal < 175){ // button 2 
    //Command to run all the time - allow calibration
    mySensorBar.clearBarStrobe();
    int i = mySensorBar.getPosition();
  }
  else if (buttonVal < 360){  // button 3 
    //Default: the IR will only be turned on during reads.
    mySensorBar.setBarStrobe();
  }
  
//  else if (buttonVal < 540){  // button 4
//    // for future use
//  }

  else if (buttonVal < 800){  // button 5 - run line follower program
    goFlag = true;
    delay(3000); //  3 sec delay to back off
    // include visual indicator later
  }
  
  if (goFlag) {
    error = mySensorBar.getPosition(); // position gives distance from midline, i.e. the error

    // want to be as fast as possible, so will just slow down necessary wheel for correction
    // rather than trying to speed up one and slow the other
    if (error < 0){           // robot has drifted right; slow down L wheel
      Rspeed = MAXSPEED;
      Lspeed = MAXSPEED + (Kp * error) + (Kd * (error - lastError)); // plus since error is negative, will result in negative values for proportionate term
      Lspeed = constrain(Lspeed, 0, MAXSPEED);
      
    }
    else if (error > 0){      // robot has drifted L; slow down R wheel         
      Rspeed = MAXSPEED - (Kp * error) - (Kd * (error - lastError)); 
      Rspeed = constrain(Rspeed, 0, MAXSPEED);
      Lspeed = MAXSPEED;
    }
    else{                   // position is zero; full on both
      Rspeed = MAXSPEED;
      Lspeed = MAXSPEED;
    }

    fwd(Lspeed,Rspeed);

  } // end if (goFlag)
} // end loop()


void halt(void)               // Stop
{
  digitalWrite(Lmotor,LOW);   
  digitalWrite(Rmotor,LOW);      
}   


void fwd(byte l,byte r)       // Move forward
{
  analogWrite (Lmotor,l);     // PWM Speed Control
  digitalWrite(Ldir,LFWD);     
  analogWrite (Rmotor,r);    
  digitalWrite(Rdir,RFWD);     
}  

// don't need these functions for basic line follower
//void rev(byte l,byte r)       // Reverse
//{
//  analogWrite (Lmotor,l);
//  digitalWrite(Ldir,LREV);   
//  analogWrite (Rmotor,r);    
//  digitalWrite(Rdir,RREV);
//}  
//void spinR(byte l, byte r)
//{
//  analogWrite (Lmotor,l);
//  digitalWrite(Ldir,LFWD);    // L fwd, R rev to spin R (clockwise)
//  analogWrite (Rmotor,r);    
//  digitalWrite(Rdir,RREV);
//}  
//void spinL(byte l, byte r)
//{
//  analogWrite (Lmotor,l);
//  digitalWrite(Ldir,LREV);    // R fwd, L rev to spin L (counterclockwise)
//  analogWrite (Rmotor,r);    
//  digitalWrite(Rdir,RFWD);
//}  
