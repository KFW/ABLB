/* ABLB
 * ActoBitty Line Bot 
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

// HOLD my code for later
//const int ButtonPin = 0;
//int buttonVal = 0;
//boolean goFlag = false;
//int currentPosition = 0;
//int lastPosition = 0;
//int error = 0;

// SparkFun MostBasicFollower code
// Define the states that the decision making machines uses:
#define IDLE_STATE 0
#define READ_LINE 1
#define GO_FORWARD 2
#define GO_LEFT 3
#define GO_RIGHT 4

uint8_t state = 0;

// Romeo standard pins
int Lmotor = 5;                // M1 Speed Control
int Rmotor = 6;                // M2 Speed Control
int Ldir = 4;                  // M1 Direction Control
int Rdir = 7;                  // M1 Direction Control
 

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

//  HOLD this section for later use
//  buttonVal = analogRead(ButtonPin);
//  
//  if (buttonVal < 30){       // button 1
//    halt();
//    goFlag = false;
//  }
//  /*
//  else if (buttonVal < 175){ // button 2 
//    // for initialization routine if needed	
//  }
//  else if (buttonVal < 360){  // button 3 
//    // for future use
//  }
//  else if (buttonVal < 540){  // button 4
//    // for future use
//  }
//  */
//  else if (buttonVal < 800){  // button 5
//    goFlag = true;
//  }
//  
//  if (goFlag) {
//    currentPosition = mySensorBar.getPosition();
//    error = currentPosition - lastPosition;
//    lastPosition = currentPosition;	
//  }

//SparkFun MostBasic Follower, modified for ABLB motor code

  uint8_t nextState = state;
  switch (state) {
  case IDLE_STATE:
    halt();       // Stops both motors
    nextState = READ_LINE;
    break;
  case READ_LINE:
    if( mySensorBar.getDensity() == 0)
    {
      nextState = IDLE_STATE;
      break;
    }
    else if( mySensorBar.getDensity() < 7 )
    {
      nextState = GO_FORWARD;
      if( mySensorBar.getPosition() < -50 )
      {
        nextState = GO_LEFT;
      }
      if( mySensorBar.getPosition() > 50 )
      {
        nextState = GO_RIGHT;
      }
    }
    else
    {
      nextState = IDLE_STATE;
    }
    break;
  case GO_FORWARD:
    fwd(128,128);
    nextState = READ_LINE;
    break;
  case GO_LEFT:
    fwd(16,160); // L a little slower R a bit faster
    nextState = READ_LINE;
    break;
  case GO_RIGHT:
    fwd(160, 16);
    nextState = READ_LINE;
    break;
  default:
    halt();       // Stops both motors
    break;
  }
  state = nextState;
  //delay(100);


} // end loop()

void halt(void)               // Stop
{
  digitalWrite(Lmotor,LOW);   
  digitalWrite(Rmotor,LOW);      
}   
void fwd(byte a,byte b)       // Move forward
{
  analogWrite (Lmotor,a);     // PWM Speed Control
  digitalWrite(Ldir,LOW);     // LOW for fwd
  analogWrite (Rmotor,b);    
  digitalWrite(Rdir,LOW);
}  
void rev(byte a,byte b)       // Reverse
{
  analogWrite (Lmotor,a);
  digitalWrite(Ldir,HIGH);   
  analogWrite (Rmotor,b);    
  digitalWrite(Rdir,HIGH);
}  
void spinR(byte a, byte b)
{
  analogWrite (Lmotor,a);
  digitalWrite(Ldir,LOW);    // L fwd, R rev to spin R (clockwise)
  analogWrite (Rmotor,b);    
  digitalWrite(Rdir,HIGH);
}  
void spinL(byte a, byte b)
{
  analogWrite (Lmotor,a);
  digitalWrite(Ldir,HIGH);    // R fwd, L rev to spin L (counterclockwise)
  analogWrite (Rmotor,b);    
  digitalWrite(Rdir,LOW);
}  
