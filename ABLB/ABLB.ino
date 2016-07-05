/* ABLB
   ActoBitty Line Bot

   base robot Actobotics ActoBitty:
   https://www.servocity.com/html/actobitty_2_wheel_robot_kit.html#.VthCuvkrI-U

   added faster motors 270 rpm
   https://www.servocity.com/html/270_rpm_micro_gearmotorblocks.html#.VzjyE5MrIUE

   microcontroller board DFRobot Romeo 2 (has built in motor controller):
   http://www.dfrobot.com/wiki/index.php/Romeo_V2-All_in_one_Controller_(R3)_(SKU:DFR0225)

   mount panel for board:
   http://www.thingiverse.com/thing:1377159

   line follower array from Sparkfun:
   https://github.com/sparkfun/Line_Follower_Array
   https://learn.sparkfun.com/tutorials/sparkfun-line-follower-array-hookup-guide
   position provided ranges from -127 (far L) to 127 (far R)

   Note: values end up being discrete values (with # sensors in parentheses) if std electrical tape used for line: +/- 0, 31, 47, 63, 79, 95, 111, 127; 
   Value then falles back to 0 (with 0 sensors detected) if too far off to the side

   PID quick tutorial
   http://letsmakerobots.com/node/39972

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

<<<<<<< HEAD

const float Kp = 0.5;
||||||| merged common ancestors
// I know we should try to avoid floating point math, but probably will need fractional values for the constants
const float Kp = 7;
=======
// I know we should try to avoid floating point math, but probably will need fractional values for the constants
const float Kp = 9;
>>>>>>> origin/dev2
const float Ki = 0;
const float Kd = 0.5;

const byte MAXSPEED = 255;
const byte RUNSPEED = 48; // slow speed

const int TIMEDELAY = 1000; // time delay for backing off in ms

const int ButtonPin = 0;

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
  //Other option: Command to run all the time; will try that for faster response at expense of worse battery
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

  static int Lspeed = RUNSPEED; // int not byte since may exceed 255 in calculations, but will ultimately be constrained
  static int Rspeed = RUNSPEED;
  static boolean goFlag = false;

  int buttonVal = analogRead(ButtonPin);

  if (buttonVal < 30) {      // button 1 - use to pause if have to stop robot; returns bar to on only during read
    halt();
    goFlag = false;
    // set bar to read only during read - can use to reset after button 2 pressed
    mySensorBar.setBarStrobe();
  }
  else if (buttonVal < 175) { // button 2 - pause robot, but also allows calibration

    halt();
    goFlag = false;
    // turn bar on for calibration
    mySensorBar.clearBarStrobe();
  }
  //  else if (buttonVal < 360){  // button 3
  //    // for future use
  //  }
  //  else if (buttonVal < 540){  // button 4
  //    // for future use
  //  }

  else if (buttonVal < 800) { // button 5 - run line follower program
    goFlag = true;
    delay(TIMEDELAY); //  time delay to back off
    // include visual indicator later
  }

  static int I = 0;
  static int lastP = 0;

  if (goFlag) {
    int P = mySensorBar.getPosition(); // position gives distance from midline, i.e. the error
    I = (I + P);
    int D = (P - lastP);
    lastP = P;

    int correction = (P * Kp) + (I * Ki) + (D * Kd);

    // since not running full speed can speed up on side of error and slow down other side.
    Lspeed = RUNSPEED + correction;
    Rspeed = RUNSPEED - correction;
    Lspeed = constrain(Lspeed, 0, MAXSPEED);
    Rspeed = constrain(Rspeed, 0, MAXSPEED);
    fwd(Lspeed, Rspeed);

  } // end if (goFlag)
} // end loop()


void halt(void)               // Stop
{
  digitalWrite(Lmotor, LOW);
  digitalWrite(Rmotor, LOW);
}


void fwd(byte l, byte r)      // Move forward
{
  analogWrite (Lmotor, l);    // PWM Speed Control
  digitalWrite(Ldir, LFWD);
  analogWrite (Rmotor, r);
  digitalWrite(Rdir, RFWD);
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
