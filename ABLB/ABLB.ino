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

SensorBar mySensorBar( 0x3E ); // default address

const int ButtonPin = 0;
int buttonVal = 0;



// Romeo standard pins
int Lspeed = 5;                // M1 Speed Control
int Rspeed = 6;                // M2 Speed Control
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
  
}

}

void loop() {
{
  buttonVal = analogRead(ButtonPin);
  
  if (buttonVal < 30){       // button 1
    halt();                  
  }
  else if (buttonVal < 175){ // button 2 
    fwd(128,128);            // motors fwd 1/2
    delay(1500);             // go for 1.5 seconds
    spinL(128,128);
    delay(400);
    fwd(128,128);
    delay(1500);
    halt();
  }
  else if (buttonVal < 360){  // button 3 
    // for later
  }
  else if (buttonVal < 540){  // button 4
    // for later
  }
  else if (buttonVal < 800){  // button 5
    // for later
  }
}

}

void halt(void)               // Stop
{
  digitalWrite(Lspeed,LOW);   
  digitalWrite(Rspeed,LOW);      
}   
void fwd(byte a,byte b)       // Move forward
{
  analogWrite (Lspeed,a);     // PWM Speed Control
  digitalWrite(Ldir,LOW);    // LOW for fwd
  analogWrite (Rspeed,b);    
  digitalWrite(Rdir,LOW);
}  
void rev(byte a,byte b)       // Reverse
{
  analogWrite (Lspeed,a);
  digitalWrite(Ldir,HIGH);   
  analogWrite (Rspeed,b);    
  digitalWrite(Rdir,HIGH);
}  
void spinR(byte a, byte b)
{
  analogWrite (Lspeed,a);
  digitalWrite(Ldir,LOW);    // L fwd, R rev to spin R (clockwise)
  analogWrite (Rspeed,b);    
  digitalWrite(Rdir,HIGH);
}  
void spinL(byte a, byte b)
{
  analogWrite (Lspeed,a);
  digitalWrite(Ldir,HIGH);    // R fwd, L rev to spin L (counterclockwise)
  analogWrite (Rspeed,b);    
  digitalWrite(Rdir,LOW);
}  
