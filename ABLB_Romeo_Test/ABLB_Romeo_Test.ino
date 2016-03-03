/* ABLB_Romeo_Test
 * ActoBitty Line Bot 
 * Initial test with DFRobot Romeo V2
 * http://www.dfrobot.com/wiki/index.php/Romeo_V2-All_in_one_Controller_(R3)_(SKU:DFR0225)
 *  
 */

const int ButtonPin = 0;
int buttonVal = 0;

// Romeo standard pins
int Lspeed = 5;    // M1 Speed Control
int Rspeed = 6;    // M2 Speed Control
int Ldir = 4;    // M1 Direction Control
int Rdir = 7;    // M1 Direction Control
 

void setup() {
  // put your setup code here, to run once:

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
    halt();
  }
  else if (buttonVal < 360){  // button 3 
    rev(128,128);
    delay(1500);
    halt();
  }
  else if (buttonVal < 540){  // button 4
    spinR(128,128);
    delay(1000);
    halt();
  }
  else if (buttonVal < 800){  // button 5
    spinL(128,128);
    delay(1000);
    halt();
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
  digitalWrite(Ldir,HIGH);    // HIGH for fwd
  analogWrite (Rspeed,b);    
  digitalWrite(Rdir,HIGH);
}  
void rev(byte a,byte b)       // Reverse
{
  analogWrite (Lspeed,a);
  digitalWrite(Ldir,LOW);   
  analogWrite (Rspeed,b);    
  digitalWrite(Rdir,LOW);
}  
void spinR(byte a, byte b)
{
  analogWrite (Lspeed,a);
  digitalWrite(Ldir,HIGH);    // L fwd, R rev to spin R (clockwise)
  analogWrite (Rspeed,b);    
  digitalWrite(Rdir,LOW);
}  
void spinL(byte a, byte b)
{
  analogWrite (Lspeed,a);
  digitalWrite(Ldir,LOW);    // R fwd, L rev to spin L (counterclockwise)
  analogWrite (Rspeed,b);    
  digitalWrite(Rdir,HIGH);
}  