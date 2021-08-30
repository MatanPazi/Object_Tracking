//Used "ARDUINO PID BALANCE CODE":
//http://electronoobs.com/eng_arduino_tut100_code1.php 
//
//And "PID Balance+Ball | full explanation & tuning":
//https://www.youtube.com/watch?v=JFTJ2SS4xyA&t=132s
//
//By Electronoobs as a reference

#include <Servo.h>

Servo myservo;  						//Create servo object to control a servo

const int DC = 112;						//Servo angle which gives a horizontal beam
int BallPos = 0;
float ErrPrev = 0.0;
int RefBallPos = 319;					//Ball position when in middle of the camera
float Err = 0.0;
float Integral = 0.0;
int ServoPos = 0;
float Ki = 0.02;						//Integral part of PID
float Kp = 0.2;							//Proportional part of PID
float rate = 1.0;						//Update rate (1 [msec])
float Kd = 0.002;						//Derivative part of PID
float Prop = 0.0;
int Diff = 0;
float D = 0.0;
unsigned long Time;
unsigned long time1 = 0;
unsigned long time2 = 0;
int x = 0;
int xprev = 0;
byte buf[2];
void setup()
{
  Serial.begin(115200); 				// Opens serial port, sets data rate to 115200 bits per sec  
  myservo.attach(9);  					// Attaches the servo on pin 9 to the servo object
  delay(1000);
  myservo.write(DC);					// Go to horizontal state
  delay(5000);
  Time = millis();
}
void loop() 
{
  while ( millis() < Time + int(rate))
  {
    // Waiting.
  }
  Time = millis();
  Serial.write('!');    				//Indicating to Python to "Send me data!"
// reply only once 2 bytes have arrived:
  while (Serial.available() < 2) 
  {
    // Waiting
  }
//  Serial.write('>');    				//For debugging. Indicating to Python to "Start reading data!"
// read the bytes in the buffer. LSB first, then MSB           
  BallPos = ((Serial.read()) | (Serial.read() << 8));
  Err = -(RefBallPos - BallPos);    	//Needed sign change due to experiments
  Prop = Err*Kp;
  Diff = ((Err - ErrPrev)*1000.0)/rate;	//Multiplication could have simply been represented in Kd
  D = Diff * Kd;
  ErrPrev = Err;
  Integral = Integral + Err*Ki;
  if (Integral > 1)						//Not allowing for much integral action, harmful in ball & beam application.
  {										//But still want a little to achieve 0 steady state error
    Integral = 1;
  }
  if (Integral < -1)
  {
    Integral = -1;
  }
  
  ServoPos = DC + Prop + Integral + D;
  if (ServoPos > DC+30)					//I found that 30 deg is enough actuation range to affect the ball's position.
  {
    ServoPos = DC+30;
  }
  if (ServoPos < DC-30)
  {
    ServoPos = DC-30;
  }  

  myservo.write(ServoPos);

//// For debugging, send data to display in Python
//  buf[0] = int(ServoPos) & 255;
//  buf[1] = (int(ServoPos) >> 8)  & 255;
//
//  Serial.write(buf, sizeof(buf));
}
