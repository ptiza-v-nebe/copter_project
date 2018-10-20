
#include "SparkFunLSM6DS3.h"
#include "Wire.h"

//Create a instance of class LSM6DS3
LSM6DS3 myIMU( I2C_MODE, 0x6A );  //I2C device address 0x6A

float resize(float x, float in_min, float in_max, float out_min, float out_max);

int control_mode = 2;
int pwm_crosswise = 5;
int pwm_along = 6;
int crosswise_joystick = 1;
int along_joystick = 2;
int height_joystick = 3;
int rotate_joystick = 4;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);  
  //Call .begin() to configure the IMUs
  if( myIMU.begin() != 0 )
  {
    Serial.println("Device error");
  }
  else  
  {
      Serial.println("Device OK!");
  }
  pinMode(pwm_crosswise,OUTPUT);//PWM in X
  pinMode(control_mode,INPUT);//set the contol mode
}

void loop()
{
  //Accelerometer
  float xsens=resize(myIMU.readFloatAccelX(),-1,1,0,100); //resize the scale from G to %
  float ysens=resize(myIMU.readFloatAccelY(),-1,1,0,100);
  float zsens=resize(myIMU.readFloatAccelZ(),-1,1,0,100);
  
  Serial.print("\nAccelerometer:\n"); // show the values on the serial monitor
  Serial.print(" X1 = ");
  Serial.println(xsens,4);
  Serial.print(" Y1 = ");
  Serial.println(ysens,4);
  Serial.print(" Z1 = ");
  Serial.println(zsens,4);

  if(digitalRead(control_mode) == LOW)
  {
    float crosswise = analogRead(crosswise_joystick);
    float along = analogRead(along_joystick);
    analogWrite(pwm_crosswise,resize(crosswise,0,100,29,145));
    analogWrite(pwm_along,resize(along,0,100,20,151));
  }else
  {
    analogWrite(pwm_crosswise,resize(xsens,0,100,29,145));
    analogWrite(pwm_along,resize(xsens,0,100,20,151));
  }
  
  delay(1000);
}


float resize(float x, float in_min, float in_max, float out_min, float out_max)//funktion to change the scale 
{
  if(x < in_min) // limit the value 
  {
   x=in_min; 
  }
  else if(x > in_max)
  {
   x=in_max; 
  }
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
