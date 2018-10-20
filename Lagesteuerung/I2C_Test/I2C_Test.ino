
#include "SparkFunLSM6DS3.h"
#include "Wire.h"

//Create a instance of class LSM6DS3
LSM6DS3 myIMU( I2C_MODE, 0x6A );  //I2C device address 0x6A

float resize(float x, float in_min, float in_max, float out_min, float out_max);

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
}

void loop()
{
  //Accelerometer
  Serial.print("\nAccelerometer:\n");
  Serial.print(" X1 = ");
  //Serial.println(myIMU.readFloatAccelX(), 4);
  float xsens=resize(myIMU.readFloatAccelX(),-1,1,0,100);
  Serial.println(xsens,4);
  Serial.print(" Y1 = ");
  //Serial.println(myIMU.readFloatAccelY(), 4);
  float ysens=resize(myIMU.readFloatAccelY(),-1,1,0,100);
  Serial.println(ysens,4);
  Serial.print(" Z1 = ");
  //Serial.println(myIMU.readFloatAccelZ(), 4);
  float zsens=resize(myIMU.readFloatAccelZ(),-1,1,0,100);
  Serial.println(zsens,4);
  
  delay(1000);
}


float resize(float x, float in_min, float in_max, float out_min, float out_max)
{
  if(x < in_min)
  {
   x=in_min; 
  }
  else if(x > in_max)
  {
   x=in_max; 
  }
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
