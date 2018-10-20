
#include "SparkFunLSM6DS3.h"
#include "Wire.h"

float resize(float x, float in_min, float in_max, float out_min, float out_max);

int pwm_y_left = 10; // y-Achse des linken Joysticks
int pwm_x_left = 5;  // x-Achse des linken Joysticks
int pwm_y_right = 6; // y-Achse des rechten Joysticks
int pwm_x_right = 9; // x-Achse des rechten Joysticks

void setup() {
  control_drone(50,50,50,50); // control_drone(x-Achse des linken Joysticks,  y-Achse des linken Joysticks , x-Achse des rechten Joysticks , y-Achse des rechten Joysticks)

}

void loop() {
  // put your main code here, to run repeatedly:

}

//Wertebereich 0 - 100 (50 Ruhelage)
void control_drone(int xl, int yl, int xr, int yr)// kennzeichnung ist x = X-Achse und l für links (rest ist analog) 
{
    analogWrite(pwm_x_left,(int)resize(xl,0,100,29,145));
    analogWrite(pwm_y_left,(int)resize(yl,0,100,20,148));

    analogWrite(pwm_x_right,resize(xr,0,100,29,145));
    analogWrite(pwm_y_right,resize(yr,0,100,20,151));
  
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
