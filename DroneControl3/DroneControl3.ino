
#include "SparkFunLSM6DS3.h"
#include "Wire.h"

//Create a instance of class LSM6DS3
LSM6DS3 myIMU( I2C_MODE, 0x6A );  //I2C device address 0x6A

//define pins
int pwm_y_left = 5; // y-Achse des linken Joysticks, P4 - Grün
int pwm_x_left = 6;  // x-Achse des linken Joysticks, P3 - Rot
int pwm_y_right = 9; // y-Achse des rechten Joysticks, P2 - Schwarz
int pwm_x_right = 10; // x-Achse des rechten Joysticks, P1 - Weiß

float resize(float x, float in_min, float in_max, float out_min, float out_max)//funktion to change the scale 
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

class DroneControl {
  public:
    DroneControl(){ }
    ~DroneControl(){ }

    void initJoystickRestingValues(){
      init_resting_p4 = analogRead(0); //P4 - Schub
      init_resting_p3 = analogRead(1); //P3 - Gieren
      init_resting_p2 = analogRead(2); //P2 - Nicken
      init_resting_p1 = analogRead(3); //P1 - Rollen
    }

    void controlDrone(float *output, float perc_y_left, float perc_x_left, float perc_y_right, float perc_x_right){
      if(perc_y_left > 100.0)
        perc_y_left = 100.0;
      if(perc_y_left < -100.0)
        perc_y_left = -100.0;
        
      if(perc_x_left > 100.0)
        perc_x_left = 100.0;
      if(perc_x_left < -100.0)
        perc_x_left = -100.0;
        
      if(perc_y_right > 100.0)
        perc_y_right = 100.0;
      if(perc_y_right < -100.0)
        perc_y_right = -100.0;
        
      if(perc_x_right > 100.0)
        perc_x_right = 100.0;
      if(perc_x_right < -100.0)
        perc_x_right = -100.0;
        
      output[0] = calculatePWM(perc_y_left, init_resting_p4, 775.0, 226.0); // P4
      output[1] = calculatePWM(perc_x_left, init_resting_p3, 825.0, 286.0); // P3
      output[2] = calculatePWM(perc_y_right, init_resting_p2, 804.0, 236.0); // P2
      output[3] = calculatePWM(perc_x_right, init_resting_p1, 793.0, 246.0); // P1
    }

    void writeToDrone(float *pwmValues){
      analogWrite(pwm_y_left, (int)pwmValues[0]); //P4 - Schub
      analogWrite(pwm_x_left, (int)pwmValues[1]); // P3 - Gieren
      analogWrite(pwm_y_right, (int)pwmValues[2]); // P2 - Nicken
      analogWrite(pwm_x_right, (int)pwmValues[3]);  // P1 - Rollen
    }

    void writeToDroneTest(){
      analogWrite(pwm_y_left, 255); //P4 - Schub
      analogWrite(pwm_x_left, 255); // P3 - Gieren
      analogWrite(pwm_y_right, 255); // P2 - Nicken
      analogWrite(pwm_x_right, 255);  // P1 - Rollen
    }

    void writeToConsole(float *values){
      Serial.println(String("P1-Rollen: ") + values[3] + String(" P2-Nicken: ") + values[2] + String(" P3-Gieren: ") + values[1] + String(" P4-Schub: ") + values[0] );
    }

    void readFromJoysticks(float* output){
      float rawValues[4];
      rawValues[0] = (float)analogRead(0); //P4 - Schub
      rawValues[1] = (float)analogRead(1); //P3 - Gieren
      rawValues[2] = (float)analogRead(2); //P2 - Nicken
      rawValues[3] = (float)analogRead(3); //P1 - Rollen
      /*output[0] = (float)analogRead(0); //P4 - Schub
      output[1] = (float)analogRead(1); //P3 - Gieren
      output[2] = (float)analogRead(2); //P2 - Nicken
      output[3] = (float)analogRead(3); //P1 - Rollen*/
     
      convertJoystickValue(rawValues[0], &output[0], init_resting_p4, 775, 226);
      convertJoystickValue(rawValues[1], &output[1], init_resting_p3, 825, 286);
      convertJoystickValue(rawValues[2], &output[2], init_resting_p2, 804, 236);
      convertJoystickValue(rawValues[3], &output[3], init_resting_p1, 793, 246);
    }

    void convertJoystickValue(float joyRawValue, float* output, int init_resting, int upValue, int downValue){
      if(joyRawValue > init_resting){
        *output = resize((float)joyRawValue, (float)upValue, (float)init_resting,  -100.0, 0.0);
      }
      
      if(joyRawValue == init_resting){
        *output = 0.0;
      }

      if(joyRawValue < init_resting){
        *output = resize((float)joyRawValue, (float)init_resting, (float)downValue, 0.0, 100.0);
      }
    }

    void readFromIMU(float* output){
      output[2] = -(myIMU.readFloatAccelY()/0.5)*100.0;
      output[3] = -(myIMU.readFloatAccelX()/0.5*100.0);
    }

  private:
    float calculatePWM(float input, int restingValue, int highValue, int lowValue) { //input is now raw joystick value
     //floats dürfen nicht mit == verglichen werden, deswegen wird ein intervall festgelegt
      if(input < 0.001 && input > -0.001)
        return restingValue/1024.0 * 255.0;

      //bei positiven prozentzahlen
      if(input > 0.0)
        return (restingValue - (restingValue - lowValue)*input/100.0)/1024.0 * 255.0;

       //bei negativen prozentzahlen
      if(input < 0.0)
        return (restingValue - (highValue - restingValue)*input/100.0)/1024.0 * 255.0;
    }
    int init_resting_p4;
    int init_resting_p3;
    int init_resting_p2;
    int init_resting_p1;
};

void initArduino(){
  //init pins
  pinMode(pwm_y_left, OUTPUT); // P4 - Schub
  pinMode(pwm_x_left, OUTPUT); // P3 - Gieren
  pinMode(pwm_y_right, OUTPUT); // P2 - Nicken
  pinMode(pwm_x_right, OUTPUT); // P1 - Rollen

  //init serial
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  //init i2c
  if( myIMU.begin() != 0 )
    Serial.println("Device error");
  else  
    Serial.println("Device OK!");
}



DroneControl dc;

void setup() {
  initArduino();
  dc.initJoystickRestingValues();
}

bool manual = true;

void loop() {   
  if(Serial.available() > 0){
    char command = Serial.read();
    if(!manual && command == '0'){ // send '0' char ascci
      manual = true;
      Serial.println("manual modus");
    }
    if(manual && command == '1'){ // send '1' char ascii
      manual = false;
      Serial.println("autopilot modus");
    }
  }

  float joystickValues[4];
  float imuValues[4] = {0,0,0,0};
  float resultPWMValues[4];
  
  if(manual){
    dc.readFromJoysticks(joystickValues);
    dc.controlDrone(resultPWMValues, joystickValues[0], joystickValues[1], joystickValues[2], joystickValues[3]);
  } else {
    dc.readFromJoysticks(joystickValues);
    dc.readFromIMU(imuValues);
    dc.controlDrone(resultPWMValues, joystickValues[0], joystickValues[1], imuValues[2], imuValues[3]);
  }
  dc.writeToDrone(resultPWMValues);
  //dc.writeToConsole(resultPWMValues);
}
