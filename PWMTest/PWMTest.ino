int pwm_y_left = 5; // y-Achse des linken Joysticks, P4 - Grün
int pwm_x_left = 6;  // x-Achse des linken Joysticks, P3 - Rot
int pwm_y_right = 9; // y-Achse des rechten Joysticks, P2 - Schwarz
int pwm_x_right = 10; // x-Achse des rechten Joysticks, P1 - Weiß

float calculatePWM(float input, float restingValue, float highValue, float lowValue){
     //floats dürfen nicht mit == verglichen werden, deswegen wird ein intervall festgelegt
    if(input < 0.001 && input > -0.001)
        return restingValue/3.3 * 255;

    //bei positiven prozentzahlen
    if(input > 0)
        return (restingValue - (restingValue - lowValue)*input/100.0)/3.3 * 255;

    //bei negativen prozentzahlen
    if(input < 0)
        return (restingValue - (highValue - restingValue)*input/100.0)/3.3 * 255;
}

void controlDrone(int *output, float perc_y_left, float perc_x_left, float perc_y_right, float perc_x_right){
  output[0] = calculatePWM(perc_y_left, 1.6950, 2.9120, 0.4026);
  output[1] = calculatePWM(perc_x_left, 1.7470, 2.8450, 0.5754);
  output[2] = calculatePWM(perc_y_right, 1.6960, 2.9520, 0.3949);
  output[3] = calculatePWM(perc_x_right, 1.6470, 2.8580, 0.5796);
}

void writeToDrone(int *pwmValues){
  //write values to controllers
  int y_left = pwmValues[0];
  int x_left = pwmValues[1];
  int y_right = pwmValues[2];
  int x_right = pwmValues[3];
  
  analogWrite(pwm_y_left, y_left);
  analogWrite(pwm_x_left, x_left);
  analogWrite(pwm_y_right, y_right); 
  analogWrite(pwm_x_right, x_right);  
}

void writeToDroneTest(){
  analogWrite(pwm_y_left, 128);
  analogWrite(pwm_x_left, 128);
  analogWrite(pwm_y_right, 128); 
  analogWrite(pwm_x_right, 128);  
}

void writeToConsole(int *pwmValues){
  int y_left = pwmValues[0];
  int x_left = pwmValues[1];
  int y_right = pwmValues[2];
  int x_right = pwmValues[3];
  Serial.println( String("") + y_left + String("; ") + x_left + String("; ") + y_right + String("; ") + x_right);
}

void setup() {
  pinMode(pwm_y_left, OUTPUT);
  pinMode(pwm_x_left, OUTPUT);
  pinMode(pwm_y_right, OUTPUT);
  pinMode(pwm_x_right, OUTPUT);
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  
}


enum State {
  idle,
  connecting,
  startEngines,
  disengageFlight,
  land,
  stopEngines,
};

State actualState = idle;
State nextState = idle;
unsigned long t = 0;
int pwmValues[4];

void loop() {
    switch(actualState){
      case idle:
        controlDrone(pwmValues, 0.0, 0.0, 0.0, 0.0);     
        if( t > 950 && t < 1050)
          nextState = connecting;
        break;
      case connecting:
        controlDrone(pwmValues, -100.0, 0.0, 0.0, 0.0);
        if( t > 1950 && t < 2050)
          nextState = startEngines;
        break;
      case startEngines:
        controlDrone(pwmValues, -55.0, 67.0, -60.0, -62.0);
        if( t > 3650 && t < 3750)
          nextState = disengageFlight;
        break;
      case disengageFlight:
        controlDrone(pwmValues, 28.0, 0.0, 0.0, 0.0);
        if( t > 5150 && t < 5250)
          nextState = land;
        break;
      case land:
        controlDrone(pwmValues, 0.0, 0.0, 0.0, 0.0);
        if( t > 6550 && t < 6650)
          nextState = stopEngines;
      case stopEngines:
        controlDrone(pwmValues, -55.0, 67.0, -60.0, -62.0);
        if( t > 9650 && t < 9750)
          nextState = idle;
        break;
      }
        
    actualState = nextState;

    writeToDrone(pwmValues);
    //writeToConsole(pwmValues);
    //writeToDroneTest();
    //Serial.println(actualState);
    
    t = millis(); //get time since start of arduino in milliseconds
}
