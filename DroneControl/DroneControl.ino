int pwm_y_left = 5; // y-Achse des linken Joysticks, P4 - Grün
int pwm_x_left = 6;  // x-Achse des linken Joysticks, P3 - Rot
int pwm_y_right = 9; // y-Achse des rechten Joysticks, P2 - Schwarz
int pwm_x_right = 10; // x-Achse des rechten Joysticks, P1 - Weiß

class DroneControl {
  public:
    DroneControl(){ }
    ~DroneControl(){ }
  
    void initJoystickRestingValues(){
      int P4 = analogRead(0); //P4 - Schub
      int P3 = analogRead(1); //P3 - Gieren
      int P2 = analogRead(2); //P2 - Nicken
      int P1 = analogRead(3); //P1 - Rollen

      init_resting_p4 = ((float)P4)/1024.0 * 3.3;
      init_resting_p3 = ((float)P3)/1024.0 * 3.3;
      init_resting_p2 = ((float)P2)/1024.0 * 3.3;
      init_resting_p1 = ((float)P1)/1024.0 * 3.3;
    }

    void controlDrone(int *output, float perc_y_left, float perc_x_left, float perc_y_right, float perc_x_right){
      output[0] = calculatePWM(perc_y_left, init_resting_p4, 2.9120, 0.4026); //resting - 1.6950
      output[1] = calculatePWM(perc_x_left, init_resting_p3, 2.8450, 0.5754); //resting - 1.7470
      output[2] = calculatePWM(perc_y_right, init_resting_p2, 2.9520, 0.3949); //resting - 1.6960
      output[3] = calculatePWM(perc_x_right, init_resting_p1, 2.8580, 0.5796); //resting - 1.6470
    }

    void writeToDrone(int *pwmValues){
      analogWrite(pwm_y_left, pwmValues[0]); //P4 - Schub
      analogWrite(pwm_x_left, pwmValues[1]); // P3 - Gieren
      analogWrite(pwm_y_right, pwmValues[2]); // P2 - Nicken
      analogWrite(pwm_x_right, pwmValues[3]);  // P1 - Rollen
    }

    void writeToDroneTest(){
      analogWrite(pwm_y_left, 255); //P4 - Schub
      analogWrite(pwm_x_left, 255); // P3 - Gieren
      analogWrite(pwm_y_right, 255); // P2 - Nicken
      analogWrite(pwm_x_right, 255);  // P1 - Rollen
    }

    void writeToConsole(int *pwmValues){
      Serial.println(String("P1-Rollen: ") + pwmValues[3] + String(" P2-Nicken: ") +pwmValues[2] + String(" P3-Gieren: ") + pwmValues[1] + String(" P4-Schub: ") + pwmValues[0] );
    }

    void readFromJoysticks(int* joystickValues, int* pwmValues){
      joystickValues[0] = analogRead(0); //P4 - Schub
      joystickValues[1] = analogRead(1); //P3 - Gieren
      joystickValues[2] = analogRead(2); //P2 - Nicken
      joystickValues[3] = analogRead(3); //P1 - Rollen
      pwmValues[0] = map(joystickValues[0], 0,1024,0,255);
      pwmValues[1] = map(joystickValues[1], 0,1024,0,255);
      pwmValues[2] = map(joystickValues[2], 0,1024,0,255);
      pwmValues[3] = map(joystickValues[3], 0,1024,0,255);  
    }

  private:
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
    
    float init_resting_p4;
    float init_resting_p3;
    float init_resting_p2;
    float init_resting_p1;
};

void initArduino(){
  pinMode(pwm_y_left, OUTPUT); // P4 - Schub
  pinMode(pwm_x_left, OUTPUT); // P3 - Gieren
  pinMode(pwm_y_right, OUTPUT); // P2 - Nicken
  pinMode(pwm_x_right, OUTPUT); // P1 - Rollen
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
}

DroneControl dc;

void setup() {
  initArduino();
  dc.initJoystickRestingValues();
}

int pwmValues[4];
int joystickValues[4];
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

  if(manual){
    dc.readFromJoysticks(joystickValues, pwmValues);
  } else {
    dc.controlDrone(pwmValues, 0.0, 40.0,0.0,0.0);
  }
  
  dc.writeToDrone(pwmValues);
  //writeToConsole(pwmValues);
  //writeToDroneTest();
}
