int pwm_y_left = 5; // y-Achse des linken Joysticks, P4 - Grün
int pwm_x_left = 6;  // x-Achse des linken Joysticks, P3 - Rot
int pwm_y_right = 9; // y-Achse des rechten Joysticks, P2 - Schwarz
int pwm_x_right = 10; // x-Achse des rechten Joysticks, P1 - Weiß

void writeToDrone(int *pwmValues) {
  analogWrite(pwm_y_left, pwmValues[0]); //P4 - Schub
  analogWrite(pwm_x_left, pwmValues[1]); // P3 - Gieren
  analogWrite(pwm_y_right, pwmValues[2]); // P2 - Nicken
  analogWrite(pwm_x_right, pwmValues[3]);  // P1 - Rollen
}

void writeToDroneTest() {
  analogWrite(pwm_y_left, 255); //P4 - Schub
  analogWrite(pwm_x_left, 255); // P3 - Gieren
  analogWrite(pwm_y_right, 255); // P2 - Nicken
  analogWrite(pwm_x_right, 255);  // P1 - Rollen
}

void writeToConsole(int *pwmValues) {
  Serial.println(String("P1-Rollen: ") + pwmValues[3] + String(" P2-Nicken: ") + pwmValues[2] + String(" P3-Gieren: ") + pwmValues[1] + String(" P4-Schub: ") + pwmValues[0] );
}


void setup() {
  pinMode(pwm_y_left, OUTPUT); // P4 - Schub
  pinMode(pwm_x_left, OUTPUT); // P3 - Gieren

  pinMode(pwm_y_right, OUTPUT); // P2 - Nicken
  pinMode(pwm_x_right, OUTPUT); // P1 - Rollen

  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
}

int pwmValues[4];

void loop() {
  int P4 = analogRead(0); //P4 - Schub
  int P3 = analogRead(1); //P3 - Gieren

  int P2 = analogRead(2); //P2 - Nicken
  int P1 = analogRead(3); //P1 - Rollen

  //linker Joystick
  pwmValues[0] = map(P4, 0, 1024, 0, 255);
  pwmValues[1] = map(P3, 0, 1024, 0, 255);

  //rechter Joystick
  pwmValues[2] = map(P2, 0, 1024, 0, 255);
  pwmValues[3] = map(P1, 0, 1024, 0, 255);

  writeToDrone(pwmValues);
  writeToConsole(pwmValues);
  //writeToDroneTest();
}
