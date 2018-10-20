int pwm_y_left = 5; // y-Achse des linken Joysticks, P4 - Grün
int pwm_x_left = 6;  // x-Achse des linken Joysticks, P3 - Rot
int pwm_y_right = 9; // y-Achse des rechten Joysticks, P2 - Schwarz
int pwm_x_right = 10; // x-Achse des rechten Joysticks, P1 - Weiß

void setup() {
  pinMode(pwm_y_left, OUTPUT); // P4 - Schub
  pinMode(pwm_x_left, OUTPUT); // P3 - Gieren
  
  pinMode(pwm_y_right, OUTPUT); // P2 - Nicken
  pinMode(pwm_x_right, OUTPUT); // P1 - Rollen

}

void loop() {
  digitalWrite(pwm_y_left,HIGH);
  digitalWrite(pwm_x_left,HIGH);
  digitalWrite(pwm_y_right,HIGH);
  digitalWrite(pwm_x_right,HIGH);

}
