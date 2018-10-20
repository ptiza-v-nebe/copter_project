void setup() {
  Serial.begin(9600);
}

void loop() {
    int P4 = analogRead(0);
    int P3 = analogRead(1);
    int P2 = analogRead(2);
    int P1 = analogRead(3);
    Serial.println(String("P1: ") + P1 + String(" P2: ") + P2 + String(" P3: ") + P3 + String(" P4: ") + P4 );
}
