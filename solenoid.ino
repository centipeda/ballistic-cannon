#include <Servo.h>

Servo serv;

void setup() {
  serv.attach(3);
  // put your setup code here, to run once:
  pinMode(5, OUTPUT);
  serv.write(35);
  digitalWrite(5, LOW);
  delay(500);
  // digitalWrite(4, HIGH);
  // delay(5000);
  digitalWrite(5, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  serv.write(35);
  delay(500); // 76 is 45 deg
  serv.write(76);
  delay(5000);
  digitalWrite(5, HIGH);
  delay(35);
  digitalWrite(5, LOW);
  delay(200);
  
  
}
