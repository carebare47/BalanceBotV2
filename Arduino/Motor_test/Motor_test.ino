
#include <L298N.h>
//Left motor pins
#define lEN 5
#define lIN1 6
#define lIN2 7

//Right motor pins
#define rEN 3
#define rIN1 8
#define rIN2 10

L298N right_motor(rEN, rIN1, rIN2);
L298N left_motor(lEN, lIN1, lIN2);


void setup() {
  // put your setup code here, to run once:

  right_motor.setSpeed(5); // an integer between 0 and 255
  left_motor.setSpeed(5); // an integer between 0 and 255

  Serial.begin(115200);
}

void serial_print(int i) {
  Serial.print(i);
  Serial.print(": ");
  Serial.println(left_motor.getSpeed());
}
int delay1 = 10;
int minimum = 0;
int maximum = 150;

void loop() {
  // put your main code here, to run repeatedly:

  right_motor.forward();
  left_motor.forward();

  // delay(400);

  for (int i = minimum; i < maximum; i++) {
    right_motor.setSpeed(i);
    left_motor.setSpeed(i);
    serial_print(i);
    right_motor.forward();
    left_motor.forward();
    delay(delay1);
  }



  for (int i = maximum; i > minimum; i--) {
    right_motor.setSpeed(i);
    left_motor.setSpeed(i);
    right_motor.forward();
    left_motor.forward();
    delay(delay1);    
  }

  right_motor.stop();
  left_motor.stop();

  delay(400);

  right_motor.backward();
  left_motor.backward();

  for (int i = minimum; i < maximum; i++) {
    right_motor.setSpeed(i);
    left_motor.setSpeed(i);
    right_motor.backward();
    left_motor.backward();
    delay(delay1);
  }



  for (int i = maximum; i > minimum; i--) {
    right_motor.setSpeed(i);
    left_motor.setSpeed(i);
    right_motor.backward();
    left_motor.backward();
    delay(20);
  }



  //return();

}
