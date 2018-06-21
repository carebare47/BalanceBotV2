/*-----( Import needed libraries )-----*/
#include <SoftwareSerial.h>
#include <string.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <L298N.h>


//#define IMU_test

//#include "LMotorController.h"
//#include "I2Cdev.h"

//#include "MPU6050_6Axis_MotionApps20.h"

//#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
//    #include "Wire.h"
//#endif


#define LOG_INPUT 0
#define MANUAL_TUNING 0
#define LOG_PID_CONSTANTS 0 //MANUAL_TUNING must be 1
#define MOVE_BACK_FORTH 0

#define MIN_ABS_SPEED 30
#include <PID_v1.h>
/*-----( Declare Constants and Pin Numbers )-----*/
#define HC_05_TXD_ARDUINO_RXD 15
#define HC_05_RXD_ARDUINO_TXD 14
#define HC_05_SETUPKEY        17
#define HC_05_PWR1            18  // Connect in parallel to HC-05 VCC
#define HC_05_PWR2            19  // Connect in parallel to HC-05 VCC


// Calculate based on max input size expected for one command
#define INPUT_SIZE 30

#define ppr 2752

//pin definition
#define lEN 5
#define lIN1 6
#define lIN2 4

#define rEN 9
#define rIN1 8
#define rIN2 7
//L298N right_motor(rEN, rIN1, rIN2);
L298N left_motor(lEN, lIN1, lIN2);

//#define aHome 186
#define aHome 195


#define lIntPin 3  // the pin we are interested in
#define lDirPin 11




//const byte leftIntPin = 3;
//const byte rightIntPin = 3;
//volatile byte rFlag = false;
volatile bool lFlag = false;
//volatile byte rDir = false;
//volatile byte lDir = false;
volatile signed long lCount = 0;
//signed long rCount = 0;

MPU6050 mpu6050(Wire);

/*-----( PID variables )------*/
//Define Variables we'll be connecting to

double aInput, aOutput;
double pSetpoint, pInput, pOutput;
double aSetpoint = aHome;


//Specify the links and initial tuning parameters
double aKp = 30, aKi = 0.1, aKd = 1;
double pKp = 10, pKi = 0.01, pKd = 1;
PID PIDp(&pInput, &pOutput, &pSetpoint, pKp, pKi, pKd, DIRECT);
PID PIDa(&aInput, &aOutput, &aSetpoint, aKp, aKi, aKd, DIRECT);


/*-----( Declare objects )-----*/
SoftwareSerial BTSerial(HC_05_TXD_ARDUINO_RXD, HC_05_RXD_ARDUINO_TXD); // RX | TX
/*-----( Declare Variables )-----*/
char inData[80];
char commandString[80];
byte index = 0;


// Install Pin change interrupt for a pin, can be called multiple times

void pciSetup(byte pin)
{
  *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
  PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
  PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

unsigned long time;
unsigned long timer_x = 0;

void setup()   /****** SETUP: RUNS ONCE ******/
{



  pinMode(HC_05_SETUPKEY, OUTPUT);  // this pin will pull the HC-05 pin 34 (key pin) HIGH to switch module to AT mode

  digitalWrite(HC_05_SETUPKEY, HIGH);  // Set command mode when powering up

  pinMode(HC_05_PWR1, OUTPUT);      // Connect in parallel to HC-05 VCC
  pinMode(HC_05_PWR2, OUTPUT);      // Connect in parallel to HC-05 VCC
  pinMode(13, OUTPUT);



  Serial.begin(9600);   // For the Arduino IDE Serial Monitor
  Serial.println("YourDuino.com HC-05 Bluetooth Module AT Command Utility V1.02");
  Serial.println("Set Serial Monitor to 'Both NL & CR' and '9600 Baud' at bottom right");
  Serial.println("Vcc Power Up DELAY");
  //delay(2000);
  Serial.println("Applying VCC Power. LED should blink SLOWLY: 2 Seconds ON/OFF");
  //digitalWrite(HC_05_PWR1, HIGH); // Power VCC
  //digitalWrite(HC_05_PWR2, HIGH);
  delay(200);
  Serial.println("Enter AT commands in top window.");
  Wire.begin();
  mpu6050.begin();
  //mpu6050.calcGyroOffsets(true);
  BTSerial.begin(9600);  // HC-05 default speed in AT command mode

  PIDa.SetMode(AUTOMATIC);
  PIDp.SetMode(AUTOMATIC);
  PIDp.SetOutputLimits(-255, 255);
  PIDa.SetOutputLimits(-255, 255);

  pinMode(lIntPin, INPUT_PULLUP);
  //pinMode(11, INPUT_PULLUP);
  //pinMode(rightIntPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(lIntPin), leftCB, RISING);
 // attachInterrupt(digitalPinToInterrupt(rightIntPin), rightCB, CHANGE);

//  pinMode(lIntPin, INPUT_PULLUP);     //set the pin to input
//  //digitalWrite(lIntPin, HIGH); //use the internal pullup resistor
//  //PBintPort::attachInterrupt(lIntPin, leftCB, RISING); // attach a PinChange Interrupt to our pin on the rising edge
//  attachPinChangeInterrupt(lIntPin, leftCB, RISING);
//  
//  // pin change interrupt (example for D12)
//  PCMSK0 |= bit (PCINT4);  // want pin 12
//  PCIFR  |= bit (PCIF0);    // clear any outstanding interrupts
//  PCICR  |= bit (PCIE0);    // enable pin change interrupts for D8 to D12
//  pinMode (12, INPUT_PULLUP);
//


  timer_x = millis();


  //left_motor.setSpeed(140);
  //left_motor.forward();
  //delay(1000);
  //left_motor.stop();
  //left_motor.backward();
  //delay(500);
  //left_motor.setSpeed(240);
  //delay(250);
  //left_motor.setSpeed(100);
  //delay(250);
  //left_motor.stop();
  Serial.println("Setup finished");


}//--(end setup )---

bool rollRX = false;
bool pitchRX = false;
bool sensRX = false;
bool x_posRX = true;
bool rollRX2 = false;
bool pitchRX2 = false;

float x_position = 0.0;
float roll = 0;
float pitch = 0;
int sensitivity = 0;
bool flag = true;
enum rx { UNDEF, s, r, p };
int timer1 = millis();
int timer2 = millis();
int duration = 200;
void do_robot_go(void);
void loop()   /****** LOOP: RUNS CONSTANTLY ******/
{

  mpu6050.update();
  // aInput = mpu6050.getAngleX();//Probably wrong axis

  //  Serial.print("X:");
  //  Serial.println(mpu6050.getAngleX());
  //  Serial.print("Y:");
  //  Serial.println(mpu6050.getAngleY());
  //  Serial.print("Z:");
  //  Serial.println(mpu6050.getAngleZ());






  timer1 = millis() - timer1;
  // READ from HC-05 and WRITE to Arduino Serial Monitor
  if (BTSerial.available()) {

    // Get next command from Serial (add 1 for final 0)
    char input[INPUT_SIZE + 1];
    byte size = BTSerial.readBytes(input, INPUT_SIZE);
    // Add the final 0 to end the C string
    input[size] = 0;

    // Read each command pair
    char* command = strtok(input, "&");
    while (command != 0)
    {
      // Split the command in two values
      char* separator = strchr(command, '=');
      if (separator != 0)
      {
        // Actually split the string in 2: replace ':' with 0
        *separator = 0;
        char ID = command[0];
        ++separator;
        int iPosition = atoi(separator);
        float fPosition = atof(separator);
        if (iPosition != 999) {
          switch (ID) {
            case 's':
              sensitivity = iPosition;
              //sensRX = true;
              // Serial.print(position); Serial.print(",");
              break;
            case 'r':
              roll = fPosition;
              rollRX = true;
              aSetpoint = roll;
              Serial.print("roll");
              Serial.println(fPosition);
              //Serial.print(position); Serial.print(",");
              break;
            case 'p':
              pitch = fPosition;
              pSetpoint = pitch;
              pitchRX = true;
              Serial.print("pitch");
              Serial.println(fPosition);
              //Serial.print(position); Serial.print(",");
              break;
            case 'x':
              Serial.print("x");
              Serial.println(fPosition);
              x_position = fPosition;
              x_posRX = true;
              break;
          }

          if (pitchRX) {
            pitchRX2 = true;
          }
          if (rollRX) {
            rollRX2 = true;
          }
          rollRX = false;
          pitchRX = false;
          //char buf[INPUT_SIZE + 1];
          //sprintf(buf, "%s = %d", &command[0], position);
          //Serial.write(buf);
          // Do something with servoId and position
        }
      }
      // Find the next command in input string
      command = strtok(0, "&");
    }




    //char buf2[30];
    //sprintf(buf2, "\nMessage period = %d\n", timer1);
    //Serial.write(buf2);
  }


  //pitchRX2 = true;
  //x_posRX = false;

  if ((pitchRX2 || rollRX2) && (!x_posRX)) {
    timer_x = millis();
    //int tracker = (int)((lCount + rCount ) / 2); //???
    int tracker = (int)lCount;
    if (roll > 1.0) {
      roll = 1.0;
    }
    if (pitch > 1.0) {
      pitch = 1.0;
    }
    if (roll < -1.0) {
      roll = -1.0;
    }
    if (pitch < -1.0) {
      pitch = -1.0;
    }
    if (roll < 0.0) {
      roll = 0.0 - roll;
    }
    if (pitch < 0.0) {
      pitch = 0.0 - pitch;
    }



    int y = map(int(roll * 100), 0, 100, 0, 254);
    analogWrite(13, y);
    //Maybe put offsets here sometime?
    //    -0.2
    //    -0.65
    //
    //    +0.2
    //    +0.65/2 = 0.325
    //    +0.525
    Serial.print(roll);
    //Serial.print((roll-0.35));
    Serial.print(",");
    //Serial.println(pitch-0.525);
    Serial.println(pitch);
    pitchRX2 = false;
    rollRX2 = false;
    // delay(10);


  } else if (x_posRX) {
    timer_x = millis();

    //Below line should go: While robot isn't settled at it's goal...
    aSetpoint = aHome; //upright
    pSetpoint = x_position; //where we've told it to go
    //    do_robot_go(); //go do robot

    //If we lose contact for 750ms, give angle mode a chance to take over.
    if ((millis() - timer_x) > 700) {
      x_posRX = false;
    }


  } else if (((!x_posRX) && (!pitchRX2) && (!rollRX2)) && ((millis() - timer_x) > 800)) {


    stop_robot(timer_x);
    pSetpoint = x_position; //where we've told it to go last hopefully
    aSetpoint = aHome;
  }


  if (lFlag){
    
    lFlag = false;
  }

  do_robot_go(); //go do robot




}//--(end main loop )---

int whereAmI(void) {
  //int tracker = (int)((lCount + rCount ) / 2);
  int tracker = (int)lCount;
  return tracker;
}

bool haltPos = true;
void stop_robot(int timer) {
  if (haltPos) {
    x_position = whereAmI();
  }

  //don't keep taking new readings or we'll at best drift into something
  haltPos = false;

  //if we've got a connection back
  if ((millis() - timer) < 7) {
    //Allow the next connection break to halt us once more
    haltPos = true;
  }
}


void do_robot_go(void) {


//  int tracker = (int)((lCount + rCount ) / 2);//oh, here it is
  int tracker = (int)lCount;
  mpu6050.update();
  aInput = ((mpu6050.getAngleX()) + 200); //Probably wrong axis
  pInput = tracker;//write this function
  PIDa.Compute();
  //PIDp.Compute();
  //Serial.println("");





  //pSetpoint = 0;
  //pInput = tracker;
  //PIDp.Compute();
  float angleWeight = 0.8;
  float positionWeight = 0.2;
  float fSpeed = ((aOutput * angleWeight) + (pOutput * positionWeight));
//  float fSpeed = aOutput;
  if (fSpeed > 0.0) {
  //  right_motor.forward();
    left_motor.backward();
  } else if (fSpeed < 0.0) {
//    right_motor.backward(); 
    left_motor.forward();
    fSpeed = fSpeed - ((2) * fSpeed);
  }
//  right_motor.setSpeed(fSpeed);
  left_motor.setSpeed(fSpeed);

  Serial.write(27);       // ESC command
  Serial.print("[2J");    // clear screen command
  Serial.write(27);
  Serial.print("[H");     // cursor to home command
  Serial.print("Tracker:");
  Serial.print(lCount);
  Serial.print("\tInput:");
  Serial.print(aInput);
  Serial.print("\tSetpoint:");
  Serial.print(aSetpoint);
  Serial.print("\tOutput:");
  Serial.print(fSpeed);

  for (int i = 0; i < 10; i++) {
    Serial.println("");
  }
  delay(50);

}
/*-----( Declare User-written Functions )-----*/
//NONE

//*********( THE END )***********



//
//ISR (PCINT0_vect)
//{
//  lFlag = true;
//  if (PINB & bit (4)) { // if it was high
//    lCount++;
//  } else {
//    lCount--;
//  }  // end of PCINT2_vect
//}
void leftCB(void) {
  lFlag = true;
  bool direc = (digitalRead(lDirPin) == HIGH) ? true: false;
  if (digitalRead(lIntPin) == HIGH){
  if (direc){
    lCount--;
  } else {
    lCount++;
  }
  }
  //(digitalRead(lDirPin)) ? lCount-- : lCount++;
}
//void rightCB(void) {
//  rFlag = true;
//  rDir = digitalRead(5);
//  (rDir) ? rCount-- : rCount++;
//}

