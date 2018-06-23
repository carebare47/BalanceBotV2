#define aHomeConst -45.6
#include <PID_v1.h>
#include <L298N.h>
#include <SoftwareSerial.h>
#include "bluetooth.h"
#include "IMU.h"
#include "serialProcessing.h"
#include "control.h"
//#define properStart
//////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////PIN MAPPINGS///////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
////Bluetooth pins
//#define HC_05_TXD_ARDUINO_RXD 15
//#define HC_05_RXD_ARDUINO_TXD 14
//#define HC_05_SETUPKEY        17
//

//Left motor pins
#define lEN 5
#define lIN1 6
#define lIN2 7

//Right motor pins
#define rEN 3
#define rIN1 8
#define rIN2 10


#define lIntPin 3  // the pin we are interested in
#define lDirPin 11
#define rIntPin 4
#define rDirPin 9

//////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////END OF PIN MAPPINGS////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////VARIABLES//////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//This one's good: -45.9
//#define aHome -45.9
//45.5
//46.3
//#define angleMax -45.56
//#define aHome -45.6

//#define angleMin -45.63
//45.5
//46.3
volatile bool lFlag = false;
volatile bool rFlag = false;
volatile signed long lCount = 0;
volatile signed long rCount = 0;

/*-----( PID variables )------*/
//Define Variables we'll be connecting to

double aInput, aOutput;
//double pSetpoint, pInput, pOutput;
extern float aHome;
double aSetpoint = aHome;


//Specify the links and initial tuning parameters
extern double aKp, aKi, aKd;
//PID PIDp(&pInput, &pOutput, &pSetpoint, pKp, pKi, pKd, DIRECT);
PID PIDa(&aInput, &aOutput, &aSetpoint, aKp, aKi, aKd, DIRECT);


L298N right_motor(rEN, rIN1, rIN2);
L298N left_motor(lEN, lIN1, lIN2);
/*-----( Declare objects )-----*/
//SoftwareSerial BTSerial(HC_05_TXD_ARDUINO_RXD, HC_05_RXD_ARDUINO_TXD); // RX | TX
/*-----( Declare Variables )-----*/
//char inData[80];
//char commandString[80];
//byte index = 0;

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
extern SoftwareSerial BTSerial;
void setup() {

  bluetoothSetup();


  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(115200);
  IMU_setup();
  while (!Serial); // wait for Leonardo enumeration, others continue immediately




  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

  PIDa.SetMode(AUTOMATIC);
  //PIDa.SetOutputLimits(-255, 255);
  PIDa.SetOutputLimits(-90, 90);
#ifdef properStart
  Serial.println("Waiting five seconds for IMU to stabalize...");
  for (int i = 0; i < 10; i++) {
    digitalWrite(LED_PIN, HIGH);
    if  ( (i % 2) == 0) {
      Serial.print(i);
      Serial.println("...");
      digitalWrite(LED_PIN, LOW);
    }
    delay(500);
  }

  for (int i = 0; i < 10; i++) {
    ((i % 2) == 0) ? digitalWrite(LED_PIN, LOW) : digitalWrite(LED_PIN, HIGH);
    delay(100);
  }
  digitalWrite(LED_PIN, LOW);
#endif
  Serial.println("Setup complete!");

  PIDa.SetSampleTime(10);
}
float angleOffset = 0;

extern bool stopFlag, printFlag, dmpReady;
bool resetAHomeFlag = false;
// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
// here to process incoming serial data after a terminator received
int SerialResetTimer = 0;
#define SerialResetTime 250
void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;


  if (BTSerial.available()) {
    angleOffset = (processBluetooth());
    Serial.print("Default aHome: ");
    Serial.print(aHome);
    Serial.print(" angle offset: ");
    Serial.print(angleOffset);
    Serial.print(" New aHome: ");
    aSetpoint = aHomeConst + angleOffset;//(float)((angleOffset-5000.0)/1000.0);
    Serial.println(aSetpoint);
    resetAHomeFlag = true;
    SerialResetTimer = millis();
  }

  if (resetAHomeFlag && ((SerialResetTimer + SerialResetTime) < millis())) {
    aSetpoint = aHomeConst;
    Serial.println("250ms has passed, resetting aHome... ");
    Serial.print("New aHome: ");
    Serial.println(aSetpoint);
    resetAHomeFlag = false;
  }


  while (Serial.available () > 0) {
    processIncomingByte (Serial.read (), PIDa, PIDa);
  }
  float tmpAngle = getAngle();
  if (tmpAngle != 1000) {
    aInput = tmpAngle;
    if (printFlag) {
      Serial.println(aInput);
    }
  }

  PIDa.Compute();
  if (stopFlag == true) {
    left_motor.stop();
    right_motor.stop();
  } else {
    //PIDa.Compute();
    float fSpeed = aOutput;

    if (fSpeed > 0.0) {
      left_motor.setSpeed(fSpeed);
      right_motor.setSpeed(fSpeed);
      left_motor.forward();
      right_motor.forward();
    } else if (fSpeed < 0.0) {
      fSpeed = fSpeed - ((2) * fSpeed);
      left_motor.setSpeed(fSpeed);
      right_motor.setSpeed(fSpeed);
      left_motor.backward();
      right_motor.backward();
    }

  }

}

