//#define aHomeConst -45.6
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

#define ppr 2752

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

double aInput;
double aOutput;
double pSetpoint = 0.0, pInput, pOutput;
extern float aHome;
double aSetpoint = aHome;


//Specify the links and initial tuning parameters
extern double aKp, aKi, aKd;
double pKp = 1, pKi = 0, pKd = 0;
PID PIDp(&pInput, &pOutput, &pSetpoint, pKp, pKi, pKd, DIRECT);
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

  // pin change interrupt (D12)
  PCMSK0 |= bit (PCINT4);  // want pin 12
  PCIFR  |= bit (PCIF0);    // clear any outstanding interrupts
  PCICR  |= bit (PCIE0);    // enable pin change interrupts for D8 to D12

  // pin change interrupt (D4)
  PCMSK2 |= bit (PCINT20);  // want pin 4
  PCIFR  |= bit (PCIF2);    // clear any outstanding interrupts
  PCICR  |= bit (PCIE2);    // enable pin change interrupts for D0 to D7

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(115200);
  IMU_setup();
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);


  PIDp.SetOutputLimits(-1, 1);
  PIDp.SetSampleTime(10);
  PIDp.SetMode(AUTOMATIC);

  PIDa.SetOutputLimits(-90, 90);
  PIDa.SetSampleTime(10);
  PIDa.SetMode(AUTOMATIC);
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


}
float angleOffset = 0;

extern bool stopFlag, printFlag, dmpReady, printFlag2, printFlag3, printFlag4, printFlagAll, printFlag5;
bool resetAHomeFlag = false;
// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
// here to process incoming serial data after a terminator received
long SerialResetTimer = 0;
#define SerialResetTime 1000
int debugTimer = 0;
int printDebugTimer(String str) {
  //void printDebugTimer(char str[]) {
  int timerM = millis();
  //  Serial.print(str);
  //  Serial.print(": ");
  //  //int timerM = millis();
  //  Serial.println(timerM);
  return timerM;

}

long gOldTime = 0;
void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  if (printFlag3) {
    Serial.print("Loop time: ");
    Serial.println((micros() - gOldTime));
    gOldTime = micros();
  }
  /*
    //Angle control from bt
    if (BTSerial.available()) {
      int avail = printDebugTimer("bt is available");
      angleOffset = (processBluetooth());
      int comp = printDebugTimer("bt has been processed");
      Serial.print(" angle offset: ");
      Serial.print(angleOffset);
      Serial.print(" New aHome: ");
      //aSetpoint = aHomeConst + angleOffset;//(float)((angleOffset-5000.0)/1000.0);
      aSetpoint = aHome + angleOffset;//(float)((angleOffset-5000.0)/1000.0);
      Serial.println(aSetpoint);
      resetAHomeFlag = true;
      SerialResetTimer = millis();
    }
  */



  //velocity control from bt
  if (BTSerial.available()) {
    //int avail = printDebugTimer("bt is available");
    angleOffset = (processBluetooth());
    //int comp = printDebugTimer("bt has been processed");
    Serial.print(" angle offset: ");
    Serial.print(angleOffset);
    Serial.print(" New aHome: ");
    //aSetpoint = aHomeConst + angleOffset;//(float)((angleOffset-5000.0)/1000.0);
    pSetpoint = angleOffset;//(float)((angleOffset-5000.0)/1000.0);
    Serial.println(pSetpoint);
    resetAHomeFlag = true;
    SerialResetTimer = millis();
  }



//  //If no commands recieved for "SerialResetTimer ms", set setpoints to aHome or 0
//  if (((SerialResetTimer + SerialResetTime) < millis())) {
//    pSetpoint = 0.0;
//    Serial.println("250ms has passed, resetting aHome... ");
//    Serial.print("New aHome: ");
//    Serial.println(pSetpoint);
//    //resetAHomeFlag = false;
//    SerialResetTimer = millis();
//  }



  //Check for hardware serial commands and act upon them
  while (Serial.available () > 0) {
    processIncomingByte (Serial.read (), PIDa, PIDa, false);
  }



  //getAngle() returns '1000' when the angle hasn't been updated.
  float tmpAngle = getAngle();
  if (tmpAngle != 1000) {
    aInput = tmpAngle;
    //aInput = aHome;
  }

  if (printFlag) {
    Serial.print("aInput: ");
    Serial.print(aInput);
  }




  //pInput = (getSpeedLeft() + getSpeedRight()) / 2;

  //if (!isnan(pInput)) {

  pInput = getSpeedLeft();
  //if (pSetpoint == (double)0.0) {
    if (printFlag5) {
      Serial.println("Not tracking pPID, resetting integral...");
    }
    PIDp.Compute(false);
    //pOutput = 0;
    //processIncomingByte (7 , PIDa, PIDa, true);
//  } else {
//    PIDp.Compute(false);
//    if (printFlag5) {
//      Serial.println("Tracking pPID");
//    }
//  }
  //    pOutput = 0.0;

  //  } else {
  //    PIDp.Compute();
  //  }
  //pOutput = 0;
  //PIDp.Compute(false);
  aSetpoint = pOutput + aHome;
  PIDa.Compute(false);

  if (printFlag2) {
    Serial.print("pInput: ");
    Serial.print(pInput);
    Serial.print(" pSetpoint: ");
    Serial.print(pSetpoint);
    Serial.print(" pOutput: ");
    Serial.println(pOutput);
  }


  if (printFlag4) {
    Serial.print("All the homes. aSetpoint = pOutput + aHome. ");
    Serial.print("aHome: ");
    Serial.print(aHome);
    Serial.print(" aSetpoint: ");
    Serial.print(aSetpoint);
    Serial.print(" pOutput: ");
    Serial.println(pOutput);
  }



  if (printFlag) {
    Serial.print(" aSetpoint: ");
    Serial.print(aSetpoint);
    //Serial.print("xx");
    Serial.print(" aOutput: ");
    Serial.print(aOutput);
  }
  if (stopFlag == true) {
    left_motor.stop();
    right_motor.stop();
    //Serial.println("");
  } else {
    //PIDa.Compute();
    float fSpeed = aOutput;
    if (printFlag) {
      Serial.print(" fSpeed: ");
      Serial.println(fSpeed);
    }
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

    if (printFlagAll) {
      Serial.print("aHome: ");
      Serial.print(aHome);
      Serial.print(" aInput: ");
      Serial.print(aInput);
      Serial.print(" aSetpoint: ");
      Serial.print(aSetpoint);
      Serial.print(" aOutput: ");
      Serial.print(aOutput);
      Serial.print(" fSpeed: ");
      Serial.print(fSpeed);

      Serial.print(" pInput: ");
      Serial.print(pInput);
      Serial.print(" pSetpoint: ");
      Serial.print(pSetpoint);
      Serial.print(" pOutput: ");
      Serial.print(pOutput);

      Serial.print(" aKp: ");
      Serial.print(aKp);
      Serial.print(" aKi: ");
      Serial.print(aKi);
      Serial.print(" aKd: ");
      Serial.println(aKd);


    }





  }

}

float countToDistanceM = (PI * 0.1) / ppr;


long old_lCount = 0, old_lTime = 0;



float getSpeedLeft(void) {
  //Serial.print("Enter GetSpeedLeft. dt: ");
  long dt = micros() - old_lTime;
  //  Serial.print(dt);
  //  Serial.print(" dt_sec: ");
  double dt_sec = ((double)dt / 1000000.0);
  //  Serial.print( dt_sec);
  //PI*0.1 (dia in meters)
  //(ppr*PI*0.1) / lCount = distance traveled in meters

  float vL;
  //float newPos = (countToDistanceM/(float)lCount);
  if (lCount != old_lCount) {
    float delta_pos = (countToDistanceM * (lCount - old_lCount));
    //    Serial.print(" delta_pos: ");
    //    Serial.print(delta_pos);
    vL = delta_pos / dt_sec;
  } else {
    vL = 0.0;
  }
  old_lCount = lCount;
  //  Serial.print(" old_lCount: ");
  //  Serial.print(old_lCount);
  old_lTime = micros();
  //  Serial.print(" old_lTime: ");
  //  Serial.println(old_lTime);
  //  Serial.print(" VL: ");
  //  Serial.print(vL);
  return (vL);
}

int old_rCount, old_time_right;
float getSpeedRight(void) {
  //int dt = millis() - old_time_right;
  int dt = micros() - old_time_right;
  float vR = rCount - old_rCount;
  vR = vR / dt;
  old_rCount = rCount;
  old_time_right = micros();
  return (vR * 1000);
}

// Left encoder
ISR (PCINT0_vect)
{
  lFlag = true;
  if (PINB & bit (4)) { // if pin D12 was high
    if (digitalRead(lDirPin) == HIGH) {
      lCount++;
    } else {
      lCount--;
    }  // end of PCINT2_vect
  }
}

//Right encoder
ISR (PCINT2_vect)
{
  rFlag = true;
  if (PIND & bit (4)) { // if pin D4 was high
    if (digitalRead(rDirPin) == HIGH) {
      rCount++;
    } else {
      rCount--;
    }  // end of PCINT2_vect
  }
}

