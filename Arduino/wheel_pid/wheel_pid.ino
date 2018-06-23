#include <PID_v1.h>
#include <L298N.h>
#include "serialProcessing.h"

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

double lInput, lOutput;
extern double lSetpoint;

//Specify the links and initial tuning parameters
//double lKp = 0, lKi = 0, lKd = 0;
extern double lKp, lKi, lKd;
//PID PIDp(&pInput, &pOutput, &pSetpoint, pKp, pKi, pKd, DIRECT);
PID left_PID(&lInput, &lOutput, &lSetpoint, lKp, lKi, lKd, DIRECT);
//PID right_PID(&aInput, &aOutput, &aSetpoint, aKp, aKi, aKd, DIRECT);


L298N right_motor(rEN, rIN1, rIN2);
L298N left_motor(lEN, lIN1, lIN2);


#define SAMPLE_TIME 10

//Ulong sample_timer = 0;
long old_sample_time = 0;

void setup() {
  left_PID.SetMode(AUTOMATIC);
  left_PID.SetOutputLimits(-100, 100);
  left_PID.SetSampleTime(SAMPLE_TIME);
  // pin change interrupt (D12)
  PCMSK0 |= bit (PCINT4);  // want pin 12
  PCIFR  |= bit (PCIF0);    // clear any outstanding interrupts
  PCICR  |= bit (PCIE0);    // enable pin change interrupts for D8 to D12

  // pin change interrupt (D4)
  PCMSK2 |= bit (PCINT20);  // want pin 4
  PCIFR  |= bit (PCIF2);    // clear any outstanding interrupts
  PCICR  |= bit (PCIE2);    // enable pin change interrupts for D0 to D7

  Serial.begin(115200);


}
uint32_t ts1 = 0, ts2 = 0;
uint32_t calculate_dt(void) {
  // uint32_t ts1 = micros();
  // ...TASK TO BE MEASURED GOES HERE
  uint32_t ts2 = micros();

  // print the time interval in microseconds
  // Serial.println(ts2 - ts1);

  float dt = (ts2 - ts1);
  uint32_t ts1 = micros();
  return dt;


}
#define ppr 2752 //pulses per revolution
#define wheel_dia 0.310
double v_left = 0;
double old_distance = 0;
double distance = 0;
float global_dt = 0;
long newTime = 0, oldTime = 0;
long local_dt = 0;
extern bool printFlag, stopFlag;


double wheel_distance_scale = wheel_dia / ppr;
void loop() {
  // put your main code here, to run repeatedly:

  while (Serial.available () > 0) {
    processIncomingByte (Serial.read (), left_PID, left_PID);
  }

  //if (lFlag) {

  if ((old_sample_time + SAMPLE_TIME) < millis()) {


    newTime = micros();
    distance = (lCount * wheel_distance_scale); // / ppr) * wheel_dia;
    local_dt = (newTime - oldTime);
    global_dt = local_dt / 100; //calculate_dt();
    if (distance != old_distance) {
      v_left = ((distance - old_distance) / global_dt ) * 10000;
    } else {
      v_left = 0;
    }
    old_distance = distance;
    oldTime = newTime;
    old_sample_time = millis();
  }
  //lFlag = false;
  //}

  lInput = v_left;
  left_PID.Compute();

  if (printFlag) {

    Serial.print("distance: ");
    Serial.print(distance);

    Serial.print("old distance: ");
    Serial.print(old_distance);

    Serial.print("v_left: ");
    Serial.print(v_left);

    Serial.print(" dt: ");
    Serial.print(global_d t);
    Serial.print(" lCount: ");
    Serial.print(lCount);

    Serial.print(" ip ");
    Serial.print(lInput);

    Serial.print(" sp ");
    Serial.print(lSetpoint);

    Serial.print(" op ");
    Serial.println(lOutput);
  }


  if (!stopFlag) {
    //  left_motor.setSpeed(lOutput);
    //  left_motor.forward();
    float fSpeed = lOutput;
    if (lSetpoint >= 0.0) {
      left_motor.setSpeed(5*fSpeed);
      //right_motor.setSpeed(fSpeed);
      left_motor.backward();
      // right_motor.backward();
    } else if (lSetpoint   < 0.0) {
      fSpeed = fSpeed - ((2) * fSpeed);
      fSpeed = fSpeed * 5;
      left_motor.setSpeed(fSpeed);
      //right_motor.setSpeed(fSpeed);
      left_motor.forward();
      //right_motor.forward();
    }
  } else {
    left_motor.stop();
    right_motor.stop();
  }


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
