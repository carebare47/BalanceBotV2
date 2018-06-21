#include <Plotter.h>


/*-----( Import needed libraries )-----*/
#include <SoftwareSerial.h>
#include <string.h>
#include <I2Cdev.h>

Plotter plot;


#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif


#include <L298N.h>
#include <PID_v1.h>


//Bluetooth pins
#define HC_05_TXD_ARDUINO_RXD 15
#define HC_05_RXD_ARDUINO_TXD 14
#define HC_05_SETUPKEY        17


// Calculate based on max input size expected for one command
#define INPUT_SIZE 30

#define ppr 2752 //pulses per revolution

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

//#define aHome 186
//#define aHome 195


#define aHome -62

#define INTERRUPT_PIN 2

#define lIntPin 3  // the pin we are interested in
#define lDirPin 11
#define rIntPin 4
#define rDirPin 9


// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high


void leftCB(void);
void stop_robot(void);

volatile bool lFlag = false;
volatile bool rFlag = false;
volatile signed long lCount = 0;
volatile signed long rCount = 0;




/*-----( PID variables )------*/
//Define Variables we'll be connecting to

double aInput, aOutput;
double pSetpoint, pInput, pOutput;
double aSetpoint = aHome;


//Specify the links and initial tuning parameters
//double aKp = 30, aKi = 0.1, aKd = 1;
double pKp = 0.5 , pKi = 0, pKd = 0;
double aKp = 70 , aKi = 240, aKd = 1.9;
PID PIDp(&pInput, &pOutput, &pSetpoint, pKp, pKi, pKd, DIRECT);
PID PIDa(&aInput, &aOutput, &aSetpoint, aKp, aKi, aKd, DIRECT);


/*-----( Declare objects )-----*/
SoftwareSerial BTSerial(HC_05_TXD_ARDUINO_RXD, HC_05_RXD_ARDUINO_TXD); // RX | TX
/*-----( Declare Variables )-----*/
char inData[80];
char commandString[80];
byte index = 0;



unsigned long time;
unsigned long timer_x = 0;



// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

void setup()   /****** SETUP: RUNS ONCE ******/
{

  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip


  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    // Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    //Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    //Serial.print(F("DMP Initialization failed (code "));
    //Serial.print(devStatus);
    //Serial.println(F(")"));
  }


  pinMode(HC_05_SETUPKEY, OUTPUT);  // this pin will pull the HC-05 pin 34 (key pin) HIGH to switch module to AT mode

  digitalWrite(HC_05_SETUPKEY, HIGH);  // Set command mode when powering up


  pinMode(13, OUTPUT);



  //  Serial.begin(9600);   // For the Arduino IDE Serial Monitor
  Serial.println("YourDuino.com HC-05 Bluetooth Module AT Command Utility V1.02");
  Serial.println("Set Serial Monitor to 'Both NL & CR' and '9600 Baud' at bottom right");

  //Wire.begin();
  //mpu6050.begin();
  //mpu6050.calcGyroOffsets(true);
  BTSerial.begin(9600);  // HC-05 default speed in AT command mode

  PIDa.SetMode(AUTOMATIC);
  PIDp.SetMode(AUTOMATIC);
  PIDp.SetOutputLimits(-255, 255);
  PIDa.SetOutputLimits(-255, 255);

  pinMode(lIntPin, INPUT_PULLUP);
  //pinMode(11, INPUT_PULLUP);
  //pinMode(rightIntPin, INPUT_PULLUP);
  //  attachInterrupt(digitalPinToInterrupt(lIntPin), leftCB, RISING);

  // pin change interrupt (D12)
  PCMSK0 |= bit (PCINT4);  // want pin 12
  PCIFR  |= bit (PCIF0);    // clear any outstanding interrupts
  PCICR  |= bit (PCIE0);    // enable pin change interrupts for D8 to D12

  // pin change interrupt (D4)
  PCMSK2 |= bit (PCINT20);  // want pin 4
  PCIFR  |= bit (PCIF2);    // clear any outstanding interrupts
  PCICR  |= bit (PCIE2);    // enable pin change interrupts for D0 to D7

  timer_x = millis();

  Serial.println("Setup finished");
  plot.Begin();
  //plot.AddTimeGraph("left wheel PID", 500, "Input", pInput, "Setpoint", pSetpoint, "Output", pOutput);
  plot.AddTimeGraph("Angle PID", 500, "Input", aInput, "Setpoint", aSetpoint, "Output", aOutput);
  //void AddTimeGraph( String title, int pointsDisplayed, String label1, Variable1Type variable1, String label2, Variable2Type variable2, ... )
  //p.AddTimeGraph( "Some title of a graph", 500, "label for x", x );



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

void loop()   /****** LOOP: RUNS CONSTANTLY ******/
{
  // if programming failed, don't try to do anything
  if (!dmpReady) return;


  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {

    PIDa.Compute();
    float fSpeed = aOutput;

    if (fSpeed > 0.0) {
      left_motor.backward();
    } else if (fSpeed < 0.0) {
      left_motor.forward();
      fSpeed = fSpeed - ((2) * fSpeed);
    }
    left_motor.setSpeed(fSpeed);
    plot.Plot();


  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    //  Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    aInput = ypr[2] * 180 / M_PI;
  }

}



//--(end main loop )---

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
//void leftCB(void) {
//  lFlag = true;
//  bool direc = (digitalRead(lDirPin) == HIGH) ? true : false;
//  if (digitalRead(lIntPin) == HIGH) {
//    if (direc) {
//      lCount--;
//    } else {
//      lCount++;
//    }
//  }
//  //(digitalRead(lDirPin)) ? lCount-- : lCount++;
//}
//void rightCB(void) {
//  rFlag = true;
//  rDir = digitalRead(5);
//  (rDir) ? rCount-- : rCount++;
//}



