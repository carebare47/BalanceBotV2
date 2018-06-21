#include <PID_v1.h>
#include <L298N.h>
#include <SoftwareSerial.h>

//#define properStart
//////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////PIN MAPPINGS///////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
////Bluetooth pins
#define HC_05_TXD_ARDUINO_RXD 15
#define HC_05_RXD_ARDUINO_TXD 14
#define HC_05_SETUPKEY        17
//

//Left motor pins
#define lEN 5
#define lIN1 6
#define lIN2 7

//Right motor pins
#define rEN 3
#define rIN1 8
#define rIN2 10

//mpu
#define INTERRUPT_PIN 2

#define lIntPin 3  // the pin we are interested in
#define lDirPin 11
#define rIntPin 4
#define rDirPin 9

#define INPUT_SIZE 30

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
float aHome = -45.65;
//#define angleMin -45.63
//45.5
//46.3
volatile bool lFlag = false;
volatile bool rFlag = false;
volatile signed long lCount = 0;
volatile signed long rCount = 0;

bool rollRX = false;
bool pitchRX = false;
bool sensRX = false;
bool x_posRX = true;

float x_position = 0.0;
float roll = 0;
float pitch = 0;
int sensitivity = 0;

/*-----( PID variables )------*/
//Define Variables we'll be connecting to

double aInput, aOutput;
//double pSetpoint, pInput, pOutput;
double aSetpoint = aHome;


//Specify the links and initial tuning parameters
//double aKp = 30, aKi = 0.1, aKd = 1;
//double pKp = 0.5 , pKi = 0, pKd = 0;
double aKp = 7.0, aKi = 7.0, aKd = 0.40;
//double aKp = 70 , aKi = 140, aKd = 4.9;

//double aKp = 40 , aKi = 0.0, aKd = 0.0;
//PID PIDp(&pInput, &pOutput, &pSetpoint, pKp, pKi, pKd, DIRECT);
PID PIDa(&aInput, &aOutput, &aSetpoint, aKp, aKi, aKd, DIRECT);


L298N right_motor(rEN, rIN1, rIN2);
L298N left_motor(lEN, lIN1, lIN2);
/*-----( Declare objects )-----*/
SoftwareSerial BTSerial(HC_05_TXD_ARDUINO_RXD, HC_05_RXD_ARDUINO_TXD); // RX | TX
/*-----( Declare Variables )-----*/
char inData[80];
char commandString[80];
byte index = 0;
//////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////MPU6050 STUFF//////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL


#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

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



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////END OF MPU6050 STUFF///////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////






// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
  // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  //  while (!Serial.available());                 // wait for data
  //  while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  //My offsets:
  mpu.setXGyroOffset(70);
  mpu.setYGyroOffset(-26);
  mpu.setZGyroOffset(108);
  mpu.setZAccelOffset(3763);
  /*
     //New pcb orientation
     Sensor readings with offsets: -8  -7  16374 -1  0 1
    Your offsets: -729  -2947 3763  70  -26 108
    Data is printed as: acelX acelY acelZ giroX giroY giroZ


  */
  /*  // supply your own gyro offsets here, scaled for min sensitivity (came with code)
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
  */
  /*
    Sensor readings with offsets: 4 -1  16389 0 0 0
    Your offsets: -2422 -1275 4542  74  -21 94

    Data is printed as: acelX acelY acelZ giroX giroY giroZ
    Check that your sensor readings are close to 0 0 16384 0 0 0
  */

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

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

  /////////////////////////////
  ///////////Bluetooth////////
  /////////////////////////////
  pinMode(HC_05_SETUPKEY, OUTPUT);  // this pin will pull the HC-05 pin 34 (key pin) HIGH to switch module to AT mode
  digitalWrite(HC_05_SETUPKEY, HIGH);  // Set command mode when powering up
  BTSerial.begin(9600);  // HC-05 default speed in AT command mode


}
char incomingByte = 'x';
#define MAX_INPUT 10

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
// here to process incoming serial data after a terminator received
bool stopFlag = true;
bool pFlag = false, iFlag = false, dFlag = false;
int iTune = 99;
bool printFlag = false;
void process_data (const char * data)
{
  // for now just display it
  // (but you could compare it to some value, convert to an integer, etc.)
  Serial.println (data);
  //  if (*data != 'c') {
  //    stopFlag = false;
  //  }

  //  if (iTune != 99) {
  //    switch (iTune) {
  //      case 0:
  //        aKp = (float)*data;
  //        Serial.println (*data);
  //        iTune = 99;
  //        break;
  //      case 1:
  //        aKi = *data;
  //        iTune = 99;
  //        break;
  //      case 2:
  //        aKd = (float)*data;
  //        iTune = 99;
  //        break;
  //    }
  //    PIDa.SetTunings(aKp, aKi, aKd);
  //  }


  if (iTune != 99) {

    switch (iTune) {
      case 0:
        aKp = Serial.parseFloat();
        iTune = 99;
        break;
      case 1:
        aKi = Serial.parseFloat();
        iTune = 99;
        break;
      case 2:
        aKd = Serial.parseFloat();
        iTune = 99;
        break;
      case 5:
        aHome = Serial.parseFloat();
        iTune = 99;
        break;
    }
    PIDa.SetTunings(aKp, aKi, aKd);
    Serial.print("New PID values: Kp = ");
    Serial.print(aKp);
    Serial.print(" Ki = ");
    Serial.print(aKi);
    Serial.print(" Kd = ");
    Serial.println(aKd);
    Serial.print("Setpoint: ");
    Serial.println(aHome);

  }


  if (*data != 'q') {
    printFlag = false;
  }

  switch (*data) {
    case 'c':
      stopFlag = true;
      iTune = 99;
      break;
    case 'l':
      Serial.print("Current PID values: Kp = ");
      Serial.print(aKp);
      Serial.print(" Ki = ");
      Serial.print(aKi);
      Serial.print(" Kd = ");
      Serial.println(aKd);
      Serial.print("Setpoint: ");
      Serial.println(aHome);
      break;
    case 'q':
      printFlag = true;
      break;
    case 'p':
      pFlag = true;
      iTune = 0;
      //Serial.println (*data);
      Serial.print("Current PID values: Kp = ");
      Serial.print(aKp);
      Serial.print(" Ki = ");
      Serial.print(aKi);
      Serial.print(" Kd = ");
      Serial.println(aKd);

      Serial.println(" Enter float:");
      break;
    case 'i':
      iFlag = true;
      iTune = 1;

      Serial.print("Current PID values: Kp = ");
      Serial.print(aKp);
      Serial.print(" Ki = ");
      Serial.print(aKi);
      Serial.print(" Kd = ");
      Serial.println(aKd);

      Serial.println(" Enter float:");
      break;
    case 'd':
      dFlag = true;
      iTune = 2;
      Serial.print("Current PID values: Kp = ");
      Serial.print(aKp);
      Serial.print(" Ki = ");
      Serial.print(aKi);
      Serial.print(" Kd = ");
      Serial.println(aKd);

      Serial.println(" Enter float:");
      break;
    case 'g':
      stopFlag = false;
      iTune = 99;
      break;
    case 't':
      //stopFlag = false;
      iTune = 5;
      break;
    case 'h':
      aSetpoint += 0.1;
      break;
    case 'j':
      //stopFlag = false;
      aSetpoint = aHome;
      break;
    case 'k':
      aSetpoint -= 0.1;
      break;


  }






}  // end of process_data

void processIncomingByte (const byte inByte)
{
  static char input_line [MAX_INPUT];
  static unsigned int input_pos = 0;

  switch (inByte)
  {

    case '\n':   // end of text
      input_line [input_pos] = 0;  // terminating null byte

      // terminator reached! process input_line here ...
      process_data (input_line);

      // reset buffer for next time
      input_pos = 0;
      break;

    case '\r':   // discard carriage return
      break;

    default:
      // keep adding if not full ... allow for terminating null byte
      if (input_pos < (MAX_INPUT - 1))
        input_line [input_pos++] = inByte;
      break;

  }  // end of switch

} // end of processIncomingByte

void bluetoothSerialExecute(char ID, int iPosition, float fPosition) {
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
        aSetpoint = aHome + roll;
        Serial.print("offset");
        Serial.println(fPosition);
        Serial.print("setpoint");
        Serial.println(aSetpoint);
        //Serial.print(position); Serial.print(",");
        break;
      case 'p':
        pitch = fPosition;
        aSetpoint = aHome;
        //        pSetpoint = pitch;
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
    //
    //    if (pitchRX) {
    //      pitchRX2 = true;
    //    }
    //    if (rollRX) {
    //      rollRX2 = true;
    //    }
    //    rollRX = false;
    //    pitchRX = false;
    //char buf[INPUT_SIZE + 1];
    //sprintf(buf, "%s = %d", &command[0], position);
    //Serial.write(buf);
    // Do something with servoId and position
  }
}

void checkBluetoothSerial(void) {
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
        bluetoothSerialExecute(ID, iPosition, fPosition);
      }
      // Find the next command in input string
      command = strtok(0, "&");
    }

  }

}
void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    // other program behavior stuff here
    // .
    // .
    // .
    // if you are really paranoid you can frequently test in between other
    // stuff to see if mpuInterrupt is true, and if so, "break;" from the
    // while() loop to immediately process the MPU data
    // .
    // .
    // .


    while (Serial.available () > 0) {
      processIncomingByte (Serial.read ());
    }
    //
    //
    //    if (Serial.available() > 0) {
    //      // read the incoming byte:
    //      incomingByte = Serial.read();
    //    }
    //
    //    switch (incomingByte) {
    //      case 'c':
    //        left_motor.stop();
    //        right_motor.stop();
    //        break;
    //      case 'p':
    //         pFlag = true;
    //         break;
    //      case 'i':
    //         iFlag = true;
    //         break;
    //      case 'd':
    //         dFlag = true;
    //         break;
    //      case '0' ... '9':
    //
    //
    //if (incomingByte == 'c') {




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

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    // Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    //#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    aInput = ypr[1] * 180 / M_PI;
    //if (45.3 < aInput < 46.3){
    //    if (angleMin < aInput < angleMax){
    //      aInput = aHome;
    //    }

    if (printFlag) {
      Serial.println(aInput);
    }
    //#ifdef PRINT_ANGLES
    //        Serial.print("ypr\t");
    //        Serial.print(ypr[0] * 180 / M_PI);
    //Serial.print("\t");
    //Serial.println(ypr[1] * 180 / M_PI);
    //Serial.print("\t");
    //Serial.println(ypr[2] * 180 / M_PI);
    //#endif
    //#endif
    //  Serial.println(ypr[2] * 180 / M_PI);

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
}

