#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <SoftwareSerial.h>
#include <string.h>

//Bluetooth pins
#define HC_05_TXD_ARDUINO_RXD 16
#define HC_05_RXD_ARDUINO_TXD 15
#define HC_05_SETUPKEY        14


// Calculate based on max input size expected for one command
#define INPUT_SIZE 30

/*-----( Declare objects )-----*/
SoftwareSerial BTSerial(HC_05_TXD_ARDUINO_RXD, HC_05_RXD_ARDUINO_TXD); // RX | TX
/*-----( Declare Variables )-----*/
char inData[80];
char commandString[80];
byte index = 0;

float sensitivity = 0;
float roll = 0;

bool rollRX = false;



void bluetoothSetup(void) {
  BTSerial.begin(9600);  // HC-05 default speed in AT command mode

}
bool hackFlag = false;
bool polarityHack;

float processBluetooth(void) {
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
        if (ID != 'r') {
          roll = 0;
        }
        switch (ID) {
          case 's':
            sensitivity = iPosition;
            //sensRX = true;
            // Serial.print(position); Serial.print(",");
            break;
          case 'r':
            roll = (fPosition - 5000) / 5000.0;
            rollRX = true;
            //            aSetpoint = roll;
            Serial.print("roll_pre");
            Serial.println(roll);
            if (!hackFlag) { //hackFlag = true;
              if (roll > 0.0) {
                polarityHack = true;
              } else {
                polarityHack = false;
              }
              hackFlag = true;
            }
            //Serial.print(position); Serial.print(",");
            break;
            //          case 'p':
            //            pitch = fPosition;
            //            pSetpoint = pitch;
            //            pitchRX = true;
            //            Serial.print("pitch");
            //            Serial.println(fPosition);
            //            //Serial.print(position); Serial.print(",");
            //            break;
            //          case 'x':
            //            Serial.print("x");
            //            Serial.println(fPosition);
            //            x_position = fPosition;
            //            x_posRX = true;
            //            break;
        }


        //char buf[INPUT_SIZE + 1];
        //sprintf(buf, "%s = %d", &command[0], position);
        //Serial.write(buf);
        // Do something with servoId and position
      }
    }
    // Find the next command in input string
    command = strtok(0, "&");
  }

  //  if (-1.0 < roll < 1.0) {
  //    roll = 0.0;
  //  }

  //

  if (hackFlag) {
    if (polarityHack) {
      if (roll < 0.0) {
        roll = roll * (-1);
      }
    }
    hackFlag = false;
  }
  Serial.print("roll_post");
  Serial.println(roll);
  polarityHack = false;
  return roll;



}


