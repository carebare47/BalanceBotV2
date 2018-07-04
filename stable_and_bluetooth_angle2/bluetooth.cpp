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
#define INPUT_SIZE 10


/*-----( Declare objects )-----*/
SoftwareSerial BTSerial(HC_05_TXD_ARDUINO_RXD, HC_05_RXD_ARDUINO_TXD); // RX | TX
/*-----( Declare Variables )-----*/
char inData[80];
char commandString[80];
byte index = 0;

float sensitivity = 0;
float roll = 0;

bool rollRX = false;

void printDebugTimerB(String str) {
  //void printDebugTimer(char str[]) {
  int timerM = millis();
  Serial.print(str);
  Serial.print(": ");
  //int timerM = millis();
  Serial.println(timerM);
  //return timerM;
}

void bluetoothSetup(void) {
  BTSerial.begin(9600);  // HC-05 default speed in AT command mode

}
bool hackFlag = false;
bool polarityHack;

bool minusSign = false;
double processBluetooth(void) {
  // Get next command from Serial (add 1 for final 0)
  char input[INPUT_SIZE + 1];
  printDebugTimerB("pre_readBytes");
  byte size = BTSerial.readBytesUntil('e', input, INPUT_SIZE);
  // Add the final 0 to end the C string
  input[size] = 0;
  if ((size == 0) || (size == 1)) {
    roll = 0.0;
  } else {

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
        iPosition -= 500;
        float fPosition = (float)iPosition / 5000;
        if (iPosition != 999) {
          switch (ID) {
            case 's':
              sensitivity = iPosition;
              //sensRX = true;
              // Serial.print(position); Serial.print(",");
              break;
            case 'r':
              roll = (fPosition*3);
              break;
          }
          if (ID != 'r') {
            roll = 0;
          }
        }
      }
      // Find the next command in input string
      //command = strtok(0, "&");
      command = 0;
    }
  }

  return roll;



}


