//#define aHomeConst
//serialProcessing.cppx

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <PID_v1.h>

double aKp = 7.0, aKi = 8.0, aKd = 0.66;


float aHome = -46.6;
//float aInput = aHome;

char incomingByte = 'x';
#define MAX_INPUT 10

bool stopFlag = false;
bool pFlag = false, iFlag = false, dFlag = false;
int iTune = 99;
bool printFlag = false, printFlag2 = false, printFlag3 = false, printFlag4 = false, printFlagAll = false, printFlag5 = false;


void process_data (const char * data, PID PIDa, PID PIDp)
{
  // for now just display it
  // (but you could compare it to some value, convert to an integer, etc.)
  //Serial.println (data);
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
      case 6:
        float aInput = Serial.parseFloat();
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
    case 'z':
      //stopFlag = false;
      iTune = 6;
      break;
    case 'n':
      printFlag2 = !printFlag2;
      break;
    case 'm':
      printFlag3 = !printFlag3;
      break;
    case 'o':
      printFlag4 = !printFlag4;
      break;
    case 't':
      //stopFlag = false;
      iTune = 5;
      break;
    case 'v':
      //stopFlag = false;
      printFlagAll = !printFlagAll;
      break;
    case 'u':
      //stopFlag = false;
      printFlag5 = !printFlag5;
      break;

  }






}  // end of process_data

void processIncomingByte (const byte inByte, PID PIDa, PID PIDp, bool process_or_reset)
{
  if (process_or_reset) {
    PIDa.SetTunings(aKp, aKi, aKd);
  } else {
    static char input_line [MAX_INPUT];
    static unsigned int input_pos = 0;

    switch (inByte)
    {

      case '\n':   // end of text
        input_line [input_pos] = 0;  // terminating null byte

        // terminator reached! process input_line here ...
        process_data (input_line, PIDa, PIDp);

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
}
