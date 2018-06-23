//serialProcessing.cppx

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <PID_v1.h>

//extern signed long lCount, rCount;

double lKp = 7.0, lKi = 8.0, lKd = 0.66;
double lSetpoint = 0;
//extern PID left_PID;



char incomingByte = 'x';
#define MAX_INPUT 10

bool stopFlag = false;
bool pFlag = false, iFlag = false, dFlag = false;
int iTune = 99;
bool printFlag = false;
void process_data (const char * data, PID left_PID, PID right_PID)
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
        lKp = Serial.parseFloat();
        iTune = 99;
        break;
      case 1:
        lKi = Serial.parseFloat();
        iTune = 99;
        break;
      case 2:
        lKd = Serial.parseFloat();
        iTune = 99;
        break;
      case 5:
        lSetpoint = Serial.parseFloat();
        iTune = 99;
        break;
    }
    left_PID.SetTunings(lKp, lKi, lKd);
    Serial.print("New PID values: Kp = ");
    Serial.print(lKp);
    Serial.print(" Ki = ");
    Serial.print(lKi);
    Serial.print(" Kd = ");
    Serial.println(lKd);
    Serial.print("Setpoint: ");
    Serial.println(lSetpoint);

  }


  //  if (*data != 'q') {
  //    printFlag = false;
  //  }

  switch (*data) {
    case 'c':
      stopFlag = true;
      iTune = 99;
      break;
    case 'l':
      left_PID.SetTunings(lKp, lKi, lKd);
      Serial.print("New PID values: Kp = ");
      Serial.print(lKp);
      Serial.print(" Ki = ");
      Serial.print(lKi);
      Serial.print(" Kd = ");
      Serial.println(lKd);
      Serial.print("Setpoint: ");
      Serial.println(lSetpoint);
      break;
    case 'a':
      printFlag = false;
      break;
    case 'q':
      printFlag = true;
      break;
    case 'p':
      pFlag = true;
      iTune = 0;
      //Serial.println (*data);
      left_PID.SetTunings(lKp, lKi, lKd);
      Serial.print("New PID values: Kp = ");
      Serial.print(lKp);
      Serial.print(" Ki = ");
      Serial.print(lKi);
      Serial.print(" Kd = ");
      Serial.println(lKd);
      Serial.print("Setpoint: ");
      Serial.println(lSetpoint);

      Serial.println(" Enter float:");
      break;
    case 'i':
      iFlag = true;
      iTune = 1;

      left_PID.SetTunings(lKp, lKi, lKd);
      Serial.print("New PID values: Kp = ");
      Serial.print(lKp);
      Serial.print(" Ki = ");
      Serial.print(lKi);
      Serial.print(" Kd = ");
      Serial.println(lKd);
      Serial.print("Setpoint: ");
      Serial.println(lSetpoint);

      Serial.println(" Enter float:");
      break;
    case 'd':
      dFlag = true;
      iTune = 2;
      left_PID.SetTunings(lKp, lKi, lKd);
      Serial.print("New PID values: Kp = ");
      Serial.print(lKp);
      Serial.print(" Ki = ");
      Serial.print(lKi);
      Serial.print(" Kd = ");
      Serial.println(lKd);
      Serial.print("Setpoint: ");
      Serial.println(lSetpoint);

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

  }






}  // end of process_data

void processIncomingByte (const byte inByte, PID left_PID, PID right_PID)
{
  static char input_line [MAX_INPUT];
  static unsigned int input_pos = 0;

  switch (inByte)
  {

    case '\n':   // end of text
      input_line [input_pos] = 0;  // terminating null byte

      // terminator reached! process input_line here ...
      process_data (input_line, left_PID, right_PID);

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
