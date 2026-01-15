//VERSION 3.0

/*  AR4 - Stepper motor robot control software
    Copyright (c) 2023, Chris Annin
    All rights reserved.

    You are free to share, copy and redistribute in any medium
    or format.  You are free to remix, transform and build upon
    this material.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

          Redistributions of source code must retain the above copyright
          notice, this list of conditions and the following disclaimer.
          Redistribution of this software in source or binary forms shall be free
          of all charges or fees to the recipient of this software.
          Redistributions in binary form must reproduce the above copyright
          notice, this list of conditions and the following disclaimer in the
          documentation and/or other materials provided with the distribution.
          you must give appropriate credit and indicate if changes were made. You may do
          so in any reasonable manner, but not in any way that suggests the
          licensor endorses you or your use.
          Selling Annin Robotics software, robots, robot parts, or any versions of robots or software based on this
          work is strictly prohibited.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL CHRIS ANNIN BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

    chris.annin@gmail.com

*/

// use this for the old AR3 encoders
// VERSION LOG
// 1.0 - 2/6/21 - initial release
// 1.1 - 2/20/21 - bug fix, calibration offset on negative axis calibration direction axis 2,4,5
// 2.0 - 10/1/22 - added lookahead and spline functionality
// 2.2 - 11/6/22 - added Move V for open cv integrated vision
// 3.0 - 2/3/23 - open loop bypass moved to teensy board / add external axis 8 & 9 / bug fix live jog drift
//       12/31/25 - remove FK/IK, use ikpy on host computer; flip J1 axis direction; soft e-stop added
//                  remove functions related to FK/IK

#include <math.h>
#include <avr/pgmspace.h>
#include <Encoder.h>

String cmdBuffer1;
String cmdBuffer2;
String cmdBuffer3;
String inData;
String recData;
String checkData;
String function;
volatile byte state = LOW;
volatile bool robotBusy = false;

const int debugg = 0;

// --- Soft E-Stop (firmware hold-stop) ------------------------------------
// NOTE: This is NOT a replacement for the existing hardware E-stop that cuts power.
// This soft E-stop is intended to stop motion immediately while keeping the drivers enabled
// so the arm holds position (no free-fall), as long as the MCU/firmware is running.
//
// Wire an NC E-stop contact to ESTOP_PIN -> GND. (INPUT_PULLUP + NC-to-GND means normal=LOW, pressed/open=HIGH)
#define ESTOP_PIN 32

volatile bool estop_latched = false;   // blocks motion until cleared by ER command
bool estop_handled = false;            // one-shot: clear buffers/flags when E-stop first latches

void estopISR();
// --------------------------------------------------------------------------

const int J1stepPin = 0;
const int J1dirPin = 1;
const int J2stepPin = 2;
const int J2dirPin = 3;
const int J3stepPin = 4;
const int J3dirPin = 5;
const int J4stepPin = 6;
const int J4dirPin = 7;
const int J5stepPin = 8;
const int J5dirPin = 9;
const int J6stepPin = 10;
const int J6dirPin = 11;
const int J7stepPin = 12;
const int J7dirPin = 13;
const int J8stepPin = 39;
const int J8dirPin = 33;
const int J9stepPin = 34;
const int J9dirPin = 35;

const int J1calPin = 26;
const int J2calPin = 27;
const int J3calPin = 28;
const int J4calPin = 29;
const int J5calPin = 30;
const int J6calPin = 31;
const int J7calPin = 36;
const int J8calPin = 37;
const int J9calPin = 38;


//const int Input39 = 39;

const int Output40 = 40;
const int Output41 = 41;


//set encoder multiplier
const float J1encMult = 5.12;
const float J2encMult = 5.12;
const float J3encMult = 5.12;
const float J4encMult = 5.12;
const float J5encMult = 2.56;
const float J6encMult = 5.12;


//set encoder pins
Encoder J1encPos(14, 15);
Encoder J2encPos(16, 17);
Encoder J3encPos(18, 19);
Encoder J4encPos(20, 21);
Encoder J5encPos(22, 23);
Encoder J6encPos(24, 25);


// GLOBAL VARS //

//define axis limits in degrees
float J1axisLimPos = 170;
float J1axisLimNeg = 170;
float J2axisLimPos = 90;
float J2axisLimNeg = 42;
float J3axisLimPos = 52;
float J3axisLimNeg = 89;
float J4axisLimPos = 165;
float J4axisLimNeg = 165;
float J5axisLimPos = 105;
float J5axisLimNeg = 105;
float J6axisLimPos = 155;
float J6axisLimNeg = 155;
float J7axisLimPos = 3450;
float J7axisLimNeg = 0;
float J8axisLimPos = 3450;
float J8axisLimNeg = 0;
float J9axisLimPos = 3450;
float J9axisLimNeg = 0;

//define total axis travel
float J1axisLim = J1axisLimPos + J1axisLimNeg;
float J2axisLim = J2axisLimPos + J2axisLimNeg;
float J3axisLim = J3axisLimPos + J3axisLimNeg;
float J4axisLim = J4axisLimPos + J4axisLimNeg;
float J5axisLim = J5axisLimPos + J5axisLimNeg;
float J6axisLim = J6axisLimPos + J6axisLimNeg;
float J7axisLim = J7axisLimPos + J7axisLimNeg;
float J8axisLim = J8axisLimPos + J8axisLimNeg;
float J9axisLim = J9axisLimPos + J9axisLimNeg;

//motor steps per degree
float J1StepDeg = 44.44444444;
float J2StepDeg = 55.55555556;
float J3StepDeg = 55.55555556;
float J4StepDeg = 42.72664356;
float J5StepDeg = 21.88888889;
float J6StepDeg = 21.3368984;
float J7StepDeg = 14.28571429;
float J8StepDeg = 14.28571429;
float J9StepDeg = 14.28571429;

//steps full movement of each axis
int J1StepLim = J1axisLim * J1StepDeg;
int J2StepLim = J2axisLim * J2StepDeg;
int J3StepLim = J3axisLim * J3StepDeg;
int J4StepLim = J4axisLim * J4StepDeg;
int J5StepLim = J5axisLim * J5StepDeg;
int J6StepLim = J6axisLim * J6StepDeg;
int J7StepLim = J7axisLim * J7StepDeg;
int J8StepLim = J8axisLim * J8StepDeg;
int J9StepLim = J9axisLim * J9StepDeg;

//step and axis zero
int J1zeroStep = J1axisLimNeg * J1StepDeg;
int J2zeroStep = J2axisLimNeg * J2StepDeg;
int J3zeroStep = J3axisLimNeg * J3StepDeg;
int J4zeroStep = J4axisLimNeg * J4StepDeg;
int J5zeroStep = J5axisLimNeg * J5StepDeg;
int J6zeroStep = J6axisLimNeg * J6StepDeg;
int J7zeroStep = J7axisLimNeg * J7StepDeg;
int J8zeroStep = J8axisLimNeg * J8StepDeg;
int J9zeroStep = J9axisLimNeg * J9StepDeg;

//start master step count at Jzerostep
int J1StepM = J1zeroStep;
int J2StepM = J2zeroStep;
int J3StepM = J3zeroStep;
int J4StepM = J4zeroStep;
int J5StepM = J5zeroStep;
int J6StepM = J6zeroStep;
int J7StepM = J7zeroStep;
int J8StepM = J8zeroStep;
int J9StepM = J9zeroStep;



//degrees from limit switch to offset calibration
float J1calBaseOff = -1;
float J2calBaseOff = 2;
float J3calBaseOff = 4.1;
float J4calBaseOff = -1.5;
float J5calBaseOff = 3;
float J6calBaseOff = -7;
float J7calBaseOff = 0;
float J8calBaseOff = 0;
float J9calBaseOff = 0;

//reset collision indicators
int J1collisionTrue = 0;
int J2collisionTrue = 0;
int J3collisionTrue = 0;
int J4collisionTrue = 0;
int J5collisionTrue = 0;
int J6collisionTrue = 0;
int TotalCollision = 0;
int KinematicError = 0;

float J7length;
float J7rot;
float J7steps;

float J8length;
float J8rot;
float J8steps;

float J9length;
float J9rot;
float J9steps;

float lineDist;

String WristCon;
int Quadrant;

unsigned long J1DebounceTime = 0;
unsigned long J2DebounceTime = 0;
unsigned long J3DebounceTime = 0;
unsigned long J4DebounceTime = 0;
unsigned long J5DebounceTime = 0;
unsigned long J6DebounceTime = 0;
unsigned long debounceDelay = 50;

String Alarm = "0";
String speedViolation = "0";
float maxSpeedDelay = 3000;
float minSpeedDelay = 350;
float linWayDistSP = 2;
String debug = "";
String flag = "";
const int TRACKrotdir = 0;

int J1EncSteps;
int J2EncSteps;
int J3EncSteps;
int J4EncSteps;
int J5EncSteps;
int J6EncSteps;

int J1LoopMode;
int J2LoopMode;
int J3LoopMode;
int J4LoopMode;
int J5LoopMode;
int J6LoopMode;

#define ROBOT_nDOFs 6
typedef float tRobotJoints[ROBOT_nDOFs];
typedef float tRobotPose[ROBOT_nDOFs];

//declare in out vars
float xyzuvw_Out[ROBOT_nDOFs];
float xyzuvw_In[ROBOT_nDOFs];
float xyzuvw_Temp[ROBOT_nDOFs];

float JangleOut[ROBOT_nDOFs];
float JangleIn[ROBOT_nDOFs];
float joints_estimate[ROBOT_nDOFs];
float SolutionMatrix[ROBOT_nDOFs][4];

//external axis
float J7_pos;
float J8_pos;
float J9_pos;

float J7_In;
float J8_In;
float J9_In;

#define Table_Size 6
typedef float Matrix4x4[16];
typedef float tRobot[66];

float pose[16];

String moveSequence;

//define rounding vars
float rndArcStart[6];
float rndArcMid[6];
float rndArcEnd[6];
float rndCalcCen[6];
String rndData;
bool rndTrue;
float rndSpeed;
bool splineTrue;
bool splineEndReceived;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//CALCULATE POSITIONS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void sendRobotPos() {

  updatePos();

  String sendPos = "A" + String(JangleIn[0], 3) + "B" + String(JangleIn[1], 3) + "C" + String(JangleIn[2], 3) + "D" + String(JangleIn[3], 3) + "E" + String(JangleIn[4], 3) + "F" + String(JangleIn[5], 3) + "M" + speedViolation + "N" + debug + "O" + flag + "P" + J7_pos + "Q" + J8_pos + "R" + J9_pos;
  //delay(5);
  Serial.println(sendPos);
  speedViolation = "0";
  flag = "";

}

void sendRobotPosSpline() {

  updatePos();

  String sendPos = "A" + String(JangleIn[0], 3) + "B" + String(JangleIn[1], 3) + "C" + String(JangleIn[2], 3) + "D" + String(JangleIn[3], 3) + "E" + String(JangleIn[4], 3) + "F" + String(JangleIn[5], 3) + "M" + speedViolation + "N" + debug + "O" + flag + "P" + J7_pos + "Q" + J8_pos + "R" + J9_pos;
  //delay(5);
  Serial.println(sendPos);
  speedViolation = "0";

}

void updatePos() {

  JangleIn[0] = (J1StepM - J1zeroStep) / J1StepDeg;
  JangleIn[1] = (J2StepM - J2zeroStep) / J2StepDeg;
  JangleIn[2] = (J3StepM - J3zeroStep) / J3StepDeg;
  JangleIn[3] = (J4StepM - J4zeroStep) / J4StepDeg;
  JangleIn[4] = (J5StepM - J5zeroStep) / J5StepDeg;
  JangleIn[5] = (J6StepM - J6zeroStep) / J6StepDeg;

  J7_pos = (J7StepM - J7zeroStep) / J7StepDeg;
  J8_pos = (J8StepM - J8zeroStep) / J8StepDeg;
  J9_pos = (J9StepM - J9zeroStep) / J9StepDeg;
}

void syncRobotPosFromEncoders () {
  // Sync internal step/angle state to the encoder-derived joint positions.
  // This is used after a soft E-stop so the controller doesn't drift if any steps were missed.
  long e1 = J1encPos.read();
  long e2 = J2encPos.read();
  long e3 = J3encPos.read();
  long e4 = J4encPos.read();
  long e5 = J5encPos.read();
  long e6 = J6encPos.read();

  J1StepM = e1 / J1encMult;
  J2StepM = e2 / J2encMult;
  J3StepM = e3 / J3encMult;
  J4StepM = e4 / J4encMult;
  J5StepM = e5 / J5encMult;
  J6StepM = e6 / J6encMult;

  JangleIn[0] = (J1StepM - J1zeroStep) / J1StepDeg;
  JangleIn[1] = (J2StepM - J2zeroStep) / J2StepDeg;
  JangleIn[2] = (J3StepM - J3zeroStep) / J3StepDeg;
  JangleIn[3] = (J4StepM - J4zeroStep) / J4StepDeg;
  JangleIn[4] = (J5StepM - J5zeroStep) / J5StepDeg;
  JangleIn[5] = (J6StepM - J6zeroStep) / J6StepDeg;

  // Keep aux axes consistent too (these are step-derived in this firmware)
  J7_pos = (J7StepM - J7zeroStep) / J7StepDeg;
  J8_pos = (J8StepM - J8zeroStep) / J8StepDeg;
  J9_pos = (J9StepM - J9zeroStep) / J9StepDeg;
}


void correctRobotPos () {

  J1StepM = J1encPos.read() / J1encMult;
  J2StepM = J2encPos.read() / J2encMult;
  J3StepM = J3encPos.read() / J3encMult;
  J4StepM = J4encPos.read() / J4encMult;
  J5StepM = J5encPos.read() / J5encMult;
  J6StepM = J6encPos.read() / J6encMult;

  JangleIn[0] = (J1StepM - J1zeroStep) / J1StepDeg;
  JangleIn[1] = (J2StepM - J2zeroStep) / J2StepDeg;
  JangleIn[2] = (J3StepM - J3zeroStep) / J3StepDeg;
  JangleIn[3] = (J4StepM - J4zeroStep) / J4StepDeg;
  JangleIn[4] = (J5StepM - J5zeroStep) / J5StepDeg;
  JangleIn[5] = (J6StepM - J6zeroStep) / J6StepDeg;

  String sendPos = "A" + String(JangleIn[0], 3) + "B" + String(JangleIn[1], 3) + "C" + String(JangleIn[2], 3) + "D" + String(JangleIn[3], 3) + "E" + String(JangleIn[4], 3) + "F" + String(JangleIn[5], 3) + "G" + String(xyzuvw_Out[0], 3) + "H" + String(xyzuvw_Out[1], 3) + "I" + String(xyzuvw_Out[2], 3) + "J" + String(xyzuvw_Out[3], 3) + "K" + String(xyzuvw_Out[4], 3) + "L" + String(xyzuvw_Out[5], 3) + "M" + speedViolation + "N" + debug + "O" + flag + "P" + J7_pos + "Q" + J8_pos + "R" + J9_pos;
  delay(5);
  Serial.println(sendPos);
  speedViolation = "0";
  flag = "";
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//DRIVE LIMIT
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void driveLimit(int J1Step, int J2Step, int J3Step, int J4Step, int J5Step, int J6Step, int J7Step, int J8Step, int J9Step, float SpeedVal) {

  //RESET COUNTERS
  int J1done = 0;
  int J2done = 0;
  int J3done = 0;
  int J4done = 0;
  int J5done = 0;
  int J6done = 0;
  int J7done = 0;
  int J8done = 0;
  int J9done = 0;

  int J1complete = 0;
  int J2complete = 0;
  int J3complete = 0;
  int J4complete = 0;
  int J5complete = 0;
  int J6complete = 0;
  int J7complete = 0;
  int J8complete = 0;
  int J9complete = 0;

  int calcStepGap = ((maxSpeedDelay - ((SpeedVal / 100) * maxSpeedDelay)) + minSpeedDelay + 300);

  //SET CAL DIRECTION
  digitalWrite(J1dirPin, HIGH);
  digitalWrite(J2dirPin, HIGH);
  digitalWrite(J3dirPin, LOW);
  digitalWrite(J4dirPin, LOW);
  digitalWrite(J5dirPin, HIGH);
  digitalWrite(J6dirPin, LOW);
  digitalWrite(J7dirPin, LOW);
  digitalWrite(J8dirPin, LOW);
  digitalWrite(J9dirPin, LOW);

  //DRIVE MOTORS FOR CALIBRATION

  int curRead;
  int J1CurState;
  int J2CurState;
  int J3CurState;
  int J4CurState;
  int J5CurState;
  int J6CurState;
  int J7CurState;
  int J8CurState;
  int J9CurState;
  int DriveLimInProc = 1;

  if (J1Step <= 0 ) {
    J1complete = 1;
  }
  if (J2Step <= 0) {
    J2complete = 1;
  }
  if (J3Step <= 0) {
    J3complete = 1;
  }
  if (J4Step <= 0) {
    J4complete = 1;
  }
  if (J5Step <= 0) {
    J5complete = 1;
  }
  if (J6Step <= 0) {
    J6complete = 1;
  }
  if (J7Step <= 0) {
    J7complete = 1;
  }
  if (J8Step <= 0) {
    J8complete = 1;
  }
  if (J9Step <= 0) {
    J9complete = 1;
  }


  while (DriveLimInProc == 1) {
    //EVAL J1
    if (digitalRead(J1calPin) == LOW) {
      J1CurState = LOW;
    }
    else {
      delayMicroseconds(10);
      if (digitalRead(J1calPin) == LOW) {
        J1CurState = LOW;
      }
      else {
        delayMicroseconds(10);
        if (digitalRead(J1calPin) == LOW) {
          J1CurState = LOW;
        }
        else {
          delayMicroseconds(10);
          if (digitalRead(J1calPin) == LOW) {
            J1CurState = LOW;
          }
          else {
            J1CurState = digitalRead(J1calPin);
          }
        }
      }
    }

    //EVAL J2
    if (digitalRead(J2calPin) == LOW) {
      J2CurState = LOW;
    }
    else {
      delayMicroseconds(10);
      if (digitalRead(J2calPin) == LOW) {
        J2CurState = LOW;
      }
      else {
        delayMicroseconds(10);
        if (digitalRead(J2calPin) == LOW) {
          J2CurState = LOW;
        }
        else {
          delayMicroseconds(10);
          if (digitalRead(J2calPin) == LOW) {
            J2CurState = LOW;
          }
          else {
            J2CurState = digitalRead(J2calPin);
          }
        }
      }
    }

    //EVAL J3
    if (digitalRead(J3calPin) == LOW) {
      J3CurState = LOW;
    }
    else {
      delayMicroseconds(10);
      if (digitalRead(J3calPin) == LOW) {
        J3CurState = LOW;
      }
      else {
        delayMicroseconds(10);
        if (digitalRead(J3calPin) == LOW) {
          J3CurState = LOW;
        }
        else {
          delayMicroseconds(10);
          if (digitalRead(J3calPin) == LOW) {
            J3CurState = LOW;
          }
          else {
            J3CurState = digitalRead(J3calPin);
          }
        }
      }
    }

    //EVAL J4
    if (digitalRead(J4calPin) == LOW) {
      J4CurState = LOW;
    }
    else {
      delayMicroseconds(10);
      if (digitalRead(J4calPin) == LOW) {
        J4CurState = LOW;
      }
      else {
        delayMicroseconds(10);
        if (digitalRead(J4calPin) == LOW) {
          J4CurState = LOW;
        }
        else {
          delayMicroseconds(10);
          if (digitalRead(J4calPin) == LOW) {
            J4CurState = LOW;
          }
          else {
            J4CurState = digitalRead(J4calPin);
          }
        }
      }
    }

    //EVAL J5
    if (digitalRead(J5calPin) == LOW) {
      J5CurState = LOW;
    }
    else {
      delayMicroseconds(10);
      if (digitalRead(J5calPin) == LOW) {
        J5CurState = LOW;
      }
      else {
        delayMicroseconds(10);
        if (digitalRead(J5calPin) == LOW) {
          J5CurState = LOW;
        }
        else {
          delayMicroseconds(10);
          if (digitalRead(J5calPin) == LOW) {
            J5CurState = LOW;
          }
          else {
            J5CurState = digitalRead(J5calPin);
          }
        }
      }
    }

    //EVAL J6
    if (digitalRead(J6calPin) == LOW) {
      J6CurState = LOW;
    }
    else {
      delayMicroseconds(10);
      if (digitalRead(J6calPin) == LOW) {
        J6CurState = LOW;
      }
      else {
        delayMicroseconds(10);
        if (digitalRead(J6calPin) == LOW) {
          J6CurState = LOW;
        }
        else {
          delayMicroseconds(10);
          if (digitalRead(J6calPin) == LOW) {
            J6CurState = LOW;
          }
          else {
            J6CurState = digitalRead(J6calPin);
          }
        }
      }
    }

    //EVAL J7
    if (digitalRead(J7calPin) == LOW) {
      J7CurState = LOW;
    }
    else {
      delayMicroseconds(10);
      if (digitalRead(J7calPin) == LOW) {
        J7CurState = LOW;
      }
      else {
        delayMicroseconds(10);
        if (digitalRead(J7calPin) == LOW) {
          J7CurState = LOW;
        }
        else {
          delayMicroseconds(10);
          if (digitalRead(J7calPin) == LOW) {
            J7CurState = LOW;
          }
          else {
            J7CurState = digitalRead(J7calPin);
          }
        }
      }
    }

    //EVAL J8
    if (digitalRead(J8calPin) == LOW) {
      J8CurState = LOW;
    }
    else {
      delayMicroseconds(10);
      if (digitalRead(J8calPin) == LOW) {
        J8CurState = LOW;
      }
      else {
        delayMicroseconds(10);
        if (digitalRead(J8calPin) == LOW) {
          J8CurState = LOW;
        }
        else {
          delayMicroseconds(10);
          if (digitalRead(J8calPin) == LOW) {
            J8CurState = LOW;
          }
          else {
            J8CurState = digitalRead(J8calPin);
          }
        }
      }
    }

    //EVAL J9
    if (digitalRead(J9calPin) == LOW) {
      J9CurState = LOW;
    }
    else {
      delayMicroseconds(10);
      if (digitalRead(J9calPin) == LOW) {
        J9CurState = LOW;
      }
      else {
        delayMicroseconds(10);
        if (digitalRead(J9calPin) == LOW) {
          J9CurState = LOW;
        }
        else {
          delayMicroseconds(10);
          if (digitalRead(J9calPin) == LOW) {
            J9CurState = LOW;
          }
          else {
            J9CurState = digitalRead(J9calPin);
          }
        }
      }
    }



    if (J1done < J1Step && J1CurState == LOW) {
      digitalWrite(J1stepPin, LOW);
      delayMicroseconds(50);
      digitalWrite(J1stepPin, HIGH);
      J1done = ++J1done;
    }
    else {
      J1complete = 1;
    }
    if (J2done < J2Step && J2CurState == LOW) {
      digitalWrite(J2stepPin, LOW);
      delayMicroseconds(50);
      digitalWrite(J2stepPin, HIGH);
      J2done = ++J2done;
    }
    else {
      J2complete = 1;
    }
    if (J3done < J3Step && J3CurState == LOW) {
      digitalWrite(J3stepPin, LOW);
      delayMicroseconds(50);
      digitalWrite(J3stepPin, HIGH);
      J3done = ++J3done;
    }
    else {
      J3complete = 1;
    }
    if (J4done < J4Step && J4CurState == LOW) {
      digitalWrite(J4stepPin, LOW);
      delayMicroseconds(50);
      digitalWrite(J4stepPin, HIGH);
      J4done = ++J4done;
    }
    else {
      J4complete = 1;
    }
    if (J5done < J5Step && J5CurState == LOW) {
      digitalWrite(J5stepPin, LOW);
      delayMicroseconds(50);
      digitalWrite(J5stepPin, HIGH);
      J5done = ++J5done;
    }
    else {
      J5complete = 1;
    }
    if (J6done < J6Step && J6CurState == LOW) {
      digitalWrite(J6stepPin, LOW);
      delayMicroseconds(50);
      digitalWrite(J6stepPin, HIGH);
      J6done = ++J6done;
    }
    else {
      J6complete = 1;
    }
    if (J7done < J7Step && J7CurState == LOW) {
      digitalWrite(J7stepPin, LOW);
      delayMicroseconds(50);
      digitalWrite(J7stepPin, HIGH);
      J7done = ++J7done;
    }
    else {
      J7complete = 1;
    }
    if (J8done < J8Step && J8CurState == LOW) {
      digitalWrite(J8stepPin, LOW);
      delayMicroseconds(50);
      digitalWrite(J8stepPin, HIGH);
      J8done = ++J8done;
    }
    else {
      J8complete = 1;
    }
    if (J9done < J9Step && J9CurState == LOW) {
      digitalWrite(J9stepPin, LOW);
      delayMicroseconds(50);
      digitalWrite(J9stepPin, HIGH);
      J9done = ++J9done;
    }
    else {
      J9complete = 1;
    }
    //jump out if complete
    if (J1complete + J2complete + J3complete + J4complete + J5complete + J6complete + J7complete + J8complete + J9complete == 9) {
      DriveLimInProc = 0;
    }
    ///////////////DELAY BEFORE RESTARTING LOOP
    delayMicroseconds(calcStepGap);
  }

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//CHECK ENCODERS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void resetEncoders() {

  J1collisionTrue = 0;
  J2collisionTrue = 0;
  J3collisionTrue = 0;
  J4collisionTrue = 0;
  J5collisionTrue = 0;
  J6collisionTrue = 0;

  //set encoders to current position
  J1encPos.write(J1StepM * J1encMult);
  J2encPos.write(J2StepM * J2encMult);
  J3encPos.write(J3StepM * J3encMult);
  J4encPos.write(J4StepM * J4encMult);
  J5encPos.write(J5StepM * J5encMult);
  J6encPos.write(J6StepM * J6encMult);
  //delayMicroseconds(5);

}

void checkEncoders() {
  //read encoders
  J1EncSteps = J1encPos.read() / J1encMult;
  J2EncSteps = J2encPos.read() / J2encMult;
  J3EncSteps = J3encPos.read() / J3encMult;
  J4EncSteps = J4encPos.read() / J4encMult;
  J5EncSteps = J5encPos.read() / J5encMult;
  J6EncSteps = J6encPos.read() / J6encMult;

  if (abs((J1EncSteps - J1StepM)) >= 15) {
    if (J1LoopMode == 0) {
      J1collisionTrue = 1;
      J1StepM = J1encPos.read() / J1encMult;
    }
  }
  if (abs((J2EncSteps - J2StepM)) >= 15) {
    if (J2LoopMode == 0) {
      J2collisionTrue = 1;
      J2StepM = J2encPos.read() / J2encMult;
    }
  }
  if (abs((J3EncSteps - J3StepM)) >= 15) {
    if (J3LoopMode == 0) {
      J3collisionTrue = 1;
      J3StepM = J3encPos.read() / J3encMult;
    }
  }
  if (abs((J4EncSteps - J4StepM)) >= 15) {
    if (J4LoopMode == 0) {
      J4collisionTrue = 1;
      J4StepM = J4encPos.read() / J4encMult;
    }
  }
  if (abs((J5EncSteps - J5StepM)) >= 15) {
    if (J5LoopMode == 0) {
      J5collisionTrue = 1;
      J5StepM = J5encPos.read() / J5encMult;
    }
  }
  if (abs((J6EncSteps - J6StepM)) >= 15) {
    if (J6LoopMode == 0) {
      J6collisionTrue = 1;
      J6StepM = J6encPos.read() / J6encMult;
    }
  }

  TotalCollision = J1collisionTrue + J2collisionTrue + J3collisionTrue + J4collisionTrue + J5collisionTrue + J6collisionTrue;
  if (TotalCollision > 0) {
    flag = "EC" + String(J1collisionTrue) + String(J2collisionTrue) + String(J3collisionTrue) + String(J4collisionTrue) + String(J5collisionTrue) + String(J6collisionTrue);
  }
}







/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//DRIVE MOTORS J
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void driveMotorsJ(int J1step, int J2step, int J3step, int J4step, int J5step, int J6step, int J7step, int J8step, int J9step, int J1dir, int J2dir, int J3dir, int J4dir, int J5dir, int J6dir, int J7dir, int J8dir, int J9dir, String SpeedType, float SpeedVal, float ACCspd, float DCCspd, float ACCramp) {

  //FIND HIGHEST STEP
  int HighStep = J1step;
  if (J2step > HighStep)
  {
    HighStep = J2step;
  }
  if (J3step > HighStep)
  {
    HighStep = J3step;
  }
  if (J4step > HighStep)
  {
    HighStep = J4step;
  }
  if (J5step > HighStep)
  {
    HighStep = J5step;
  }
  if (J6step > HighStep)
  {
    HighStep = J6step;
  }
  if (J7step > HighStep)
  {
    HighStep = J7step;
  }
  if (J8step > HighStep)
  {
    HighStep = J8step;
  }
  if (J9step > HighStep)
  {
    HighStep = J9step;
  }

  //FIND ACTIVE JOINTS
  int J1active = 0;
  int J2active = 0;
  int J3active = 0;
  int J4active = 0;
  int J5active = 0;
  int J6active = 0;
  int J7active = 0;
  int J8active = 0;
  int J9active = 0;
  int Jactive = 0;

  if (J1step >= 1)
  {
    J1active = 1;
  }
  if (J2step >= 1)
  {
    J2active = 1;
  }
  if (J3step >= 1)
  {
    J3active = 1;
  }
  if (J4step >= 1)
  {
    J4active = 1;
  }
  if (J5step >= 1)
  {
    J5active = 1;
  }
  if (J6step >= 1)
  {
    J6active = 1;
  }
  if (J7step >= 1)
  {
    J7active = 1;
  }
  if (J8step >= 1)
  {
    J8active = 1;
  }
  if (J9step >= 1)
  {
    J9active = 1;
  }
  Jactive = (J1active + J2active + J3active + J4active + J5active + J6active + J7active + J8active + J9active);

  int J1_PE = 0;
  int J2_PE = 0;
  int J3_PE = 0;
  int J4_PE = 0;
  int J5_PE = 0;
  int J6_PE = 0;
  int J7_PE = 0;
  int J8_PE = 0;
  int J9_PE = 0;

  int J1_SE_1 = 0;
  int J2_SE_1 = 0;
  int J3_SE_1 = 0;
  int J4_SE_1 = 0;
  int J5_SE_1 = 0;
  int J6_SE_1 = 0;
  int J7_SE_1 = 0;
  int J8_SE_1 = 0;
  int J9_SE_1 = 0;

  int J1_SE_2 = 0;
  int J2_SE_2 = 0;
  int J3_SE_2 = 0;
  int J4_SE_2 = 0;
  int J5_SE_2 = 0;
  int J6_SE_2 = 0;
  int J7_SE_2 = 0;
  int J8_SE_2 = 0;
  int J9_SE_2 = 0;

  int J1_LO_1 = 0;
  int J2_LO_1 = 0;
  int J3_LO_1 = 0;
  int J4_LO_1 = 0;
  int J5_LO_1 = 0;
  int J6_LO_1 = 0;
  int J7_LO_1 = 0;
  int J8_LO_1 = 0;
  int J9_LO_1 = 0;

  int J1_LO_2 = 0;
  int J2_LO_2 = 0;
  int J3_LO_2 = 0;
  int J4_LO_2 = 0;
  int J5_LO_2 = 0;
  int J6_LO_2 = 0;
  int J7_LO_2 = 0;
  int J8_LO_2 = 0;
  int J9_LO_2 = 0;

  //reset
  int J1cur = 0;
  int J2cur = 0;
  int J3cur = 0;
  int J4cur = 0;
  int J5cur = 0;
  int J6cur = 0;
  int J7cur = 0;
  int J8cur = 0;
  int J9cur = 0;

  int J1_PEcur = 0;
  int J2_PEcur = 0;
  int J3_PEcur = 0;
  int J4_PEcur = 0;
  int J5_PEcur = 0;
  int J6_PEcur = 0;
  int J7_PEcur = 0;
  int J8_PEcur = 0;
  int J9_PEcur = 0;

  int J1_SE_1cur = 0;
  int J2_SE_1cur = 0;
  int J3_SE_1cur = 0;
  int J4_SE_1cur = 0;
  int J5_SE_1cur = 0;
  int J6_SE_1cur = 0;
  int J7_SE_1cur = 0;
  int J8_SE_1cur = 0;
  int J9_SE_1cur = 0;

  int J1_SE_2cur = 0;
  int J2_SE_2cur = 0;
  int J3_SE_2cur = 0;
  int J4_SE_2cur = 0;
  int J5_SE_2cur = 0;
  int J6_SE_2cur = 0;
  int J7_SE_2cur = 0;
  int J8_SE_2cur = 0;
  int J9_SE_2cur = 0;

  int highStepCur = 0;
  float curDelay = 0;

  float speedSP;
  float moveDist;

  //int J4EncSteps;

  //SET DIRECTIONS

  /// J1 ///
  //flip J1 direction
  if (J1dir) {
    digitalWrite(J1dirPin, LOW);
  }
  else {
    digitalWrite(J1dirPin, HIGH);
  }
  /// J2 ///
  if (J2dir) {
    digitalWrite(J2dirPin, LOW);
  }
  else {
    digitalWrite(J2dirPin, HIGH);
  }
  /// J3 ///
  if (J3dir) {
    digitalWrite(J3dirPin, LOW);
  }
  else {
    digitalWrite(J3dirPin, HIGH);
  }
  /// J4 ///
  if (J4dir) {
    digitalWrite(J4dirPin, HIGH);
  }
  else {
    digitalWrite(J4dirPin, LOW);
  }
  /// J5 ///
  if (J5dir) {
    digitalWrite(J5dirPin, LOW);
  }
  else {
    digitalWrite(J5dirPin, HIGH);
  }
  /// J6 ///
  if (J6dir) {
    digitalWrite(J6dirPin, LOW);
  }
  else {
    digitalWrite(J6dirPin, HIGH);
  }
  /// J7 ///
  if (J7dir) {
    digitalWrite(J7dirPin, HIGH);
  }
  else {
    digitalWrite(J7dirPin, LOW);
  }
  /// J8 ///
  if (J8dir) {
    digitalWrite(J8dirPin, HIGH);
  }
  else {
    digitalWrite(J8dirPin, LOW);
  }
  /// J9 ///
  if (J9dir) {
    digitalWrite(J9dirPin, HIGH);
  }
  else {
    digitalWrite(J9dirPin, LOW);
  }

  /////CALC SPEEDS//////
  float calcStepGap;

  //determine steps
  float ACCStep = HighStep * (ACCspd / 100);
  float NORStep = HighStep * ((100 - ACCspd - DCCspd) / 100);
  float DCCStep = HighStep * (DCCspd / 100);

  //set speed for seconds or mm per sec
  if (SpeedType == "s") {
    speedSP = (SpeedVal * 1000000) * .8;
  }
  else if (SpeedType == "m") {
    lineDist = pow((pow((xyzuvw_In[0] - xyzuvw_Out[0]), 2) + pow((xyzuvw_In[1] - xyzuvw_Out[1]), 2) + pow((xyzuvw_In[2] - xyzuvw_Out[2]), 2)), .5);
    speedSP = ((lineDist / SpeedVal) * 1000000) * .8;
  }

  //calc step gap for seconds or mm per sec
  if (SpeedType == "s" or SpeedType == "m" ) {
    float zeroStepGap = speedSP / HighStep;
    float zeroACCstepInc = (zeroStepGap * (100 / ACCramp)) / ACCStep;
    float zeroACCtime = ((ACCStep) * zeroStepGap) + ((ACCStep - 9) * (((ACCStep) * (zeroACCstepInc / 2))));
    float zeroNORtime = NORStep * zeroStepGap;
    float zeroDCCstepInc = (zeroStepGap * (100 / ACCramp)) / DCCStep;
    float zeroDCCtime = ((DCCStep) * zeroStepGap) + ((DCCStep - 9) * (((DCCStep) * (zeroDCCstepInc / 2))));
    float zeroTOTtime = zeroACCtime + zeroNORtime + zeroDCCtime;
    float overclockPerc = speedSP / zeroTOTtime;
    calcStepGap = zeroStepGap * overclockPerc;
    if (calcStepGap <= minSpeedDelay) {
      calcStepGap = minSpeedDelay;
      speedViolation = "1";
    }
  }

  //calc step gap for percentage
  else if (SpeedType == "p") {
    calcStepGap = (maxSpeedDelay - ((SpeedVal / 100) * maxSpeedDelay));
    if (calcStepGap < minSpeedDelay) {
      calcStepGap = minSpeedDelay;
    }
  }

  //calculate final step increments
  float calcACCstepInc = (calcStepGap * (100 / ACCramp)) / ACCStep;
  float calcDCCstepInc = (calcStepGap * (100 / ACCramp)) / DCCStep;
  float calcACCstartDel = (calcACCstepInc * ACCStep) * 2;

  //set starting delay
  if (rndTrue == true) {
    curDelay = rndSpeed;
    rndTrue = false;
  }
  else {
    curDelay = calcACCstartDel;
  }


  // Soft E-stop decel support
  bool estopBraking = false;
  unsigned long estopBrakeStart = 0;
///// DRIVE MOTORS /////
  while (J1cur < J1step || J2cur < J2step || J3cur < J3step || J4cur < J4step || J5cur < J5step || J6cur < J6step || J7cur < J7step || J8cur < J8step || J9cur < J9step)
  {
    // Soft E-stop: ramp down step rate briefly, then stop stepping (hold torque)
    if (estop_latched) {
      if (!estopBraking) {
        estopBraking = true;
        estopBrakeStart = micros();
      }
      // Increase delay to reduce speed smoothly
      curDelay = (curDelay * 1.08f) + 5.0f;
      // Stop stepping after a short braking window
      if ((micros() - estopBrakeStart) > 200000UL || curDelay > 60000.0f) {
        return;
      }
    }
    ////DELAY CALC/////
    if (highStepCur <= ACCStep) {
      curDelay = curDelay - (calcACCstepInc);
    }
    else if (highStepCur >= (HighStep - DCCStep)) {
      curDelay = curDelay + (calcDCCstepInc);
    }
    else {
      curDelay = calcStepGap;
    }



    float distDelay = 60;
    if (debugg == 1) {
      distDelay = 0;
    }
    float disDelayCur = 0;


    /////// J1 ////////////////////////////////
    ///find pulse every
    if (J1cur < J1step)
    {
      J1_PE = (HighStep / J1step);
      ///find left over 1
      J1_LO_1 = (HighStep - (J1step * J1_PE));
      ///find skip 1
      if (J1_LO_1 > 0)
      {
        J1_SE_1 = (HighStep / J1_LO_1);
      }
      else
      {
        J1_SE_1 = 0;
      }
      ///find left over 2
      if (J1_SE_1 > 0)
      {
        J1_LO_2 = HighStep - ((J1step * J1_PE) + ((J1step * J1_PE) / J1_SE_1));
      }
      else
      {
        J1_LO_2 = 0;
      }
      ///find skip 2
      if (J1_LO_2 > 0)
      {
        J1_SE_2 = (HighStep / J1_LO_2);
      }
      else
      {
        J1_SE_2 = 0;
      }
      /////////  J1  ///////////////
      if (J1_SE_2 == 0)
      {
        J1_SE_2cur = (J1_SE_2 + 1);
      }
      if (J1_SE_2cur != J1_SE_2)
      {
        J1_SE_2cur = ++J1_SE_2cur;
        if (J1_SE_1 == 0)
        {
          J1_SE_1cur = (J1_SE_1 + 1);
        }
        if (J1_SE_1cur != J1_SE_1)
        {
          J1_SE_1cur = ++J1_SE_1cur;
          J1_PEcur = ++J1_PEcur;
          if (J1_PEcur == J1_PE)
          {
            J1cur = ++J1cur;
            J1_PEcur = 0;
            digitalWrite(J1stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J1dir == 0) {
              J1StepM == --J1StepM;
            }
            else {
              J1StepM == ++J1StepM;
            }
          }
        }
        else
        {
          J1_SE_1cur = 0;
        }
      }
      else
      {
        J1_SE_2cur = 0;
      }
    }


    /////// J2 ////////////////////////////////
    ///find pulse every
    if (J2cur < J2step)
    {
      J2_PE = (HighStep / J2step);
      ///find left over 1
      J2_LO_1 = (HighStep - (J2step * J2_PE));
      ///find skip 1
      if (J2_LO_1 > 0)
      {
        J2_SE_1 = (HighStep / J2_LO_1);
      }
      else
      {
        J2_SE_1 = 0;
      }
      ///find left over 2
      if (J2_SE_1 > 0)
      {
        J2_LO_2 = HighStep - ((J2step * J2_PE) + ((J2step * J2_PE) / J2_SE_1));
      }
      else
      {
        J2_LO_2 = 0;
      }
      ///find skip 2
      if (J2_LO_2 > 0)
      {
        J2_SE_2 = (HighStep / J2_LO_2);
      }
      else
      {
        J2_SE_2 = 0;
      }
      /////////  J2  ///////////////
      if (J2_SE_2 == 0)
      {
        J2_SE_2cur = (J2_SE_2 + 1);
      }
      if (J2_SE_2cur != J2_SE_2)
      {
        J2_SE_2cur = ++J2_SE_2cur;
        if (J2_SE_1 == 0)
        {
          J2_SE_1cur = (J2_SE_1 + 1);
        }
        if (J2_SE_1cur != J2_SE_1)
        {
          J2_SE_1cur = ++J2_SE_1cur;
          J2_PEcur = ++J2_PEcur;
          if (J2_PEcur == J2_PE)
          {
            J2cur = ++J2cur;
            J2_PEcur = 0;
            digitalWrite(J2stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J2dir == 0) {
              J2StepM == --J2StepM;
            }
            else {
              J2StepM == ++J2StepM;
            }
          }
        }
        else
        {
          J2_SE_1cur = 0;
        }
      }
      else
      {
        J2_SE_2cur = 0;
      }
    }

    /////// J3 ////////////////////////////////
    ///find pulse every
    if (J3cur < J3step)
    {
      J3_PE = (HighStep / J3step);
      ///find left over 1
      J3_LO_1 = (HighStep - (J3step * J3_PE));
      ///find skip 1
      if (J3_LO_1 > 0)
      {
        J3_SE_1 = (HighStep / J3_LO_1);
      }
      else
      {
        J3_SE_1 = 0;
      }
      ///find left over 2
      if (J3_SE_1 > 0)
      {
        J3_LO_2 = HighStep - ((J3step * J3_PE) + ((J3step * J3_PE) / J3_SE_1));
      }
      else
      {
        J3_LO_2 = 0;
      }
      ///find skip 2
      if (J3_LO_2 > 0)
      {
        J3_SE_2 = (HighStep / J3_LO_2);
      }
      else
      {
        J3_SE_2 = 0;
      }
      /////////  J3  ///////////////
      if (J3_SE_2 == 0)
      {
        J3_SE_2cur = (J3_SE_2 + 1);
      }
      if (J3_SE_2cur != J3_SE_2)
      {
        J3_SE_2cur = ++J3_SE_2cur;
        if (J3_SE_1 == 0)
        {
          J3_SE_1cur = (J3_SE_1 + 1);
        }
        if (J3_SE_1cur != J3_SE_1)
        {
          J3_SE_1cur = ++J3_SE_1cur;
          J3_PEcur = ++J3_PEcur;
          if (J3_PEcur == J3_PE)
          {
            J3cur = ++J3cur;
            J3_PEcur = 0;
            digitalWrite(J3stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J3dir == 0) {
              J3StepM == --J3StepM;
            }
            else {
              J3StepM == ++J3StepM;
            }
          }
        }
        else
        {
          J3_SE_1cur = 0;
        }
      }
      else
      {
        J3_SE_2cur = 0;
      }
    }


    /////// J4 ////////////////////////////////
    ///find pulse every
    if (J4cur < J4step)
    {
      J4_PE = (HighStep / J4step);
      ///find left over 1
      J4_LO_1 = (HighStep - (J4step * J4_PE));
      ///find skip 1
      if (J4_LO_1 > 0)
      {
        J4_SE_1 = (HighStep / J4_LO_1);
      }
      else
      {
        J4_SE_1 = 0;
      }
      ///find left over 2
      if (J4_SE_1 > 0)
      {
        J4_LO_2 = HighStep - ((J4step * J4_PE) + ((J4step * J4_PE) / J4_SE_1));
      }
      else
      {
        J4_LO_2 = 0;
      }
      ///find skip 2
      if (J4_LO_2 > 0)
      {
        J4_SE_2 = (HighStep / J4_LO_2);
      }
      else
      {
        J4_SE_2 = 0;
      }
      /////////  J4  ///////////////
      if (J4_SE_2 == 0)
      {
        J4_SE_2cur = (J4_SE_2 + 1);
      }
      if (J4_SE_2cur != J4_SE_2)
      {
        J4_SE_2cur = ++J4_SE_2cur;
        if (J4_SE_1 == 0)
        {
          J4_SE_1cur = (J4_SE_1 + 1);
        }
        if (J4_SE_1cur != J4_SE_1)
        {
          J4_SE_1cur = ++J4_SE_1cur;
          J4_PEcur = ++J4_PEcur;
          if (J4_PEcur == J4_PE)
          {
            J4cur = ++J4cur;
            J4_PEcur = 0;
            digitalWrite(J4stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J4dir == 0) {
              J4StepM == --J4StepM;
            }
            else {
              J4StepM == ++J4StepM;
            }
          }
        }
        else
        {
          J4_SE_1cur = 0;
        }
      }
      else
      {
        J4_SE_2cur = 0;
      }
    }


    /////// J5 ////////////////////////////////
    ///find pulse every
    if (J5cur < J5step)
    {
      J5_PE = (HighStep / J5step);
      ///find left over 1
      J5_LO_1 = (HighStep - (J5step * J5_PE));
      ///find skip 1
      if (J5_LO_1 > 0)
      {
        J5_SE_1 = (HighStep / J5_LO_1);
      }
      else
      {
        J5_SE_1 = 0;
      }
      ///find left over 2
      if (J5_SE_1 > 0)
      {
        J5_LO_2 = HighStep - ((J5step * J5_PE) + ((J5step * J5_PE) / J5_SE_1));
      }
      else
      {
        J5_LO_2 = 0;
      }
      ///find skip 2
      if (J5_LO_2 > 0)
      {
        J5_SE_2 = (HighStep / J5_LO_2);
      }
      else
      {
        J5_SE_2 = 0;
      }
      /////////  J5  ///////////////
      if (J5_SE_2 == 0)
      {
        J5_SE_2cur = (J5_SE_2 + 1);
      }
      if (J5_SE_2cur != J5_SE_2)
      {
        J5_SE_2cur = ++J5_SE_2cur;
        if (J5_SE_1 == 0)
        {
          J5_SE_1cur = (J5_SE_1 + 1);
        }
        if (J5_SE_1cur != J5_SE_1)
        {
          J5_SE_1cur = ++J5_SE_1cur;
          J5_PEcur = ++J5_PEcur;
          if (J5_PEcur == J5_PE)
          {
            J5cur = ++J5cur;
            J5_PEcur = 0;
            digitalWrite(J5stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J5dir == 0) {
              J5StepM == --J5StepM;
            }
            else {
              J5StepM == ++J5StepM;
            }
          }
        }
        else
        {
          J5_SE_1cur = 0;
        }
      }
      else
      {
        J5_SE_2cur = 0;
      }
    }


    /////// J6 ////////////////////////////////
    ///find pulse every
    if (J6cur < J6step)
    {
      J6_PE = (HighStep / J6step);
      ///find left over 1
      J6_LO_1 = (HighStep - (J6step * J6_PE));
      ///find skip 1
      if (J6_LO_1 > 0)
      {
        J6_SE_1 = (HighStep / J6_LO_1);
      }
      else
      {
        J6_SE_1 = 0;
      }
      ///find left over 2
      if (J6_SE_1 > 0)
      {
        J6_LO_2 = HighStep - ((J6step * J6_PE) + ((J6step * J6_PE) / J6_SE_1));
      }
      else
      {
        J6_LO_2 = 0;
      }
      ///find skip 2
      if (J6_LO_2 > 0)
      {
        J6_SE_2 = (HighStep / J6_LO_2);
      }
      else
      {
        J6_SE_2 = 0;
      }
      /////////  J6  ///////////////
      if (J6_SE_2 == 0)
      {
        J6_SE_2cur = (J6_SE_2 + 1);
      }
      if (J6_SE_2cur != J6_SE_2)
      {
        J6_SE_2cur = ++J6_SE_2cur;
        if (J6_SE_1 == 0)
        {
          J6_SE_1cur = (J6_SE_1 + 1);
        }
        if (J6_SE_1cur != J6_SE_1)
        {
          J6_SE_1cur = ++J6_SE_1cur;
          J6_PEcur = ++J6_PEcur;
          if (J6_PEcur == J6_PE)
          {
            J6cur = ++J6cur;
            J6_PEcur = 0;
            digitalWrite(J6stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J6dir == 0) {
              J6StepM == --J6StepM;
            }
            else {
              J6StepM == ++J6StepM;
            }
          }
        }
        else
        {
          J6_SE_1cur = 0;
        }
      }
      else
      {
        J6_SE_2cur = 0;
      }
    }


    /////// J7 ////////////////////////////////
    ///find pulse every
    if (J7cur < J7step)
    {
      J7_PE = (HighStep / J7step);
      ///find left over 1
      J7_LO_1 = (HighStep - (J7step * J7_PE));
      ///find skip 1
      if (J7_LO_1 > 0)
      {
        J7_SE_1 = (HighStep / J7_LO_1);
      }
      else
      {
        J7_SE_1 = 0;
      }
      ///find left over 2
      if (J7_SE_1 > 0)
      {
        J7_LO_2 = HighStep - ((J7step * J7_PE) + ((J7step * J7_PE) / J7_SE_1));
      }
      else
      {
        J7_LO_2 = 0;
      }
      ///find skip 2
      if (J7_LO_2 > 0)
      {
        J7_SE_2 = (HighStep / J7_LO_2);
      }
      else
      {
        J7_SE_2 = 0;
      }
      /////////  J7  ///////////////
      if (J7_SE_2 == 0)
      {
        J7_SE_2cur = (J7_SE_2 + 1);
      }
      if (J7_SE_2cur != J7_SE_2)
      {
        J7_SE_2cur = ++J7_SE_2cur;
        if (J7_SE_1 == 0)
        {
          J7_SE_1cur = (J7_SE_1 + 1);
        }
        if (J7_SE_1cur != J7_SE_1)
        {
          J7_SE_1cur = ++J7_SE_1cur;
          J7_PEcur = ++J7_PEcur;
          if (J7_PEcur == J7_PE)
          {
            J7cur = ++J7cur;
            J7_PEcur = 0;
            digitalWrite(J7stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J7dir == 0) {
              J7StepM == --J7StepM;
            }
            else {
              J7StepM == ++J7StepM;
            }
          }
        }
        else
        {
          J7_SE_1cur = 0;
        }
      }
      else
      {
        J7_SE_2cur = 0;
      }
    }




    /////// J8 ////////////////////////////////
    ///find pulse every
    if (J8cur < J8step)
    {
      J8_PE = (HighStep / J8step);
      ///find left over 1
      J8_LO_1 = (HighStep - (J8step * J8_PE));
      ///find skip 1
      if (J8_LO_1 > 0)
      {
        J8_SE_1 = (HighStep / J8_LO_1);
      }
      else
      {
        J8_SE_1 = 0;
      }
      ///find left over 2
      if (J8_SE_1 > 0)
      {
        J8_LO_2 = HighStep - ((J8step * J8_PE) + ((J8step * J8_PE) / J8_SE_1));
      }
      else
      {
        J8_LO_2 = 0;
      }
      ///find skip 2
      if (J8_LO_2 > 0)
      {
        J8_SE_2 = (HighStep / J8_LO_2);
      }
      else
      {
        J8_SE_2 = 0;
      }
      /////////  J8  ///////////////
      if (J8_SE_2 == 0)
      {
        J8_SE_2cur = (J8_SE_2 + 1);
      }
      if (J8_SE_2cur != J8_SE_2)
      {
        J8_SE_2cur = ++J8_SE_2cur;
        if (J8_SE_1 == 0)
        {
          J8_SE_1cur = (J8_SE_1 + 1);
        }
        if (J8_SE_1cur != J8_SE_1)
        {
          J8_SE_1cur = ++J8_SE_1cur;
          J8_PEcur = ++J8_PEcur;
          if (J8_PEcur == J8_PE)
          {
            J8cur = ++J8cur;
            J8_PEcur = 0;
            digitalWrite(J8stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J8dir == 0) {
              J8StepM == --J8StepM;
            }
            else {
              J8StepM == ++J8StepM;
            }
          }
        }
        else
        {
          J8_SE_1cur = 0;
        }
      }
      else
      {
        J8_SE_2cur = 0;
      }
    }


    /////// J9 ////////////////////////////////
    ///find pulse every
    if (J9cur < J9step)
    {
      J9_PE = (HighStep / J9step);
      ///find left over 1
      J9_LO_1 = (HighStep - (J9step * J9_PE));
      ///find skip 1
      if (J9_LO_1 > 0)
      {
        J9_SE_1 = (HighStep / J9_LO_1);
      }
      else
      {
        J9_SE_1 = 0;
      }
      ///find left over 2
      if (J9_SE_1 > 0)
      {
        J9_LO_2 = HighStep - ((J9step * J9_PE) + ((J9step * J9_PE) / J9_SE_1));
      }
      else
      {
        J9_LO_2 = 0;
      }
      ///find skip 2
      if (J9_LO_2 > 0)
      {
        J9_SE_2 = (HighStep / J9_LO_2);
      }
      else
      {
        J9_SE_2 = 0;
      }
      /////////  J9  ///////////////
      if (J9_SE_2 == 0)
      {
        J9_SE_2cur = (J9_SE_2 + 1);
      }
      if (J9_SE_2cur != J9_SE_2)
      {
        J9_SE_2cur = ++J9_SE_2cur;
        if (J9_SE_1 == 0)
        {
          J9_SE_1cur = (J9_SE_1 + 1);
        }
        if (J9_SE_1cur != J9_SE_1)
        {
          J9_SE_1cur = ++J9_SE_1cur;
          J9_PEcur = ++J9_PEcur;
          if (J9_PEcur == J9_PE)
          {
            J9cur = ++J9cur;
            J9_PEcur = 0;
            digitalWrite(J9stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J9dir == 0) {
              J9StepM == --J9StepM;
            }
            else {
              J9StepM == ++J9StepM;
            }
          }
        }
        else
        {
          J9_SE_1cur = 0;
        }
      }
      else
      {
        J9_SE_2cur = 0;
      }
    }


    // inc cur step
    highStepCur = ++highStepCur;
    digitalWrite(J1stepPin, HIGH);
    digitalWrite(J2stepPin, HIGH);
    digitalWrite(J3stepPin, HIGH);
    digitalWrite(J4stepPin, HIGH);
    digitalWrite(J5stepPin, HIGH);
    digitalWrite(J6stepPin, HIGH);
    digitalWrite(J7stepPin, HIGH);
    digitalWrite(J8stepPin, HIGH);
    digitalWrite(J9stepPin, HIGH);
    if (debugg == 0) {
      delayMicroseconds(curDelay - disDelayCur);
    }

  }
  //set rounding speed to last move speed
  rndSpeed = curDelay;
}





/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//DRIVE MOTORS L
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void driveMotorsL(int J1step, int J2step, int J3step, int J4step, int J5step, int J6step, int J7step, int J8step, int J9step, int J1dir, int J2dir, int J3dir, int J4dir, int J5dir, int J6dir, int J7dir, int J8dir, int J9dir, float curDelay) {

  //FIND HIGHEST STEP
  int HighStep = J1step;
  if (J2step > HighStep)
  {
    HighStep = J2step;
  }
  if (J3step > HighStep)
  {
    HighStep = J3step;
  }
  if (J4step > HighStep)
  {
    HighStep = J4step;
  }
  if (J5step > HighStep)
  {
    HighStep = J5step;
  }
  if (J6step > HighStep)
  {
    HighStep = J6step;
  }
  if (J7step > HighStep)
  {
    HighStep = J7step;
  }
  if (J8step > HighStep)
  {
    HighStep = J8step;
  }
  if (J9step > HighStep)
  {
    HighStep = J9step;
  }

  //FIND ACTIVE JOINTS
  int J1active = 0;
  int J2active = 0;
  int J3active = 0;
  int J4active = 0;
  int J5active = 0;
  int J6active = 0;
  int J7active = 0;
  int J8active = 0;
  int J9active = 0;
  int Jactive = 0;

  if (J1step >= 1)
  {
    J1active = 1;
  }
  if (J2step >= 1)
  {
    J2active = 1;
  }
  if (J3step >= 1)
  {
    J3active = 1;
  }
  if (J4step >= 1)
  {
    J4active = 1;
  }
  if (J5step >= 1)
  {
    J5active = 1;
  }
  if (J6step >= 1)
  {
    J6active = 1;
  }
  if (J7step >= 1)
  {
    J7active = 1;
  }
  if (J8step >= 1)
  {
    J8active = 1;
  }
  if (J9step >= 1)
  {
    J9active = 1;
  }
  Jactive = (J1active + J2active + J3active + J4active + J5active + J6active + J7active + J8active + J9active);

  int J1_PE = 0;
  int J2_PE = 0;
  int J3_PE = 0;
  int J4_PE = 0;
  int J5_PE = 0;
  int J6_PE = 0;
  int J7_PE = 0;
  int J8_PE = 0;
  int J9_PE = 0;

  int J1_SE_1 = 0;
  int J2_SE_1 = 0;
  int J3_SE_1 = 0;
  int J4_SE_1 = 0;
  int J5_SE_1 = 0;
  int J6_SE_1 = 0;
  int J7_SE_1 = 0;
  int J8_SE_1 = 0;
  int J9_SE_1 = 0;

  int J1_SE_2 = 0;
  int J2_SE_2 = 0;
  int J3_SE_2 = 0;
  int J4_SE_2 = 0;
  int J5_SE_2 = 0;
  int J6_SE_2 = 0;
  int J7_SE_2 = 0;
  int J8_SE_2 = 0;
  int J9_SE_2 = 0;

  int J1_LO_1 = 0;
  int J2_LO_1 = 0;
  int J3_LO_1 = 0;
  int J4_LO_1 = 0;
  int J5_LO_1 = 0;
  int J6_LO_1 = 0;
  int J7_LO_1 = 0;
  int J8_LO_1 = 0;
  int J9_LO_1 = 0;

  int J1_LO_2 = 0;
  int J2_LO_2 = 0;
  int J3_LO_2 = 0;
  int J4_LO_2 = 0;
  int J5_LO_2 = 0;
  int J6_LO_2 = 0;
  int J7_LO_2 = 0;
  int J8_LO_2 = 0;
  int J9_LO_2 = 0;

  //reset
  int J1cur = 0;
  int J2cur = 0;
  int J3cur = 0;
  int J4cur = 0;
  int J5cur = 0;
  int J6cur = 0;
  int J7cur = 0;
  int J8cur = 0;
  int J9cur = 0;

  int J1_PEcur = 0;
  int J2_PEcur = 0;
  int J3_PEcur = 0;
  int J4_PEcur = 0;
  int J5_PEcur = 0;
  int J6_PEcur = 0;
  int J7_PEcur = 0;
  int J8_PEcur = 0;
  int J9_PEcur = 0;

  int J1_SE_1cur = 0;
  int J2_SE_1cur = 0;
  int J3_SE_1cur = 0;
  int J4_SE_1cur = 0;
  int J5_SE_1cur = 0;
  int J6_SE_1cur = 0;
  int J7_SE_1cur = 0;
  int J8_SE_1cur = 0;
  int J9_SE_1cur = 0;

  int J1_SE_2cur = 0;
  int J2_SE_2cur = 0;
  int J3_SE_2cur = 0;
  int J4_SE_2cur = 0;
  int J5_SE_2cur = 0;
  int J6_SE_2cur = 0;
  int J7_SE_2cur = 0;
  int J8_SE_2cur = 0;
  int J9_SE_2cur = 0;

  int highStepCur = 0;

  float speedSP;
  float moveDist;

  //process lookahead
  if (splineTrue == true) {
    processSerial();
  }

  //SET DIRECTIONS

  /// J1 ///
  //flip J1 direction
  if (J1dir) {
    digitalWrite(J1dirPin, LOW);
  }
  else {
    digitalWrite(J1dirPin, HIGH);
  }
  /// J2 ///
  if (J2dir) {
    digitalWrite(J2dirPin, LOW);
  }
  else {
    digitalWrite(J2dirPin, HIGH);
  }
  /// J3 ///
  if (J3dir) {
    digitalWrite(J3dirPin, LOW);
  }
  else {
    digitalWrite(J3dirPin, HIGH);
  }
  /// J4 ///
  if (J4dir) {
    digitalWrite(J4dirPin, HIGH);
  }
  else {
    digitalWrite(J4dirPin, LOW);
  }
  /// J5 ///
  if (J5dir) {
    digitalWrite(J5dirPin, LOW);
  }
  else {
    digitalWrite(J5dirPin, HIGH);
  }
  /// J6 ///
  if (J6dir) {
    digitalWrite(J6dirPin, LOW);
  }
  else {
    digitalWrite(J6dirPin, HIGH);
  }

  /// J7 ///
  if (J7dir) {
    digitalWrite(J7dirPin, HIGH);
  }
  else {
    digitalWrite(J7dirPin, LOW);
  }

  /// J8 ///
  if (J8dir) {
    digitalWrite(J8dirPin, HIGH);
  }
  else {
    digitalWrite(J8dirPin, LOW);
  }

  /// J9 ///
  if (J9dir) {
    digitalWrite(J9dirPin, HIGH);
  }
  else {
    digitalWrite(J9dirPin, LOW);
  }


  J1collisionTrue = 0;
  J2collisionTrue = 0;
  J3collisionTrue = 0;
  J4collisionTrue = 0;
  J5collisionTrue = 0;
  J6collisionTrue = 0;


  // Soft E-stop decel support
  bool estopBraking = false;
  unsigned long estopBrakeStart = 0;
///// DRIVE MOTORS /////
  while (J1cur < J1step || J2cur < J2step || J3cur < J3step || J4cur < J4step || J5cur < J5step || J6cur < J6step || J7cur < J7step || J8cur < J8step || J9cur < J9step)
  {
    // Soft E-stop: ramp down step rate briefly, then stop stepping (hold torque)
    if (estop_latched) {
      if (!estopBraking) {
        estopBraking = true;
        estopBrakeStart = micros();
      }
      // Increase delay to reduce speed smoothly
      curDelay = (curDelay * 1.08f) + 5.0f;
      // Stop stepping after a short braking window
      if ((micros() - estopBrakeStart) > 200000UL || curDelay > 60000.0f) {
        return;
      }
    }
    float distDelay = 60;
    float disDelayCur = 0;

    //process lookahead
    if (splineTrue == true) {
      processSerial();
    }

    /////// J1 ////////////////////////////////
    ///find pulse every
    if (J1cur < J1step)
    {
      J1_PE = (HighStep / J1step);
      ///find left over 1
      J1_LO_1 = (HighStep - (J1step * J1_PE));
      ///find skip 1
      if (J1_LO_1 > 0)
      {
        J1_SE_1 = (HighStep / J1_LO_1);
      }
      else
      {
        J1_SE_1 = 0;
      }
      ///find left over 2
      if (J1_SE_1 > 0)
      {
        J1_LO_2 = HighStep - ((J1step * J1_PE) + ((J1step * J1_PE) / J1_SE_1));
      }
      else
      {
        J1_LO_2 = 0;
      }
      ///find skip 2
      if (J1_LO_2 > 0)
      {
        J1_SE_2 = (HighStep / J1_LO_2);
      }
      else
      {
        J1_SE_2 = 0;
      }
      /////////  J1  ///////////////
      if (J1_SE_2 == 0)
      {
        J1_SE_2cur = (J1_SE_2 + 1);
      }
      if (J1_SE_2cur != J1_SE_2)
      {
        J1_SE_2cur = ++J1_SE_2cur;
        if (J1_SE_1 == 0)
        {
          J1_SE_1cur = (J1_SE_1 + 1);
        }
        if (J1_SE_1cur != J1_SE_1)
        {
          J1_SE_1cur = ++J1_SE_1cur;
          J1_PEcur = ++J1_PEcur;
          if (J1_PEcur == J1_PE)
          {
            J1cur = ++J1cur;
            J1_PEcur = 0;
            digitalWrite(J1stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J1dir == 0) {
              J1StepM == --J1StepM;
            }
            else {
              J1StepM == ++J1StepM;
            }
          }
        }
        else
        {
          J1_SE_1cur = 0;
        }
      }
      else
      {
        J1_SE_2cur = 0;
      }
    }


    /////// J2 ////////////////////////////////

    ///find pulse every
    if (J2cur < J2step)
    {
      J2_PE = (HighStep / J2step);
      ///find left over 1
      J2_LO_1 = (HighStep - (J2step * J2_PE));
      ///find skip 1
      if (J2_LO_1 > 0)
      {
        J2_SE_1 = (HighStep / J2_LO_1);
      }
      else
      {
        J2_SE_1 = 0;
      }
      ///find left over 2
      if (J2_SE_1 > 0)
      {
        J2_LO_2 = HighStep - ((J2step * J2_PE) + ((J2step * J2_PE) / J2_SE_1));
      }
      else
      {
        J2_LO_2 = 0;
      }
      ///find skip 2
      if (J2_LO_2 > 0)
      {
        J2_SE_2 = (HighStep / J2_LO_2);
      }
      else
      {
        J2_SE_2 = 0;
      }
      /////////  J2  ///////////////
      if (J2_SE_2 == 0)
      {
        J2_SE_2cur = (J2_SE_2 + 1);
      }
      if (J2_SE_2cur != J2_SE_2)
      {
        J2_SE_2cur = ++J2_SE_2cur;
        if (J2_SE_1 == 0)
        {
          J2_SE_1cur = (J2_SE_1 + 1);
        }
        if (J2_SE_1cur != J2_SE_1)
        {
          J2_SE_1cur = ++J2_SE_1cur;
          J2_PEcur = ++J2_PEcur;
          if (J2_PEcur == J2_PE)
          {
            J2cur = ++J2cur;
            J2_PEcur = 0;
            digitalWrite(J2stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J2dir == 0) {
              J2StepM == --J2StepM;
            }
            else {
              J2StepM == ++J2StepM;
            }
          }
        }
        else
        {
          J2_SE_1cur = 0;
        }
      }
      else
      {
        J2_SE_2cur = 0;
      }
    }

    /////// J3 ////////////////////////////////
    ///find pulse every
    if (J3cur < J3step)
    {
      J3_PE = (HighStep / J3step);
      ///find left over 1
      J3_LO_1 = (HighStep - (J3step * J3_PE));
      ///find skip 1
      if (J3_LO_1 > 0)
      {
        J3_SE_1 = (HighStep / J3_LO_1);
      }
      else
      {
        J3_SE_1 = 0;
      }
      ///find left over 2
      if (J3_SE_1 > 0)
      {
        J3_LO_2 = HighStep - ((J3step * J3_PE) + ((J3step * J3_PE) / J3_SE_1));
      }
      else
      {
        J3_LO_2 = 0;
      }
      ///find skip 2
      if (J3_LO_2 > 0)
      {
        J3_SE_2 = (HighStep / J3_LO_2);
      }
      else
      {
        J3_SE_2 = 0;
      }
      /////////  J3  ///////////////
      if (J3_SE_2 == 0)
      {
        J3_SE_2cur = (J3_SE_2 + 1);
      }
      if (J3_SE_2cur != J3_SE_2)
      {
        J3_SE_2cur = ++J3_SE_2cur;
        if (J3_SE_1 == 0)
        {
          J3_SE_1cur = (J3_SE_1 + 1);
        }
        if (J3_SE_1cur != J3_SE_1)
        {
          J3_SE_1cur = ++J3_SE_1cur;
          J3_PEcur = ++J3_PEcur;
          if (J3_PEcur == J3_PE)
          {
            J3cur = ++J3cur;
            J3_PEcur = 0;
            digitalWrite(J3stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J3dir == 0) {
              J3StepM == --J3StepM;
            }
            else {
              J3StepM == ++J3StepM;
            }
          }
        }
        else
        {
          J3_SE_1cur = 0;
        }
      }
      else
      {
        J3_SE_2cur = 0;
      }
    }


    /////// J4 ////////////////////////////////
    ///find pulse every
    if (J4cur < J4step)
    {
      J4_PE = (HighStep / J4step);
      ///find left over 1
      J4_LO_1 = (HighStep - (J4step * J4_PE));
      ///find skip 1
      if (J4_LO_1 > 0)
      {
        J4_SE_1 = (HighStep / J4_LO_1);
      }
      else
      {
        J4_SE_1 = 0;
      }
      ///find left over 2
      if (J4_SE_1 > 0)
      {
        J4_LO_2 = HighStep - ((J4step * J4_PE) + ((J4step * J4_PE) / J4_SE_1));
      }
      else
      {
        J4_LO_2 = 0;
      }
      ///find skip 2
      if (J4_LO_2 > 0)
      {
        J4_SE_2 = (HighStep / J4_LO_2);
      }
      else
      {
        J4_SE_2 = 0;
      }
      /////////  J4  ///////////////
      if (J4_SE_2 == 0)
      {
        J4_SE_2cur = (J4_SE_2 + 1);
      }
      if (J4_SE_2cur != J4_SE_2)
      {
        J4_SE_2cur = ++J4_SE_2cur;
        if (J4_SE_1 == 0)
        {
          J4_SE_1cur = (J4_SE_1 + 1);
        }
        if (J4_SE_1cur != J4_SE_1)
        {
          J4_SE_1cur = ++J4_SE_1cur;
          J4_PEcur = ++J4_PEcur;
          if (J4_PEcur == J4_PE)
          {
            J4cur = ++J4cur;
            J4_PEcur = 0;
            digitalWrite(J4stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J4dir == 0) {
              J4StepM == --J4StepM;
            }
            else {
              J4StepM == ++J4StepM;
            }
          }
        }
        else
        {
          J4_SE_1cur = 0;
        }
      }
      else
      {
        J4_SE_2cur = 0;
      }
    }


    /////// J5 ////////////////////////////////
    ///find pulse every
    if (J5cur < J5step)
    {
      J5_PE = (HighStep / J5step);
      ///find left over 1
      J5_LO_1 = (HighStep - (J5step * J5_PE));
      ///find skip 1
      if (J5_LO_1 > 0)
      {
        J5_SE_1 = (HighStep / J5_LO_1);
      }
      else
      {
        J5_SE_1 = 0;
      }
      ///find left over 2
      if (J5_SE_1 > 0)
      {
        J5_LO_2 = HighStep - ((J5step * J5_PE) + ((J5step * J5_PE) / J5_SE_1));
      }
      else
      {
        J5_LO_2 = 0;
      }
      ///find skip 2
      if (J5_LO_2 > 0)
      {
        J5_SE_2 = (HighStep / J5_LO_2);
      }
      else
      {
        J5_SE_2 = 0;
      }
      /////////  J5  ///////////////
      if (J5_SE_2 == 0)
      {
        J5_SE_2cur = (J5_SE_2 + 1);
      }
      if (J5_SE_2cur != J5_SE_2)
      {
        J5_SE_2cur = ++J5_SE_2cur;
        if (J5_SE_1 == 0)
        {
          J5_SE_1cur = (J5_SE_1 + 1);
        }
        if (J5_SE_1cur != J5_SE_1)
        {
          J5_SE_1cur = ++J5_SE_1cur;
          J5_PEcur = ++J5_PEcur;
          if (J5_PEcur == J5_PE)
          {
            J5cur = ++J5cur;
            J5_PEcur = 0;
            digitalWrite(J5stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J5dir == 0) {
              J5StepM == --J5StepM;
            }
            else {
              J5StepM == ++J5StepM;
            }
          }
        }
        else
        {
          J5_SE_1cur = 0;
        }
      }
      else
      {
        J5_SE_2cur = 0;
      }
    }


    /////// J6 ////////////////////////////////
    ///find pulse every
    if (J6cur < J6step)
    {
      J6_PE = (HighStep / J6step);
      ///find left over 1
      J6_LO_1 = (HighStep - (J6step * J6_PE));
      ///find skip 1
      if (J6_LO_1 > 0)
      {
        J6_SE_1 = (HighStep / J6_LO_1);
      }
      else
      {
        J6_SE_1 = 0;
      }
      ///find left over 2
      if (J6_SE_1 > 0)
      {
        J6_LO_2 = HighStep - ((J6step * J6_PE) + ((J6step * J6_PE) / J6_SE_1));
      }
      else
      {
        J6_LO_2 = 0;
      }
      ///find skip 2
      if (J6_LO_2 > 0)
      {
        J6_SE_2 = (HighStep / J6_LO_2);
      }
      else
      {
        J6_SE_2 = 0;
      }
      /////////  J6  ///////////////
      if (J6_SE_2 == 0)
      {
        J6_SE_2cur = (J6_SE_2 + 1);
      }
      if (J6_SE_2cur != J6_SE_2)
      {
        J6_SE_2cur = ++J6_SE_2cur;
        if (J6_SE_1 == 0)
        {
          J6_SE_1cur = (J6_SE_1 + 1);
        }
        if (J6_SE_1cur != J6_SE_1)
        {
          J6_SE_1cur = ++J6_SE_1cur;
          J6_PEcur = ++J6_PEcur;
          if (J6_PEcur == J6_PE)
          {
            J6cur = ++J6cur;
            J6_PEcur = 0;
            digitalWrite(J6stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J6dir == 0) {
              J6StepM == --J6StepM;
            }
            else {
              J6StepM == ++J6StepM;
            }
          }
        }
        else
        {
          J6_SE_1cur = 0;
        }
      }
      else
      {
        J6_SE_2cur = 0;
      }
    }


    /////// J7 ////////////////////////////////
    ///find pulse every
    if (J7cur < J7step)
    {
      J7_PE = (HighStep / J7step);
      ///find left over 1
      J7_LO_1 = (HighStep - (J7step * J7_PE));
      ///find skip 1
      if (J7_LO_1 > 0)
      {
        J7_SE_1 = (HighStep / J7_LO_1);
      }
      else
      {
        J7_SE_1 = 0;
      }
      ///find left over 2
      if (J7_SE_1 > 0)
      {
        J7_LO_2 = HighStep - ((J7step * J7_PE) + ((J7step * J7_PE) / J7_SE_1));
      }
      else
      {
        J7_LO_2 = 0;
      }
      ///find skip 2
      if (J7_LO_2 > 0)
      {
        J7_SE_2 = (HighStep / J7_LO_2);
      }
      else
      {
        J7_SE_2 = 0;
      }
      /////////  J7  ///////////////
      if (J7_SE_2 == 0)
      {
        J7_SE_2cur = (J7_SE_2 + 1);
      }
      if (J7_SE_2cur != J7_SE_2)
      {
        J7_SE_2cur = ++J7_SE_2cur;
        if (J7_SE_1 == 0)
        {
          J7_SE_1cur = (J7_SE_1 + 1);
        }
        if (J7_SE_1cur != J7_SE_1)
        {
          J7_SE_1cur = ++J7_SE_1cur;
          J7_PEcur = ++J7_PEcur;
          if (J7_PEcur == J7_PE)
          {
            J7cur = ++J7cur;
            J7_PEcur = 0;
            digitalWrite(J7stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J7dir == 0) {
              J7StepM == --J7StepM;
            }
            else {
              J7StepM == ++J7StepM;
            }
          }
        }
        else
        {
          J7_SE_1cur = 0;
        }
      }
      else
      {
        J7_SE_2cur = 0;
      }
    }


    /////// J8 ////////////////////////////////
    ///find pulse every
    if (J8cur < J8step)
    {
      J8_PE = (HighStep / J8step);
      ///find left over 1
      J8_LO_1 = (HighStep - (J8step * J8_PE));
      ///find skip 1
      if (J8_LO_1 > 0)
      {
        J8_SE_1 = (HighStep / J8_LO_1);
      }
      else
      {
        J8_SE_1 = 0;
      }
      ///find left over 2
      if (J8_SE_1 > 0)
      {
        J8_LO_2 = HighStep - ((J8step * J8_PE) + ((J8step * J8_PE) / J8_SE_1));
      }
      else
      {
        J8_LO_2 = 0;
      }
      ///find skip 2
      if (J8_LO_2 > 0)
      {
        J8_SE_2 = (HighStep / J8_LO_2);
      }
      else
      {
        J8_SE_2 = 0;
      }
      /////////  J8  ///////////////
      if (J8_SE_2 == 0)
      {
        J8_SE_2cur = (J8_SE_2 + 1);
      }
      if (J8_SE_2cur != J8_SE_2)
      {
        J8_SE_2cur = ++J8_SE_2cur;
        if (J8_SE_1 == 0)
        {
          J8_SE_1cur = (J8_SE_1 + 1);
        }
        if (J8_SE_1cur != J8_SE_1)
        {
          J8_SE_1cur = ++J8_SE_1cur;
          J8_PEcur = ++J8_PEcur;
          if (J8_PEcur == J8_PE)
          {
            J8cur = ++J8cur;
            J8_PEcur = 0;
            digitalWrite(J8stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J8dir == 0) {
              J8StepM == --J8StepM;
            }
            else {
              J8StepM == ++J8StepM;
            }
          }
        }
        else
        {
          J8_SE_1cur = 0;
        }
      }
      else
      {
        J8_SE_2cur = 0;
      }
    }


    /////// J9 ////////////////////////////////
    ///find pulse every
    if (J9cur < J9step)
    {
      J9_PE = (HighStep / J9step);
      ///find left over 1
      J9_LO_1 = (HighStep - (J9step * J9_PE));
      ///find skip 1
      if (J9_LO_1 > 0)
      {
        J9_SE_1 = (HighStep / J9_LO_1);
      }
      else
      {
        J9_SE_1 = 0;
      }
      ///find left over 2
      if (J9_SE_1 > 0)
      {
        J9_LO_2 = HighStep - ((J9step * J9_PE) + ((J9step * J9_PE) / J9_SE_1));
      }
      else
      {
        J9_LO_2 = 0;
      }
      ///find skip 2
      if (J9_LO_2 > 0)
      {
        J9_SE_2 = (HighStep / J9_LO_2);
      }
      else
      {
        J9_SE_2 = 0;
      }
      /////////  J9  ///////////////
      if (J9_SE_2 == 0)
      {
        J9_SE_2cur = (J9_SE_2 + 1);
      }
      if (J9_SE_2cur != J9_SE_2)
      {
        J9_SE_2cur = ++J9_SE_2cur;
        if (J9_SE_1 == 0)
        {
          J9_SE_1cur = (J9_SE_1 + 1);
        }
        if (J9_SE_1cur != J9_SE_1)
        {
          J9_SE_1cur = ++J9_SE_1cur;
          J9_PEcur = ++J9_PEcur;
          if (J9_PEcur == J9_PE)
          {
            J9cur = ++J9cur;
            J9_PEcur = 0;
            digitalWrite(J9stepPin, LOW);
            delayMicroseconds(distDelay);
            disDelayCur = disDelayCur + distDelay;
            if (J9dir == 0) {
              J9StepM == --J9StepM;
            }
            else {
              J9StepM == ++J9StepM;
            }
          }
        }
        else
        {
          J9_SE_1cur = 0;
        }
      }
      else
      {
        J9_SE_2cur = 0;
      }
    }




    // inc cur step
    highStepCur = ++highStepCur;
    digitalWrite(J1stepPin, HIGH);
    digitalWrite(J2stepPin, HIGH);
    digitalWrite(J3stepPin, HIGH);
    digitalWrite(J4stepPin, HIGH);
    digitalWrite(J5stepPin, HIGH);
    digitalWrite(J6stepPin, HIGH);
    digitalWrite(J7stepPin, HIGH);
    digitalWrite(J8stepPin, HIGH);
    digitalWrite(J9stepPin, HIGH);
    delayMicroseconds(curDelay - disDelayCur);

  }
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//READ DATA
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void processSerial() {
  if (Serial.available() > 0 and cmdBuffer3 == "") {
    char recieved = Serial.read();
    recData += recieved;
    // Process message when new line character is recieved
    if (recieved == '\n') {
      //place data in last position
      cmdBuffer3 = recData;
      //determine if move command
      recData.trim();
      String procCMDtype = recData.substring(0, 2);
      if (procCMDtype == "SS") {
        splineTrue = false;
        splineEndReceived = true;
      }
      if (splineTrue == true) {
        if (moveSequence == "") {
          moveSequence = "firsMoveActive";
        }
        //close serial so next command can be read in
        if (Alarm == "0") {
          sendRobotPosSpline();
        }
        else {
          Serial.println(Alarm);
          Alarm = "0";
        }
      }

      recData = ""; // Clear recieved buffer

      shiftCMDarray();


      //if second position is empty and first move command read in process second move ahead of time
      if (procCMDtype == "ML" and moveSequence == "firsMoveActive" and cmdBuffer2 == "" and cmdBuffer1 != "" and splineTrue == true) {
        moveSequence = "secondMoveProcessed";
        while (cmdBuffer2 == "") {
          if (Serial.available() > 0) {
            char recieved = Serial.read();
            recData += recieved;
            if (recieved == '\n') {
              cmdBuffer2 = recData;
              recData.trim();
              procCMDtype = recData.substring(0, 2);
              if (procCMDtype == "ML") {
                //close serial so next command can be read in
                delay(5);
                if (Alarm == "0") {
                  sendRobotPosSpline();
                }
                else {
                  Serial.println(Alarm);
                  Alarm = "0";
                }
              }
              recData = ""; // Clear recieved buffer
            }
          }
        }
      }
    }
  }
}


void shiftCMDarray() {
  if (cmdBuffer1 == "") {
    //shift 2 to 1
    cmdBuffer1 = cmdBuffer2;
    cmdBuffer2 = "";
  }
  if (cmdBuffer2 == "") {
    //shift 3 to 2
    cmdBuffer2 = cmdBuffer3;
    cmdBuffer3 = "";
  }
  if (cmdBuffer1 == "") {
    //shift 2 to 1
    cmdBuffer1 = cmdBuffer2;
    cmdBuffer2 = "";
  }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//MAIN
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Soft E-stop interrupt: set request flag (do minimal work in ISR)
void estopISR() {
  estop_latched = true; // latch immediately even if we're inside a blocking move
}

void setup() {
  // run once:
  // faster speed to reduce Serial latency
  Serial.begin(115200);

  // Soft E-stop input (firmware hold-stop)
  pinMode(ESTOP_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), estopISR, RISING);



  pinMode(J1stepPin, OUTPUT);
  pinMode(J1dirPin, OUTPUT);
  pinMode(J2stepPin, OUTPUT);
  pinMode(J2dirPin, OUTPUT);
  pinMode(J3stepPin, OUTPUT);
  pinMode(J3dirPin, OUTPUT);
  pinMode(J4stepPin, OUTPUT);
  pinMode(J4dirPin, OUTPUT);
  pinMode(J5stepPin, OUTPUT);
  pinMode(J5dirPin, OUTPUT);
  pinMode(J6stepPin, OUTPUT);
  pinMode(J6dirPin, OUTPUT);
  pinMode(J7stepPin, OUTPUT);
  pinMode(J7dirPin, OUTPUT);
  pinMode(J8stepPin, OUTPUT);
  pinMode(J8dirPin, OUTPUT);
  pinMode(J9stepPin, OUTPUT);
  pinMode(J9dirPin, OUTPUT);

  pinMode(J1calPin, INPUT);
  pinMode(J2calPin, INPUT);
  pinMode(J3calPin, INPUT);
  pinMode(J4calPin, INPUT);
  pinMode(J5calPin, INPUT);
  pinMode(J6calPin, INPUT);
  pinMode(J7calPin, INPUT);
  pinMode(J8calPin, INPUT);
  pinMode(J9calPin, INPUT);


  //pinMode(Input39, INPUT_PULLUP);


  pinMode(Output40, OUTPUT);
  pinMode(Output41, OUTPUT);



  digitalWrite(J1stepPin, HIGH);
  digitalWrite(J2stepPin, HIGH);
  digitalWrite(J3stepPin, HIGH);
  digitalWrite(J4stepPin, HIGH);
  digitalWrite(J5stepPin, HIGH);
  digitalWrite(J6stepPin, HIGH);
  digitalWrite(J7stepPin, HIGH);
  digitalWrite(J8stepPin, HIGH);
  digitalWrite(J9stepPin, HIGH);

  //clear command buffer array
  cmdBuffer1 = "";
  cmdBuffer2 = "";
  cmdBuffer3 = "";
  //reset move command flag
  moveSequence = "";
  flag = "";
  rndTrue = false;
  splineTrue = false;
  splineEndReceived = false;
}


void loop() {

  ////////////////////////////////////
  ///////////start loop///////////////

// ---------------- Soft E-stop handling ----------------
// Fail-safe: with NC->GND + INPUT_PULLUP, a pressed E-stop (or broken wire) reads HIGH.
if (digitalRead(ESTOP_PIN) == HIGH) {
  estop_latched = true;
}

// One-shot actions when E-stop first latches (clear buffers, report not-busy, etc.)
if (estop_latched && !estop_handled) {
  estop_handled = true;
  robotBusy = false; // make sure UI/host sees we're stopped

  // Clear any buffered commands to prevent "resume" surprises
  cmdBuffer1 = "";
  cmdBuffer2 = "";
  cmdBuffer3 = "";
  recData = "";
  inData = "";

  Serial.println("!! ESTOP LATCHED (soft hold-stop) !!");
}

// If E-stop is active, only allow the ER command to clear it.
// We still call processSerial() so the host can send ER.
if (estop_latched) {
  processSerial();

  if (cmdBuffer1 != "") {
    String tmp = cmdBuffer1;
    tmp.trim();
    String fn = tmp.substring(0, 2);

    if (fn == "ER") {
      // Clear E-stop latch (and resync internal position from encoders)
      // This prevents "lost step" drift after an abrupt stop.
      robotBusy = false;
      //estop_request = false;

      // Stop any residual motion intent
      speedViolation = "0";
      flag = "";
      // Ensure step/angle state matches the encoders before resuming
      syncRobotPosFromEncoders();

      estop_latched = false;


      estop_handled = false;
      // Clear buffers
      cmdBuffer1 = "";
      cmdBuffer2 = "";
      cmdBuffer3 = "";
      recData = "";
      inData = "";

      Serial.println("ESTOP CLEARED (SYNCED)");
    }
    //----- Query Robot Status  ---------------------------------------------------
    //-----------------------------------------------------------------------
    else if (function == "QS")
    {
      // Status query (single-line): keep legacy BUSY/IDLE first, add ESTOP flag for GUI
      Serial.print(robotBusy ? "BUSY" : "IDLE");
      Serial.print(" ESTOP=");
      Serial.println(estop_latched ? "1" : "0");
    }
    else if (function == "QE")
    {
      // Query E-stop state only
      Serial.print("ESTOP=");
      Serial.println(estop_latched ? "1" : "0");
    }
    else
    {
      // Ignore everything else while latched
      cmdBuffer1 = "";
      cmdBuffer2 = "";
      cmdBuffer3 = "";
      recData = "";
      inData = "";
      Serial.println("!! ESTOP ACTIVE - SEND ER TO CLEAR !!");
    }
  }

  // Skip the rest of loop while latched
  return;
}
// ------------------------------------------------------


  if (splineEndReceived == false) {
    processSerial();
  }
  //dont start unless at least one command has been read in
  if (cmdBuffer1 != "") {
    //process data
    inData = cmdBuffer1;
    inData.trim();
    String function = inData.substring(0, 2);
    inData = inData.substring(2);
    KinematicError = 0;

    //-----SPLINE START------------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "SL") {
      splineTrue = true;
      delay(5);
      Serial.print("SL");
      moveSequence = "";
      flag = "";
      rndTrue = false;
      splineEndReceived = false;
    }

    //----- SPLINE STOP  ----------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "SS") {
      delay(5);
      sendRobotPos();
      splineTrue = false;
      splineEndReceived = false;
    }

    //-----COMMAND TO CLOSE---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "CL")
    {
      delay(5);
      Serial.end();
    }

    //-----COMMAND TEST LIMIT SWITCHES---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "TL") {

      String J1calTest = "0";
      String J2calTest = "0";
      String J3calTest = "0";
      String J4calTest = "0";
      String J5calTest = "0";
      String J6calTest = "0";

      if (digitalRead(J1calPin) == HIGH) {
        J1calTest = "1";
      }
      if (digitalRead(J2calPin) == HIGH) {
        J2calTest = "1";
      }
      if (digitalRead(J3calPin) == HIGH) {
        J3calTest = "1";
      }
      if (digitalRead(J4calPin) == HIGH) {
        J4calTest = "1";
      }
      if (digitalRead(J5calPin) == HIGH) {
        J5calTest = "1";
      }
      if (digitalRead(J6calPin) == HIGH) {
        J6calTest = "1";
      }
      String TestLim = " J1 = " + J1calTest + "   J2 = " + J2calTest + "   J3 = " + J3calTest + "   J4 = " + J4calTest + "   J5 = " + J5calTest + "   J6 = " + J6calTest;
      delay(5);
      Serial.println(TestLim);
    }


    //-----COMMAND SET ENCODERS TO 1000---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "SE")
    {
      J1encPos.write(1000);
      J2encPos.write(1000);
      J3encPos.write(1000);
      J4encPos.write(1000);
      J5encPos.write(1000);
      J6encPos.write(1000);
      delay(5);
      Serial.print("Done");
    }

    //-----COMMAND READ ENCODERS---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "RE")
    {
      J1EncSteps = J1encPos.read();
      J2EncSteps = J2encPos.read();
      J3EncSteps = J3encPos.read();
      J4EncSteps = J4encPos.read();
      J5EncSteps = J5encPos.read();
      J6EncSteps = J6encPos.read();
      String Read = " J1 = " + String(J1EncSteps) + "   J2 = " + String(J2EncSteps) + "   J3 = " + String(J3EncSteps) + "   J4 = " + String(J4EncSteps) + "   J5 = " + String(J5EncSteps) + "   J6 = " + String(J6EncSteps);
      delay(5);
      Serial.println(Read);
    }

    //-----COMMAND REQUEST POSITION---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "RP")
    {
      //close serial so next command can be read in
      //delay(5);
      if (Alarm == "0") {
        sendRobotPos();
      }
      else {
        Serial.println(Alarm);
        Alarm = "0";
      }
    }



    //-----COMMAND HOME POSITION---------------------------------------------------
    //-----------------------------------------------------------------------

    //For debugging
    if (function == "HM")
    {

      int J1dir;
      int J2dir;
      int J3dir;
      int J4dir;
      int J5dir;
      int J6dir;
      int J7dir;
      int J8dir;
      int J9dir;


      String SpeedType = "p";
      float SpeedVal = 25.0;
      float ACCspd = 10.0;
      float DCCspd = 10.0;
      float ACCramp = 20.0;

      JangleIn[0] = 0.00;
      JangleIn[1] = 0.00;
      JangleIn[2] = 0.00;
      JangleIn[3] = 0.00;
      JangleIn[4] = 0.00;
      JangleIn[5] = 0.00;


      //calc destination motor steps
      int J1futStepM = J1axisLimNeg * J1StepDeg;
      int J2futStepM = J2axisLimNeg * J2StepDeg;
      int J3futStepM = J3axisLimNeg * J3StepDeg;
      int J4futStepM = J4axisLimNeg * J4StepDeg;
      int J5futStepM = J5axisLimNeg * J5StepDeg;
      int J6futStepM = J6axisLimNeg * J6StepDeg;

      //calc delta from current to destination
      int J1stepDif = J1StepM - J1futStepM;
      int J2stepDif = J2StepM - J2futStepM;
      int J3stepDif = J3StepM - J3futStepM;
      int J4stepDif = J4StepM - J4futStepM;
      int J5stepDif = J5StepM - J5futStepM;
      int J6stepDif = J6StepM - J6futStepM;
      int J7stepDif = 0;
      int J8stepDif = 0;
      int J9stepDif = 0;

      //determine motor directions
      if (J1stepDif <= 0) {
        J1dir = 1;
      }
      else {
        J1dir = 0;
      }

      if (J2stepDif <= 0) {
        J2dir = 1;
      }
      else {
        J2dir = 0;
      }

      if (J3stepDif <= 0) {
        J3dir = 1;
      }
      else {
        J3dir = 0;
      }

      if (J4stepDif <= 0) {
        J4dir = 1;
      }
      else {
        J4dir = 0;
      }

      if (J5stepDif <= 0) {
        J5dir = 1;
      }
      else {
        J5dir = 0;
      }

      if (J6stepDif <= 0) {
        J6dir = 1;
      }
      else {
        J6dir = 0;
      }

      J7dir = 0;
      J8dir = 0;
      J9dir = 0;



      resetEncoders();

      driveMotorsJ(abs(J1stepDif), abs(J2stepDif), abs(J3stepDif), abs(J4stepDif), abs(J5stepDif), abs(J6stepDif), abs(J7stepDif), abs(J8stepDif), abs(J9stepDif), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
      checkEncoders();
      sendRobotPos();
      delay(5);
      Serial.println("Done");
    }


    //-----COMMAND CORRECT POSITION---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "CP")
    {
      correctRobotPos();
    }

    //-----COMMAND CALIBRATE EXTERNAL AXIS---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "CE")
    {
      int J7lengthStart = inData.indexOf('A');
      int J7rotStart = inData.indexOf('B');
      int J7stepsStart = inData.indexOf('C');
      int J8lengthStart = inData.indexOf('D');
      int J8rotStart = inData.indexOf('E');
      int J8stepsStart = inData.indexOf('F');
      int J9lengthStart = inData.indexOf('G');
      int J9rotStart = inData.indexOf('H');
      int J9stepsStart = inData.indexOf('I');

      J7length = inData.substring(J7lengthStart + 1, J7rotStart).toFloat();
      J7rot = inData.substring(J7rotStart + 1, J7stepsStart).toFloat();
      J7steps = inData.substring(J7stepsStart + 1, J8lengthStart).toFloat();

      J8length = inData.substring(J8lengthStart + 1, J8rotStart).toFloat();
      J8rot = inData.substring(J8rotStart + 1, J8stepsStart).toFloat();
      J8steps = inData.substring(J8stepsStart + 1, J9lengthStart).toFloat();

      J9length = inData.substring(J9lengthStart + 1, J9rotStart).toFloat();
      J9rot = inData.substring(J9rotStart + 1, J9stepsStart).toFloat();
      J9steps = inData.substring(J9stepsStart + 1).toFloat();

      J7axisLimNeg = 0;
      J7axisLimPos = J7length;
      J7axisLim = J7axisLimPos + J7axisLimNeg;
      J7StepDeg = J7steps / J7rot;
      J7StepLim = J7axisLim * J7StepDeg;

      J8axisLimNeg = 0;
      J8axisLimPos = J8length;
      J8axisLim = J8axisLimPos + J8axisLimNeg;
      J8StepDeg = J8steps / J8rot;
      J8StepLim = J8axisLim * J8StepDeg;

      J9axisLimNeg = 0;
      J9axisLimPos = J9length;
      J9axisLim = J9axisLimPos + J9axisLimNeg;
      J9StepDeg = J9steps / J9rot;
      J9StepLim = J9axisLim * J9StepDeg;

      delay(5);
      Serial.print("Done");
    }

    //-----COMMAND ZERO J7---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "Z7")
    {
      J7StepM = 0;
      sendRobotPos();
    }

    //-----COMMAND ZERO J8---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "Z8")
    {
      J8StepM = 0;
      sendRobotPos();
    }

    //-----COMMAND ZERO J9---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "Z9")
    {
      J9StepM = 0;
      sendRobotPos();
    }





    //-----COMMAND TO WAIT TIME---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "WT")
    {
      int WTstart = inData.indexOf('S');
      float WaitTime = inData.substring(WTstart + 1).toFloat();
      int WaitTimeMS = WaitTime * 1000;
      delay(WaitTimeMS);
      Serial.println("WTdone");
    }

    //-----COMMAND IF INPUT THEN JUMP---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "JF")
    {
      int IJstart = inData.indexOf('X');
      int IJTabstart = inData.indexOf('T');
      int IJInputNum = inData.substring(IJstart + 1, IJTabstart).toInt();
      if (digitalRead(IJInputNum) == HIGH)
      {
        delay(5);
        Serial.println("T");
      }
      if (digitalRead(IJInputNum) == LOW)
      {
        delay(5);
        Serial.println("F");
      }
    }
    //-----COMMAND SET OUTPUT ON---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "ON")
    {
      int ONstart = inData.indexOf('X');
      int outputNum = inData.substring(ONstart + 1).toInt();
      digitalWrite(outputNum, HIGH);
      delay(5);
      Serial.println("Done");
    }
    //-----COMMAND SET OUTPUT OFF---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "OF")
    {
      int ONstart = inData.indexOf('X');
      int outputNum = inData.substring(ONstart + 1).toInt();
      digitalWrite(outputNum, LOW);
      delay(5);
      Serial.println("Done");
    }
    //-----COMMAND TO WAIT INPUT ON---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "WI")
    {
      int WIstart = inData.indexOf('N');
      int InputNum = inData.substring(WIstart + 1).toInt();
      while (digitalRead(InputNum) == LOW) {
        delay(100);
      }
      delay(5);
      Serial.println("Done");
    }
    //-----COMMAND TO WAIT INPUT OFF---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "WO")
    {
      int WIstart = inData.indexOf('N');
      int InputNum = inData.substring(WIstart + 1).toInt();
      while (digitalRead(InputNum) == HIGH) {
        delay(100);
      }
      delay(5);
      Serial.println("Done");
    }

    //-----COMMAND SEND POSITION---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "SP")
    {
      int J1angStart = inData.indexOf('A');
      int J2angStart = inData.indexOf('B');
      int J3angStart = inData.indexOf('C');
      int J4angStart = inData.indexOf('D');
      int J5angStart = inData.indexOf('E');
      int J6angStart = inData.indexOf('F');
      int J7angStart = inData.indexOf('G');
      int J8angStart = inData.indexOf('H');
      int J9angStart = inData.indexOf('I');
      J1StepM = ((inData.substring(J1angStart + 1, J2angStart).toFloat()) + J1axisLimNeg) * J1StepDeg;
      J2StepM = ((inData.substring(J2angStart + 1, J3angStart).toFloat()) + J2axisLimNeg) * J2StepDeg;
      J3StepM = ((inData.substring(J3angStart + 1, J4angStart).toFloat()) + J3axisLimNeg) * J3StepDeg;
      J4StepM = ((inData.substring(J4angStart + 1, J5angStart).toFloat()) + J4axisLimNeg) * J4StepDeg;
      J5StepM = ((inData.substring(J5angStart + 1, J6angStart).toFloat()) + J5axisLimNeg) * J5StepDeg;
      J6StepM = ((inData.substring(J6angStart + 1, J7angStart).toFloat()) + J6axisLimNeg) * J6StepDeg;
      J7StepM = ((inData.substring(J7angStart + 1, J8angStart).toFloat()) + J7axisLimNeg) * J7StepDeg;
      J8StepM = ((inData.substring(J8angStart + 1, J9angStart).toFloat()) + J8axisLimNeg) * J8StepDeg;
      J9StepM = ((inData.substring(J9angStart + 1).toFloat()) + J9axisLimNeg) * J9StepDeg;
      delay(5);
      Serial.println("Done");
    }


    //-----COMMAND ECHO TEST MESSAGE---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "TM")
    {
      int J1start = inData.indexOf('A');
      int J2start = inData.indexOf('B');
      int J3start = inData.indexOf('C');
      int J4start = inData.indexOf('D');
      int J5start = inData.indexOf('E');
      int J6start = inData.indexOf('F');
      int WristConStart = inData.indexOf('W');
      JangleIn[0] = inData.substring(J1start + 1, J2start).toFloat();
      JangleIn[1] = inData.substring(J2start + 1, J3start).toFloat();
      JangleIn[2] = inData.substring(J3start + 1, J4start).toFloat();
      JangleIn[3] = inData.substring(J4start + 1, J5start).toFloat();
      JangleIn[4] = inData.substring(J5start + 1, J6start).toFloat();
      JangleIn[5] = inData.substring(J6start + 1, WristConStart).toFloat();
      WristCon = inData.substring(WristConStart + 1);
      WristCon.trim();

      //SolveInverseKinematic();

      String echo = "";
      delay(5);
      Serial.println(inData);


    }
    //-----COMMAND TO CALIBRATE---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "LL")
    {
      robotBusy = true;
      Serial.println("ACK");   // or "ACK MJ", "ACK ML" etc.

      int J1start = inData.indexOf('A');
      int J2start = inData.indexOf('B');
      int J3start = inData.indexOf('C');
      int J4start = inData.indexOf('D');
      int J5start = inData.indexOf('E');
      int J6start = inData.indexOf('F');
      int J7start = inData.indexOf('G');
      int J8start = inData.indexOf('H');
      int J9start = inData.indexOf('I');

      int J1calstart = inData.indexOf('J');
      int J2calstart = inData.indexOf('K');
      int J3calstart = inData.indexOf('L');
      int J4calstart = inData.indexOf('M');
      int J5calstart = inData.indexOf('N');
      int J6calstart = inData.indexOf('O');
      int J7calstart = inData.indexOf('P');
      int J8calstart = inData.indexOf('Q');
      int J9calstart = inData.indexOf('R');



      ///
      int J1req = inData.substring(J1start + 1, J2start).toInt();
      int J2req = inData.substring(J2start + 1, J3start).toInt();
      int J3req = inData.substring(J3start + 1, J4start).toInt();
      int J4req = inData.substring(J4start + 1, J5start).toInt();
      int J5req = inData.substring(J5start + 1, J6start).toInt();
      int J6req = inData.substring(J6start + 1, J7start).toInt();
      int J7req = inData.substring(J7start + 1, J8start).toInt();
      int J8req = inData.substring(J8start + 1, J9start).toInt();
      int J9req = inData.substring(J9start + 1, J1calstart).toInt();



      float J1calOff = inData.substring(J1calstart + 1, J2calstart).toFloat();
      float J2calOff = inData.substring(J2calstart + 1, J3calstart).toFloat();
      float J3calOff = inData.substring(J3calstart + 1, J4calstart).toFloat();
      float J4calOff = inData.substring(J4calstart + 1, J5calstart).toFloat();
      float J5calOff = inData.substring(J5calstart + 1, J6calstart).toFloat();
      float J6calOff = inData.substring(J6calstart + 1, J7calstart).toFloat();
      float J7calOff = inData.substring(J7calstart + 1, J8calstart).toFloat();
      float J8calOff = inData.substring(J8calstart + 1, J9calstart).toFloat();
      float J9calOff = inData.substring(J9calstart + 1).toFloat();
      ///
      float SpeedIn;
      ///
      int J1Step = 0;
      int J2Step = 0;
      int J3Step = 0;
      int J4Step = 0;
      int J5Step = 0;
      int J6Step = 0;
      int J7Step = 0;
      int J8Step = 0;
      int J9Step = 0;
      ///
      int J1stepCen = 0;
      int J2stepCen = 0;
      int J3stepCen = 0;
      int J4stepCen = 0;
      int J5stepCen = 0;
      int J6stepCen = 0;
      int J7stepCen = 0;
      int J8stepCen = 0;
      int J9stepCen = 0;
      Alarm = "0";

      //--IF JOINT IS CALLED FOR CALIBRATION PASS ITS STEP LIMIT OTHERWISE PASS 0---
      if (J1req == 1) {
        J1Step = J1StepLim;
      }
      if (J2req == 1) {
        J2Step = J2StepLim;
      }
      if (J3req == 1) {
        J3Step = J3StepLim;
      }
      if (J4req == 1) {
        J4Step = J4StepLim;
      }
      if (J5req == 1) {
        J5Step = J5StepLim;
      }
      if (J6req == 1) {
        J6Step = J6StepLim;
      }
      if (J7req == 1) {
        J7Step = J7StepLim;
      }
      if (J8req == 1) {
        J8Step = J8StepLim;
      }
      if (J9req == 1) {
        J9Step = J9StepLim;
      }

      //--CALL FUNCT TO DRIVE TO LIMITS--
      SpeedIn = 80;
      driveLimit(J1Step, J2Step, J3Step, J4Step, J5Step, J6Step, J7Step, J8Step, J9Step, SpeedIn);
      delay(500);

      //BACKOFF
      digitalWrite(J1dirPin, LOW);
      digitalWrite(J2dirPin, LOW);
      digitalWrite(J3dirPin, HIGH);
      digitalWrite(J4dirPin, HIGH);
      digitalWrite(J5dirPin, LOW);
      digitalWrite(J6dirPin, HIGH);
      digitalWrite(J7dirPin, HIGH);
      digitalWrite(J8dirPin, HIGH);
      digitalWrite(J9dirPin, HIGH);

      int BacOff = 0;
      while (BacOff <= 250)
      {
        if (J1req == 1) {
          digitalWrite(J1stepPin, LOW);
          delayMicroseconds(5);
          digitalWrite(J1stepPin, HIGH);
          delayMicroseconds(5);
        }
        if (J2req == 1) {
          digitalWrite(J2stepPin, LOW);
          delayMicroseconds(5);
          digitalWrite(J2stepPin, HIGH);
          delayMicroseconds(5);
        }
        if (J3req == 1) {
          digitalWrite(J3stepPin, LOW);
          delayMicroseconds(5);
          digitalWrite(J3stepPin, HIGH);
          delayMicroseconds(5);
        }
        if (J4req == 1) {
          digitalWrite(J4stepPin, LOW);
          delayMicroseconds(5);
          digitalWrite(J4stepPin, HIGH);
          delayMicroseconds(5);
        }
        if (J5req == 1) {
          digitalWrite(J5stepPin, LOW);
          delayMicroseconds(5);
          digitalWrite(J5stepPin, HIGH);
          delayMicroseconds(5);
        }
        if (J6req == 1) {
          digitalWrite(J6stepPin, LOW);
          delayMicroseconds(5);
          digitalWrite(J6stepPin, HIGH);
          delayMicroseconds(5);
        }
        if (J7req == 1) {
          digitalWrite(J7stepPin, LOW);
          delayMicroseconds(5);
          digitalWrite(J7stepPin, HIGH);
          delayMicroseconds(5);
        }
        if (J8req == 1) {
          digitalWrite(J8stepPin, LOW);
          delayMicroseconds(5);
          digitalWrite(J8stepPin, HIGH);
          delayMicroseconds(5);
        }
        if (J9req == 1) {
          digitalWrite(J9stepPin, LOW);
          delayMicroseconds(5);
          digitalWrite(J9stepPin, HIGH);
          delayMicroseconds(5);
        }
        BacOff = ++BacOff;
        delayMicroseconds(4000);
      }

      //--CALL FUNCT TO DRIVE BACK TO LIMITS SLOWLY--
      SpeedIn = .02;
      driveLimit(J1Step, J2Step, J3Step, J4Step, J5Step, J6Step, J7Step, J8Step, J9Step, SpeedIn);

      //OVERDRIVE - MAKE SURE LIMIT SWITCH STAYS MADE
      digitalWrite(J1dirPin, HIGH);
      digitalWrite(J2dirPin, HIGH);
      digitalWrite(J3dirPin, LOW);
      digitalWrite(J4dirPin, LOW);
      digitalWrite(J5dirPin, HIGH);
      digitalWrite(J6dirPin, LOW);
      digitalWrite(J7dirPin, HIGH);
      digitalWrite(J8dirPin, HIGH);
      digitalWrite(J9dirPin, HIGH);

      int OvrDrv = 0;
      while (OvrDrv <= 50)
      {
        if (J1req == 1) {
          digitalWrite(J1stepPin, LOW);
          delayMicroseconds(5);
          digitalWrite(J1stepPin, HIGH);
          delayMicroseconds(5);
        }
        if (J2req == 1) {
          digitalWrite(J2stepPin, LOW);
          delayMicroseconds(5);
          digitalWrite(J2stepPin, HIGH);
          delayMicroseconds(5);
        }
        if (J3req == 1) {
          digitalWrite(J3stepPin, LOW);
          delayMicroseconds(5);
          digitalWrite(J3stepPin, HIGH);
          delayMicroseconds(5);
        }
        if (J4req == 1) {
          digitalWrite(J4stepPin, LOW);
          delayMicroseconds(5);
          digitalWrite(J4stepPin, HIGH);
          delayMicroseconds(5);
        }
        if (J5req == 1) {
          digitalWrite(J5stepPin, LOW);
          delayMicroseconds(5);
          digitalWrite(J5stepPin, HIGH);
          delayMicroseconds(5);
        }
        if (J6req == 1) {
          digitalWrite(J6stepPin, LOW);
          delayMicroseconds(5);
          digitalWrite(J6stepPin, HIGH);
          delayMicroseconds(5);
        }
        if (J7req == 1) {
          digitalWrite(J7stepPin, LOW);
          delayMicroseconds(5);
          digitalWrite(J7stepPin, HIGH);
          delayMicroseconds(5);
        }
        if (J8req == 1) {
          digitalWrite(J8stepPin, LOW);
          delayMicroseconds(5);
          digitalWrite(J8stepPin, HIGH);
          delayMicroseconds(5);
        }
        if (J9req == 1) {
          digitalWrite(J9stepPin, LOW);
          delayMicroseconds(5);
          digitalWrite(J9stepPin, HIGH);
          delayMicroseconds(5);
        }
        OvrDrv = ++OvrDrv;
        delayMicroseconds(3000);
      }

      //SEE IF ANY SWITCHES NOT MADE
      delay(500);
      ///
      if (J1req == 1) {
        if (digitalRead(J1calPin) == LOW) {
          Alarm = "1";
        }
      }
      if (J2req == 1) {
        if (digitalRead(J2calPin) == LOW) {
          Alarm = "2";
        }
      }
      if (J3req == 1) {
        if (digitalRead(J3calPin) == LOW) {
          Alarm = "3";
        }
      }
      if (J4req == 1) {
        if (digitalRead(J4calPin) == LOW) {
          Alarm = "4";
        }
      }
      if (J5req == 1) {
        if (digitalRead(J5calPin) == LOW) {
          Alarm = "5";
        }
      }
      if (J6req == 1) {
        if (digitalRead(J6calPin) == LOW) {
          Alarm = "6";
        }
      }
      if (J7req == 1) {
        if (digitalRead(J7calPin) == LOW) {
          Alarm = "7";
        }
      }
      if (J8req == 1) {
        if (digitalRead(J8calPin) == LOW) {
          Alarm = "8";
        }
      }
      if (J9req == 1) {
        if (digitalRead(J9calPin) == LOW) {
          Alarm = "9";
        }
      }
      ///
      if (Alarm == "0") {

        //set master steps and center step
        if (J1req == 1) {
          J1StepM = ((J1axisLim) + J1calBaseOff + J1calOff) * J1StepDeg;
          J1stepCen = ((J1axisLimPos) + J1calBaseOff + J1calOff) * J1StepDeg;
        }
        if (J2req == 1) {
          J2StepM = (0 + J2calBaseOff + J2calOff) * J2StepDeg;
          J2stepCen = ((J2axisLimNeg) - J2calBaseOff - J2calOff) * J2StepDeg;
        }
        if (J3req == 1) {
          J3StepM = ((J3axisLim) + J3calBaseOff + J3calOff) * J3StepDeg;
          J3stepCen = ((J3axisLimPos) + J3calBaseOff + J3calOff) * J3StepDeg;
        }
        if (J4req == 1) {
          J4StepM = (0 + J4calBaseOff + J4calOff) * J4StepDeg;
          J4stepCen = ((J4axisLimNeg) - J4calBaseOff - J4calOff) * J4StepDeg;
        }
        if (J5req == 1) {
          J5StepM = (0 + J5calBaseOff + J5calOff) * J5StepDeg;
          J5stepCen = ((J5axisLimNeg) - J5calBaseOff - J5calOff) * J5StepDeg;
        }
        if (J6req == 1) {
          J6StepM = ((J6axisLim) + J6calBaseOff + J6calOff) * J6StepDeg;
          J6stepCen = ((J6axisLimNeg) + J6calBaseOff + J6calOff) * J6StepDeg;
        }
        if (J7req == 1) {
          J7StepM = (0 + J7calBaseOff + J7calOff) * J7StepDeg;
          J7stepCen = 0;
        }
        if (J8req == 1) {
          J8StepM = (0 + J8calBaseOff + J8calOff) * J8StepDeg;
          J8stepCen = 0;
        }
        if (J9req == 1) {
          J9StepM = (0 + J9calBaseOff + J9calOff) * J9StepDeg;
          J9stepCen = 0;
        }
        //move to center
        int J1dir = 0;
        int J2dir = 1;
        int J3dir = 0;
        int J4dir = 1;
        int J5dir = 1;
        int J6dir = 0;
        int J7dir = 1;
        int J8dir = 1;
        int J9dir = 1;
        float ACCspd = 10;
        float DCCspd = 10;
        String SpeedType = "p";
        float SpeedVal = 80;
        float ACCramp = 50;

        driveMotorsJ(J1stepCen, J2stepCen, J3stepCen, J4stepCen, J5stepCen, J6stepCen, J7stepCen, J8stepCen, J9stepCen, J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
        robotBusy = false;
        Serial.println("DONE"); // or "DONE MJ"
        sendRobotPos();

      }
      else {
        delay(5);
        robotBusy = false;
        Serial.println("DONE");   // optional, but consistent
        Serial.println(Alarm);
        Alarm = "0";
      }

      inData = ""; // Clear recieved buffer
    }

    //----- Query Robot Status  ---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "QS")
    {
      // Status query (single-line): keep legacy BUSY/IDLE first, add ESTOP flag for GUI
      Serial.print(robotBusy ? "BUSY" : "IDLE");
      Serial.print(" ESTOP=");
      Serial.println(estop_latched ? "1" : "0");
    }

    if (function == "QE")
    {
      // Query E-stop state only
      Serial.print("ESTOP=");
      Serial.println(estop_latched ? "1" : "0");
    }




//----- LIVE JOINT JOG  ---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "LJ")
    {



      int J1dir;
      int J2dir;
      int J3dir;
      int J4dir;
      int J5dir;
      int J6dir;
      int J7dir;
      int J8dir;
      int J9dir;

      int J1axisFault = 0;
      int J2axisFault = 0;
      int J3axisFault = 0;
      int J4axisFault = 0;
      int J5axisFault = 0;
      int J6axisFault = 0;
      int J7axisFault = 0;
      int J8axisFault = 0;
      int J9axisFault = 0;
      int TotalAxisFault = 0;

      bool JogInPoc = true;
      Alarm = "0";


      int VStart = inData.indexOf("V");
      int SPstart = inData.indexOf("S");
      int AcStart = inData.indexOf("Ac");
      int DcStart = inData.indexOf("Dc");
      int RmStart = inData.indexOf("Rm");
      int WristConStart = inData.indexOf("W");
      int LoopModeStart = inData.indexOf("Lm");


      float Vector = inData.substring(VStart + 1, SPstart).toFloat();
      String SpeedType = inData.substring(SPstart + 1, SPstart + 2);
      float SpeedVal = inData.substring(SPstart + 2, AcStart).toFloat();
      float ACCspd = 100;
      float DCCspd = 100;
      float ACCramp = 100;
      String WristCon = inData.substring(WristConStart + 1, LoopModeStart);
      String LoopMode = inData.substring(LoopModeStart + 2);
      LoopMode.trim();
      J1LoopMode = LoopMode.substring(0, 1).toInt();
      J2LoopMode = LoopMode.substring(1, 2).toInt();
      J3LoopMode = LoopMode.substring(2, 3).toInt();
      J4LoopMode = LoopMode.substring(3, 4).toInt();
      J5LoopMode = LoopMode.substring(4, 5).toInt();
      J6LoopMode = LoopMode.substring(5).toInt();


      inData = ""; // Clear recieved buffer


      //clear serial
      delay(5);
      Serial.println();
      updatePos();

      float J1Angle = JangleIn[0];
      float J2Angle = JangleIn[1];
      float J3Angle = JangleIn[2];
      float J4Angle = JangleIn[3];
      float J5Angle = JangleIn[4];
      float J6Angle = JangleIn[5];
      float J7Angle = J7_pos;
      float J8Angle = J8_pos;
      float J9Angle = J9_pos;
      float xyzuvw_In[6];


      while (JogInPoc = true) {


        if (Vector == 10) {
          J1Angle = JangleIn[0] - .25;
        }
        if (Vector == 11) {
          J1Angle = JangleIn[0] + .25;
        }

        if (Vector == 20) {
          J2Angle = JangleIn[1] - .25;
        }
        if (Vector == 21) {
          J2Angle = JangleIn[1] + .25;
        }

        if (Vector == 30) {
          J3Angle = JangleIn[2] - .25;
        }
        if (Vector == 31) {
          J3Angle = JangleIn[2] + .25;
        }

        if (Vector == 40) {
          J4Angle = JangleIn[3] - .25;
        }
        if (Vector == 41) {
          J4Angle = JangleIn[3] + .25;
        }

        if (Vector == 50) {
          J5Angle = JangleIn[4] - .25;
        }
        if (Vector == 51) {
          J5Angle = JangleIn[4] + .25;
        }

        if (Vector == 60) {
          J6Angle = JangleIn[5] - .25;
        }
        if (Vector == 61) {
          J6Angle = JangleIn[5] + .25;
        }
        if (Vector == 70) {
          J7Angle = J7_pos - .25;
        }
        if (Vector == 71) {
          J7Angle = J7_pos + .25;
        }
        if (Vector == 80) {
          J8Angle = J8_pos - .25;
        }
        if (Vector == 81) {
          J8Angle = J8_pos + .25;
        }
        if (Vector == 90) {
          J9Angle = J9_pos - .25;
        }
        if (Vector == 91) {
          J9Angle = J9_pos + .25;
        }

        //calc destination motor steps
        int J1futStepM = (J1Angle + J1axisLimNeg) * J1StepDeg;
        int J2futStepM = (J2Angle + J2axisLimNeg) * J2StepDeg;
        int J3futStepM = (J3Angle + J3axisLimNeg) * J3StepDeg;
        int J4futStepM = (J4Angle + J4axisLimNeg) * J4StepDeg;
        int J5futStepM = (J5Angle + J5axisLimNeg) * J5StepDeg;
        int J6futStepM = (J6Angle + J6axisLimNeg) * J6StepDeg;
        int J7futStepM = (J7Angle + J7axisLimNeg) * J7StepDeg;
        int J8futStepM = (J8Angle + J8axisLimNeg) * J8StepDeg;
        int J9futStepM = (J9Angle + J9axisLimNeg) * J9StepDeg;


        //calc delta from current to destination
        int J1stepDif = J1StepM - J1futStepM;
        int J2stepDif = J2StepM - J2futStepM;
        int J3stepDif = J3StepM - J3futStepM;
        int J4stepDif = J4StepM - J4futStepM;
        int J5stepDif = J5StepM - J5futStepM;
        int J6stepDif = J6StepM - J6futStepM;
        int J7stepDif = J7StepM - J7futStepM;
        int J8stepDif = J8StepM - J8futStepM;
        int J9stepDif = J9StepM - J9futStepM;

        //determine motor directions
        if (J1stepDif <= 0) {
          J1dir = 1;
        }
        else {
          J1dir = 0;
        }

        if (J2stepDif <= 0) {
          J2dir = 1;
        }
        else {
          J2dir = 0;
        }

        if (J3stepDif <= 0) {
          J3dir = 1;
        }
        else {
          J3dir = 0;
        }

        if (J4stepDif <= 0) {
          J4dir = 1;
        }
        else {
          J4dir = 0;
        }

        if (J5stepDif <= 0) {
          J5dir = 1;
        }
        else {
          J5dir = 0;
        }

        if (J6stepDif <= 0) {
          J6dir = 1;
        }
        else {
          J6dir = 0;
        }

        if (J7stepDif <= 0) {
          J7dir = 1;
        }
        else {
          J7dir = 0;
        }

        if (J8stepDif <= 0) {
          J8dir = 1;
        }
        else {
          J8dir = 0;
        }

        if (J9stepDif <= 0) {
          J9dir = 1;
        }
        else {
          J9dir = 0;
        }




        //determine if requested position is within axis limits
        if ((J1dir == 1 and (J1StepM + J1stepDif > J1StepLim)) or (J1dir == 0 and (J1StepM - J1stepDif < 0))) {
          J1axisFault = 1;
        }
        if ((J2dir == 1 and (J2StepM + J2stepDif > J2StepLim)) or (J2dir == 0 and (J2StepM - J2stepDif < 0))) {
          J2axisFault = 1;
        }
        if ((J3dir == 1 and (J3StepM + J3stepDif > J3StepLim)) or (J3dir == 0 and (J3StepM - J3stepDif < 0))) {
          J3axisFault = 1;
        }
        if ((J4dir == 1 and (J4StepM + J4stepDif > J4StepLim)) or (J4dir == 0 and (J4StepM - J4stepDif < 0))) {
          J4axisFault = 1;
        }
        if ((J5dir == 1 and (J5StepM + J5stepDif > J5StepLim)) or (J5dir == 0 and (J5StepM - J5stepDif < 0))) {
          J5axisFault = 1;
        }
        if ((J6dir == 1 and (J6StepM + J6stepDif > J6StepLim)) or (J6dir == 0 and (J6StepM - J6stepDif < 0))) {
          J6axisFault = 1;
        }
        if ((J7dir == 1 and (J7StepM + J7stepDif > J7StepLim)) or (J7dir == 0 and (J7StepM - J7stepDif < 0))) {
          J7axisFault = 1;
        }
        if ((J8dir == 1 and (J8StepM + J8stepDif > J8StepLim)) or (J8dir == 0 and (J8StepM - J8stepDif < 0))) {
          J8axisFault = 1;
        }
        if ((J9dir == 1 and (J9StepM + J9stepDif > J9StepLim)) or (J9dir == 0 and (J9StepM - J9stepDif < 0))) {
          J9axisFault = 1;
        }
        TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault + J7axisFault + J8axisFault + J9axisFault;

        //send move command if no axis limit error
        if (TotalAxisFault == 0 && KinematicError == 0) {
          resetEncoders();

          driveMotorsJ(abs(J1stepDif), abs(J2stepDif), abs(J3stepDif), abs(J4stepDif), abs(J5stepDif), abs(J6stepDif), abs(J7stepDif), abs(J8stepDif), abs(J9stepDif), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
          //checkEncoders();
          J1EncSteps = J1encPos.read() / J1encMult;
          J2EncSteps = J2encPos.read() / J2encMult;
          J3EncSteps = J3encPos.read() / J3encMult;
          J4EncSteps = J4encPos.read() / J4encMult;
          J5EncSteps = J5encPos.read() / J5encMult;
          J6EncSteps = J6encPos.read() / J6encMult;

          if (Vector == 10 or Vector == 11) {
            if (J1LoopMode == 0) {
              if (abs((J1EncSteps - J1StepM)) >= 5) {
                J1collisionTrue = 1;
                J1StepM = J1encPos.read() / J1encMult;
              }
            }
          }

          if (Vector == 20 or Vector == 21) {
            if (J2LoopMode == 0) {
              if (abs((J2EncSteps - J2StepM)) >= 5) {
                J2collisionTrue = 1;
                J2StepM = J2encPos.read() / J2encMult;
              }
            }
          }

          if (Vector == 30 or Vector == 31) {
            if (J3LoopMode == 0) {
              if (abs((J3EncSteps - J3StepM)) >= 5) {
                J3collisionTrue = 1;
                J3StepM = J3encPos.read() / J3encMult;
              }
            }
          }

          if (Vector == 40 or Vector == 41) {
            if (J4LoopMode == 0) {
              if (abs((J4EncSteps - J4StepM)) >= 5) {
                J4collisionTrue = 1;
                J4StepM = J4encPos.read() / J4encMult;
              }
            }
          }

          if (Vector == 50 or Vector == 51) {
            if (J5LoopMode == 0) {
              if (abs((J5EncSteps - J5StepM)) >= 5) {
                J5collisionTrue = 1;
                J5StepM = J5encPos.read() / J5encMult;
              }
            }
          }

          if (Vector == 60 or Vector == 61) {
            if (J6LoopMode == 0) {
              if (abs((J6EncSteps - J6StepM)) >= 5) {
                J6collisionTrue = 1;
                J6StepM = J6encPos.read() / J6encMult;
              }
            }
          }

          updatePos();
        }

        //stop loop if any serial command is recieved - but the expected command is "S" to stop the loop.

        char recieved = Serial.read();
        inData += recieved;
        if (recieved == '\n') {
          break;
        }

        //end loop
      }

      TotalCollision = J1collisionTrue + J2collisionTrue + J3collisionTrue + J4collisionTrue + J5collisionTrue + J6collisionTrue;
      if (TotalCollision > 0) {
        flag = "EC" + String(J1collisionTrue) + String(J2collisionTrue) + String(J3collisionTrue) + String(J4collisionTrue) + String(J5collisionTrue) + String(J6collisionTrue);
      }

      //send move command if no axis limit error
      if (TotalAxisFault == 0 && KinematicError == 0) {
        sendRobotPos();
      }
      else if (KinematicError == 1) {
        Alarm = "ER";
        delay(5);
        Serial.println(Alarm);
        Alarm = "0";
      }
      else {
        Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault) + String(J7axisFault) + String(J8axisFault) + String(J9axisFault);
        delay(5);
        Serial.println(Alarm);
        Alarm = "0";
      }

      inData = ""; // Clear recieved buffer
      ////////MOVE COMPLETE///////////
    }



    //----- MOVE IN JOINTS ROTATION  ---------------------------------------------------
    //-----------------------------------------------------------------------

    if (function == "RJ") {
      int J1dir;
      int J2dir;
      int J3dir;
      int J4dir;
      int J5dir;
      int J6dir;
      int J7dir;
      int J8dir;
      int J9dir;

      int J1axisFault = 0;
      int J2axisFault = 0;
      int J3axisFault = 0;
      int J4axisFault = 0;
      int J5axisFault = 0;
      int J6axisFault = 0;
      int J7axisFault = 0;
      int J8axisFault = 0;
      int J9axisFault = 0;
      int TotalAxisFault = 0;

      int J1stepStart = inData.indexOf("A");
      int J2stepStart = inData.indexOf("B");
      int J3stepStart = inData.indexOf("C");
      int J4stepStart = inData.indexOf("D");
      int J5stepStart = inData.indexOf("E");
      int J6stepStart = inData.indexOf("F");
      int J7Start = inData.indexOf("J7");
      int J8Start = inData.indexOf("J8");
      int J9Start = inData.indexOf("J9");
      int SPstart = inData.indexOf("S");
      int AcStart = inData.indexOf("Ac");
      int DcStart = inData.indexOf("Dc");
      int RmStart = inData.indexOf("Rm");
      int WristConStart = inData.indexOf("W");
      int LoopModeStart = inData.indexOf("Lm");

      float J1Angle;
      float J2Angle;
      float J3Angle;
      float J4Angle;
      float J5Angle;
      float J6Angle;

      J1Angle = inData.substring(J1stepStart + 1, J2stepStart).toFloat();
      J2Angle = inData.substring(J2stepStart + 1, J3stepStart).toFloat();
      J3Angle = inData.substring(J3stepStart + 1, J4stepStart).toFloat();
      J4Angle = inData.substring(J4stepStart + 1, J5stepStart).toFloat();
      J5Angle = inData.substring(J5stepStart + 1, J6stepStart).toFloat();
      J6Angle = inData.substring(J6stepStart + 1, J7Start).toFloat();
      J7_In = inData.substring(J7Start + 2, J8Start).toFloat();
      J8_In = inData.substring(J8Start + 2, J9Start).toFloat();
      J9_In = inData.substring(J9Start + 2, SPstart).toFloat();
      String SpeedType = inData.substring(SPstart + 1, SPstart + 2);
      float SpeedVal = inData.substring(SPstart + 2, AcStart).toFloat();
      float ACCspd = inData.substring(AcStart + 2, DcStart).toFloat();
      float DCCspd = inData.substring(DcStart + 2, RmStart).toFloat();
      float ACCramp = inData.substring(RmStart + 2, WristConStart).toFloat();
      String WristCon = inData.substring(WristConStart + 1, LoopModeStart);
      String LoopMode = inData.substring(LoopModeStart + 2);
      LoopMode.trim();
      J1LoopMode = LoopMode.substring(0, 1).toInt();
      J2LoopMode = LoopMode.substring(1, 2).toInt();
      J3LoopMode = LoopMode.substring(2, 3).toInt();
      J4LoopMode = LoopMode.substring(3, 4).toInt();
      J5LoopMode = LoopMode.substring(4, 5).toInt();
      J6LoopMode = LoopMode.substring(5).toInt();

      int J1futStepM = (J1Angle + J1axisLimNeg) * J1StepDeg;
      int J2futStepM = (J2Angle + J2axisLimNeg) * J2StepDeg;
      int J3futStepM = (J3Angle + J3axisLimNeg) * J3StepDeg;
      int J4futStepM = (J4Angle + J4axisLimNeg) * J4StepDeg;
      int J5futStepM = (J5Angle + J5axisLimNeg) * J5StepDeg;
      int J6futStepM = (J6Angle + J6axisLimNeg) * J6StepDeg;
      int J7futStepM = (J7_In + J7axisLimNeg) * J7StepDeg;
      int J8futStepM = (J8_In + J8axisLimNeg) * J8StepDeg;
      int J9futStepM = (J9_In + J9axisLimNeg) * J9StepDeg;

      //calc delta from current to destination
      int J1stepDif = J1StepM - J1futStepM;
      int J2stepDif = J2StepM - J2futStepM;
      int J3stepDif = J3StepM - J3futStepM;
      int J4stepDif = J4StepM - J4futStepM;
      int J5stepDif = J5StepM - J5futStepM;
      int J6stepDif = J6StepM - J6futStepM;
      int J7stepDif = J7StepM - J7futStepM;
      int J8stepDif = J8StepM - J8futStepM;
      int J9stepDif = J9StepM - J9futStepM;


      //determine motor directions
      if (J1stepDif <= 0) {
        J1dir = 1;
      }
      else {
        J1dir = 0;
      }

      if (J2stepDif <= 0) {
        J2dir = 1;
      }
      else {
        J2dir = 0;
      }

      if (J3stepDif <= 0) {
        J3dir = 1;
      }
      else {
        J3dir = 0;
      }

      if (J4stepDif <= 0) {
        J4dir = 1;
      }
      else {
        J4dir = 0;
      }

      if (J5stepDif <= 0) {
        J5dir = 1;
      }
      else {
        J5dir = 0;
      }

      if (J6stepDif <= 0) {
        J6dir = 1;
      }
      else {
        J6dir = 0;
      }

      if (J7stepDif <= 0) {
        J7dir = 1;
      }
      else {
        J7dir = 0;
      }

      if (J8stepDif <= 0) {
        J8dir = 1;
      }
      else {
        J8dir = 0;
      }

      if (J9stepDif <= 0) {
        J9dir = 1;
      }
      else {
        J9dir = 0;
      }


      //determine if requested position is within axis limits
      if ((J1dir == 1 and (J1StepM + J1stepDif > J1StepLim)) or (J1dir == 0 and (J1StepM - J1stepDif < 0))) {
        J1axisFault = 1;
      }
      if ((J2dir == 1 and (J2StepM + J2stepDif > J2StepLim)) or (J2dir == 0 and (J2StepM - J2stepDif < 0))) {
        J2axisFault = 1;
      }
      if ((J3dir == 1 and (J3StepM + J3stepDif > J3StepLim)) or (J3dir == 0 and (J3StepM - J3stepDif < 0))) {
        J3axisFault = 1;
      }
      if ((J4dir == 1 and (J4StepM + J4stepDif > J4StepLim)) or (J4dir == 0 and (J4StepM - J4stepDif < 0))) {
        J4axisFault = 1;
      }
      if ((J5dir == 1 and (J5StepM + J5stepDif > J5StepLim)) or (J5dir == 0 and (J5StepM - J5stepDif < 0))) {
        J5axisFault = 1;
      }
      if ((J6dir == 1 and (J6StepM + J6stepDif > J6StepLim)) or (J6dir == 0 and (J6StepM - J6stepDif < 0))) {
        J6axisFault = 1;
      }
      if ((J7dir == 1 and (J7StepM + J7stepDif > J7StepLim)) or (J7dir == 0 and (J7StepM - J7stepDif < 0))) {
        J7axisFault = 1;
      }
      if ((J8dir == 1 and (J8StepM + J8stepDif > J8StepLim)) or (J8dir == 0 and (J8StepM - J8stepDif < 0))) {
        J8axisFault = 1;
      }
      if ((J9dir == 1 and (J9StepM + J9stepDif > J9StepLim)) or (J9dir == 0 and (J9StepM - J9stepDif < 0))) {
        J9axisFault = 1;
      }
      TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault + J7axisFault + J8axisFault + J9axisFault;


      //send move command if no axis limit error
      if (TotalAxisFault == 0 && KinematicError == 0) {
        resetEncoders();
     
        driveMotorsJ(abs(J1stepDif), abs(J2stepDif), abs(J3stepDif), abs(J4stepDif), abs(J5stepDif), abs(J6stepDif), abs(J7stepDif), abs(J8stepDif), abs(J9stepDif), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp);
        checkEncoders();
        sendRobotPos();
      }
      else if (KinematicError == 1) {
        Alarm = "ER";
        delay(5);
        Serial.println(Alarm);
      }
      else {
        Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault) + String(J7axisFault) + String(J8axisFault) + String(J9axisFault);
        delay(5);
        Serial.println(Alarm);
      }


      inData = ""; // Clear recieved buffer
      ////////MOVE COMPLETE///////////

    }





    else
    {
      inData = ""; // Clear recieved buffer
    }

    //shift cmd buffer
    inData = "";
    cmdBuffer1 = "";
    shiftCMDarray();

  }
}

