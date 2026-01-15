/*  AR3 Annin Robot Control Software Arduino Mega 2560 sketch
    Copyright (c) 2022, Chris Annin
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
          Selling AR2 software, robots, robot parts, or any versions of robots or software based on this
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

    Log:

*/


#include <Servo.h>

String inData;


Servo servo0;
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;
Servo servo7;

//set gripper servo limits in microseconds
const int gripperMin = 780;
const int gripperMax = 1820;

const int Input0 = 0;
const int Input1 = 1;
const int Input2 = 2;
const int Input3 = 3;
const int Input4 = 4;
const int Input5 = 5;
const int Input6 = 6;
const int Input7 = 7;
const int Input8 = 8;
const int Input9 = 9;
const int Input10 = 10;
const int Input11 = 11;
const int Input12 = 12;
const int Input13 = 13;
const int Input14 = 14;
const int Input15 = 15;
const int Input16 = 16;
const int Input17 = 17;
const int Input18 = 18;
const int Input19 = 19;
const int Input20 = 20;
const int Input21 = 21;
const int Input22 = 22;
const int Input23 = 23;
const int Input24 = 24;
const int Input25 = 25;
const int Input26 = 26;
const int Input27 = 27;


//28~35 for the relay
const int ch0 = 28;
const int ch1 = 29;
const int ch2 = 30;
const int ch3 = 31;
const int ch4 = 32;
const int ch5 = 33;
const int ch6 = 34;
const int ch7 = 35;

const int stepPin1 = 36;//stepper motor 1, move wok up/down
const int dirPin1 = 37;//stepper motor 1, move wok up/down
const int stepPin2 = 38;//stepper motor 2, yaw 
const int dirPin2 = 39;//stepper motor 2, yaw 
const int stepPin3 = 40;//stepper motor 3, roll 
const int dirPin3 = 41;//stepper motor 3, roll 
const int Output42 = 42;
const int Output43 = 43;
const int Output44 = 44;
const int Output45 = 45;
const int Output46 = 46;
const int Output47 = 47;
const int Output48 = 48;
const int Output49 = 49;
const int Output50 = 50;
const int Output51 = 51;
const int Output52 = 52;
const int Output53 = 53;

//global variables
const float microStep = 2;
const int delayHigh = 500/microStep; //400 microseconds is the minimal I can set for 1 microstep setting TB6600
const int delayLow = 500/microStep; //400 microseconds is the minimal I can set for 1 microstep setting TB6600
const int delayHighMS = 30; //delay in milliseconds, for the rotation movement, use this longer delay to slow down
const int delayLowMS = 30; 
const int delayTimeMS = 1000;

void setup() {
  // run once:
  Serial.begin(115200);

  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(A4, OUTPUT);
  pinMode(A5, OUTPUT);
  pinMode(A6, OUTPUT);
  pinMode(A7, OUTPUT);

  pinMode(A8, INPUT);
  pinMode(A9, INPUT);
  pinMode(A10, INPUT);
  pinMode(A11, INPUT);
  pinMode(A12, INPUT);
  pinMode(A13, INPUT);
  pinMode(A14, INPUT);
  pinMode(A15, INPUT);


  pinMode(Input0, INPUT_PULLUP);
  pinMode(Input1, INPUT_PULLUP);
  pinMode(Input2, INPUT_PULLUP);
  pinMode(Input3, INPUT_PULLUP);
  pinMode(Input4, INPUT_PULLUP);
  pinMode(Input5, INPUT_PULLUP);
  pinMode(Input6, INPUT_PULLUP);
  pinMode(Input7, INPUT_PULLUP);
  pinMode(Input8, INPUT_PULLUP);
  pinMode(Input9, INPUT_PULLUP);
  pinMode(Input10, INPUT_PULLUP);
  pinMode(Input11, INPUT_PULLUP);
  pinMode(Input12, INPUT_PULLUP);
  pinMode(Input13, INPUT_PULLUP);
  pinMode(Input14, INPUT_PULLUP);
  pinMode(Input15, INPUT_PULLUP);
  pinMode(Input16, INPUT_PULLUP);
  pinMode(Input17, INPUT_PULLUP);
  pinMode(Input18, INPUT_PULLUP);
  pinMode(Input19, INPUT_PULLUP);
  pinMode(Input20, INPUT_PULLUP);
  pinMode(Input21, INPUT_PULLUP);
  pinMode(Input22, INPUT_PULLUP);
  pinMode(Input23, INPUT_PULLUP);
  pinMode(Input24, INPUT_PULLUP);
  pinMode(Input25, INPUT_PULLUP);
  pinMode(Input26, INPUT_PULLUP);
  pinMode(Input27, INPUT_PULLUP);

  //relay
  pinMode(ch0, OUTPUT);
  pinMode(ch1, OUTPUT);
  pinMode(ch2, OUTPUT);
  pinMode(ch3, OUTPUT);
  pinMode(ch4, OUTPUT);
  pinMode(ch5, OUTPUT);
  pinMode(ch6, OUTPUT);
  pinMode(ch7, OUTPUT);

  //motors
  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(stepPin3, OUTPUT);
  pinMode(dirPin3, OUTPUT);
  pinMode(Output42, OUTPUT);
  pinMode(Output43, OUTPUT);
  pinMode(Output44, OUTPUT);
  pinMode(Output45, OUTPUT);
  pinMode(Output46, OUTPUT);
  pinMode(Output47, OUTPUT);
  pinMode(Output48, OUTPUT);
  pinMode(Output49, OUTPUT);
  pinMode(Output50, OUTPUT);
  pinMode(Output51, OUTPUT);
  pinMode(Output52, OUTPUT);
  pinMode(Output53, OUTPUT);

  servo0.attach(A0, gripperMin, gripperMax);
  //below only works for continuous servo, avoid twitching at power up, not working
  //it seems continuous servos always twitch on power up
  //servo1.attach(A1);
  //servo1.writeMicroseconds(1500);//turn servo off
  //servo1.detach();
  //servo2.attach(A2);
  //servo2.writeMicroseconds(1500);//turn servo off
  //servo2.detach();
  //servo3.attach(A3);
  //servo3.writeMicroseconds(1500);//turn servo off
  //servo3.detach();
  //servo4.attach(A4);
  //servo4.writeMicroseconds(1500);//turn servo off
  //servo4.detach();
  
  servo5.attach(A5);
  servo6.attach(A6);
  servo7.attach(A7);

  // HIGH will disconnect switch
  digitalWrite(ch0, HIGH);
  digitalWrite(ch1, HIGH);
  digitalWrite(ch2, HIGH);
  digitalWrite(ch3, HIGH);
  digitalWrite(ch4, HIGH);
  digitalWrite(ch5, HIGH);
  digitalWrite(ch6, HIGH);
  digitalWrite(ch7, HIGH);
}

void loop() {
  //start loop
  while (Serial.available() > 0)
  {
    char recieved = Serial.read();
    inData += recieved;
    // Process message when new line character is recieved
    if (recieved == '\n')
    {
      String function = inData.substring(0, 2);


      //-----COMMAND TO MOVE SERVO---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "SV")
      {
        int SVstart = inData.indexOf('V');
        int POSstart = inData.indexOf('P');
        int servoNum = inData.substring(SVstart + 1, POSstart).toInt();
        int servoPOS = inData.substring(POSstart + 1).toInt();
        if (servoNum == 0)
        {
          //constrain the input to the gripper openning range 0~110mm
          servoPOS = constrain(servoPOS, 0, 110);
          //map the input to the servo range
          servoPOS = map(servoPOS, 0, 110, gripperMin, gripperMax);
          //constrain again to make sure it's within the servo range
          servoPOS = constrain(servoPOS, gripperMin, gripperMax);
          servo0.writeMicroseconds(servoPOS);
        }
        if (servoNum == 1)
        {
          //this is a continuous servo
          if(servoPOS<80)
          {
            servo1.attach(A1);
            servo1.writeMicroseconds(1000);
          }
          else if(servoPOS>100)
          {
            servo1.attach(A1);
            servo1.writeMicroseconds(2000);
          }
          else
          {
            servo1.writeMicroseconds(1500);//turn servo off
            servo1.detach();
          }
        }
        if (servoNum == 2)
        {
          //this is a continuous servo
          if(servoPOS<80)
          {
            servo2.attach(A2);
            servo2.writeMicroseconds(1000);
          }
          else if(servoPOS>100)
          {
            servo2.attach(A2);
            servo2.writeMicroseconds(2000);
          }
          else
          {
            servo2.writeMicroseconds(1500);//turn servo off
            servo2.detach();
          }
        }
        if (servoNum == 3)
        {
          //this is a continuous servo
          if(servoPOS<80)
          {
            servo3.attach(A3);
            servo3.writeMicroseconds(1000);
          }
          else if(servoPOS>100)
          {
            servo3.attach(A3);
            servo3.writeMicroseconds(2000);
          }
          else
          {
            servo3.writeMicroseconds(1500);//turn servo off
            servo3.detach();
          }
        }
        if (servoNum == 4)
        {
          //this is a continuous servo
          if(servoPOS<80)
          {
            servo4.attach(A4);
            servo4.writeMicroseconds(1000);
          }
          else if(servoPOS>100)
          {
            servo4.attach(A4);
            servo4.writeMicroseconds(2000);
          }
          else
          {
            servo4.writeMicroseconds(1500);//turn servo off
            servo4.detach();
          }
        }
        if (servoNum == 5)
        {
          servo5.write(servoPOS);
        }
        if (servoNum == 6)
        {
          servo6.write(servoPOS);
        }
        if (servoNum == 7)
        {
          servo7.write(servoPOS);
        }
        Serial.println("Servo Done");
      }
      //control continuous servos
      if (function == "SC")
      {
        int SVstart = inData.indexOf('C');
        int timeStart = inData.indexOf('T');
        int servoNum = inData.substring(SVstart + 1, timeStart).toInt();
        int servoTime = inData.substring(timeStart + 1).toInt();
        if (servoNum == 0)
        {
          //do nothing
        }
        if (servoNum == 1)
        {
          servo1.attach(A1);
          servo1.writeMicroseconds(1000);
          delay(servoTime);
          servo1.writeMicroseconds(1500);//turn servo off
          servo1.detach();
        }
        if (servoNum == 2)
        {
          servo2.attach(A2);
          servo2.writeMicroseconds(1000);
          delay(servoTime);
          servo2.writeMicroseconds(1500);//turn servo off
          servo2.detach();
        }
        if (servoNum == 3)
        {
          servo3.attach(A3);
          servo3.writeMicroseconds(1000);
          delay(servoTime);
          servo3.writeMicroseconds(1500);//turn servo off
          servo3.detach();
        }
        if (servoNum == 4)
        {
          servo4.attach(A4);
          servo4.writeMicroseconds(1000);
          delay(servoTime);
          servo4.writeMicroseconds(1500);//turn servo off
          servo4.detach();
        }
        if (servoNum == 5)
        {
        }
        if (servoNum == 6)
        {
        }
        if (servoNum == 7)
        {
        }
        Serial.println("Servo Continuous Done");
      }
      //control stepper motors
      else if (function == "ST")
      {
        int STstart = inData.indexOf('T');
        int POSstart = inData.indexOf('P');
        int limitSwitchstart = inData.indexOf("LS");
        int stepperNum = inData.substring(STstart + 1, POSstart).toInt();
        int stepperPos = inData.substring(POSstart + 1, limitSwitchstart).toInt();
        int checkLS = inData.substring(limitSwitchstart + 2).toInt();
       
        //stepper number 1, move wok up/down
        if (stepperNum == 1)
        {        
          if(stepperPos > 0)
          {
            digitalWrite(dirPin1,HIGH); //Enables the motor to move in a particular direction, HIGH is up
          }
          else
          {
            digitalWrite(dirPin1,LOW); //Enables the motor to move in a particular direction, LOW is down
          }
  
          stepperPos = abs(stepperPos); //in mm
          //when microstep is 1:
          //angle/step is 1.8 degrees, lead screw pitch is 4mm
          //Makes 200 pulses for making one full cycle rotation

          // map degrees to steps
          //int steps = map(stepperPos, 0, 360, 0, 480);

          for(long x = 0; x < 200 * microStep / 4 * stepperPos; x++) {
            digitalWrite(stepPin1,HIGH); 
            delayMicroseconds(delayHigh); 
            digitalWrite(stepPin1,LOW); 
            delayMicroseconds(delayLow); 
          }      
  
          digitalWrite(stepPin1,HIGH);//why? to make the stepper hold the torque?
        }
        //stepper number 2, yaw, rotate toward to the front of the table, or toward the back
        else if (stepperNum == 2)
        {        
          if(stepperPos > 0)
          {
            digitalWrite(dirPin2,HIGH); //HIGH: turn towards the front of the table
          }
          else
          {
            digitalWrite(dirPin2,LOW); //LOW: turn towards the back of the table
          }
  
          stepperPos = abs(stepperPos); //in degrees
          //when microstep is 1:
          //angle/step is 1.8 degrees
          //Makes 200 pulses for making one full cycle rotation

          // map degrees to steps
          //48/20=2.4 gear ratio
          int steps = map(stepperPos, 0, 360, 0, 200 * microStep * 2.4);
          int accSteps = steps / 10;
          int delayMS2 = 20;
          
          for(long x = 0; x < steps; x++) {
            if(x < accSteps)
            {
              delayMS2 = 40;
            }
            else if(x > steps - accSteps)
            {
              delayMS2 = 40;
            }
            else
            {
              delayMS2 = 20;
            }

            digitalWrite(stepPin2,HIGH); 
            delay(delayMS2); 
            digitalWrite(stepPin2,LOW); 
            delay(delayMS2); 
          }          
 
          digitalWrite(stepPin2,HIGH);//why? to make the stepper hold the torque?
        }
        //stepper number 3, roll, rotate toward to the front of the table, or toward the back
        else if (stepperNum == 3)
        {        
          if(stepperPos > 0)
          {
            digitalWrite(dirPin3,HIGH); //toward the front of the table
          }
          else
          {
            digitalWrite(dirPin3,LOW); //toward the back of the table
          }
  
          stepperPos = abs(stepperPos); //in degrees
          //when microstep is 1:
          //angle/step is 1.8 degrees, internal gear ratio is 19:1
          //Makes 200x19 pulses for making one full cycle rotation

          // map degrees to steps
          //48/20=2.4 gear ratio
          //18240<32767, so int is OK here
          int steps = map(stepperPos, 0, 360, 0, 200 * microStep * 2.4 * 19);

          for(long x = 0; x < steps; x++) {
            digitalWrite(stepPin3,HIGH); 
            delay(1); 
            digitalWrite(stepPin3,LOW); 
            delay(1); 
          }          
 
          digitalWrite(stepPin3,HIGH);//why? to make the stepper hold the torque?
        }
        //don't do anything for the rest of the motor index
        else
        {}
        
        Serial.println("Stepper Done");
      }
      
      //control DC motors
      else if (function == "DC")
      {
        Serial.print("DC motor Done");
      }
      
      //control 8 channel relay
      else if (function == "SW")
      {
        int SWstart = inData.indexOf('W');
        int POSstart = inData.indexOf('P');
        int swChannel = inData.substring(SWstart + 1, POSstart).toInt();
        int swBool = inData.substring(POSstart + 1).toInt();
        int portNum = ch0;
        
        //map swChannel to a pin
        if(swChannel == 0)
        {
          portNum = ch0;
        }
        else if(swChannel == 1)
        {
          portNum = ch1;
        }
        else if(swChannel == 2)
        {
          portNum = ch2;
        }
        else if(swChannel == 3)
        {
          portNum = ch3;
        }
        else if(swChannel == 4)
        {
          portNum = ch4;
        }
        else if(swChannel == 5)
        {
          portNum = ch5;
        }
        else if(swChannel == 6)
        {
          portNum = ch6;
        }
        else if(swChannel == 7)
        {
          portNum = ch7;
        }

        //check control mode
        if(swBool == 1)
        {
          digitalWrite(portNum, LOW);
        }
        else if(swBool == 0)
        {
          digitalWrite(portNum, HIGH);
        }
        else if(swBool >= 2)
        {
          //flip switch on, delay swBool ms, then flip switch off
          digitalWrite(portNum, LOW);
          delay(swBool);
          digitalWrite(portNum, HIGH);
        }

        Serial.println("Relay control Done");
      }
      //-----COMMAND IF INPUT THEN JUMP---------------------------------------------------
      //-----------------------------------------------------------------------
      else if (function == "JF")
      {
        int IJstart = inData.indexOf('X');
        int IJTabstart = inData.indexOf('T');
        int IJInputNum = inData.substring(IJstart + 1, IJTabstart).toInt();
        if (digitalRead(IJInputNum) == HIGH)
        {
          Serial.println("T");
        }
        if (digitalRead(IJInputNum) == LOW)
        {
          Serial.println("F");
        }
      }
      //-----COMMAND SET OUTPUT ON---------------------------------------------------
      //-----------------------------------------------------------------------
      else if (function == "ON")
      {
        int ONstart = inData.indexOf('X');
        int outputNum = inData.substring(ONstart + 1).toInt();
        digitalWrite(outputNum, HIGH);
        Serial.print("Done");
      }
      //-----COMMAND SET OUTPUT OFF---------------------------------------------------
      //-----------------------------------------------------------------------
      else if (function == "OF")
      {
        int ONstart = inData.indexOf('X');
        int outputNum = inData.substring(ONstart + 1).toInt();
        digitalWrite(outputNum, LOW);
        Serial.print("Done");
      }
      //-----COMMAND TO WAIT INPUT ON---------------------------------------------------
      //-----------------------------------------------------------------------
      else if (function == "WI")
      {
        int WIstart = inData.indexOf('N');
        int InputNum = inData.substring(WIstart + 1).toInt();
        while (digitalRead(InputNum) == LOW) {
          delay(100);
        }
        Serial.print("Done");
      }
      //-----COMMAND TO WAIT INPUT OFF---------------------------------------------------
      //-----------------------------------------------------------------------
      else if (function == "WO")
      {
        int WIstart = inData.indexOf('N');
        int InputNum = inData.substring(WIstart + 1).toInt();

        while (digitalRead(InputNum) == HIGH) {
          delay(100);
        }
        Serial.print("Done");
      }
      //-----COMMAND ECHO TEST MESSAGE---------------------------------------------------
      //-----------------------------------------------------------------------
      else if (function == "TM")
      {
        String echo = inData.substring(2);
        Serial.println(echo);
      }

      inData = ""; // Clear recieved buffer
    }
  }
}
