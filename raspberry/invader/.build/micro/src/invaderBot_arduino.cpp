#include <Arduino.h>

void motorTask();
void batteryTask();
void setup();
void loop();
#line 1 "src/invaderBot_arduino.ino"
const int CELL_1 = A0;
const int CELL_2 = A1;
const int CELL_3 = A2;

//              13, 11, 10, 9
int pinPwm[] = { 0,  0,  0, 0};

//                  R L
int motorSpeed[] = {0,0};

void motorTask()
{
  if (motorSpeed[0] >= 0)
  {
    pinPwm[0] = motorSpeed[0];
    pinPwm[1] = 0;
  }

  if (motorSpeed[0] < 0)
  {
    pinPwm[0] = 0;
    pinPwm[1] = abs(motorSpeed[0]);
  }

  // ------------------
  
  if (motorSpeed[1] >= 0)
  {
    pinPwm[2] = motorSpeed[1];
    pinPwm[3] = 0;
  }

  if (motorSpeed[1] < 0)
  {
    pinPwm[2] = 0;
    pinPwm[3] = abs(motorSpeed[1]);
  }

  // ------------------
  
  /*if (pinPwm[0] != pinPwm[1]) // Safety
  {
    pinPwm[0] = 0;
    pinPwm[1] = 0;
  }
  if (pinPwm[2] != pinPwm[3]) // Safety
  {
    pinPwm[2] = 0;
    pinPwm[3] = 0;
  }*/
  analogWrite(13, pinPwm[0]);
  delay(1);
  analogWrite(11, pinPwm[1]);
  delay(1);
  analogWrite(10, pinPwm[2]);
  delay(1);
  analogWrite(9, pinPwm[3]);
  delay(1);

  Serial.print(pinPwm[0]);
  Serial.print(" ");
  Serial.print(pinPwm[1]);
  Serial.print(" ");
  Serial.print(pinPwm[2]);
  Serial.print(" ");
  Serial.print(pinPwm[3]);
  Serial.println(" ");

  Serial.print(motorSpeed[0]);
  Serial.print(" ");
  Serial.print(motorSpeed[1]);
  Serial.println(" ");
}

void batteryTask()
{
  int c1, c2, c3;
  c1 = analogRead(CELL_1)*5; // an approximation
  c2 = analogRead(CELL_2)*5; // an approximation
  c3 = analogRead(CELL_3)*5; // an approximation

  c2 = c2*2-c1;
  c3 = c3*3-c2-c1;

  Serial.print("Battery: ");
  Serial.print(c1);
  Serial.print(" ");
  Serial.print(c2);
  Serial.print(" ");
  Serial.println(c3);

  if (true)
  {
    digitalWrite(6, HIGH);
  }
}

void setup() {
  Serial.begin(9600);
  
  // Motor Driver;
  pinMode(13, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(7, OUTPUT);
  delay(100);
  digitalWrite(8, HIGH);
  digitalWrite(7, HIGH);

  // Battery Monitor;
  pinMode(6, OUTPUT);
  batteryTask();
}

void loop() {
  char firstChar;
  int value;

  // batteryTask();
  
  if (Serial.available())
  {
    firstChar = Serial.read();

    if (firstChar == 'R' || 
        firstChar == 'L' ||
        firstChar == 'C')
    {
      value = Serial.parseInt();
      if (value > 255 || value < -255)
      {
        value = 0;
      }
    }
    else if(firstChar == 'G')
    {
      motorTask();
    }

    switch (firstChar)
    {
      case 'R': // Right Motor
        motorSpeed[0] = value;
        break;
      case 'L': // Left Motor
        motorSpeed[1] = value;
        break;
      case 'C':
        Serial.print("MUST_INVADE_");
        Serial.println(value*value); // Check code.
      default:
        break;
    }

     motorTask();
  }

  // delay(100);

  return;
  
  digitalWrite(13, LOW);
  digitalWrite(11, LOW);
  delay(100);
  digitalWrite(13, HIGH);
  delay(2000);
  digitalWrite(13, LOW);
  digitalWrite(11, LOW);
  delay(100);
  digitalWrite(11, HIGH);
  delay(2000);
}
