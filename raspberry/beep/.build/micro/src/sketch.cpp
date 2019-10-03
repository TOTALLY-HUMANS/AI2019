#include <Arduino.h>

void setup();
void loop();
#line 1 "src/sketch.ino"
void setup()

{

    Serial.begin(9600);
    pinMode(11, OUTPUT);
    pinMode(13, OUTPUT);
    pinMode(7, OUTPUT);
    pinMode(8, OUTPUT);
    digitalWrite(7, HIGH);
    digitalWrite(8, HIGH);
}


void loop()

{
    
    analogWrite(11, 0);
    delay(1);
    analogWrite(13, 100);
    
}
