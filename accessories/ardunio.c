#include <Servo.h>
Servo left;
Servo right;

void setup()
{
Serial.begin(9600);
pinMode(9, OUTPUT);
pinMode(5, INPUT_PULLUP);
pinMode(6, INPUT_PULLUP);
left.attach(11);
right.attach(10);
left.write(0);
right.write(0);
Serial.setTimeout(50);
}

int ctrl = 0;
String input;
char next;
bool brake;
bool butt;
bool pwr = 1;

void loop()
{
  butt = digitalRead(5);
  brake = digitalRead(6);
  if(!butt){
    pwr = true;
  }
  if(brake){
    pwr = false;
  }
  digitalWrite(9, !pwr);

    if(Serial.available() > 0){
        input = Serial.readString();
        ctrl = input.toInt();
        Serial.println(ctrl);
     }
     left.write(ctrl+127);
     right.write(ctrl+127);
}