#include <Servo.h>
Servo left;
Servo right;

void setup()
{
Serial.begin(9600);
pinMode(7, INPUT_PULLUP);
pinMode(6, INPUT_PULLUP);
left.attach(11);
right.attach(10);
left.write(127);
right.write(127);

}

int rb;
int lb;
int ctrl;

void loop()
{

lb = not digitalRead(7);
rb = not digitalRead(6);
Serial.print(lb);
Serial.println(rb);

  if(lb){
     ctrl = 77;
  }
  if(rb){
     ctrl = 177;
  }
  if(not rb and not lb){
     ctrl = 127;    
  }
left.write(ctrl);
right.write(ctrl);
}