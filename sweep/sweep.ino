#include <Servo.h>

Servo myservo; 

int pos = 0;    

void setup() {
  myservo.attach(9);  
}

void loop() {
  for (pos = 45; pos <= 135; pos += 1) { 
    myservo.write(pos-10);              
    delay(20);                       
  }
  for (pos = 135; pos >= 45; pos -= 1) {
    myservo.write(pos-10);             
    delay(20);                       
  }
  myservo.write(90); 

}

