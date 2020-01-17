#include <Servo.h>

Servo myservo; 

int pos = 0;    

void setup() {
  myservo.attach(9);
  Serial.begin(9600);  
}

void loop() { 
//  myservo.write(60);
  for (pos = 0; pos <= 120; pos += 1) { 
    myservo.write(pos);
    Serial.println(pos);          
    delay(1);                       
  }
  for (pos = 120; pos >= 0; pos -= 1) {
    myservo.write(pos);         
    Serial.println(pos);     
    delay(1);                       
  } 
 
}

