#include <LiquidCrystal.h>
#include <Ultrasonic.h>
#include <Servo.h>

LiquidCrystal lcd(12, 11, 5, 4, 3, 2); 
Ultrasonic ultrasonic(10,13); // make ultrasonic object
Servo myservo;  // create servo object to control a servo

const int potkPin = 1; // pins for potentiometers for setting p i and d coeffs
const int potkIin = 2;
const int potkDin = 3;
const int buttonPin = 1;// button input
const float setPoint = 9; // desired ball distance from sonar device 
int pos = 0;    // servo command variable
int period = 50; // refresh rate period
float kp=10; // proportional
float ki=0;  // integral
float kd=0;  // control
float PID_p, PID_i, PID_d, PID_total; // values calculated after applying coefficients
float measure; // where the ball is currently
float sum;
float error; // calculated error 
float lastError;
float time;
void setup() {
  // set up the LCD's number of columns and rows:
  Serial.begin(9600);
  lcd.begin(16, 2);
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  myservo.write(90); //Put the servco at angle 125, so the balance is in the middle
  // potentiometers set to analog pins
//  while (digitalRead(buttonPin) == HIGH){ //wait for user to press button after setting PID vals
//    lcd.clear();
//    lcd.setCursor(0, 0);
//    lcd.print("  P    I     D  ");
//    //get pot values
//    kp = map(analogRead(potkPin),0,1023,0,100);
//    ki = map(analogRead(potkIin),0,1023,0,100);
//    kd = map(analogRead(potkDin),0,1023,0,100);
//    lcd.setCursor(0, 1);
//    lcd.print(kp);
//    lcd.setCursor(5, 1);
//    lcd.print(ki);
//    lcd.setCursor(10,1);
//    lcd.print(kd);
//    }  
  kp=1;
  kd=0;
  ki=0; 
  time=millis();
}

void loop() {  
  if (millis() > time+period){
    time = millis();
    sum=0;
    for (int i=1;i<100;i++){
      sum = sum+ultrasonic.Ranging(CM);
    }
    measure = sum/100;// Calculate error to feed back (might have to call function to average value)
    if (measure > 18){
      measure = 18;
    }
    error = setPoint - measure;
//    Serial.println(measure);
    // Proportioanl control
    PID_p = error*kp; // Apply proportional control using set kp values
//    Serial.print("PID_p=");
//    Serial.println(PID_p);
    
    // Derivative control
    PID_d = kd*((error-lastError)/period);
//    Serial.print("PID_d=");
//    Serial.println(PID_d);

    // Integral control (might have to remove spurious values)
    PID_i = PID_i + ki*error;
//    Serial.print("PID_i=");
//    Serial.println(PID_i);
    

    PID_total = PID_p + PID_i + PID_d;

    if (PID_total > 150){
      PID_total =150;
    }
    if (PID_total < -150){
      PID_total =-150;
    }
    pos = map(PID_total,-150,150,120,0); // map the PID command to a servo angle
    Serial.println(error);
    Serial.println(pos);
    myservo.write(pos); // command to servo
    lastError = error;
    delay(3);
  }



}




