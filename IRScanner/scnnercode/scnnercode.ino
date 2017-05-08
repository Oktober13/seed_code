#include <Servo.h>

Servo servo1;
int x;
int irSensorPin = A0;

int d_pos = 1;
int apos = 90;
int t0;

void setup()
{
  int x = 0;
  pinMode(1, OUTPUT);
  pinMode(irSensorPin, INPUT);
  servo1.attach(9);
  pinMode(10, OUTPUT);
  Serial.begin(9600); //initialize baud rate
  delay(10);
  t0 = millis();
 return; 
}

void loop()
{

  
  for(int phi = 0; phi<59; phi++) {

  for (int theta = 80; theta <=130; theta++) {
    
//  Serial.print("[");

  for (int i=0;i <=7; i++) {
    apos = servo1.read();
    x = analogRead(irSensorPin);
//    Serial.print("[");
    Serial.print(apos-100);
    Serial.print(",");
    Serial.print(x);  //Print value of IR sensor
    Serial.print(",");
    Serial.print(phi);
    if (i!=7){
      Serial.print(";");
    }
    
  }

  //servo1.write(100);
  
//  if(apos <= 70)         //This is the bottom angle
//  {
//    d_pos = 1;          //Change angle in the positive direction
//    //delay(500);
//  }
//  else if(apos >= 150)  //This is the top angle
//  {
//    d_pos = -1;         //Change angle in the negative direction
//    //delay(500);
//  }
  
  
  servo1.write(theta);
  
  
  
  Serial.print(";");

  
  }

    
  digitalWrite(10, HIGH);
  delay(300);
  digitalWrite(10,LOW);
  delay(300);
//  Serial.println(x);


  
}
delay(100000);
}

