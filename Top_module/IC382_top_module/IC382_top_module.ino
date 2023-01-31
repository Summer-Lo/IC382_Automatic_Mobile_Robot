#include <Stepper.h>
#include <Servo.h>  

#define A_BUTTON_PIN 2
#define B_BUTTON_PIN 3
#define L_SERVO_PIN 9
#define R_SERVO_PIN 10
#define DIR_PIN 8
#define PUL_PIN 6
#define ENA_PIN 7
#define EXTEND_SENSOR 13
#define RETRACT_SENSOR 12
#define stepsPerRevolution 400 

// define the servo motor action
int L_up_servo = 0;
int R_up_servo = 180;
int L_down_servo = 90;
int R_down_servo = 90;

// define stepper direction
boolean stepper_ena = LOW;
boolean stepper_disa = HIGH;
boolean dir_extend = LOW;
boolean dir_retract = HIGH;
int stepper_speed = 30;

Servo Left_servo;   // define servo motor name
Servo Right_servo;  // define servo motor name

void setup() {
  Serial.begin(9600);

  // GPIO config
  pinMode(PUL_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);
  
  pinMode(RETRACT_SENSOR, INPUT_PULLUP);
  pinMode(EXTEND_SENSOR, INPUT_PULLUP);
  pinMode(A_BUTTON_PIN, INPUT_PULLUP);
  pinMode(B_BUTTON_PIN, INPUT_PULLUP);
  
  //digitalWrite(DIR_PIN, LOW);   // LOW = PUSH, HIGH = PULL
  //digitalWrite(ENA_PIN, stepper_disa);   // LOW = ENABLE, HIGH = DISABLE

  // Servo config and Home position
  Left_servo.attach(L_SERVO_PIN);    // config the Left servo motor pin
  Right_servo.attach(R_SERVO_PIN);   // config the Right servo motor pin
  
  Left_servo.write(L_down_servo);       // Open
  Right_servo.write(R_down_servo);      // Open
  
  // Stepper motor Home position
  
  Serial.println("<<START>> for HOME position");
  digitalWrite(ENA_PIN,stepper_ena);
  digitalWrite(DIR_PIN,dir_retract);
  while(digitalRead(RETRACT_SENSOR) == 1)     // 1 == not activiate, 0 == detected
  {
    Serial.println(digitalRead(RETRACT_SENSOR));
    analogWrite(PUL_PIN,stepper_speed);
    delay(100);
  }
  analogWrite(PUL_PIN,0);
  digitalWrite(ENA_PIN,stepper_disa);
  Serial.println("<<END>> for HOME position");
  
}
void loop() {
    // GPIO Digital output genereate pulse
    if(digitalRead(A_BUTTON_PIN) == 0)
    {
      Serial.println("<<START>> extending for [Stepper Motor]");
      digitalWrite(ENA_PIN,stepper_ena);
      digitalWrite(DIR_PIN,dir_extend);
      while(digitalRead(EXTEND_SENSOR) == 1)
      {
        analogWrite(PUL_PIN,stepper_speed);
      }
      analogWrite(PUL_PIN,0);
      digitalWrite(ENA_PIN,stepper_disa);
      Serial.println("<<END>> extending for [Stepper Motor]");
      Serial.println("<<START>> uping for [Servo Motor]");
      Left_servo.write(L_up_servo);      // Open
      Right_servo.write(R_up_servo);   // Open
      delay(2000);
      Serial.println("<<END>> uping for [Servo Motor]");
      Serial.println("<<START>> retracting for [Stepper Motor]");
      digitalWrite(ENA_PIN,stepper_ena);
      digitalWrite(DIR_PIN,dir_retract);
      while(digitalRead(RETRACT_SENSOR) == 1)
      {
        analogWrite(PUL_PIN,stepper_speed);
      }
      analogWrite(PUL_PIN,0);
      digitalWrite(ENA_PIN,stepper_disa);
      Serial.println("<<END>> retracting for [Stepper Motor]");
    }

    else if(digitalRead(B_BUTTON_PIN) == 0)
    {
      Serial.println("<<START>> extending for [Stepper Motor]");
      digitalWrite(ENA_PIN,stepper_ena);
      digitalWrite(DIR_PIN,dir_extend);
      while(digitalRead(EXTEND_SENSOR) == 1)
      {
        analogWrite(PUL_PIN,stepper_speed);
      }
      analogWrite(PUL_PIN,0);
      digitalWrite(ENA_PIN,stepper_disa);
      Serial.println("<<END>> extending for [Stepper Motor]");
      Serial.println("<<START>> uping for [Servo Motor]");
      Left_servo.write(L_down_servo);      // Open
      Right_servo.write(R_down_servo);   // Open
      delay(2000);
      Serial.println("<<END>> downing for [Servo Motor]");
      Serial.println("<<START>> retracting for [Stepper Motor]");
      digitalWrite(ENA_PIN,stepper_ena);
      digitalWrite(DIR_PIN,dir_retract);
      while(digitalRead(RETRACT_SENSOR) == 1)
      {
        analogWrite(PUL_PIN,stepper_speed);
      }
      analogWrite(PUL_PIN,0);
      digitalWrite(ENA_PIN,stepper_disa);
      Serial.println("<<END>> retracting for [Stepper Motor]");
    }
    /*
    Serial.println("<<START>> for [Stepper Motor]");
    digitalWrite(ENA_PIN,stepper_ena);
    digitalWrite(DIR_PIN,dir_extend);
    analogWrite(PUL_PIN,stepper_speed);
    Serial.println("<<END>> for [Stepper Motor]");
    Serial.println("<<START>> for [Servo Motor]");
    Left_servo.write(L_down_servo);      // Open
    Right_servo.write(R_down_servo);   // Open
    delay(2000);
    Serial.println("<<END>> for [Servo Motor]");
    */
    /*
    for(int x=0; x<3*stepsPerRevolution; x++){
    digitalWrite(PUL_PIN, HIGH);
    delayMicroseconds(600);
    digitalWrite(PUL_PIN, LOW);
    delayMicroseconds(600);
    }
    */
    /*
    if(digitalRead(END_SENSOR) == 1)
    {
    analogWrite(PUL_PIN,10);
    }
    else if(digitalRead(END_SENSOR) == 0)
    {
      Serial.println("Detected");
      analogWrite(PUL_PIN,0);
      analogWrite(L_SERVO_PIN,45);
      analogWrite(R_SERVO_PIN,45);
    }
    */
  
}
