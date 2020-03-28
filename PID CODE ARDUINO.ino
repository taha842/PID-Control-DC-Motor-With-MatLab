/*
  Muhammad Taha
  0334-3992626
  taha_munir842@hotmail.com
*/

#include <SoftwareSerial.h>
#include <PID_v1.h>
#define ENCODEROutput 50
int in1Pin = 11;                   // L293D INPUT PIN
int in2Pin = 13;                   // L293D INPUT PIN
int enablePin = 5;                 // L293D ENABLE PIN
int encoderpin1 = 2;               // ENCODER PIN A
int encoderpin2 = 3;               // ENCODER PIN B
volatile long encoderValue = 0;    // ENCODER VALUE STORE
int rpm = 0;                       // RPM VALUE STORE
int motorPwm = 0;                  // MOTOR PWM VALUE
unsigned int integerValue = 0;     // Max value is 65535
char incomingByte;
float z;
double Setpoint, Input, Output;    // Declare variables
double Kp = 1, Ki =5, Kd = 0.1;   // set values of PID
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup()
{
  Serial.begin(9600);
  pinMode(encoderpin1, INPUT_PULLUP); // Set hall sensor A as Input pullup
  // Attach interrupt at hall sensor A on each rising signal
  attachInterrupt(digitalPinToInterrupt(encoderpin1), updateEncoder, FALLING);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(encoderpin1, INPUT);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);
  myPID.SetSampleTime(0);
  Setpoint = 0;
}

// LOOP_START

void loop()
{
 
  //Output = 0;
  RPM();
  //Serial.print("Motor Speed = ");
  //Serial.print(rpm);

  //Serial.print("Setpoint = ");
  //Serial.println(Setpoint);

  Input = rpm;
  myPID.Compute();
  delay(300);
  analogWrite(enablePin , Output);
  //Serial.print("OUPUT = ");
  //Serial.println (Output);
  //Serial.println (rpm);
  Serial.write(rpm);

 

  // Serial Communication
  SERIALs();     // Read data from Serial, store to 'command'

  if ( integerValue == 500 ) // Normal_Mode Counter Clockwise
  {
    REVERSEs();
  }
  else if ( integerValue == 600) // Normal_Mode Clockwise
  {
    FORWARD();
  }
  else if ( integerValue == 700 ) // STOP Motor
  {
    STOP();
  }
  else if ( (integerValue >= 0) && (integerValue <= 120 ))
  {
    Setpoint = integerValue;
  }
  else if  (integerValue == 800)
  {
    STOP();
    Setpoint = 0;
    integerValue = 0;
    Output = 0;
    Input = 0;
    rpm = 0;
    delay (500);

  }
  else
  {
    //Serial.println(integerValue);
    delay (500);
  }
}


// FUNCTIONS

void STOP()
{
  digitalWrite(in2Pin, LOW);
  digitalWrite(in1Pin, LOW);
  digitalWrite(enablePin, LOW);
  //Serial.println("MOTOR STOP");
}

void FORWARD()
{
  digitalWrite(in2Pin, HIGH);
  digitalWrite(in1Pin, LOW);
  //Serial.println("MOTOR DIRECTION: CLOCKWISE");
  delay (500);
}

void REVERSEs()
{
  digitalWrite(in1Pin, HIGH);
  digitalWrite(in2Pin, LOW);
  //Serial.println("MOTOR DIRECTION: COUNETER CLOCKWISE");
  delay (500);
}

void updateEncoder()
{
  // Add encoderValue by 1, each time it detects rising signal
  // from hall sensor A
  encoderValue++;
}

void RPM()
{
  rpm = (float)(encoderValue * 60 / ENCODEROutput);
  delay (500);
  encoderValue = 0;
}

void SERIALs()
{
  integerValue = 900;
  if (Serial.available() > 0)
  {
    char buffer[] = {' ', ' ', ' ', ' ', ' ', ' ', ' '}; // Receive up to 7 bytes
    while (!Serial.available()); // Wait for characters
    Serial.readBytesUntil('n', buffer, 7);
    integerValue = atoi(buffer);

  }
}

