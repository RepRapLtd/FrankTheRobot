#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;

const int del = 1000;

const char rRIN = D6;
const char rFIN = D5;

const char lRIN = D8;
const char lFIN = D7;

const char rSense = D9;
const char lSense = D10;

const char aSelect = D2;
const char aRead = A0;
const char photo = false;
const char voltage = !photo;

const char stop = 0;
const char forward = 1;
const char backward = 2;

float rateLimit = 0.25;
uint32_t timingBudget = 100000;
uint8_t periodPrePclks = 18; // Must be even
uint8_t periodFinalPclks = 14; // Must be even

unsigned int defaultDistance = 2000;

void SetupLIDAR()
{
  Serial.println("Sensor starting.");
  
  Wire.begin();

  sensor.init();
  sensor.setTimeout(500);
  if(!sensor.setSignalRateLimit(rateLimit))
  {
    Serial.println("Rate limit out of range.");
    sensor.setSignalRateLimit(0.25);
  }

  if(!sensor.setMeasurementTimingBudget(timingBudget))
  {
    Serial.println("Timing budget out of range.");
    sensor.setMeasurementTimingBudget(33000);
  }

  if(!sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, periodPrePclks))
  {
    Serial.println("Pulse pre period wrong.");
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 14);
  }

  if(!sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, periodFinalPclks))
  {
    Serial.println("Pulse final period wrong.");
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 10);
  }

  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  sensor.startContinuous();    
}

unsigned int LIDARDistance()
{
  unsigned int r = sensor.readRangeContinuousMillimeters();
  if (sensor.timeoutOccurred()) { Serial.println("\nSensor timeout"); }
  if(r > 7500)
  {
    if(r > 9000)
      Serial.println("Sensor reset needed.");
    r = defaultDistance;
  }
  return r;  
}

void LeftMotor(char direction)
{
  switch(direction)
  {
    case forward:
      digitalWrite(lRIN, LOW);
      digitalWrite(lFIN, HIGH);
      break;
           
    case backward:
      digitalWrite(lFIN, LOW);
      digitalWrite(lRIN, HIGH);
      break;
      
    case stop:
    default:
      digitalWrite(lRIN, HIGH);
      digitalWrite(lFIN, HIGH);
  } 
}

void RightMotor(char direction)
{
  switch(direction)
  {
    case forward:
      digitalWrite(rRIN, LOW);
      digitalWrite(rFIN, HIGH);
      break;
           
    case backward:
      digitalWrite(rFIN, LOW);
      digitalWrite(rRIN, HIGH);
      break;
      
    case stop:
    default:
      digitalWrite(rRIN, HIGH);
      digitalWrite(rFIN, HIGH);
  } 
}

void setup() 
{
  Serial.begin(9600);
  Serial.println("begin");
  pinMode(rRIN, OUTPUT);
  pinMode(rFIN, OUTPUT);
  pinMode(lRIN, OUTPUT);
  pinMode(lFIN, OUTPUT);
  pinMode(rSense, INPUT);
  pinMode(lSense, INPUT);
  pinMode(aSelect, OUTPUT);
  pinMode(aRead, OUTPUT);
  
  SetupLIDAR();
}

void Report(int sel)
{
  digitalWrite(aSelect,sel);
  int v = analogRead(aRead);
  if(sel)
    Serial.print("voltage: ");
  else
    Serial.print("photo: ");
  Serial.print(": ");
  Serial.println(v);
  Serial.print("L, R sense: ");
  Serial.print(digitalRead(lSense));
  Serial.print(" ");
  Serial.println(digitalRead(rSense));
  Serial.print("d: ");
  Serial.println(LIDARDistance());
  Serial.println();
  delay(del);
}

void loop() 
{
  Serial.println("forward");
  LeftMotor(forward);
  RightMotor(forward);
  Report(0);
  Serial.println("backward");
  LeftMotor(backward);
  RightMotor(backward);
  Report(1);
  Serial.println("stop");
  LeftMotor(stop);
  RightMotor(stop);
  Report(0);  
}
