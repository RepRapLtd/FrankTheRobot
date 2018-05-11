#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;

const int del = 1000;

// Arduino pins

const char rRIN = D6; // Right motor pins
const char rFIN = D5;

const char lRIN = D8; // Left motor pins
const char lFIN = D7;

const char rSense = D11; // Wheel sensors
const char lSense = D10;

const char aSelect = D2;     // Select analogue channel
const char aRead = A0;       // Read analogue voltage
const char photo = false;    // Send to aSelect to read the photosensor
const char voltage = !photo; // Send to aSelect to read the battery voltage

// Directions, and stop

const char stop = 0;
const char forward = 1;
const char backward = 2;

// LIDAR numbers

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

// Return the distance in mm

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

// Control the leftmotor

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

// Control the right motor

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

// Send what's going on down the USB

void Report()
{
  digitalWrite(aSelect,voltage);
  int v = analogRead(aRead);
  Serial.print("voltage: ");
  Serial.println(v);
  digitalWrite(aSelect,photo);
  v = analogRead(aRead);
  Serial.print("photo: ");
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

void loop() 
{
  Serial.println("forward");
  LeftMotor(forward);
  RightMotor(forward);
  Report();
  Serial.println("stop");
  LeftMotor(stop);
  RightMotor(stop);
  Report();  
  Serial.println("backward");
  LeftMotor(backward);
  RightMotor(backward);
  Report();
  Serial.println("stop");
  LeftMotor(stop);
  RightMotor(stop);
  Report();  
}
