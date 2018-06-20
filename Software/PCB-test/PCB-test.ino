/*
 * Test program for Frank The Robot
 * 
 * This allows all the electronic and mechanical functions to
 * be run and tested individually.
 * 
 * Adrian Bowyer
 * RepRap Ltd
 * http://reprapltd.com
 * 
 * 8 June 2018
 * 
 * Licence: GPL
 */

#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;

#define WEMOS_MINI

#ifdef WEMOS_R1
// Wemos D1 R1 "Arduino" pins

const char rRIN = D6; // Right motor pins
const char rFIN = D5; //

const char lRIN = D8; // Left motor pins
const char lFIN = D7; //

const char rSense = D9; // Wheel sensors NB: LED pin - LED pulls up?
const char lSense = D10; // (Input pulldown)

const char led = LED_BUILTIN; // D9

const char aSelect = D2;     // Select analogue channel
const char aRead = A0;       // Read analogue voltage
#endif


#ifdef WEMOS_R2
// Wemos D1 R2 "Arduino" pins

const char rRIN = D4; // Right motor pins
const char rFIN = D3;

const char lRIN = D6; // Left motor pins
const char lFIN = D8;

const char rSense = D7; // Wheel sensors
const char lSense = D5;

const char led = LED_BUILTIN;

const char aSelect = D0;     // Select analogue channel
const char aRead = A0;       // Read analogue voltage
#endif

#ifdef WEMOS_MINI
// Wemos Mini "Arduino" pins

const char rRIN = D8; // Right motor pins
const char rFIN = D6;

const char lRIN = D4; // Left motor pins
const char lFIN = D3;

const char rSense = D7; // Wheel sensors
const char lSense = D5;

const char led = LED_BUILTIN;

const char aSelect = D0;     // Select analogue channel
const char aRead = A0;       // Read analogue voltage
#endif

const char photo = false;    // Send to aSelect to read the photosensor
const char voltage = !photo; // Send to aSelect to read the battery voltage

// Motor directions, stop and motor control data

const char stop = 0;
const char forward = 1;
const char backward = 2;
volatile char leftDirection, rightDirection; // What each motor is doing
unsigned char lfPWM, lbPWM, rfPWM, rbPWM;    // PWM to control each motor speed (forward != backward in general)
volatile long leftCount, rightCount; // Counts of the wheel opto-slots
volatile long leftTime, rightTime;  // milliseconds that the motor's been turning
volatile char lStop, rStop; // Flag for when a motor has just stopped


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
  if (sensor.timeoutOccurred())
  {
    Serial.println("\nSensor timeout");
  }
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
      analogWrite(lFIN, lfPWM);
      break;
           
    case backward:
      digitalWrite(lFIN, LOW);
      analogWrite(lRIN, lbPWM);
      break;
      
    case stop:
    default:
      digitalWrite(lRIN, HIGH);
      digitalWrite(lFIN, HIGH);
  }

  // Remember what we're doing
  
  leftDirection = direction; 
}

// Control the right motor

void RightMotor(char direction)
{
  switch(direction)
  {
    case forward:
      digitalWrite(rRIN, LOW);
      analogWrite(rFIN, rfPWM);
      break;
           
    case backward:
      digitalWrite(rFIN, LOW);
      analogWrite(rRIN, rbPWM);
      break;
      
    case stop:
    default:
      digitalWrite(rRIN, HIGH);
      digitalWrite(rFIN, HIGH);
  }

  // Remember what we're doing
  
  rightDirection = direction;  
}


void PrintDistance()
{
  Serial.print("LIDAR distanc (mm): ");
  Serial.println(LIDARDistance());  
}

void PrintWheels()
{
  Serial.print("L, R wheel sensors: ");
  Serial.print(digitalRead(lSense));
  Serial.print(", ");
  Serial.println(digitalRead(rSense));  
}

void PrintIllumination()
{
  digitalWrite(aSelect,photo);
  int v = analogRead(aRead);
  Serial.print("Illumination voltage: ");
  Serial.println(v);  
}

void PrintBattery()
{
  digitalWrite(aSelect,voltage);
  int v = analogRead(aRead);
  Serial.print("Battery voltage: ");
  Serial.println(v);  
}

// Set the right motor running for n opto-wheel-slot pulses

void RunRight(long n, char dir)
{
  rightCount = n;
  RightMotor(dir);
  rightTime = millis();
}

// Set the left motor running for n opto-wheel-slot pulses

void RunLeft(long n, char dir)
{
  leftCount = n;
  LeftMotor(dir);
  leftTime = millis();
}

// This function gets called by the interrupt on the falling edge
// of the left wheel opto-slot sensor signal.

void LeftInterupt()
{
  if(leftDirection == stop)
    return;
  leftCount--; // Countdown
  if(leftCount > 0)
    return;
  LeftMotor(stop);
  leftTime = millis() - leftTime;
  lStop = true;
}


// This function gets called by the interrupt on the falling edge
// of the right wheel opto-slot sensor signal.

void RightInterupt()
{
  if(rightDirection == stop)
    return;
  rightCount--; // Countdown
  if(rightCount > 0)
    return;
  RightMotor(stop);
  rightTime = millis() - rightTime;
  rStop = true;
}

// This gets called when the main loop detects that the left motor has stopped.

void LeftStopped()
{
  lStop = false;
  Serial.print("\nLeft motor turned for ");
  Serial.print(leftTime);
  Serial.println(" ms.");
}

// This gets called when the main loop detects that the right motor has stopped.

void RightStopped()
{
  rStop = false;
  Serial.print("\nRight motor turned for ");
  Serial.print(rightTime);
  Serial.println(" ms.");
}

// Remind the user what can be done...

void Prompt()
{
  Serial.println("\nCommands:");
  Serial.println(" L n - turn left motor for n steps forward.");
  Serial.println(" l n - turn left motor for n steps backward.");
  Serial.println(" R n - turn right motor for n steps forward.");
  Serial.println(" r n - turn right motor for n steps backward.");
  Serial.println(" c - calibrate motor speeds.");
  Serial.println(" d - print distance reading.");
  Serial.println(" w - print states of wheel sensors.");
  Serial.println(" v - print battery voltage.");
  Serial.println(" i - print illumination voltage.");
  Serial.println();
}

void setup() 
{
  Serial.begin(9600);
  Serial.println("begin");
  
  pinMode(rRIN, OUTPUT);
  pinMode(rFIN, OUTPUT);
  pinMode(lRIN, OUTPUT);
  pinMode(lFIN, OUTPUT);
  pinMode(rSense, INPUT_PULLUP);
  pinMode(lSense, INPUT_PULLUP);
  pinMode(aSelect, OUTPUT);
  pinMode(aRead, INPUT);

  RightMotor(stop);
  LeftMotor(stop);
  
  attachInterrupt(digitalPinToInterrupt(rSense), RightInterupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(lSense), LeftInterupt, FALLING);
  
  SetupLIDAR();
 
  leftCount = 0;
  rightCount = 0;
  leftTime = 0;
  rightTime = 0;
  lStop = false;
  rStop = false;
  lfPWM = 255;
  lbPWM = 255;
  rfPWM = 255;
  rbPWM = 255;
  
  Prompt();
  
  Serial.print("Command: ");  
}

void CalibrateMotors()
{

}


void loop()
{
  char c;
  long n;

  // Has the user typed a command?
  
  c = 0;
  if (Serial.available() > 0) 
      c = (char)Serial.read();
  if(c == '\n' || c == '\r' || c == ' ') // Throw away white space
    c = 0;
      
  if(c)
  {
    Serial.println(c);
    
    switch(c)
    {
      case 'L':
        n = Serial.parseInt();
        RunLeft(n, forward);
        break;
  
     case 'l':
        n = Serial.parseInt();
        RunLeft(n, backward);
        break;
  
     case 'R':
        n = Serial.parseInt();
        RunRight(n, forward);
        break;
  
     case 'r':
        n = Serial.parseInt();
        RunRight(n, backward);
        break;

     case 'c':
        CalibrateMotors();
        break;
  
     case 'd':
        PrintDistance();
        break;
  
     case 'w':
        PrintWheels();
        break;
  
     case 'v':
        PrintBattery();
        break;
  
     case 'i':
        PrintIllumination();
        break; 
    
      case 0:
        break;
  
      default:
        Serial.println("??");
        Prompt();
        break;
    }
    Serial.print("Command: ");
  }

  // Check if an interrupt function has stopped a motor and,
  // if so, report how long the motor turned.
  
  if(lStop)
  {
    LeftStopped();
    Serial.print("Command: ");
  }
    
  if(rStop)
  {
    RightStopped();
    Serial.print("Command: ");
  }
}


