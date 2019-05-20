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
 * 
 * Additional programming:
 * 
 * George Newton Shaw
 * 29 June 2018
 *
 */

#include <Wire.h>
#include <VL53L0X.h>
#include <ESP8266WiFi.h>
#include "localwifi.h"

VL53L0X sensor;

#define WEMOS_R2 // Black PCB; no can

#ifdef WEMOS_R1
// Wemos D1 R1 "Arduino" pins

const char rRIN = D6; // Right motor pins
const char rFIN = D5; //

const char lRIN = D8; // Left motor pins
const char lFIN = D10; //

const char rSense = D9; // Wheel sensors NB: LED pin - LED pulls up?
const char lSense = D7; // (Input pulldown)

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

// Comms

WiFiServer server(80);
bool USB = true;
bool wifi = !USB;

// A->D

const char photo = false;    // Send to aSelect to read the photosensor
const char voltage = !photo; // Send to aSelect to read the battery voltage

// Motor directions, stop and motor control data

const char stop = 0;
const char forward = 1;
const char backward = 2;
volatile char leftDirection, rightDirection; // What each motor is doing
int lfPWM, lbPWM, rfPWM, rbPWM;    // PWM to control each motor speed (forward != backward in general)
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

void SetupWiFi()
{
  Serial.print("\nConnecting to ");
  Serial.print(ssid);
 
  WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(" .");
  }
  Serial.println("\nWiFi connected");
 
  // Start the server
  server.begin();
  Serial.println("Server started");
 
  // Print the IP address
  Serial.print("Use this URL : ");
  Serial.print("http://");
  Serial.print(WiFi.localIP());
  Serial.println("/");
}

// Control the leftmotor

void LeftMotor(char direction)
{
  switch(direction)
  {
    case forward:
      digitalWrite(lRIN, LOW);
      analogWrite(lFIN, lfPWM);
      //digitalWrite(lFIN, HIGH);
      break;
           
    case backward:
      digitalWrite(lFIN, LOW);
      analogWrite(lRIN, lbPWM);
      //digitalWrite(lRIN, HIGH);
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
      //digitalWrite(rFIN, HIGH);
      break;
           
    case backward:
      digitalWrite(rFIN, LOW);
      analogWrite(rRIN, rbPWM);
      //digitalWrite(rRIN, HIGH);
      break;
      
    case stop:
    default:
      digitalWrite(rRIN, HIGH);
      digitalWrite(rFIN, HIGH);
  }

  // Remember what we're doing
  
  rightDirection = direction;  
}


void PrintDistance(bool usb)
{
  Serial.print("LIDAR distanc (mm): ");
  Serial.println(LIDARDistance());  
}

void PrintWheels(bool usb)
{
  Serial.print("L, R wheel sensors: ");
  Serial.print(digitalRead(lSense));
  Serial.print(", ");
  Serial.println(digitalRead(rSense));  
}

void PrintIllumination(bool usb)
{
  digitalWrite(aSelect,photo);
  int v = analogRead(aRead);
  Serial.print("Illumination voltage: ");
  Serial.println(v);  
}

void PrintBattery(bool usb)
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

void StopMotors()
{
  RightMotor(stop);
  LeftMotor(stop);
  leftCount = 0;
  rightCount = 0;
  leftTime = 0;
  rightTime = 0;
  lStop = false;
  rStop = false;
}


void WiFiResponse(WiFiClient &client)
{
  // Return the response
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println(""); //  do not forget this one
  client.println("<!DOCTYPE HTML>");
  client.println("<html>");
 
  client.print("Led pin is now: ");
 
  if(value == HIGH) {
    client.print("On");  
  } else {
    client.print("Off");
  }
  client.println("<br><br>");
  client.println("Click <a href=\"/LED=ON\">here</a> turn the LED on pin 5 ON<br>");
  client.println("Click <a href=\"/LED=OFF\">here</a> turn the LED on pin 5 OFF<br>");
  client.println("</html>");  
}



// Remind the user what can be done...

void Prompt()
{
  Serial.println("\nCommands:");
  Serial.println(" L n - turn left motor for n steps forward.");
  Serial.println(" l n - turn left motor for n steps backward.");
  Serial.println(" R n - turn right motor for n steps forward.");
  Serial.println(" r n - turn right motor for n steps backward.");
  Serial.println(" c n - clockwise for n steps.");
  Serial.println(" a n - anticlockwise for n steps.");
  Serial.println(" f n - forward for n steps.");
  Serial.println(" r n - reverse for n steps.");
  Serial.println(" d - print distance reading.");
  Serial.println(" w - print states of wheel sensors.");
  Serial.println(" v - print battery voltage.");
  Serial.println(" i - print illumination voltage.");
  Serial.println(" D - initialise LIDAR.");
  Serial.println(" s - stop motors.");  
  Serial.println();
}

int GetInteger(bool usb)
{
  int n;
  if(usb)
  {
    n = Serial.parseInt();
    Serial.print(n);
  } else
  {
    n = 1;
  }
  return n;
}

void Interpret(String request, bool usb)
{
  int n;

    // Match the request
 
  int value = LOW;
  if (request.indexOf("/LED=ON") != -1) {
    digitalWrite(ledPin, HIGH);
    value = HIGH;
  } 
  if (request.indexOf("/LED=OFF") != -1){
    digitalWrite(ledPin, LOW);
    value = LOW;
  }
  
  switch(c)
  {
    case 'L':
      n = GetInteger(usb);
      RunLeft(n, forward);
      break;

   case 'l':
      n = GetInteger(usb);
      RunLeft(n, backward);
      break;

   case 'R':
      n = GetInteger(usb);
      RunRight(n, forward);
      break;

   case 'r':
      n = GetInteger(usb);
      RunRight(n, backward);
      break;

   case 'c':
      n = GetInteger(usb);
      RunRight(n, forward);
      RunLeft(n, backward);
      break;

   case 'a':
      n = GetInteger(usb);
      RunRight(n, backward);
      RunLeft(n, forward);
      break;

   case 'f':
      n = GetInteger(usb);
      RunRight(n, forward);
      RunLeft(n, forward);
      break;
      
   case 'b':
      n = GetInteger(usb);
      RunRight(n, backward);
      RunLeft(n, backward);
      break;

   case 'd':
      PrintDistance(usb);
      break;

   case 'w':
      PrintWheels(usb);
      break;

   case 'v':
      PrintBattery(usb);
      break;

   case 'i':
      PrintIllumination(usb);
      break; 
  
    case 0:
      break;

    case 'D':
      SetupLIDAR();
      break;

    case 's':
      StopMotors();
      break;
      

    default:
      if(usb)
      {
        Serial.println("??");
        Prompt();
      }
      break;
  }  
}


void GetUSB()
{
  if(!Serial.available())
  {
    return;
  }
  String request = Serial.readStringUntil('\n');
  Interpret(request, USB);
}

void GetWiFi()
{
  WiFiClient client = server.available();
  if (!client) 
  {
    return;
  }
 
  while(!client.available())
  {
    delay(1);
  }
 
  // Read the first line of the request
  String request = client.readStringUntil('\r');
  client.flush();
  Serial.println(request);
  int r = request.lastIndexOf("frank");
  if(r < 0)
  {
    Serial.println("\nUnexpected request");
  } else
  {
    request = request.substring(r + 5);
    Interpret(request, wifi);
  }

  delay(1);
}

void setup() 
{
  Serial.begin(9600);
  Serial.println("Frank control program");
  
  pinMode(rRIN, OUTPUT);
  pinMode(rFIN, OUTPUT);
  pinMode(lRIN, OUTPUT);
  pinMode(lFIN, OUTPUT);
  pinMode(rSense, INPUT_PULLUP);
  pinMode(lSense, INPUT_PULLUP);
  pinMode(aSelect, OUTPUT);
  pinMode(aRead, INPUT);

  StopMotors();
  
  attachInterrupt(digitalPinToInterrupt(rSense), RightInterupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(lSense), LeftInterupt, FALLING);

 // Max PWM is 1023
 
  lfPWM = 600;
  lbPWM = 600;
  rfPWM = 600;
  rbPWM = 600;
  
  Prompt();
  
  Serial.print("Command: ");  
}


void loop()
{
  // Check the two input streams:
  
  GetUSB();
  GetWiFi();

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


