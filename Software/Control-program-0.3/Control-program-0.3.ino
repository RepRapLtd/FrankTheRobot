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

const float rateLimit = 0.25;
const uint32_t timingBudget = 100000;
const uint8_t periodPrePclks = 18; // Must be even
const uint8_t periodFinalPclks = 14; // Must be even
const int defaultDistance = 2000;
const int lidarTimeout = -1;
const int lidarResetNeeded = -2;

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

Stream* Output(WiFiClient* client)
{
  if(client)
  {
    return client;
  } else
  {
    return (Stream *)(&Serial);
  }  
}

void Endline(WiFiClient* client)
{
  if(client)
  {
    client->println("<br>");
  } else
  {
    Serial.println();
  }    
}

void PrintSpace(WiFiClient* client)
{
  if(client)
  {
    client->print("&nbsp;");
  } else
  {
    Serial.print(' ');
  }    
}

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

int LIDARDistance()
{
  int r = (int)sensor.readRangeContinuousMillimeters();
  if (sensor.timeoutOccurred())
  {
    return lidarTimeout;
  }
  if(r > 7500)
  {
    if(r > 9000)
      return lidarResetNeeded;
    r = defaultDistance;
  }
  return r;  
}

void PrintDistance(WiFiClient* client)
{
  Stream *op = Output(client);
  int r = LIDARDistance();
  switch(r)
  {
    case lidarTimeout:
      op->print("LIDAR timed out.");
      break;

    case lidarResetNeeded:
      op->print("LIDAR reset needed.");
      break;

    default:
      op->print("LIDAR distance (mm): ");
      op->print(r);
  }
  Endline(client); 
}

void PrintWheels(WiFiClient* client)
{
  Stream *op = Output(client);
  op->print("L, R wheel sensors: ");
  op->print(digitalRead(lSense));
  op->print(", ");
  op->print(digitalRead(rSense));
  Endline(client);
}

void PrintIllumination(WiFiClient* client)
{
  Stream *op = Output(client);  
  digitalWrite(aSelect,photo);
  int v = analogRead(aRead);
  op->print("Illumination voltage: ");
  op->print(v);
  Endline(client); 
}

void PrintBattery(WiFiClient* client)
{
  Stream *op = Output(client); 
  digitalWrite(aSelect,voltage);
  int v = analogRead(aRead);
  op->print("Battery voltage: ");
  op->print(v);
  Endline(client); 
}

void PrintStatus(WiFiClient* client)
{
  Stream *op = Output(client);
  Endline(client);
  op->print("Status:");
  PrintSpace(client); 
  PrintDistance(client);
  PrintSpace(client);
  PrintWheels(client);
  PrintSpace(client);
  PrintIllumination(client);
  PrintSpace(client);
  PrintBattery(client);
  Endline(client);
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


void WiFiResponse(WiFiClient* client)
{
  // Return the response
  client->println("HTTP/1.1 200 OK");
  client->println("Content-Type: text/html");
  client->println(""); //  do not forget this one
  client->println("<!DOCTYPE HTML>");
  client->println("<html>\n<br>");

  PrintStatus(client);
  
  client->println("<br><br><a href=\"/frankc10\">Turn clockwise 10 steps</a> <br>");
  client->println("<a href=\"/franka10\">Turn anticlockwise 10 steps</a> <br>");  
  client->println("<a href=\"/frankf10\">Move forward 10 steps</a> <br>");
  client->println("<a href=\"/frankr10\">Move backwarward 10 steps</a> <br>");
  client->println("</html>");  
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
  Serial.println(" W - setup WiFi.");
  Serial.println(" v - print battery voltage.");
  Serial.println(" i - print illumination voltage.");
  Serial.println(" D - initialise LIDAR.");
  Serial.println(" s - stop motors.");  
  Serial.println();
}

int GetInteger(String request, WiFiClient* client)
{
  int n= request.substring(1).toInt();
  if(!client)
  {
    Serial.print(n);
  } 
}

void Interpret(String request, WiFiClient* client)
{
  int n;

  char c = request.charAt(0);
  
  switch(c)
  {
    case 'L':
      n = GetInteger(request, client);
      RunLeft(n, forward);
      break;

   case 'l':
      n = GetInteger(request, client);
      RunLeft(n, backward);
      break;

   case 'R':
      n = GetInteger(request, client);
      RunRight(n, forward);
      break;

   case 'r':
      n = GetInteger(request, client);
      RunRight(n, backward);
      break;

   case 'c':
      n = GetInteger(request, client);
      RunRight(n, forward);
      RunLeft(n, backward);
      break;

   case 'a':
      n = GetInteger(request, client);
      RunRight(n, backward);
      RunLeft(n, forward);
      break;

   case 'f':
      n = GetInteger(request, client);
      RunRight(n, forward);
      RunLeft(n, forward);
      break;
      
   case 'b':
      n = GetInteger(request, client);
      RunRight(n, backward);
      RunLeft(n, backward);
      break;

   case 'W':
      SetupWiFi();
      break;

   case 'd':
      PrintDistance(client);
      break;

   case 'w':
      PrintWheels(client);
      break;

   case 'v':
      PrintBattery(client);
      break;

   case 'i':
      PrintIllumination(client);
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
      if(!client)
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
  Interpret(request, 0);
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
    Interpret(request, &client);
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


