/*
 * 
 * 470 ohm -> 13
 * 10K -> +5
 * two centre pins -> GND
 * collector -> A0 
 * 
 */



const int modulatePin = 13;
const int sensePin = A0;
const int pwmA = 5;
const int pwmB = 6;
const int factor255 = 17;
const int stabiliseTime = 100;
const long sampleTime = 5000;

enum condition {stopped, forward, backward};
condition state = stopped;

enum stage {inactive, fStarting, fRising, fFalling, fStopping, rStarting, rRising, rFalling, rStopping};
stage test = inactive;
long testTime;
bool sensor = false;
int pulseCount = 0;
bool gRun = false;


int pwm = 200;
int oldPWM = 200;
bool graphLive = false;
long graphTime = 0;
int trough = 1024;
int peak = 0;

void PrintInt(int v)
{
    if(v < 0) v = 0;
    if (v < 100) Serial.print('0');
    if (v < 10) Serial.print('0');
    Serial.print(v);
    Serial.print(' ');  
}

void Forward()
{
  analogWrite(pwmA, 0);
  analogWrite(pwmB, pwm);
  state = forward;
}

void Backward()
{
  analogWrite(pwmB, 0);
  analogWrite(pwmA, pwm);
  state = backward;
}

void Stop()
{
  analogWrite(pwmA, 0);
  analogWrite(pwmB, 0);
  state = stopped;
}

int OptoReading(bool printBar)
{
  int vDark = analogRead(sensePin);
  digitalWrite(modulatePin, HIGH);
  int vLight = analogRead(sensePin);
  digitalWrite(modulatePin, LOW);
  int vDiff = vDark - vLight;
  if(printBar)
  {
    PrintInt(vLight);
    PrintInt(vDiff);
    int stars = vDiff/10;
    for (int i = 0; i < stars; i++)
      Serial.print('*');
    Serial.println();
    if(vDark > peak)
      peak = vDark;
    if(vLight < trough)
      trough = vLight;    
  }
  return vDiff;  
}

void Prompt()
{
  Serial.println("\nCommands:");
  Serial.println(" f: Go forwards.");
  Serial.println(" b: Go backwards.");
  Serial.println(" s: Stop.");
  Serial.println(" pNNN: Set PWM to NNN.");
  Serial.println(" g: Plot sensor readings for 10 seconds and set peak and trough values.");
  Serial.println(" c: Output PWM/RPM curves.\n");
}

void Timings()
{
  if(graphLive)
  {
    long t = millis();
    if(t - 10000 >= graphTime)
    {
      graphLive = false;
      Serial.print("\nTrough (light): ");
      Serial.print(trough);
      Serial.print(", Peak (dark): ");
      Serial.print(peak);
      Serial.println('\n');
    }       
  }
}


bool Trigger()
{
  bool s = sensor;
  int reading = OptoReading(false);
  if(reading < (peak - trough)/3)
    sensor = false;
  if(reading > ((peak - trough)*2)/3)
    sensor = true;
  return s ^ sensor; 
}

void TurnOnOrOff()
{
  switch (state)
  {
    case forward:
      Forward();
      break;
    case backward:
      Backward();
      break;
    case stopped:
      Stop();
      break;
    default:
      ;
  }
  delay(stabiliseTime);
  testTime = millis();
}

bool TestAtSpeed()
{
  if(Trigger())
    pulseCount++;
  long t = millis();
  if(t - testTime > sampleTime)
  {
    float rpm = 60000.0*(float)pulseCount/((float)sampleTime*24.0);
    Serial.print(' ');
    Serial.print(pwm);
    Serial.print(", ");
    Serial.println(rpm);
    testTime = t;
    pulseCount = 0;
    return true;
  }
  return false;
}

void Test()
{
  switch(test)
  {
    case fStarting:
      oldPWM = pwm;
      Serial.println("\nPWM/speed test");
    case rStarting:
      pwm = 0;
      Stop();
      delay(stabiliseTime);
      if(test == fStarting)
      {
        Forward();
        Serial.print("Forward");
        test = fRising;
      } else
      {
        Backward();
        Serial.print("Backward");
        test = rRising;
      }
      Serial.println(" rising curve:");
      Serial.println(" PWM, RPM");
      TurnOnOrOff();
      pulseCount = 0;
      Trigger();      
      break;

    case fRising:
    case rRising:
      if(TestAtSpeed())
      {
        pwm += factor255;
        if(pwm > 255)
        {
          pwm = 255;
          Serial.println(" falling curve:");
          Serial.println(" PWM, RPM");
          if(test == fRising)
            test = fFalling;
          else
            test = rFalling;
        } else
          TurnOnOrOff();
      }
      break;


    case fFalling:
    case rFalling:
      if(TestAtSpeed())
      {
        pwm -= factor255;
        if(pwm < 0)
        {
          pwm = 0;
          Serial.println(" falling curve:");
          Serial.println(" PWM, RPM");
          if(test == fRising)
            test = fFalling;
          else
            test = rFalling;
        } else
          TurnOnOrOff();
      }
      break;

    case fStopping:
      test = rStarting;
      break;
      
    case rStopping:
      pwm = oldPWM;
      test = inactive;
      break;

    default:
    ;
  }
}

void setup() 
{
  Serial.begin(9600);
  Serial.println("Opto sensor test");
  pinMode(modulatePin, OUTPUT);
  digitalWrite(modulatePin, LOW);
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  Stop();
  pinMode(sensePin, INPUT);
}

void loop() 
{
  Timings();

  if(test != inactive)
  {
    Test();
  }else if(graphLive)
  {
    int v = OptoReading(true); 
    delay(30);
  } else if(Serial.available() > 0)
  {
    int c = Serial.read();
    switch(c)
    {
      case 'f':
        Forward();
        break;
      case 'b':
        Backward();
        break;
      case 's':
        Stop();
        break;
      case 'p':
        pwm = Serial.parseInt();
        TurnOnOrOff();
        break;
      case 'g':
        graphLive = true;
        trough = 1024;
        peak = 0;
        graphTime = millis();
        gRun = true;
        break;
      case 'c':
        if(gRun)
          test = fStarting;
        else
          Serial.println("You need to run a g test before a c test to set the opto range.");
        break;
      case '\n':
        break;
      default:
        Serial.print("\nUnrecognised command: ");
        Serial.println((char)c);
        Prompt();
    }
  }
}
