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
const int stopped = 0;
const int forward = 1;
const int backward = 2;

int pwm = 200;
int state = stopped;

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
  }
  return vDiff;  
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
  int v = OptoReading(true); 
  delay(30);
  if(Serial.available() > 0)
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
        break;
      case '\n':
        break;
      default:
        Serial.println("Valid inputs: f, b, s, pnnn.");
    }
  }
}
