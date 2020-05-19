const int modulatePin = 13;
const int sensePin = A0;

void setup() 
{
  Serial.begin(9600);
  Serial.println("Opto sensor test");
  pinMode(modulatePin, OUTPUT);
  digitalWrite(modulatePin, LOW);
  pinMode(sensePin, INPUT);
}

void PrintInt(int v)
{
    if(v < 0) v = 0;
    if (v < 100) Serial.print('0');
    if (v < 10) Serial.print('0');
    Serial.print(v);
    Serial.print(' ');  
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

void loop() 
{
  int v = OptoReading(true); 
  delay(30);
}
