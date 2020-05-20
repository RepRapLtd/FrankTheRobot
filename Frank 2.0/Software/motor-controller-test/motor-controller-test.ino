const int ain1 = 4;
const int ain2 = 2;
const int apwm = 3;

void setup() 
{
  Serial.begin(9600);
  Serial.println("Motor controller test");
  pinMode(ain1, OUTPUT);
  digitalWrite(ain1, LOW);
  pinMode(ain2, OUTPUT);
  digitalWrite(ain2, LOW);
  pinMode(apwm, OUTPUT);
  analogWrite(apwm, 0);
  Prompt();
}

void Prompt()
{
   Serial.println("AIN1 AIN2 APWM");  
}

void loop() 
{
  int a1 = 0;
  int a2 = 0;
  int ap = 0;
  
  if(Serial.available() > 0)
  {
    a1 = Serial.parseInt();
    a2 = Serial.parseInt();
    ap = Serial.parseInt();
    Serial.print(a1);
    Serial.print(' ');
    Serial.print(a2);
    Serial.print(' ');
    Serial.println(ap);
    digitalWrite(ain1, a1);
    digitalWrite(ain2, a2);
    analogWrite(apwm, ap);            
    Prompt();
  }
}
