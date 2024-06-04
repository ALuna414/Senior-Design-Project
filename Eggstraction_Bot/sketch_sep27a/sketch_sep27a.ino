//Circuit
// Arduino Uno  -->   TCRT5000
// 5v           --->   VCC
// Grnd         --->   Grnd
// A0           --->   A0
// D8           --->   D0


const int pinIRa = A0;
const int pinIRb = A1;
const int pinLED = 13;
int IRvalueA = 0;
int IRvalueB = 0;

void setup()
{
  Serial.begin(9600);
  pinMode(pinIRa,INPUT);
  pinMode(pinIRb,INPUT);
  pinMode(pinLED,OUTPUT);

}

void loop()
{
  Serial.print("Analog Reading (left) =");
  Serial.print(IRvalueA);
  Serial.print(", Analog Reading (right)=");
  Serial.println(IRvalueB);

  delay(100);
  
  IRvalueA = analogRead(pinIRa);
  IRvalueB = analogRead(pinIRb);


}
