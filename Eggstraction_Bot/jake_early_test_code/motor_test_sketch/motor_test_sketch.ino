const int pinIRa = A0;
const int pinIRb = A1;
const int pinLED = 13;
const int in1 = 10;
const int in2 = 11;
const int in3 = 6;
const int in4 = 9;
const int en_a = 3;
const int en_b = 5;
bool debug = true;
int IRvalueA = 0;
int IRvalueB = 0;
int initial_readingsA[5];
int initial_readingsB[5];

bool dataOut[4] = {true, false, true, false}; // Initial state is red for both sensors

int maxInitialValueA = 0;
int maxInitialValueB = 0;
float minThresholdA = 0;  // Declare as global variables
float maxThresholdA = 0;
float minThresholdB = 0;
float maxThresholdB = 0;
int left = 0;
int right = 0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  pinMode(pinIRa, INPUT);
  pinMode(pinIRb, INPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(en_a, OUTPUT);
  pinMode(en_b, OUTPUT);
}
int motorsMapDeadzone(int pwm)
{
  if(pwm == 0)
    return 0;
    
  const int minPWM = 50;
  const int maxPWM = 255;
  bool neg = pwm < 0;
  if(neg)
    pwm *= -1;

  pwm = minPWM + pwm * ((maxPWM - minPWM) / (float)maxPWM);

  if(neg)
    pwm *= -1;

  return pwm;
}

void setMotors(int left, int right)
{
  left = motorsMapDeadzone(left);
  right = motorsMapDeadzone(right);
  
  // if(esp.rx_time_out_flag)
  // {
  //   left = 0;
  //   right = 0;
  // }
  if(left < 0)
  {
    digitalWrite(in4, LOW);
    analogWrite(in3, -left);
    Serial.println("left backwards");
  }
  else
  {
    digitalWrite(in3, LOW);
    analogWrite(in4, left);
    Serial.println("left forwards");
  }

  if(right < 0)
  {
    digitalWrite(in2, LOW);
    analogWrite(in1, -right);
    Serial.println("right backwards");
  }
  else
  {
    digitalWrite(in1, LOW);
    analogWrite(in2, right);
    Serial.println("right forwards");
  }
}

void serialMotorControl()
{
  switch(Serial.read())
  {
    case 'l':
    {
      left = Serial.parseInt();
      break;
    }
    case 'r':
    {
      right = Serial.parseInt();
      break;
    }
    case 'b':
    {
      right = left = Serial.parseInt();
      break;
    }
    // default:
    //   left = right = 0;
  }
  setMotors(left, right);
}

void loop() {
  Serial.print("left: "); Serial.print(left);
  Serial.print("\tright: "); Serial.println(right);
  Serial.println("enter new values - left(l), right(r), both(b): ");
  while(!Serial.available()){}

  serialMotorControl();
}
