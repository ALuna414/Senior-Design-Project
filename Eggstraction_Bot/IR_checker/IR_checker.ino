const int LED_START_INDICATOR = 7;
const int SWITCH_0 = 2;
const int SWITCH_1 = 17; //A3
const int BUTTON_START = 4;

//IR 
const int pinIRa = A1;
const int pinIRb = A0;
const int SIZE = 5;
int IRa_last_readings[SIZE];
int IRb_last_readings[SIZE];
int IRaIndex = 0;
int IRbIndex = 0;

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
//part of the setup is how upon start, the bot will wait 5 seconds and then start moving. 
//During these 5 seconds, the initial_readings array will be filled with values for the red square

void handleIR();

void setup()
{
  Serial.begin(115200);

  pinMode(pinIRa, INPUT);
  pinMode(pinIRb, INPUT);
  
  for (int i = 0; i < SIZE; i++) {
    IRa_last_readings[i] = 0;
    IRb_last_readings[i] = 0;
  }

  for (int i = 0; i < 5; i++) {
    initial_readingsA[i] = analogRead(pinIRa);
    initial_readingsB[i] = analogRead(pinIRb);

    if (initial_readingsA[i] > maxInitialValueA) {
      maxInitialValueA = initial_readingsA[i]; // Find the maximum value from initial readings for sensor A
    }

    if (initial_readingsB[i] > maxInitialValueB) {
      maxInitialValueB = initial_readingsB[i]; // Find the maximum value from initial readings for sensor B
    }

    delay(500); // Delay between readings
  }

  // Calculate thresholds based on the maximum initial value
  minThresholdA = maxInitialValueA * 1.05;
  maxThresholdA = maxInitialValueA * 2;  
  
  minThresholdB = maxInitialValueB * 1.1;
  maxThresholdB = maxInitialValueB * 2;
// 


  Serial.println("Thresholds for Sensor A:");
  Serial.print("Min: ");
  Serial.println(minThresholdA);
  Serial.print("Max: ");
  Serial.println(maxThresholdA);

  Serial.println("Thresholds for Sensor B:");
  Serial.print("Min: ");
  Serial.println(minThresholdB);
  Serial.print("Max: ");
  Serial.println(maxThresholdB);
  
}

void loop()
{
  IRvalueA = analogRead(pinIRa);
  IRvalueB = analogRead(pinIRb);
  
  IRa_last_readings[IRaIndex] = IRvalueA;
  IRb_last_readings[IRbIndex] = IRvalueB;

  IRaIndex = (IRaIndex + 1) % SIZE;
  IRbIndex = (IRbIndex + 1) % SIZE;

  float avg_A = 0;
  float avg_B = 0;

  for (int i = 0; i < SIZE; i++){
    avg_A += IRa_last_readings[i];
    avg_B += IRb_last_readings[i];
  }

  avg_A /= SIZE;
  avg_B /= SIZE;

  Serial.print("left sensor: ");
  Serial.print(IRvalueA); Serial.print("\n");
  Serial.print("left sensor AVERAGE: ");
  Serial.print(avg_A); Serial.print("\nleft sensor sees: ");

  if (isGray(avg_A, minThresholdA, maxThresholdA)) {
    Serial.print("gray\n");
  } else if (isBlack(avg_A, maxThresholdA)) {
    Serial.print("black\n");
  } else {
    Serial.print("red\n");
  }

  Serial.print("right sensor: ");
  Serial.print(IRvalueB); Serial.print("\n");
  Serial.print("right sensor AVERAGE: ");
  Serial.print(avg_B); Serial.print("\nright sensor sees: ");
  if (isGray(avg_B, minThresholdB, maxThresholdB)) {
    Serial.println("gray\n");
  } else if (isBlack(avg_B, maxThresholdB)) {
    Serial.println("black\n");
  } else {
    Serial.println("red\n");
  }

  delay(250);
}

bool isGray(int value, float minThreshold, float maxThreshold) {
  return (value >= minThreshold && value <= maxThreshold);
}

bool isBlack(int value, float threshold) {
  return (value > threshold);
}
