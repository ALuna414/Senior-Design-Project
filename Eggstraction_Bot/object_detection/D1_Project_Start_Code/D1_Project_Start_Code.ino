#define CAMERA_MODEL_AI_THINKER
int LED_START_INDICATOR = 7;
int SWITCH_0 = 2;
int SWITCH_1 = 3;
int BUTTON_START = 4;
bool start_light = false;
String ReadColorButtons() {
  int color_bit0 = digitalRead(SWITCH_0);
  int color_bit1 = digitalRead(SWITCH_1);
  if (color_bit1 == 0 && color_bit0 == 0) {
    Serial.println("COLOR #1 Was chosen: 00");
    //if (digitalRead(BUTTON_START) == HIGH)
    return "COLOR-1";
  }
  if (color_bit1 == 0 && color_bit0 == 1) {
    Serial.println("COLOR #2 Was chosen: 01");
    //if (digitalRead(BUTTON_START) == HIGH)
    return "COLOR-2";
  }
  if (color_bit1 == 1 && color_bit0 == 0) {
    Serial.println("COLOR #3 Was chosen: 10");
    //if (digitalRead(BUTTON_START) == HIGH)
    return "COLOR-3";
  }
  if (color_bit1 == 1 && color_bit0 == 1) {
    Serial.println("COLOR #4 Was chosen: 11");
    //if (digitalRead(BUTTON_START) == HIGH)
    return "COLOR-4";
  }
  return ""; // Add a default return statement to avoid compilation warning
}

void BlinkStartLED() {
  Serial.println("Blinking START LED!");
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_START_INDICATOR, HIGH);
    delay(500);
    digitalWrite(LED_START_INDICATOR, LOW);
    delay(500);
  }
  //digitalWrite(LED_START_INDICATOR, HIGH);
}

void StartSequence() {
  String color_target = ReadColorButtons();
  Serial.print("The color returned by the ReadColorButtons() Function:");
  Serial.println(color_target);
  if (start_light)
    BlinkStartLED();
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_START_INDICATOR, OUTPUT);
  pinMode(SWITCH_0, INPUT);
  pinMode(SWITCH_1, INPUT);
  pinMode(BUTTON_START, INPUT);
  digitalWrite(LED_START_INDICATOR, LOW);
}

void loop() {
  StartSequence();
  if (digitalRead(BUTTON_START) == HIGH)
    start_light = true;
}
