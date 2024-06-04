struct __attribute__((packed)) UNO_TX
{
  bool grabber_signal;
  float motor_control_linear;
  float motor_control_angular;
} unoTX;

void setup() {
  Serial.begin(115200);

  Serial.print("int:\t"); Serial.println(sizeof(int));
  Serial.print("long:\t"); Serial.println(sizeof(long));
  Serial.print("float:\t"); Serial.println(sizeof(float));
  Serial.print("double:\t"); Serial.println(sizeof(double));
  Serial.print("bool:\t"); Serial.println(sizeof(bool));
  Serial.print("unoTX: "); Serial.println(sizeof(UNO_TX));

}

void loop() {
  // put your main code here, to run repeatedly:

}
