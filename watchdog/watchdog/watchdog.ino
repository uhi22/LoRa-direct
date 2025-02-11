#define PIN_RESET_OUT 4

void setup(void) {
  pinMode(PIN_RESET_OUT, OUTPUT);
  digitalWrite(PIN_RESET_OUT, 1);
  Serial.begin(115200);
}

void loop(void) {
  uint16_t i;
  for (i=300; i>0; i--) {
      Serial.println(i);
      delay(1000);
  }
  digitalWrite(PIN_RESET_OUT, 0);
  delay(1000);
  digitalWrite(PIN_RESET_OUT, 1);
}
