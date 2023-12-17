void setup() {
  Serial1.begin(115200);
}

void loop() {
  if (Serial1.available()) {
    Serial.write(Serial1.read());
  }
}
