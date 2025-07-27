
void setup() {
  Serial.begin(115200); // 115200 baud
  analogReference(INTERNAL); // 1.1V reference
  ADCSRA = (ADCSRA & 0xF8) | 0x04; // Prescaler = 32 (~38.5 kHz)
  DIDR0 |= (1 << ADC0D); // Disable digital input on A0
}

void loop() {
  int sample = analogRead(A0); // Read A0 (10-bit, 0–1.1V)
  static int count = 0;
    Serial.write((uint8_t)(sample >> 8)); // High byte
    Serial.write((uint8_t)sample); // Low byte
  
  
}