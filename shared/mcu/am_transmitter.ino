// analogsound sending  
/*#define FRQ 7       // 1.6 MHz (OCR2A = 4)
#define ANTENNA PB3    // Pin 11 (OC2A)
const char *text="hey it is a cloduy day";
void setup() {
  // Timer2: CTC mode, toggle OC2A (PB3 / Pin 11)
  TCCR2A = (1 << COM2A0) | (1 << WGM21);
  TCCR2B = (1 << CS20);       // No prescaler
  OCR2A = FRQ;
  DDRB | (1 << ANTENNA);     // Enable output on PB3

  // Timer1: PWM for gating
  TCCR1A = (1 << WGM11) | (1 << WGM10);
  TCCR1B = (1 << WGM12) | (1 << CS10);
  TIMSK1 = (1 << OCIE1A) | (1 << TOIE1);

  // ADC: A0 analog input
  ADMUX = (1 << REFS1) | (1 << REFS0);
  ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADATE);
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1);
  DIDR0 = (1 << ADC0D);
}

ISR(TIMER1_OVF_vect) {
  uint8_t a=0;
 uint16_t adc = ADC;
  OCR1A = adc;
 // OCR1A=a;
  //a++;
  //if(a=255){a=0;}
  //int i=0;
  //OCR1A=text[i];
  //i++;
  //if(i==strlen(text)){i=0;}
  
  DDRB |= (1 << ANTENNA);   // Turn RF ON
}

ISR(TIMER1_COMPA_vect) {
  DDRB &= ~(1 << ANTENNA);  // Turn RF OFF
*
void loop() {}*/ 
/*
#define ANTENNA PB3 // Pin 11 (OC2A)
#define FRQ 7       // ~1.14 MHz carrier
const char *text = "hey it is a cloduy day";

void setup() {
  // Timer2: ~1.14 MHz carrier
  TCCR2A = (1 << COM2A0) | (1 << WGM21); // CTC, toggle OC2A
  TCCR2B = (1 << CS20); // No prescaler
  OCR2A = FRQ;
  DDRB |= (1 << ANTENNA); // Output pin

  / Timer1: 10 ms bit duration (100 bps)
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | (1 << CS12) | (1 << CS10); // CTC, prescaler 1024
  TIMSK1 = (1 << OCIE1A); // Enable compare interrupt
  OCR1A = 156; // 10 ms = 16MHz / 1024 * 0.01 ≈ 156
}

ISR(TIMER1_COMPA_vect) {
  static uint8_t i = 0, bit = 0;
  if (text[i] == 0) i = 0; // Loop text
  uint8_t value = (text[i] >> (7 - bit)) & 1; // Get bit
  if (value) {
    TCCR2A |= (1 << COM2A0); // Carrier ON for '1'
  } else {
    TCCR2A &= ~(1 << COM2A0); // Carrier OFF for '0'
  }
  bit++;
  if (bit == 8) {
    bit = 0;
    i++;
    TCCR2A &= ~(1 << COM2A0); // Gap between chars
  }
}

//morse code sender
void loop() {}*/
#define ANTENNA PB3 // Pin 11 (OC2A)
#define FRQ 8     // ~2 MHz carrier (16 MHz / (2 * (3 + 1)))
const char *morse = "... --- ..."; // SOS

void setup() {
  // Timer2: ~2 MHz carrier
  TCCR2A = (1 << COM2A0) | (1 << WGM21); // CTC, toggle OC2A
  TCCR2B = (1 << CS20); // No prescaler
  OCR2A = FRQ; // ~2 MHz
  DDRB |= (1 << ANTENNA); // Output pin
}

void send_tone(int duration_ms) {
  unsigned long start = millis();
  while (millis() - start < duration_ms) {
    TCCR2A |= (1 << COM2A0); // Carrier ON
    delayMicroseconds(500); // Half period (1 kHz = 500 µs)
    TCCR2A &= ~(1 << COM2A0); // Carrier OFF
    delayMicroseconds(500); // Half period
  }
}

void loop() {
  for (int i = 0; morse[i]; i++) {
    if (morse[i] == '.') {
      send_tone(120); // Dot: 1.2 s of 1 kHz tone
      delay(120); // Space: 1.2 s
    } else if (morse[i] == '-') {
      send_tone(360); // Dash: 3.6 s of 1 kHz tone
      delay(120); // Space: 1.2 s
    }
  }
  delay(360); // Word space: 3.6 s
}