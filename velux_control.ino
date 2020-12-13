#define BIT_DURATION 450 // us
#define SEQUENCE_PAUSE 30 // ms
#define SENSOR_PIN PB3
#define LED_PIN PB1
#define POTI_PIN PB2 //Changing this is not enough to change the analog input pin. Also change the corresponing section in setup()
#define UNSIGNED_LONG_MAX_VALUE 4294967295
#define SIGNAL_DURATION 20 // ms
#define NUM_BITS_VELUX 24
#define NUM_BITS_SEQUENCE 21
#define MAX_SIGNAL_SIZE 45
// #define DEBUG

// define sequences as long integers but in reverse bit order so that first bit in the sequence
// can be set with 1 << 0 and the consecutive ones with 1 << 1...num_bits
#define SEQUENCE_VELUX 15263976 // 000101110001011100010111 00000000
#define SEQUENCE_OPEN 585454    // 011101110111011100010 00000000000
#define SEQUENCE_CLOSE 978574   // 011100010111011101110 00000000000

// define minimum and maximum close delay time in ms
#define MINIMUM_CLOSE_DELAY 300000 // 5 min
#define MAXIMUM_CLOSE_DELAY 1800000 // 0.5 h

#define SENSOR_PIN_MASK (1<<SENSOR_PIN);

unsigned long signal_start;

unsigned long sequence_velux = 0;
unsigned long sequence_part_two = 0;
byte sequence[MAX_SIGNAL_SIZE];
unsigned long sequence_times[MAX_SIGNAL_SIZE];
unsigned long timer_start_time;
unsigned long close_delay = 60000;
bool timer_started = false;


void setup() {
  // Set all pins to input with pullup enabled to make sure we don't have any dangling pins
  DDRB = 0b0;
  PORTB = 0b00111111;
  // Now set the pins we use to the mode they're needed
  // LED pin should be an output and off by default
  DDRB |= (1<<LED_PIN);
  PORTB &= ~(1<<LED_PIN);
  // Sensor pin stays input, but disable pullup
  // PORTB &= ~(1<<SENSOR_PIN);
  //  // Set voltage reference to use AREF pin (PB0)
  //  ADMUX |= (1<<REFS0);
  //  ADMUX &= ~(1<<REFS1);
  //  // Use pin PB2 as analog input pin
  //  ADMUX |= (1<<MUX0);
  //  ADMUX &= ~(1<<MUX1);
  //  ADMUX &= ~(1<<MUX2);
  //  ADMUX &= ~(1<<MUX3);
  //  // Left adjust analog data (so we only get 8bit resolution
  //  ADMUX |= (1<<ADLAR);
  // Combine all of that into one command
  ADMUX = 0b01100001;
  // Enable the ADC
  ADCSRA |= (1<<ADEN);
  // Enable auto trigger
  ADCSRA |= (1<<ADATE);
  // Set prescaler to 128, we don't need fast conversions, so we can use the slowest ADC clock available
  ADCSRA |= (1<<ADPS2);
  ADCSRA |= (1<<ADPS1);
  ADCSRA |= (1<<ADPS0);
  // Set ADC to free running mode
  ADCSRB = 0b0;
  // Disable digital input on our ADC pin
  DIDR0 |= (1<<ADC1D);
  // Finally set the "start conversion" bit to start free running mode
  ADCSRA |= (1<<ADSC);
  

// For debugging when running on an Arduino it can be useful to have some serial output and allow some
// commands to be sent via Serial. On Attiny Serial is not available, so don't inlcude it there
#ifdef Serial
  Serial.begin(115200);
#endif

}

void loop() {
  // put your main code here, to run repeatedly:
#ifdef Serial
  if (Serial.available()) {
    char cmd = Serial.read();

    switch (cmd) {
      case 'O': send_open();
#ifndef DEBUG
                Serial.write('O');
#endif
                break;
      case 'C': send_close();
#ifndef DEBUG
                Serial.write('C');
#endif
                break;
      case 'D': float delay_time = Serial.parseFloat();
                if (delay_time > 0) {
                  close_delay = (unsigned long) delay_time;
                }
                Serial.print(close_delay); Serial.println("D");
      // case 'R': print_last_received_sequence();
    }
  }
#endif

  int val = PINB & SENSOR_PIN_MASK;
  if (val == 0) {
    signal_start = millis();
    sense(val);
    
#ifdef DEBUG
    Serial.println("Received command sequence.");
    for (int i=0; i<MAX_SIGNAL_SIZE; i++) {
      Serial.print(sequence[i]);
    }
    Serial.println("");
    for (int i=0; i<MAX_SIGNAL_SIZE; i++) {
      Serial.println(sequence_times[i]);
    }
    Serial.println("");
#endif
    decode_recorded_sequence();
#ifdef DEBUG
    Serial.print("Decoded Velux part: "); Serial.println(sequence_velux);
    Serial.print("Decoded part 2: "); Serial.println(sequence_part_two);
#endif
    if (sequence_velux == SEQUENCE_VELUX && sequence_part_two == SEQUENCE_OPEN) {
      delay(SEQUENCE_PAUSE + SIGNAL_DURATION); // Pause so that the repetition of the signal sent by the controller does not disturb us
      timer_start_time = millis();
      timer_started = true;
#ifdef DEBUG
      Serial.println("Starting timer.");
#endif
    }
  }

  if (timer_started) {
    unsigned long now = millis();
    unsigned long elapsed;

    byte analog_data = ADCH;
    close_delay = (MAXIMUM_CLOSE_DELAY - MINIMUM_CLOSE_DELAY) / 256.0 * analog_data + MINIMUM_CLOSE_DELAY;

    // Switch on the timer LED
    PORTB |= (1<<LED_PIN);  // digitalWrite(LED_PIN, HIGH);
    
    if (now < timer_start_time) {
      elapsed = now + UNSIGNED_LONG_MAX_VALUE - timer_start_time;
    } else {
      elapsed = now - timer_start_time;
    }

    if (elapsed > close_delay) {
      timer_started = false;
      send_close();
      // Switch off timer LED
      PORTB &= ~(1<<LED_PIN);
    }
  }
}

const unsigned long one = 1;

void send_bits(unsigned long bits, int num_bits) {
  for (unsigned long i=0; i < num_bits; i++) {
    if (bits & one << i) {
      PORTB |= (1<<SENSOR_PIN);  // digitalWrite(SENSOR_PIN, HIGH);
#ifdef DEBUG
      Serial.write('1');
#endif      
    } else {
      PORTB &= ~(1<<SENSOR_PIN);  // digitalWrite(SENSOR_PIN, LOW);
#ifdef DEBUG
      Serial.write('0');
#endif
    }
    delayMicroseconds(BIT_DURATION);
  }
}

void send_open() {
  DDRB |= (1<<SENSOR_PIN);  // pinMode(SENSOR_PIN, OUTPUT);
  PORTB |= (1<<SENSOR_PIN);  // digitalWrite(SENSOR_PIN, HIGH);
  send_bits(SEQUENCE_VELUX, NUM_BITS_VELUX);
  send_bits(SEQUENCE_OPEN, NUM_BITS_SEQUENCE);
  PORTB |= (1<<SENSOR_PIN);  // digitalWrite(SENSOR_PIN, HIGH);
  delay(SEQUENCE_PAUSE);
  send_bits(SEQUENCE_VELUX, NUM_BITS_VELUX);
  send_bits(SEQUENCE_OPEN, NUM_BITS_SEQUENCE);
  PORTB |= (1<<SENSOR_PIN);  // digitalWrite(SENSOR_PIN, HIGH);
  DDRB &= ~(1<<SENSOR_PIN);  // pinMode(SENSOR_PIN, INPUT);
  PORTB |= (1<<SENSOR_PIN);  // digitalWrite(SENSOR_PIN, HIGH);
  
}

void send_close() {
  DDRB |= (1<<SENSOR_PIN);  // pinMode(SENSOR_PIN, OUTPUT);
  PORTB |= (1<<SENSOR_PIN);  // digitalWrite(SENSOR_PIN, HIGH);
  send_bits(SEQUENCE_VELUX, NUM_BITS_VELUX);
  send_bits(SEQUENCE_CLOSE, NUM_BITS_SEQUENCE);
  PORTB |= (1<<SENSOR_PIN);  // digitalWrite(SENSOR_PIN, HIGH);
  delay(SEQUENCE_PAUSE);
  send_bits(SEQUENCE_VELUX, NUM_BITS_VELUX);
  send_bits(SEQUENCE_CLOSE, NUM_BITS_SEQUENCE);
  PORTB |= (1<<SENSOR_PIN);  // digitalWrite(SENSOR_PIN, HIGH);
  DDRB &= ~(1<<SENSOR_PIN);  // pinMode(SENSOR_PIN, INPUT);
  PORTB |= (1<<SENSOR_PIN);  // digitalWrite(SENSOR_PIN, HIGH);
}

void sense(int start_value) {
  unsigned long now;
  unsigned long offset = 0;
  unsigned long counter = 0;
  int last_value = start_value;
  int value = start_value;
  
  while (true) {
    now = millis();
    if (now < signal_start ) {
      offset = UNSIGNED_LONG_MAX_VALUE - signal_start;
      signal_start = 0;
    }
    if (now - signal_start + offset > SIGNAL_DURATION) {
      break;
    }
    
    value = PINB & SENSOR_PIN_MASK;  // value = digitalRead(SENSOR_PIN);
    if (value != last_value) {
      sequence_times[counter] = micros();
      sequence[counter] = value;
      counter++;
      last_value = value;
    }
    
  }
}

void decode_recorded_sequence() {
  sequence_velux = 0;
  sequence_part_two = 0;
  unsigned long value;
  unsigned long duration;
  unsigned long t0;
  unsigned long t1;
  int num_bits;
  unsigned long bit_index = 3;
  unsigned long *sequence_ptr = &sequence_velux;
  bool part_two = false;
  
  for (int i=1; i < MAX_SIGNAL_SIZE; i++) {
    value = sequence[i-1];
    t0 = sequence_times[i-1];
    t1 = sequence_times[i];
    if (t1 > t0) {
      duration = t1 - t0;
    } else {
      duration = t1 + UNSIGNED_LONG_MAX_VALUE - t0;
    }

    num_bits = duration / BIT_DURATION;

    if (num_bits == 1 || num_bits == 0) {
      num_bits = 1;
    } else {
      num_bits = 3;
    }

    for (int i=0; i < num_bits; i++) {
      if (value) {
        *sequence_ptr |= one << bit_index;
      } else {
        *sequence_ptr &= ~(one << bit_index); 
      }
      bit_index++;
    }

    if (bit_index >= NUM_BITS_VELUX) {
      bit_index = 0;
      part_two = true;
      sequence_ptr = &sequence_part_two;
    }

    if (part_two && bit_index >= NUM_BITS_SEQUENCE) {
      break;
    }
    
  }
}
