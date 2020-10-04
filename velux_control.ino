#define SHORT_PULSE 430
#define LONG_PULSE 1210
#define BIT_DURATION 450
#define SEQUENCE_PAUSE 30
#define SENSOR_PIN 10
#define OUTPUT_PIN 10
#define UNSIGNED_LONG_MAX_VALUE 4294967295
#define SIGNAL_DURATION 20
#define NUM_BITS_VELUX 24
#define NUM_BITS_SEQUENCE 21
#define MAX_SIGNAL_SIZE 45
// #define DEBUG

// define sequences as long integers but in reverse bit order so that first bit in the sequence
// can be set with 1 << 0 and the consecutive ones with 1 << 1...num_bits
#define SEQUENCE_VELUX 15263976 // 000101110001011100010111 00000000
#define SEQUENCE_OPEN 585454    // 011101110111011100010 00000000000
#define SEQUENCE_CLOSE 978574   // 011100010111011101110 00000000000

unsigned long signal_start;

unsigned long sequence_velux = 0;
unsigned long sequence_part_two = 0;
byte sequence[MAX_SIGNAL_SIZE];
unsigned long sequence_times[MAX_SIGNAL_SIZE];
unsigned long timer_start_time;
unsigned long close_delay = 60000;
bool timer_started = false;


void setup() {
  // put your setup code here, to run once:
  // Set output pin to input too so that it has a high impedance and does not disrupt existing controller.
  pinMode(OUTPUT_PIN, INPUT);
  pinMode(SENSOR_PIN, INPUT);

  //attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), sense, CHANGE);

  Serial.begin(115200);

}

void loop() {
  // put your main code here, to run repeatedly:
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
  int val = digitalRead(SENSOR_PIN);
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
    if (now < timer_start_time) {
      elapsed = now + UNSIGNED_LONG_MAX_VALUE - timer_start_time;
    } else {
      elapsed = now - timer_start_time;
    }

    if (elapsed > close_delay) {
      timer_started = false;
      send_close();
    }
  }
}

const unsigned long one = 1;

void send_bits(unsigned long bits, int num_bits) {
  for (unsigned long i=0; i < num_bits; i++) {
    if (bits & one << i) {
      digitalWrite(OUTPUT_PIN, HIGH);
#ifdef DEBUG
      Serial.write('1');
#endif      
    } else {
      digitalWrite(OUTPUT_PIN, LOW);
#ifdef DEBUG
      Serial.write('0');
#endif
    }
    delayMicroseconds(BIT_DURATION);
  }
}

void send_open() {
  pinMode(OUTPUT_PIN, OUTPUT);
  digitalWrite(OUTPUT_PIN, HIGH);
  send_bits(SEQUENCE_VELUX, NUM_BITS_VELUX);
  send_bits(SEQUENCE_OPEN, NUM_BITS_SEQUENCE);
  digitalWrite(OUTPUT_PIN, HIGH);
  delay(SEQUENCE_PAUSE);
  send_bits(SEQUENCE_VELUX, NUM_BITS_VELUX);
  send_bits(SEQUENCE_OPEN, NUM_BITS_SEQUENCE);
  digitalWrite(OUTPUT_PIN, HIGH);
  pinMode(OUTPUT_PIN, INPUT);
  
}

void send_close() {
  pinMode(OUTPUT_PIN, OUTPUT);
  digitalWrite(OUTPUT_PIN, HIGH);
  send_bits(SEQUENCE_VELUX, NUM_BITS_VELUX);
  send_bits(SEQUENCE_CLOSE, NUM_BITS_SEQUENCE);
  digitalWrite(OUTPUT_PIN, HIGH);
  delay(SEQUENCE_PAUSE);
  send_bits(SEQUENCE_VELUX, NUM_BITS_VELUX);
  send_bits(SEQUENCE_CLOSE, NUM_BITS_SEQUENCE);
  digitalWrite(OUTPUT_PIN, HIGH);
  pinMode(OUTPUT_PIN, INPUT);
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

    value = digitalRead(SENSOR_PIN);
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
