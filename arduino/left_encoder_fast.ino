#define left_signalA 2            // Encoder signal A connected to digital pin 2 (interrupt pin)
#define left_signalB 3            // Encoder signal B connected to digital pin 3 (direction detection)

volatile long int left_counter = 0;
volatile unsigned long prev_t = 0;
volatile float velocity = 0;
volatile long int prev_enc = 0;
const unsigned long interval = 100000; // Update interval in microseconds

void setup() {
  Serial.begin(9600);
  
  pinMode(left_signalA, INPUT_PULLUP);
  pinMode(left_signalB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(left_signalA), updateEncoderCount, FALLING);
}

void loop() {
  // Update and publish encoder ticks
  Serial.print("TICKS:");
  Serial.println((1) * left_counter);
  
  // Get current time for velocity calculation
  unsigned long current_t = micros();
  
  // Calculate velocity at fixed intervals
  if (current_t - prev_t > interval) {
    float delta_t = ((float)(current_t - prev_t)) / 1000000.0; // Convert to seconds
    
    // Calculate velocity
    velocity = (float)(left_counter - prev_enc) / delta_t; // Velocity in ticks per second
    prev_t = current_t;
    prev_enc = left_counter;
    
    // Convert velocity from ticks/second to RPM
    float rpm_value = velocity / 600.0 * 60.0;
    
    Serial.print("RPM:");
    Serial.println((int)rpm_value);
  }
  
  delay(100); // Send data every 100ms
}

void updateEncoderCount() {
  if (digitalRead(left_signalB) == LOW)
    left_counter++;
  else
    left_counter--;
}