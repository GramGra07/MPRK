#include <ESP32Encoder.h>

#define ENCODER_PIN_A 18  // Quadrature signal A
#define ENCODER_PIN_B 19  // Quadrature signal B

volatile int encoderPosition = 0;  // Tracks the encoder position
volatile int lastEncoded = 0;      // Stores the last encoder state

void setup() {
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
pinMode(ENCODER_PIN_B, INPUT_PULLUP);

  // Attach interrupts to handle encoder signals
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), updateEncoder, CHANGE);

  Serial.begin(9600);  // Initialize serial communication
}

void loop() {
  // Print the encoder position to the Serial Monitor
  Serial.print("Encoder Position: ");
  Serial.println(encoderPosition);
  delay(100);  // Update every 100ms
}

void updateEncoder() {
  int MSB = digitalRead(ENCODER_PIN_A);  // Most significant bit
  int LSB = digitalRead(ENCODER_PIN_B);  // Least significant bit

  int encoded = (MSB << 1) | LSB;        // Combine A and B signals
  int sum = (lastEncoded << 2) | encoded;  // Track state changes

  // Determine direction and update position
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    encoderPosition++;
  } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    encoderPosition--;
  }

  lastEncoded = encoded;  // Update the last state
}

