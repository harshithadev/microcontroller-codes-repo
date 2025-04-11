#include <Encoder.h>

// Motor control pins (not used here, just kept for consistency)
#define PWM1 5   // Left Motor PWM
#define DIR1 6   // Left Motor Direction
#define PWM2 8   // Right Motor PWM
#define DIR2 9   // Right Motor Direction

// Encoder pins
#define ENC1_A 2
#define ENC1_B 3
#define ENC2_A 4
#define ENC2_B 7

// Encoder objects using your naming
Encoder motor1Encoder(ENC1_A, ENC1_B);  // Left Motor Encoder
Encoder motor2Encoder(ENC2_A, ENC2_B);  // Right Motor Encoder

void setup() {
  Serial.begin(115200);
  Serial.println("Encoder Pulse Count Debug - Rotate wheels manually");
}

void loop() {
  static long prevCount1 = 0;
  static long prevCount2 = 0;

  long count1 = motor1Encoder.read();
  long count2 = motor2Encoder.read();

  if (count1 != prevCount1 || count2 != prevCount2) {
    Serial.print("Left Encoder (motor1): ");
    Serial.print(count1);
    Serial.print(" | Right Encoder (motor2): ");
    Serial.println(count2);
    prevCount1 = count1;
    prevCount2 = count2;
  }

  delay(10);
}
