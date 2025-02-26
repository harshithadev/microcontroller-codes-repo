#include <Encoder.h>

// Motor 1 driver pins (Left Motor)
// Motor 1  driver pins
#define PWM1 5   // Left Motor PWM
#define DIR1 6  // Left Motor Direction

// Motor 2  driver pins
#define PWM2 9   // Right Motor PWM
#define DIR2 10   // Right Motor Direction
 
// Encoder 1 pins
#define ENC1_A 3 // Left Motor (Motor 1) Encoder A
#define ENC1_B 2 // Left Motor (Motor 1) Encoder B

// Encoder 2 pins
#define ENC2_A 7  // Right Motor (Motor 2) Encoder A
#define ENC2_B 4 // Right Motor (Motor 2) Encoder B
 

// Create encoder objects
Encoder motor1Encoder(ENC1_A, ENC1_B);
Encoder motor2Encoder(ENC2_A, ENC2_B);

// Define run time (1 minute = 60000ms)
#define RUN_TIME 60000

void setup() {
    Serial.begin(115200);
    
    pinMode(PWM1, OUTPUT);
    pinMode(DIR1, OUTPUT);
    pinMode(PWM2, OUTPUT);
    pinMode(DIR2, OUTPUT);

    Serial.println("Starting 1-minute test...");
    
    // Set both motors to move forward
    digitalWrite(DIR1, HIGH);
    digitalWrite(DIR2, HIGH);
    analogWrite(PWM1, 250);  // Moderate speed
    analogWrite(PWM2, 250);
    
    delay(RUN_TIME);  // Run for exactly 1 minute
    
    // Stop motors
    analogWrite(PWM1, 0);
    analogWrite(PWM2, 0);

    // Read final encoder counts
    long pulses1 = motor1Encoder.read();
    long pulses2 = motor2Encoder.read();

    Serial.println("Test complete. Encoder pulse counts:");
    Serial.print("Motor 1 (Left) Pulses: "); Serial.println(pulses1);
    Serial.print("Motor 2 (Right) Pulses: "); Serial.println(pulses2);
}

void loop() {
    // Nothing in loop since it's a one-time test
}
