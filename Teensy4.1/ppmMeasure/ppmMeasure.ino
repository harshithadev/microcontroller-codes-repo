#include <Encoder.h>

// Motor 1 driver pins (Left Motor)
#define PWM1 5   
#define DIR1 6  

// Motor 2 driver pins (Right Motor)
#define PWM2 9   
#define DIR2 10  

// Encoder 1 pins (Left Motor)
#define ENC1_A 2  
#define ENC1_B 3  

// Encoder 2 pins (Right Motor)
#define ENC2_A 4  
#define ENC2_B 7  

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
