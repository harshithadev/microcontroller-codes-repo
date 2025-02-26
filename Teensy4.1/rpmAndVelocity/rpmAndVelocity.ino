#include <Encoder.h>

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
 
// Encoder constants
#define CPR 155500      // Counts Per Revolution
#define GEAR_RATIO 102  // 1:13 gear ratio
#define WHEEL_RADIUS 0.075  // Wheel radius in meters
#define SAMPLE_TIME 120    // Sampling time in milliseconds (reduced for faster updates)

// Create encoder objects
Encoder motor1Encoder(ENC1_A, ENC1_B);
Encoder motor2Encoder(ENC2_A, ENC2_B);

// Variables for tracking encoder counts
long prevCount1 = 0, prevCount2 = 0;
unsigned long prevTime = 0;

// Function to calculate and print RPM & velocity
void printRPMVelocity(String movement) {
    long count1 = motor1Encoder.read();
    long count2 = motor2Encoder.read();

    // No need to Fix Motor 2's negative RPM issue
    float rpm1 = (count1 * 60.0) / CPR;
    float rpm2 = (count2 * 60.0) / CPR;  // Inverted

    // Convert RPM to linear velocity (m/s)
    float velocity1 = rpm1 * (2 * 3.14159 * WHEEL_RADIUS) / 60.0;
    float velocity2 = rpm2 * (2 * 3.14159 * WHEEL_RADIUS) / 60.0;

    // Print values after movement
    Serial.print(movement + " -> Motor 1 RPM: "); 
    Serial.print(rpm1);
    Serial.print(", Velocity: "); Serial.print(velocity1); Serial.print(" m/s  ||  ");
    Serial.print("Motor 2 RPM: "); 
    Serial.println(rpm2);
    Serial.print(", Velocity: "); Serial.println(velocity2); Serial.print(" m/s");

    // Reset encoder counts for the next movement
    motor1Encoder.write(0);
    motor2Encoder.write(0);
}

void setup() {
    Serial.begin(115200);

    pinMode(PWM1, OUTPUT);
    pinMode(DIR1, OUTPUT);
    pinMode(PWM2, OUTPUT);
    pinMode(DIR2, OUTPUT);

    Serial.println("Motor test begins!");
}

void loop() {
    // Move forward
    Serial.println("Moving Forward...");
    moveMotors(120, 120);
    delay(2000);
    moveMotors(0, 0);
    printRPMVelocity("FORWARD");

    // // Move backward
    // Serial.println("Moving Backward...");
    // moveMotors(-120, -120);
    // delay(2000);
    // moveMotors(0, 0);
    // printRPMVelocity("BACKWARD");

    // // Turn left
    // Serial.println("Turning Left...");
    // moveMotors(-120, 120);
    // delay(2000);
    // moveMotors(0, 0);
    // printRPMVelocity("LEFT TURN");

    // // Turn right
    // Serial.println("Turning Right...");
    // moveMotors(120, -120);
    // delay(2000);
    // moveMotors(0, 0);
    // printRPMVelocity("RIGHT TURN");

    // // Stop for 2 seconds
    // Serial.println("Stopping...");
    // moveMotors(0, 0);
    // delay(2000);
}

// Function to control motors
void moveMotors(int speed1, int speed2) {
    if (speed1 >= 0) {
        digitalWrite(DIR1, HIGH);
        analogWrite(PWM1, speed1);
    } else {
        digitalWrite(DIR1, LOW);
        analogWrite(PWM1, -speed1);
    }

    if (speed2 >= 0) {
        digitalWrite(DIR2, HIGH);
        analogWrite(PWM2, speed2);
    } else {
        digitalWrite(DIR2, LOW);
        analogWrite(PWM2, -speed2);
    }
}
