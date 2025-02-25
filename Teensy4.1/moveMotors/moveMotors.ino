#include <Arduino.h>
#include <Encoder.h>

// Define motor driver pins (Updated for correct movement)
#define PWM1 9   // Left Motor (Motor 1) PWM
#define DIR1 10  // Left Motor (Motor 1) Direction
#define PWM2 5   // Right Motor (Motor 2) PWM
#define DIR2 6   // Right Motor (Motor 2) Direction

// Define encoder pins
#define ENC1_A 2
#define ENC1_B 3
#define ENC2_A 4
#define ENC2_B 7

// Create encoder objects
Encoder motor1Encoder(ENC1_A, ENC1_B);
Encoder motor2Encoder(ENC2_A, ENC2_B);

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
    Serial.println("Moving Forward");
    moveMotors(225, 225);
    delay(2000);

    // Move backward
    Serial.println("Moving Backward");
    moveMotors(-150, -150);
    delay(2000);

    // Turn left
    Serial.println("Turning Left");
    moveMotors(-150, 150);
    delay(2000);

    // Turn right
    Serial.println("Turning Right");
    moveMotors(150, -150);
    delay(2000);

    // Stop
    Serial.println("Stopping");
    moveMotors(0, 0);
    delay(2000);

    // Read encoder values
    long pos1 = motor1Encoder.read();
    long pos2 = motor2Encoder.read();
    Serial.print("Encoder1: "); Serial.print(pos1);
    Serial.print(" Encoder2: "); Serial.println(pos2);
}
