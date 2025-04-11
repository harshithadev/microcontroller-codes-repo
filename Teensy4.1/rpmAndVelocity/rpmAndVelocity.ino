#include <Encoder.h>

// Motor 1 driver pins
#define PWM1 5   // Left Motor PWM
#define DIR1 6   // Left Motor Direction

// Motor 2 driver pins
#define PWM2 9   // Right Motor PWM
#define DIR2 10  // Right Motor Direction

// Encoder 1 pins
#define ENC1_A 2 // Left Motor (Motor 1) Encoder A
#define ENC1_B 3 // Left Motor (Motor 1) Encoder B

// Encoder 2 pins
#define ENC2_A 7 // Right Motor (Motor 2) Encoder A
#define ENC2_B 4 // Right Motor (Motor 2) Encoder B

// Encoder constants
#define CPR 153500     // Counts Per Revolution (CPR for the motor shaft)
#define SAMPLE_TIME 120    // Sampling time in milliseconds (reduced for faster updates)

// Wheel constants
#define WHEEL_RADIUS 0.075  // Wheel radius in meters (provided)

// Create encoder objects
Encoder motor1Encoder(ENC1_A, ENC1_B);
Encoder motor2Encoder(ENC2_A, ENC2_B);

// Variables for tracking encoder counts
long prevCount1 = 0, prevCount2 = 0;
unsigned long prevTime = 0;
float motor1RPM = 0, motor2RPM = 0;
float motor1Velocity = 0, motor2Velocity = 0;  // Linear velocities of the motors (m/s)

// PWM values to control motor speed (0-255)
int pwmValue1 = 25;  // Motor 1 PWM (50% speed as an example)
int pwmValue2 = 25;  // Motor 2 PWM (50% speed as an example)

void setup() {
  Serial.begin(9600);
  pinMode(PWM1, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(DIR2, OUTPUT);

  // Set initial PWM values for the motors
  analogWrite(PWM1, pwmValue1);
  analogWrite(PWM2, pwmValue2);

  // Set motor directions to forward (HIGH or LOW depends on your motor driver)
  digitalWrite(DIR1, HIGH); // Motor 1 forward
  digitalWrite(DIR2, HIGH); // Motor 2 forward
}

void loop() {
  unsigned long currentTime = millis();

  // Check if it's time to sample
  if (currentTime - prevTime >= SAMPLE_TIME) {
    long count1 = motor1Encoder.read();  // Read encoder count for motor 1
    long count2 = motor2Encoder.read();  // Read encoder count for motor 2

    // Calculate RPM for motor 1 and motor 2
    // Convert SAMPLE_TIME to seconds by dividing by 1000
    float sampleTimeInSeconds = SAMPLE_TIME / 1000.0;

    motor1RPM = (float)(count1 - prevCount1) * 60.0 / (CPR * sampleTimeInSeconds);
    motor2RPM = (float)(count2 - prevCount2) * 60.0 / (CPR * sampleTimeInSeconds);

    // Calculate linear velocity in m/s for motor 1 and motor 2
    // Convert RPM to radians per second, then multiply by wheel radius to get velocity
    motor1Velocity = (motor1RPM * 2 * PI / 60) * WHEEL_RADIUS;
    motor2Velocity = (motor2RPM * 2 * PI / 60) * WHEEL_RADIUS;

    // Update previous encoder counts
    prevCount1 = count1;
    prevCount2 = count2;

    // Output the RPM and velocity values for both motors
    Serial.print("Motor 1 RPM: ");
    Serial.println(motor1RPM);

    Serial.print("Motor 1 Linear Velocity (m/s): ");
    Serial.println(motor1Velocity);

    Serial.print("Motor 2 RPM: ");
    Serial.println(motor2RPM);

    Serial.print("Motor 2 Linear Velocity (m/s): ");
    Serial.println(motor2Velocity);

    // Update the previous time
    prevTime = currentTime;
  }
}
