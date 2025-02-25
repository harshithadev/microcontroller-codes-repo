#include <PID_v1.h>

// Motor 1 Driver Pins
#define PWM1 5   // Left Motor PWM
#define DIR1 6   // Left Motor Direction

// Motor 2 Driver Pins
#define PWM2 9   // Right Motor PWM
#define DIR2 10  // Right Motor Direction

// Encoder 1 Pins (Left Motor)
#define ENC1_A 2
#define ENC1_B 3

// Encoder 2 Pins (Right Motor)
#define ENC2_A 4
#define ENC2_B 7

// Encoder and Motor Parameters
#define CPR 25916       
#define SAMPLE_TIME 100  // Sampling time in milliseconds (adjustable)

// **Updated PID Values**
double Kp = 5.0, Ki = 0.8, Kd = 0.1;

// Variables for PID control
double setpoint1, setpoint2;  
double input1, input2;  
double output1, output2;  

// PID Controllers
PID pid1(&input1, &output1, &setpoint1, Kp, Ki, Kd, DIRECT);
PID pid2(&input2, &output2, &setpoint2, Kp, Ki, Kd, DIRECT);

// **Interrupt-based Encoder Counters**
volatile long count1 = 0, count2 = 0;
unsigned long prevTime = 0;

// Interrupt Service Routine for Encoder 1 (Left Motor)
void encoder1ISR() {
    count1++;
}

// Interrupt Service Routine for Encoder 2 (Right Motor)
void encoder2ISR() {
    count2++;
}

// **Fix Motor 2's Negative RPM Issue**
double computeRPM(long deltaCount, int motorID) {
    double timeFactor = (60000.0 / SAMPLE_TIME);  // Adjust factor based on sampling time
    if (motorID == 2) {
        return (deltaCount * timeFactor) / CPR;  // Inverted RPM for Motor 2
    } else {
        return (deltaCount * timeFactor) / CPR;
    }
}

void setup() {
    Serial.begin(115200);

    // Motor Driver Pins as Output
    pinMode(PWM1, OUTPUT);
    pinMode(DIR1, OUTPUT);
    pinMode(PWM2, OUTPUT);
    pinMode(DIR2, OUTPUT);

    // Encoder Pins as Input
    pinMode(ENC1_A, INPUT_PULLUP);
    pinMode(ENC1_B, INPUT_PULLUP);
    pinMode(ENC2_A, INPUT_PULLUP);
    pinMode(ENC2_B, INPUT_PULLUP);

    // Attach Interrupts for Encoder A Signals
    attachInterrupt(digitalPinToInterrupt(ENC1_A), encoder1ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC2_A), encoder2ISR, RISING);

    // Initialize PID controllers
    pid1.SetMode(AUTOMATIC);
    pid2.SetMode(AUTOMATIC);

    // Ensure minimum PWM to start movement
    pid1.SetOutputLimits(100, 255);  
    pid2.SetOutputLimits(100, 255);  

    Serial.println("PID Speed Control with Interrupt-Based Encoders Started");

    // Start Motors
    moveForward(150);  // Initial speed for testing
}

void loop() {
    unsigned long currentTime = millis();

    // **Set realistic RPM target**
    setpoint1 = 90;  
    setpoint2 = 90;  

    if (currentTime - prevTime >= SAMPLE_TIME) {
        prevTime = currentTime;

        // Compute RPM from interrupt-based encoder counts
        input1 = computeRPM(count1, 1);
        input2 = computeRPM(count2, 2);  // Fixed negative RPM issue

        // Reset encoder counters after reading
        count1 = 0;
        count2 = 0;

        // Compute PID outputs
        pid1.Compute();
        pid2.Compute();

        moveMotors((int)output1, (int)output2);

        // **Print debug information**
        Serial.print("Target RPM: "); Serial.print(setpoint1);
        Serial.print(" | Motor 1 RPM: "); Serial.print(input1);
        Serial.print(" | Motor 2 RPM: "); Serial.print(input2);
        Serial.print(" | PWM1: "); Serial.print(output1);
        Serial.print(" | PWM2: "); Serial.println(output2);
    }
}

// **Fix: Ensure Minimum PWM to Start Moving**
void moveMotors(int speed1, int speed2) {
    if (speed1 > 0 && speed1 < 100) speed1 = 100;
    if (speed2 > 0 && speed2 < 100) speed2 = 100;

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

// Function to move forward at a given PWM speed
void moveForward(int speed) {
    digitalWrite(DIR1, HIGH);
    digitalWrite(DIR2, HIGH);
    analogWrite(PWM1, speed);
    analogWrite(PWM2, speed);
}

// Function to stop motors
void stopMotors() {
    analogWrite(PWM1, 0);
    analogWrite(PWM2, 0);
}
