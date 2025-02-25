#include <Encoder.h>
#include <PID_v1.h>

// Motor 1  driver pins
#define PWM1 5   // Left Motor PWM
#define DIR1 6  // Left Motor Direction

// Motor 2  driver pins
#define PWM2 9   // Right Motor PWM
#define DIR2 10   // Right Motor Direction
 
// Encoder 1 pins
#define ENC1_A 2 // Left Motor (Motor 1) Encoder A
#define ENC1_B 3 // Left Motor (Motor 1) Encoder B

// Encoder 2 pins
#define ENC2_A 4  // Right Motor (Motor 2) Encoder A
#define ENC2_B 7  // Right Motor (Motor 2) Encoder B
 
// Encoder and motor parameters
#define CPR 25916       
#define SAMPLE_TIME 100  
 
// **Updated PID Values**
double Kp = 3.7, Ki = 0.8, Kd = 0.1;
 
// Variables for PID control
double setpoint1, setpoint2;  
double input1, input2;  
double output1, output2;  
 
// PID Controllers
PID pid1(&input1, &output1, &setpoint1, Kp, Ki, Kd, DIRECT);
PID pid2(&input2, &output2, &setpoint2, Kp, Ki, Kd, DIRECT);
 
// Encoder objects
Encoder motor1Encoder(ENC1_A, ENC1_B);
Encoder motor2Encoder(ENC2_A, ENC2_B);
 
long prevCount1 = 0, prevCount2 = 0;
unsigned long prevTime = 0;
 
// **Fix Motor 2's Negative RPM Issue**
double computeRPM(long deltaCount, int motorID) {
    double timeFactor = (60000.0 / SAMPLE_TIME);  // Adjust factor based on sampling time
    if (motorID == 2) {
        return -(deltaCount * timeFactor) / CPR;  // Inverted RPM for Motor 2
    } else {
        return (deltaCount * timeFactor) / CPR;
    }
}
void setup() {
    Serial.begin(115200);
    pinMode(PWM1, OUTPUT);
    pinMode(DIR1, OUTPUT);
    pinMode(PWM2, OUTPUT);
    pinMode(DIR2, OUTPUT);
 
    pid1.SetMode(AUTOMATIC);
    pid2.SetMode(AUTOMATIC);
 
    // **Ensure minimum PWM to start movement**
    pid1.SetOutputLimits(100, 255);  
    pid2.SetOutputLimits(100, 255);  
 
    Serial.println("PID Speed Control Test");
}
 
void loop() {
    unsigned long currentTime = millis();
 
    // **Set realistic RPM target**
    setpoint1 = 90;  
    setpoint2 = 90;  
 
    if (currentTime - prevTime >= SAMPLE_TIME) {
        long count1 = motor1Encoder.read();
        long count2 = motor2Encoder.read();
 
        input1 = computeRPM(count1 - prevCount1, 1);
        input2 = computeRPM(count2 - prevCount2, 2);  // Fixed negative RPM issue
 
        prevCount1 = count1;
        prevCount2 = count2;
        prevTime = currentTime;
 
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