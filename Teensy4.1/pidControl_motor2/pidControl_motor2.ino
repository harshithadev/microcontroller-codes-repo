
#include <Encoder.h>
#include <PID_v1.h>
 
// Motor 2 driver pins
#define PWM2 9   // Left Motor PWM
#define DIR2 10  // Left Motor Direction
 
#define ENC2_A 4  // Right Motor (Motor 2) Encoder A
#define ENC2_B 7  // Right Motor (Motor 2) Encoder B
 
 
// Encoder and motor parameters
#define CPR 25916       
#define SAMPLE_TIME 120  
 
// **PID Values for Single Motor**
double Kp = 2.5, Ki = 0.8, Kd = 0.1;
 
// Variables for PID control
double setpoint, input, output;
 
// Single PID Controller for Motor 2
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
 
// Encoder object for Motor 2
Encoder motor2Encoder(ENC2_A, ENC2_B); 
 
 
long prevCount2 = 0;
unsigned long prevTime = 0;
 
// **Compute RPM for Motor 2**
double computeRPM(long deltaCount) {
    return -(deltaCount * 60.0) / CPR;
}
 
void setup() {
    Serial.begin(115200);
    pinMode(PWM2, OUTPUT);
    pinMode(DIR2, OUTPUT);
 
    pid.SetMode(AUTOMATIC);
    pid.SetOutputLimits(100, 200);  // Ensure minimum PWM for movement
 
    Serial.println("PID Speed Control Test - Motor 2 Only");
}
 
void loop() {
    unsigned long currentTime = millis();
 
    // **Set Target RPM**
    setpoint = 20;  
 
    if (currentTime - prevTime >= SAMPLE_TIME) {
        long count2 = motor2Encoder.read();
 
        // **Reset encoder counts every cycle**
        motor2Encoder.write(0);
 
        // **Calculate RPM for Motor 2**
        input = computeRPM(count2);
 
        prevTime = currentTime;
 
        pid.Compute();
 
        // **Apply PID output to Motor 2**
        moveMotor((int)output);
 
        // **Print Debug Information**
        Serial.print("Target RPM: "); Serial.print(setpoint);
        Serial.print(" | Motor 2 RPM: "); Serial.print(input);
        Serial.print(" | PWM2: "); Serial.println(output);
    }
}
 
void moveMotor(int speed2) {
    if (speed2 > 0 && speed2 < 100) speed2 = 100;
 
    if (speed2 >= 0) {
        digitalWrite(DIR2, HIGH);
        analogWrite(PWM2, speed2);
    } else {
        digitalWrite(DIR2, LOW);
        analogWrite(PWM2, -speed2);
    }
}