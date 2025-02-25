#include <Encoder.h>
#include <PID_v1.h>

// Motor 1 driver pins (Right Motor)
#define PWM1 5   // Left Motor PWM
#define DIR1 6   // Left Motor Direction

#define ENC1_A 2  
#define ENC1_B 3  

// Encoder and motor parameters
#define CPR 25916       
#define SAMPLE_TIME 120  

// **PID Values for Motor 1**
double Kp = 2.5, Ki = 0.8, Kd = 0.1;

// Variables for PID control
double setpoint, input, output;

// Single PID Controller for Motor 1
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// Encoder object for Motor 1
Encoder motor1Encoder(ENC1_A, ENC1_B); 

long prevCount1 = 0;
unsigned long prevTime = 0;

// **Compute RPM for Motor 1**
double computeRPM(long deltaCount) {
    return (deltaCount * 60.0) / CPR;
}

void setup() {
    Serial.begin(115200);
    pinMode(PWM1, OUTPUT);
    pinMode(DIR1, OUTPUT);

    pid.SetMode(AUTOMATIC);
    pid.SetOutputLimits(100, 200);  // Ensure minimum PWM for movement

    Serial.println("PID Speed Control Test - Motor 1 Only");
}

void loop() {
    unsigned long currentTime = millis();

    // **Set Target RPM**
    setpoint = 20;  

    if (currentTime - prevTime >= SAMPLE_TIME) {
        long count1 = motor1Encoder.read();

        // **Reset encoder counts every cycle**
        motor1Encoder.write(0);

        // **Calculate RPM for Motor 1**
        input = computeRPM(count1);

        prevTime = currentTime;

        pid.Compute();

        // **Apply PID output to Motor 1**
        moveMotor((int)output);

        // **Print Debug Information**
        Serial.print("Target RPM: "); Serial.print(setpoint);
        Serial.print(" | Motor 1 RPM: "); Serial.print(input);
        Serial.print(" | PWM1: "); Serial.println(output);
    }
}

void moveMotor(int speed) {
    if (speed > 0 && speed < 100) speed = 100;

    if (speed >= 0) {
        digitalWrite(DIR1, HIGH);
        analogWrite(PWM1, speed);
    } else {
        digitalWrite(DIR1, LOW);
        analogWrite(PWM1, -speed);
    }
}
