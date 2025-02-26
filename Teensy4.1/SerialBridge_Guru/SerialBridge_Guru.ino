#include <Encoder.h>
#include <PID_v1.h>

// Motor 1  driver pins
#define PWM1 0   // Left Motor PWM
#define DIR1 1 // Left Motor Direction

// Motor 2  driver pins
#define PWM2 2  // Right Motor PWM
#define DIR2 3   // Right Motor Direction
 
// Encoder 1 pins
#define ENC1_A 6 // Left Motor (Motor 1) Encoder A
#define ENC1_B 8 // Left Motor (Motor 1) Encoder B

// Encoder 2 pins
#define ENC2_A 4  // Right Motor (Motor 2) Encoder A
#define ENC2_B 5 // Right Motor (Motor 2) Encoder B
 
// Encoder constants
#define CPR 25916       
#define SAMPLE_TIME 100  

// PID Tuning Parameters
double Kp = 3.7, Ki = 0.8, Kd = 0.1;

// Variables for PID control
double setpoint1 = 0, setpoint2 = 0;  
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

// Compute RPM function
double computeRPM(long deltaCount, int motorID) {
    double timeFactor = (60000.0 / SAMPLE_TIME);
    return -(deltaCount * timeFactor) / CPR;
}

// **NEW: Function to Continuously Read Incoming Setpoint**
void checkSerialForSetpoint() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        if (command.startsWith("SETPOINT")) {
            sscanf(command.c_str(), "SETPOINT %lf %lf", &setpoint1, &setpoint2);
            Serial.println("OK - Setpoint Updated");  // Confirmation message
        }
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

    pid1.SetOutputLimits(-255, 255);  
    pid2.SetOutputLimits(-255, 255);  

    Serial.println("PID Speed Control with ROS2 Integration");
}

void loop() {
    unsigned long currentTime = millis();

    // **NEW: Continuously check for new SETPOINT messages**
    checkSerialForSetpoint();

    // **PID Control Loop**
    if (currentTime - prevTime >= SAMPLE_TIME) {
        long count1 = motor1Encoder.read();
        long count2 = motor2Encoder.read();

        input1 = computeRPM(count1 - prevCount1, 1);
        input2 = computeRPM(count2 - prevCount2, 2);

        prevCount1 = count1;
        prevCount2 = count2;
        prevTime = currentTime;

        pid1.Compute();
        pid2.Compute();

        moveMotors((int)output1, (int)output2);

        // **Send Actual RPM Back to ROS2**
        Serial.print("Setpoint ");
        Serial.print(setpoint1);
        Serial.print("RPM ");
        Serial.print(input1);
        Serial.print(" ");
        Serial.println(input2);
    }
}

// **Move Motors with PID Output**
void moveMotors(int speed1, int speed2) {
    // if (speed1 > 0 && speed1 < 100) speed1 = 100;
    // if (speed2 > 0 && speed2 < 100) speed2 = 100;

    digitalWrite(DIR1, speed1 >= 0 ? HIGH : LOW);
    analogWrite(PWM1, abs(speed1));

    digitalWrite(DIR2, speed2 >= 0 ? HIGH : LOW);
    analogWrite(PWM2, abs(speed2));
}
