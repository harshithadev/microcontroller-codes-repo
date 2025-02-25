#include <Arduino.h>

// Motor Control Pins
#define MOTOR_LEFT_PWM 5
#define MOTOR_RIGHT_PWM 6

// Variables for velocity
float linear_x = 0.0;
float angular_z = 0.0;

void setup() {
    Serial.begin(115200);  // USB Serial for ROS2 communication
    pinMode(MOTOR_LEFT_PWM, OUTPUT);
    pinMode(MOTOR_RIGHT_PWM, OUTPUT);
}

void loop() {
    // Read incoming data from ROS2
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');  // Read a full command
        parseCommand(command);
    }
    // Simulate motor feedback
    sendMotorFeedback();
    delay(50); // Adjust for smooth communication
}

// Parse incoming ROS2 commands
void parseCommand(String cmd) {
    if (cmd.startsWith("VEL")) {
        sscanf(cmd.c_str(), "VEL %f %f", &linear_x, &angular_z);
        Serial.println("OK");  // Acknowledge receipt
        controlMotors();
    }
}

// Control motors based on velocity command
void controlMotors() {
    int left_speed = int((linear_x - angular_z) * 255);
    int right_speed = int((linear_x + angular_z) * 255);
    
    analogWrite(MOTOR_LEFT_PWM, constrain(left_speed, 0, 255));
    analogWrite(MOTOR_RIGHT_PWM, constrain(right_speed, 0, 255));
}

// Send encoder/motor feedback to ROS2
void sendMotorFeedback() {
    float rpm_left = 100.0;  // Placeholder (use encoder readings in real case)
    float rpm_right = 98.0;
    Serial.print("RPM "); Serial.print(rpm_left); Serial.print(" lol "); Serial.println(rpm_right);
}
