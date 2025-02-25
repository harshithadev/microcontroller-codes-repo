#include <Encoder.h>
#include <PID_v1.h>

// Motor 1 (Left) driver pins
#define PWM1 5   
#define DIR1 6  
// Motor 2 (Right) driver pins
#define PWM2 9   
#define DIR2 10  

// Encoder 1 (Left) pins
#define ENC1_A 2 
#define ENC1_B 3 
// Encoder 2 (Right) pins
#define ENC2_A 4  
#define ENC2_B 7  

// Encoder and motor parameters
#define CPR 25916       
#define SAMPLE_TIME 100  

// Variables for PID control
double setpoint1 = 0, setpoint2 = 0;  
double input1, input2;  
double output1 = 255;

// Encoder objects
Encoder motor1Encoder(ENC1_A, ENC1_B);
Encoder motor2Encoder(ENC2_A, ENC2_B);

long prevCount1 = 0, prevCount2 = 0;
unsigned long prevTime = 0;

// Compute RPM function
double computeRPM(long deltaCount, int motorID) {
    double timeFactor = (60000.0 / SAMPLE_TIME);
    return (motorID == 2) ? -(deltaCount * timeFactor) / CPR : (deltaCount * timeFactor) / CPR;
}


void setup() {
    Serial.begin(115200);
    pinMode(PWM1, OUTPUT);
    pinMode(DIR1, OUTPUT);
    pinMode(PWM2, OUTPUT);
    pinMode(DIR2, OUTPUT);
}

void loop() {
    unsigned long currentTime = millis();

    if (currentTime - prevTime >= SAMPLE_TIME) {
        long count1 = motor1Encoder.read();
        long count2 = motor2Encoder.read();

        input1 = computeRPM(count1 - prevCount1, 1);
        input2 = computeRPM(count2 - prevCount2, 2);

        prevCount1 = count1;
        prevCount2 = count2;
        prevTime = currentTime;

        moveMotors((int)output1, (int)output1);

        Serial.print("PWM ");
        Serial.print(output1);

        Serial.print("RPM ");
        Serial.print(input1);
        Serial.print(" ");
        Serial.println(input2);
    }
}

// **Move Motors with PID Output**
void moveMotors(int speed1, int speed2) {

    digitalWrite(DIR1, speed1 >= 0 ? HIGH : LOW);
    analogWrite(PWM1, abs(speed1));

    digitalWrite(DIR2, speed2 >= 0 ? HIGH : LOW);
    analogWrite(PWM2, abs(speed2));
}
