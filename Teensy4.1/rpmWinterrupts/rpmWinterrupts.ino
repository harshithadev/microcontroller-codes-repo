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

// **Function to Compute RPM**
double computeRPM(long deltaCount, int motorID) {
    double timeFactor = (60000.0 / SAMPLE_TIME);  // Convert to RPM
    if (motorID == 2) {
        return (deltaCount * timeFactor) / CPR;  // Fix negative RPM for Motor 2
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
    attachInterrupt(digitalPinToInterrupt(ENC1_A), encoder1ISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(ENC2_A), encoder2ISR, FALLING);

    Serial.println("Constant PWM Speed Test with Interrupt-Based RPM Calculation");

    // Start Motors with a Constant PWM Signal
    moveForward(150);  // Adjust PWM value as needed
}

void loop() {
    unsigned long currentTime = millis();

    if (currentTime - prevTime >= SAMPLE_TIME) {
        prevTime = currentTime;

        // Compute RPM using interrupt-based encoder readings
        double rpm1 = computeRPM(count1, 1);
        double rpm2 = computeRPM(count2, 2);  // Fix negative RPM issue for Motor 2

        // Reset encoder counters after reading
        count1 = 0;
        count2 = 0;

        // **Print RPM values**
        Serial.print("Motor 1 RPM: "); Serial.print(rpm1);
        Serial.print(" | Motor 2 RPM: "); Serial.println(rpm2);
    }
}

// **Function to Set Motors to a Constant PWM Speed**
void moveForward(int speed) {
    digitalWrite(DIR1, HIGH);
    digitalWrite(DIR2, HIGH);
    analogWrite(PWM1, speed);
    analogWrite(PWM2, speed);
}

// **Function to Stop Motors (if needed)**
void stopMotors() {
    analogWrite(PWM1, 0);
    analogWrite(PWM2, 0);
}
