

// //////////////////////////////////////////////////wheel speed////////////////////////////////////////////////////////
// #include <math.h>
// #include <Arduino.h>
// #include <ESP32Encoder.h> 

// // Hardware Instance Handles
// ESP32Encoder enc_FL;
// ESP32Encoder enc_FR;
// ESP32Encoder enc_RL;
// ESP32Encoder enc_RR;

// // CHANGED: Converted to an array to handle independent encoder resolutions
// const int TICKS_PER_REV[4] = {1300, 1300, 1300, 1300}; // FL, FR, RL, RR

// const unsigned long INNER_LOOP_MS = 10; 
// unsigned long last_inner_time = 0;

// const double WHEEL_RADIUS = 0.06;

// double actual_wheel_speeds[4] = {0.0, 0.0, 0.0, 0.0};
// long prev_counts[4] = {0, 0, 0, 0};

// double smoothed_wheel_speeds[4] = {0.0, 0.0, 0.0, 0.0};
// const double ALPHA = 0.3; // Adjust this between 0.1 (smoother) and 0.5 (snappier)

// // Motor Pin Definitions
// const int motor1_pin1 = 16, motor1_pin2 = 17; 
// const int motor2_pin1 = 18, motor2_pin2 = 19; 
// const int motor3_pin1 = 23, motor3_pin2 = 4; 
// const int motor4_pin1 = 13, motor4_pin2 = 14;

// // Encoder Pins 
// const int PIN_FL_ENC_A = 39; const int PIN_FL_ENC_B = 36;
// const int PIN_FR_ENC_A = 34; const int PIN_FR_ENC_B = 35;
// const int PIN_RL_ENC_A = 26; const int PIN_RL_ENC_B = 25;
// const int PIN_RR_ENC_A = 32; const int PIN_RR_ENC_B = 33;

// // LEDC CONFIGURATION (For ESP32 Core v2.x.x)
// const int CH_M1_P1 = 0; const int CH_M1_P2 = 1;
// const int CH_M2_P1 = 2; const int CH_M2_P2 = 3;
// const int CH_M3_P1 = 4; const int CH_M3_P2 = 5;
// const int CH_M4_P1 = 6; const int CH_M4_P2 = 7;

// const int PWM_FREQ = 5000;  
// const int PWM_RES  = 8;     

// void setMotor1(int pwm); void setMotor2(int pwm); 
// void setMotor3(int pwm); void setMotor4(int pwm);

// void setAllMotors(int p1, int p2, int p3, int p4) {
//     setMotor1(p1); setMotor2(p2); setMotor3(p3); setMotor4(p4);
// }

// void setMotor1(int pwm) {
//     int p = constrain((int)abs(pwm), 0, 150);
//     if (pwm >= 0) { ledcWrite(CH_M1_P1, p); ledcWrite(CH_M1_P2, 0); } 
//     else { ledcWrite(CH_M1_P1, 0); ledcWrite(CH_M1_P2, p); }
// }
// void setMotor2(int pwm) {
//     int p = constrain((int)abs(pwm), 0, 150);
//     if (pwm >= 0) { ledcWrite(CH_M2_P1, p); ledcWrite(CH_M2_P2, 0); } 
//     else { ledcWrite(CH_M2_P1, 0); ledcWrite(CH_M2_P2, p); }
// }
// void setMotor3(int pwm) {
//     int p = constrain((int)abs(pwm), 0, 150);
//     if (pwm >= 0) { ledcWrite(CH_M3_P1, p); ledcWrite(CH_M3_P2, 0); } 
//     else { ledcWrite(CH_M3_P1, 0); ledcWrite(CH_M3_P2, p); }
// }
// void setMotor4(int pwm) {
//     int p = constrain((int)abs(pwm), 0, 150);
//     if (pwm >= 0) { ledcWrite(CH_M4_P1, p); ledcWrite(CH_M4_P2, 0); } 
//     else { ledcWrite(CH_M4_P1, 0); ledcWrite(CH_M4_P2, p); }
// }


// void setup() {
//     Serial.begin(115200);
 

//     ESP32Encoder::useInternalWeakPullResistors = puType::up;
    
//     enc_FL.attachFullQuad(PIN_FL_ENC_A, PIN_FL_ENC_B);
//     enc_FR.attachFullQuad(PIN_FR_ENC_A, PIN_FR_ENC_B);
//     enc_RL.attachFullQuad(PIN_RL_ENC_A, PIN_RL_ENC_B);
//     enc_RR.attachFullQuad(PIN_RR_ENC_A, PIN_RR_ENC_B);
    
//     ledcSetup(CH_M1_P1, PWM_FREQ, PWM_RES); ledcSetup(CH_M1_P2, PWM_FREQ, PWM_RES);
//     ledcSetup(CH_M2_P1, PWM_FREQ, PWM_RES); ledcSetup(CH_M2_P2, PWM_FREQ, PWM_RES);
//     ledcSetup(CH_M3_P1, PWM_FREQ, PWM_RES); ledcSetup(CH_M3_P2, PWM_FREQ, PWM_RES);
//     ledcSetup(CH_M4_P1, PWM_FREQ, PWM_RES); ledcSetup(CH_M4_P2, PWM_FREQ, PWM_RES);

//     ledcAttachPin(motor1_pin1, CH_M1_P1); ledcAttachPin(motor1_pin2, CH_M1_P2);
//     ledcAttachPin(motor2_pin1, CH_M2_P1); ledcAttachPin(motor2_pin2, CH_M2_P2);
//     ledcAttachPin(motor3_pin1, CH_M3_P1); ledcAttachPin(motor3_pin2, CH_M3_P2);
//     ledcAttachPin(motor4_pin1, CH_M4_P1); ledcAttachPin(motor4_pin2, CH_M4_P2);
// }

// void loop() {


//     unsigned long now = millis();
//     if (now - last_inner_time >= INNER_LOOP_MS) {
//         double dt = (now - last_inner_time) / 1000.0;
//         last_inner_time = now;

//         long current_counts[4];
//         current_counts[0] = enc_FL.getCount();
//         current_counts[1] = enc_FR.getCount();
//         current_counts[2] = enc_RL.getCount();
//         current_counts[3] = enc_RR.getCount();

//         // CHANGED: Restored and updated mathematical processing loop
// for (int i = 0; i < 4; i++) {
//     long delta_ticks = current_counts[i] - prev_counts[i];
    
//     prev_counts[i] = current_counts[i];

//     double revolutions = (double)delta_ticks / TICKS_PER_REV[i];
//     double distance = revolutions * (2.0 * PI * WHEEL_RADIUS);
//     double raw_speed = distance / dt; 

//     // THE LOW PASS FILTER: Smooth out the raw spikes
//     smoothed_wheel_speeds[i] = (ALPHA * raw_speed) + ((1.0 - ALPHA) * smoothed_wheel_speeds[i]);
// }

// // Print the clean, smoothed speeds instead
// Serial.printf("Smooth Speeds -> FL: %.2f | FR: %.2f | RL: %.2f | RR: %.2f\n", 
//               smoothed_wheel_speeds[0], smoothed_wheel_speeds[1], 
//               smoothed_wheel_speeds[2], smoothed_wheel_speeds[3]);            

//         // Uncomment to activate motor testing loop
//         setAllMotors(150, 150, 150, 150);
//     }
// }






////////////////////////ENCODER TICKS??????????????????????????????????????????????????????????????
#include <math.h>
#include <Arduino.h>
#include <ESP32Encoder.h> 

// Hardware Instance Handles
ESP32Encoder enc_FL;
ESP32Encoder enc_FR;
ESP32Encoder enc_RL;
ESP32Encoder enc_RR;

const int TICKS_PER_REV = 1300;
const unsigned long INNER_LOOP_MS = 10; 
unsigned long last_inner_time = 0;

const double WHEEL_RADIUS = 0.05;

double actual_wheel_speeds[4] = {0.0, 0.0, 0.0, 0.0};
long prev_counts[4] = {0, 0, 0, 0}; 

// Motor Pin Definitions
const int motor1_pin1 = 16, motor1_pin2 = 17; 
const int motor2_pin1 = 18, motor2_pin2 = 19; 
const int motor3_pin1 = 23, motor3_pin2 = 4; 
const int motor4_pin1 = 13, motor4_pin2 = 14;

// Encoder Pins 
const int PIN_FL_ENC_A = 36; const int PIN_FL_ENC_B = 39;
const int PIN_FR_ENC_A = 34; const int PIN_FR_ENC_B = 35;
const int PIN_RL_ENC_A = 25; const int PIN_RL_ENC_B = 26;
const int PIN_RR_ENC_A = 32; const int PIN_RR_ENC_B = 33;

// LEDC CONFIGURATION (For ESP32 Core v2.x.x)
const int CH_M1_P1 = 0; const int CH_M1_P2 = 1;
const int CH_M2_P1 = 2; const int CH_M2_P2 = 3;
const int CH_M3_P1 = 4; const int CH_M3_P2 = 5;
const int CH_M4_P1 = 6; const int CH_M4_P2 = 7;

const int PWM_FREQ = 5000;  
const int PWM_RES  = 8;     

void setMotor1(int pwm); void setMotor2(int pwm); 
void setMotor3(int pwm); void setMotor4(int pwm);

void setAllMotors(int p1, int p2, int p3, int p4) {
    setMotor1(p1); setMotor2(p2); setMotor3(p3); setMotor4(p4);
}

void setMotor1(int pwm) {
    int p = constrain((int)abs(pwm), 0, 150);
    if (pwm >= 0) { ledcWrite(CH_M1_P1, p); ledcWrite(CH_M1_P2, 0); } 
    else { ledcWrite(CH_M1_P1, 0); ledcWrite(CH_M1_P2, p); }
}
void setMotor2(int pwm) {
    int p = constrain((int)abs(pwm), 0, 150);
    if (pwm >= 0) { ledcWrite(CH_M2_P1, p); ledcWrite(CH_M2_P2, 0); } 
    else { ledcWrite(CH_M2_P1, 0); ledcWrite(CH_M2_P2, p); }
}
void setMotor3(int pwm) {
    int p = constrain((int)abs(pwm), 0, 150);
    if (pwm >= 0) { ledcWrite(CH_M3_P1, p); ledcWrite(CH_M3_P2, 0); } 
    else { ledcWrite(CH_M3_P1, 0); ledcWrite(CH_M3_P2, p); }
}
void setMotor4(int pwm) {
    int p = constrain((int)abs(pwm), 0, 150);
    if (pwm >= 0) { ledcWrite(CH_M4_P1, p); ledcWrite(CH_M4_P2, 0); } 
    else { ledcWrite(CH_M4_P1, 0); ledcWrite(CH_M4_P2, p); }
}

void setup() {
    Serial.begin(115200);

    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    
    enc_FL.attachFullQuad(PIN_FL_ENC_A, PIN_FL_ENC_B);
    enc_FR.attachFullQuad(PIN_FR_ENC_A, PIN_FR_ENC_B);
    enc_RL.attachFullQuad(PIN_RL_ENC_A, PIN_RL_ENC_B);
    enc_RR.attachFullQuad(PIN_RR_ENC_A, PIN_RR_ENC_B);
    
    ledcSetup(CH_M1_P1, PWM_FREQ, PWM_RES); ledcSetup(CH_M1_P2, PWM_FREQ, PWM_RES);
    ledcSetup(CH_M2_P1, PWM_FREQ, PWM_RES); ledcSetup(CH_M2_P2, PWM_FREQ, PWM_RES);
    ledcSetup(CH_M3_P1, PWM_FREQ, PWM_RES); ledcSetup(CH_M3_P2, PWM_FREQ, PWM_RES);
    ledcSetup(CH_M4_P1, PWM_FREQ, PWM_RES); ledcSetup(CH_M4_P2, PWM_FREQ, PWM_RES);

    ledcAttachPin(motor1_pin1, CH_M1_P1); ledcAttachPin(motor1_pin2, CH_M1_P2);
    ledcAttachPin(motor2_pin1, CH_M2_P1); ledcAttachPin(motor2_pin2, CH_M2_P2);
    ledcAttachPin(motor3_pin1, CH_M3_P1); ledcAttachPin(motor3_pin2, CH_M3_P2);
    ledcAttachPin(motor4_pin1, CH_M4_P1); ledcAttachPin(motor4_pin2, CH_M4_P2);
}

void loop() {
    unsigned long now = millis();
    if (now - last_inner_time >= INNER_LOOP_MS) {
        last_inner_time = now;

        long current_counts[4];
        current_counts[0] = enc_FL.getCount();
        current_counts[1] = enc_FR.getCount();
        current_counts[2] = enc_RL.getCount();
        current_counts[3] = enc_RR.getCount();

        // Fixed print syntax: Changed %.2f to %ld for long integers
        Serial.printf("Raw Ticks -> FL: %ld | FR: %ld | RL: %ld | RR: %ld\n", 
                      current_counts[0], current_counts[1], 
                      current_counts[2], current_counts[3]);              

        // Optional: Uncomment this line when you want to spin the wheels to test
        // setAllMotors(150, 150, 150, 150);
    }
}


