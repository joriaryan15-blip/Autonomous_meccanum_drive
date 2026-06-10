#include <Arduino.h>
#include <ESP32Encoder.h> 
#include <math.h>

// ─────────────────────────────────────────────────────────────────────────────
// 1. HARDWARE PIN DEFINITIONS & LEDC CHANNELS (ESP32 Core v2.x.x)
// ─────────────────────────────────────────────────────────────────────────────
// Front Left Motor (Motor 1)
const int PIN_FL_RPWM = 16; const int PIN_FL_LPWM = 17;
const int CH_FL_RPWM  = 0;  const int CH_FL_LPWM  = 1;

// Front Right Motor (Motor 2)
const int PIN_FR_RPWM = 18; const int PIN_FR_LPWM = 19;
const int CH_FR_RPWM  = 2;  const int CH_FR_LPWM  = 3;

// Rear Left Motor (Motor 3)
const int PIN_RL_RPWM = 23; const int PIN_RL_LPWM = 4;
const int CH_RL_RPWM  = 4;  const int CH_RL_LPWM  = 5;

// Rear Right Motor (Motor 4)
const int PIN_RR_RPWM = 13; const int PIN_RR_LPWM = 14;
const int CH_RR_RPWM  = 6;  const int CH_RR_LPWM  = 7;

// Encoder Pins 
const int PIN_FL_ENC_A = 36; const int PIN_FL_ENC_B = 39;
const int PIN_FR_ENC_A = 34; const int PIN_FR_ENC_B = 35;
const int PIN_RL_ENC_A = 25; const int PIN_RL_ENC_B = 26;
const int PIN_RR_ENC_A = 32; const int PIN_RR_ENC_B = 33;

// PWM Properties
const int PWM_FREQ = 5000;  
const int PWM_RES  = 8;     

// ─────────────────────────────────────────────────────────────────────────────
// 2. ROBOT GEOMETRY & TUNING CONFIGURATIONS
// ─────────────────────────────────────────────────────────────────────────────
const double WHEEL_RADIUS = 0.05; // 50mm
const double LX = 0.30;           
const double LY = 0.30;           
const int TICKS_PER_REV = 1300;   

// UPDATED: Set maximum safe speed ceiling to your target max limit
const double MAX_SAFE_SPEED_MS = 1.0; 

// Tuning Gains
double KP_INNER_VEL = 700.0; // Starting Proportional Gain estimate
double KI_INNER_VEL = 0.0;  // Kept at 0.0 for initial P-only tuning

// Low Pass Filter 
double smoothed_wheel_speeds[4] = {0.0, 0.0, 0.0, 0.0};
const double ALPHA = 0.3; 

// Core Loop Timing
const unsigned long INNER_LOOP_MS = 10; 
unsigned long last_inner_time = 0;

// Kinematic Motion Commands (Robot Level Targets)
double target_robot_vx = 1.5; // Will automatically scale to max limits safely
double target_robot_vy = 0.0;
double target_robot_omega = 0.0;

double target_wheel_speeds[4] = {0.0, 0.0, 0.0, 0.0};
double actual_wheel_speeds[4] = {0.0, 0.0, 0.0, 0.0};
long prev_counts[4]           = {0, 0, 0, 0};
double wheel_integrals[4]     = {0.0, 0.0, 0.0, 0.0};

// Hardware Instance Handles
ESP32Encoder enc_FL;
ESP32Encoder enc_FR;
ESP32Encoder enc_RL;
ESP32Encoder enc_RR;

// ─────────────────────────────────────────────────────────────────────────────
// 3. LEDC LOW-LEVEL MOTOR DRIVER FUNCTIONS
// ─────────────────────────────────────────────────────────────────────────────
void setMotor1(int pwm) { // Front Left
    int p = constrain((int)abs(pwm), 0, 255);
    if (pwm >= 0) { ledcWrite(CH_FL_RPWM, p); ledcWrite(CH_FL_LPWM, 0); } 
    else          { ledcWrite(CH_FL_RPWM, 0); ledcWrite(CH_FL_LPWM, p); }
}

void setMotor2(int pwm) { // Front Right
    int p = constrain((int)abs(pwm), 0, 255);
    if (pwm >= 0) { ledcWrite(CH_FR_RPWM, p); ledcWrite(CH_FR_LPWM, 0); } 
    else          { ledcWrite(CH_FR_RPWM, 0); ledcWrite(CH_FR_LPWM, p); }
}

void setMotor3(int pwm) { // Rear Left
    int p = constrain((int)abs(pwm), 0, 255);
    if (pwm >= 0) { ledcWrite(CH_RL_RPWM, p); ledcWrite(CH_RL_LPWM, 0); } 
    else          { ledcWrite(CH_RL_RPWM, 0); ledcWrite(CH_RL_LPWM, p); }
}

void setMotor4(int pwm) { // Rear Right
    int p = constrain((int)abs(pwm), 0, 255);
    if (pwm >= 0) { ledcWrite(CH_RR_RPWM, p); ledcWrite(CH_RR_LPWM, 0); } 
    else          { ledcWrite(CH_RR_RPWM, 0); ledcWrite(CH_RR_LPWM, p); }
}

// ─────────────────────────────────────────────────────────────────────────────
// 4. KINEMATICS & SERIAL INTERACTION
// ─────────────────────────────────────────────────────────────────────────────
void mecanumInverseKinematics(double vx, double vy, double omega, double* speeds) {
    speeds[0] = vx - vy - (LX + LY) * omega; // FL
    speeds[1] = vx + vy + (LX + LY) * omega; // FR
    speeds[2] = vx + vy - (LX + LY) * omega; // RL
    speeds[3] = vx - vy + (LX + LY) * omega; // RR
}

void checkSerialKpTuning() {
    if (Serial.available() > 0) {
        String inputStr = Serial.readStringUntil('\n'); 
        inputStr.trim(); 
        if (inputStr.length() > 0) {
            KP_INNER_VEL = inputStr.toDouble(); 
            Serial.print(">>> Dynamic KP updated to: ");
            Serial.println(KP_INNER_VEL);
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 5. SETUP & MAIN LOOP
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    ledcSetup(CH_FL_RPWM, PWM_FREQ, PWM_RES); ledcSetup(CH_FL_LPWM, PWM_FREQ, PWM_RES);
    ledcSetup(CH_FR_RPWM, PWM_FREQ, PWM_RES); ledcSetup(CH_FR_LPWM, PWM_FREQ, PWM_RES);
    ledcSetup(CH_RL_RPWM, PWM_FREQ, PWM_RES); ledcSetup(CH_RL_LPWM, PWM_FREQ, PWM_RES);
    ledcSetup(CH_RR_RPWM, PWM_FREQ, PWM_RES); ledcSetup(CH_RR_LPWM, PWM_FREQ, PWM_RES);

    ledcAttachPin(PIN_FL_RPWM, CH_FL_RPWM); ledcAttachPin(PIN_FL_LPWM, CH_FL_LPWM);
    ledcAttachPin(PIN_FR_RPWM, CH_FR_RPWM); ledcAttachPin(PIN_FR_LPWM, CH_FR_LPWM);
    ledcAttachPin(PIN_RL_RPWM, CH_RL_RPWM); ledcAttachPin(PIN_RL_LPWM, CH_RL_LPWM);
    ledcAttachPin(PIN_RR_RPWM, CH_RR_RPWM); ledcAttachPin(PIN_RR_LPWM, CH_RR_LPWM);

    ESP32Encoder::useInternalWeakPullResistors = puType::up;

    enc_FL.attachFullQuad(PIN_FL_ENC_A, PIN_FL_ENC_B);
    enc_FR.attachFullQuad(PIN_FR_ENC_A, PIN_FR_ENC_B);
    enc_RL.attachFullQuad(PIN_RL_ENC_A, PIN_RL_ENC_B);
    enc_RR.attachFullQuad(PIN_RR_ENC_A, PIN_RR_ENC_B);

    Serial.println("Kinematics Tuning System online. Type your new KP value into the input box.");
}

void loop() {
    checkSerialKpTuning();
    unsigned long now = millis();

    if (now - last_inner_time >= INNER_LOOP_MS) {
        double dt = (now - last_inner_time) / 1000.0;
        last_inner_time = now;

        long current_counts[4];
        current_counts[0] = enc_FL.getCount();
        current_counts[1] = enc_FR.getCount();
        current_counts[2] = enc_RL.getCount();
        current_counts[3] = enc_RR.getCount();

        // Process raw math safely without clearCount() register wipes
        for (int i = 0; i < 4; i++) {
            long delta_ticks = current_counts[i] - prev_counts[i];
            
            // Software direction correction for Left side motors (FL and RL)
            if (i == 0 || i == 2) { delta_ticks = -delta_ticks; }

            prev_counts[i] = current_counts[i];

            double revolutions = (double)delta_ticks / TICKS_PER_REV;
            double distance = revolutions * (2.0 * PI * WHEEL_RADIUS);
            double raw_speed = distance / dt; 

            smoothed_wheel_speeds[i] = (ALPHA * raw_speed) + ((1.0 - ALPHA) * smoothed_wheel_speeds[i]);
            actual_wheel_speeds[i] = smoothed_wheel_speeds[i];
        }

        // Generate target wheel velocities
        mecanumInverseKinematics(target_robot_vx, target_robot_vy, target_robot_omega, target_wheel_speeds);

        // Cap velocities scaling down linearly if limits are exceeded
        double max_requested = 0.0;
        for(int i = 0; i < 4; i++) {
            if(abs(target_wheel_speeds[i]) > max_requested) {
                max_requested = abs(target_wheel_speeds[i]);
            }
        }
        if (max_requested > MAX_SAFE_SPEED_MS) {
            double scale = MAX_SAFE_SPEED_MS / max_requested;
            for(int i = 0; i < 4; i++) {
                target_wheel_speeds[i] *= scale;
            }
        }

        // Execute PID controller loop
        for (int i = 0; i < 4; i++) {
            double error = target_wheel_speeds[i] - actual_wheel_speeds[i];
            
            // Integral tracking (reserved for later steps)
            wheel_integrals[i] += error * dt;
            wheel_integrals[i] = constrain(wheel_integrals[i], -10.0, 10.0);

            // FEEDFORWARD CALCULATION: Maps 1.5 m/s linearly to 122.4 PWM
            double ff_term = target_wheel_speeds[i] * (122.4 / 1.5);

            // PROPORTIONAL TERM
            double p_term = error * KP_INNER_VEL;
            
            // INTEGRAL TERM
            double i_term = wheel_integrals[i] * KI_INNER_VEL;

            // Combine calculations into a single command output
            double raw_pwm_command = ff_term + p_term + i_term;

            // Constrain within hardware safety boundaries
            int final_pwm = constrain((int)raw_pwm_command, -255, 255);

            switch(i) {
                case 0: setMotor1(final_pwm); break;
                case 1: setMotor2(final_pwm); break;
                case 2: setMotor3(final_pwm); break;
                case 3: setMotor4(final_pwm); break;
            }
        }

        // Output format optimized for the Arduino Serial Plotter
        Serial.printf("Target:%.2f,FL:%.2f,FR:%.2f,RL:%.2f,RR:%.2f\n", 
                      target_wheel_speeds[0], actual_wheel_speeds[0], actual_wheel_speeds[1], 
                      actual_wheel_speeds[2], actual_wheel_speeds[3]);
    } 
}
