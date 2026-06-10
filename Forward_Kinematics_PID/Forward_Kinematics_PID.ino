///////////////////FORWARD KINEMATICS PID?//////////////////////////////////////////////////



#include <Arduino.h>
#include <ESP32Encoder.h> 
#include <Adafruit_BNO08x.h>
#include <math.h>

// Target
double TARGET_DISTANCE_X = 0.0;
double TARGET_DISTANCE_Y = 0.0; 
double TARGET_ANGLE_DEG  = 90.0;

// BNO08x Global Setup
#define BNO08X_RESET -1
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
double imu_yaw_rad = 0.0; // Global tracking of IMU heading

// ─────────────────────────────────────────────────────────────────────────────
// 1. HARDWARE PIN DEFINITIONS & CONFIGURATIONS
// ─────────────────────────────────────────────────────────────────────────────
const int PIN_FL_RPWM = 16; const int PIN_FL_LPWM = 17;
const int CH_FL_RPWM  = 0;  const int CH_FL_LPWM  = 1;
const int PIN_FR_RPWM = 18; const int PIN_FR_LPWM = 19;
const int CH_FR_RPWM  = 2;  const int CH_FR_LPWM  = 3;
const int PIN_RL_RPWM = 23; const int PIN_RL_LPWM = 4;
const int CH_RL_RPWM  = 4;  const int CH_RL_LPWM  = 5;
const int PIN_RR_RPWM = 13; const int PIN_RR_LPWM = 14;
const int CH_RR_RPWM  = 6;  const int CH_RR_LPWM  = 7;

const int PIN_FL_ENC_A = 39; const int PIN_FL_ENC_B = 36;
const int PIN_FR_ENC_A = 34; const int PIN_FR_ENC_B = 35;
const int PIN_RL_ENC_A = 26; const int PIN_RL_ENC_B = 25;
const int PIN_RR_ENC_A = 32; const int PIN_RR_ENC_B = 33;

const int PWM_FREQ = 5000;  const int PWM_RES  = 8;     
const double WHEEL_RADIUS = 0.05; // 50mm
const double LX = 0.30; const double LY = 0.30;           
const int TICKS_PER_REV = 1300;   

// Timing
const unsigned long INNER_LOOP_MS = 10; 
unsigned long last_inner_time = 0;

// Encoders & Wheel Filtering
ESP32Encoder enc_FL; ESP32Encoder enc_FR; ESP32Encoder enc_RL; ESP32Encoder enc_RR;
double smoothed_wheel_speeds[4] = {0.0, 0.0, 0.0, 0.0};
const double ALPHA = 0.3; 
long prev_counts[4] = {0, 0, 0, 0};

// ─────────────────────────────────────────────────────────────────────────────
// 2. CASCADED PID TUNING VARIABLES
// ─────────────────────────────────────────────────────────────────────────────
double KP_INNER_VEL = 700.0; 
double KI_INNER_VEL = 0.0;  
double actual_wheel_speeds[4] = {0.0, 0.0, 0.0, 0.0};
double target_wheel_speeds[4] = {0.0, 0.0, 0.0, 0.0};
double wheel_integrals[4]     = {0.0, 0.0, 0.0, 0.0};

double KP_DIST = 2.5; 
double KP_ANGLE = 4.0; 

const double MAX_ALLOWED_SPEED_MS = 0.5;   
const double MAX_ALLOWED_OMEGA_RAD = 1.5;  

// Real-Time Odometry States (Absolute positions)
double robot_pose_x = 0.0;
double robot_pose_y = 0.0;

// Robot Velocity States
double actual_robot_vx = 0.0, actual_robot_vy = 0.0, actual_robot_omega = 0.0;
double target_robot_vx = 0.0, target_robot_vy = 0.0, target_robot_omega = 0.0;

// ─────────────────────────────────────────────────────────────────────────────
// 3. LOW LEVEL DRIVERS & KINEMATICS MATRIX FUNCTIONS
// ─────────────────────────────────────────────────────────────────────────────
void setMotor1(int pwm) { int p = constrain((int)abs(pwm), 0, 255); if (pwm >= 0) { ledcWrite(CH_FL_RPWM, p); ledcWrite(CH_FL_LPWM, 0); } else { ledcWrite(CH_FL_RPWM, 0); ledcWrite(CH_FL_LPWM, p); } }
void setMotor2(int pwm) { int p = constrain((int)abs(pwm), 0, 255); if (pwm >= 0) { ledcWrite(CH_FR_RPWM, p); ledcWrite(CH_FR_LPWM, 0); } else { ledcWrite(CH_FR_RPWM, 0); ledcWrite(CH_FR_LPWM, p); } }
void setMotor3(int pwm) { int p = constrain((int)abs(pwm), 0, 255); if (pwm >= 0) { ledcWrite(CH_RL_RPWM, p); ledcWrite(CH_RL_LPWM, 0); } else { ledcWrite(CH_RL_RPWM, 0); ledcWrite(CH_RL_LPWM, p); } }
void setMotor4(int pwm) { int p = constrain((int)abs(pwm), 0, 255); if (pwm >= 0) { ledcWrite(CH_RR_RPWM, p); ledcWrite(CH_RR_LPWM, 0); } else { ledcWrite(CH_RR_RPWM, 0); ledcWrite(CH_RR_LPWM, p); } }

void mecanumInverseKinematics(double vx, double vy, double omega, double* speeds) {
    speeds[0] = vx - vy - (LX + LY) * omega; // FL
    speeds[1] = vx + vy + (LX + LY) * omega; // FR
    speeds[2] = vx + vy - (LX + LY) * omega; // RL
    speeds[3] = vx - vy + (LX + LY) * omega; // RR
}

void mecanumForwardKinematics(double* w_speeds, double& vx, double& vy, double& omega) {
    vx = (w_speeds[0] + w_speeds[1] + w_speeds[2] + w_speeds[3]) / 4.0;
    vy = (-w_speeds[0] + w_speeds[1] + w_speeds[2] - w_speeds[3]) / 4.0;
    omega = (-w_speeds[0] + w_speeds[1] - w_speeds[2] + w_speeds[3]) / (4.0 * (LX + LY));
}

void setReports(void) {
    Serial.println("Setting desired reports");
    if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
        Serial.println("Could not enable game vector");
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 4. SETUP & MAIN LOOP
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
    enc_FL.attachFullQuad(PIN_FL_ENC_A, PIN_FL_ENC_B); enc_FR.attachFullQuad(PIN_FR_ENC_A, PIN_FR_ENC_B);
    enc_RL.attachFullQuad(PIN_RL_ENC_A, PIN_RL_ENC_B); enc_RR.attachFullQuad(PIN_RR_ENC_A, PIN_RR_ENC_B);

    // Note: I2C address changed back to standard 0x4A unless your ADR pin is pulled high to 0x4B
    if (!bno08x.begin_I2C(0x4B)) {
        Serial.println("Failed to find BNO08x chip");
        while (1) { delay(10); }
    }
    Serial.println("BNO08x Found!");
    setReports();
    delay(100);
}

void loop() {
    unsigned long now = millis();

    // Check IMU asynchronously (NO loop breaking 'return;')
    if (bno08x.wasReset()) {
        setReports();
    }

    if (bno08x.getSensorEvent(&sensorValue)) {
        if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
            float r = sensorValue.un.gameRotationVector.real;
            float i = sensorValue.un.gameRotationVector.i;
            float j = sensorValue.un.gameRotationVector.j;
            float k = sensorValue.un.gameRotationVector.k;

            // Save continuously as Global Radians
            imu_yaw_rad = atan2(2.0f * (r * k + i * j), 1.0f - 2.0f * (j * j + k * k));
        }
    }

    // Fixed Inner Loop Execute block
    if (now - last_inner_time >= INNER_LOOP_MS) {
        double dt = (now - last_inner_time) / 1000.0;
        last_inner_time = now;

        // Read Hardware Encoders
        long current_counts[4] = {enc_FL.getCount(), enc_FR.getCount(), enc_RL.getCount(), enc_RR.getCount()};

        for (int i = 0; i < 4; i++) {
            long delta_ticks = current_counts[i] - prev_counts[i]; 
            prev_counts[i] = current_counts[i];

            double revolutions = (double)delta_ticks / TICKS_PER_REV;
            double distance = revolutions * (2.0 * PI * WHEEL_RADIUS);
            double raw_speed = distance / dt; 

            smoothed_wheel_speeds[i] = (ALPHA * raw_speed) + ((1.0 - ALPHA) * smoothed_wheel_speeds[i]);
            actual_wheel_speeds[i] = smoothed_wheel_speeds[i];
        }

        // Run Forward Kinematics 
        mecanumForwardKinematics(actual_wheel_speeds, actual_robot_vx, actual_robot_vy, actual_robot_omega);
        
        double delta_x = actual_robot_vx * dt;
        double delta_y = actual_robot_vy * dt;

        // CRITICAL FIX: Use IMU Heading (imu_yaw_rad) instead of drifted encoder-odometry math
        robot_pose_x += delta_x * cos(imu_yaw_rad) - delta_y * sin(imu_yaw_rad);
        robot_pose_y += delta_x * sin(imu_yaw_rad) + delta_y * cos(imu_yaw_rad);

        // Calculate positioning discrepancies on global grid
        double global_error_x = TARGET_DISTANCE_X - robot_pose_x;
        double global_error_y = TARGET_DISTANCE_Y - robot_pose_y;
        
        // Target calculation unit match (Radians everywhere)
        double target_angle_rad = TARGET_ANGLE_DEG * (PI / 180.0);
        double error_theta = target_angle_rad - imu_yaw_rad; 

        // Angle normalization (-PI to +PI)
        while (error_theta > PI)  error_theta -= 2.0 * PI;
        while (error_theta < -PI) error_theta += 2.0 * PI;

        // 2D Frame Transformation Matrix
        double local_error_x = global_error_x * cos(imu_yaw_rad) + global_error_y * sin(imu_yaw_rad);
        double local_error_y = -global_error_x * sin(imu_yaw_rad) + global_error_y * cos(imu_yaw_rad);

        double linear_error_distance = sqrt(global_error_x * global_error_x + global_error_y * global_error_y);

        // Linear Target Allocation
        if (linear_error_distance < 0.005) {
            target_robot_vx = 0.0;
            target_robot_vy = 0.0;
        } else {
            target_robot_vx = local_error_x * KP_DIST;
            target_robot_vy = local_error_y * KP_DIST;
        }

        // Rotational Target Allocation
        if (abs(error_theta) < 0.02) {
            target_robot_omega = 0.0;
        } else {
            target_robot_omega = error_theta * KP_ANGLE;
        }

        // Speed caps 
        target_robot_vx = constrain(target_robot_vx, -MAX_ALLOWED_SPEED_MS, MAX_ALLOWED_SPEED_MS);
        target_robot_vy = constrain(target_robot_vy, -MAX_ALLOWED_SPEED_MS, MAX_ALLOWED_SPEED_MS);
        target_robot_omega = constrain(target_robot_omega, -MAX_ALLOWED_OMEGA_RAD, MAX_ALLOWED_OMEGA_RAD);

        // Inverse Kinematics Engine
        mecanumInverseKinematics(target_robot_vx, target_robot_vy, target_robot_omega, target_wheel_speeds);

        for (int i = 0; i < 4; i++) {
            double error = target_wheel_speeds[i] - actual_wheel_speeds[i];
            wheel_integrals[i] = constrain(wheel_integrals[i] + error * dt, -10.0, 10.0);

            double ff_term = target_wheel_speeds[i] * (122.4 / 1.5);
            double p_term  = error * KP_INNER_VEL;
            double i_term  = wheel_integrals[i] * KI_INNER_VEL;

            int final_pwm = constrain((int)(ff_term + p_term + i_term), -255, 255);

            switch(i) {
                case 0: setMotor1(final_pwm); break;
                case 1: setMotor2(final_pwm); break;
                case 2: setMotor3(final_pwm); break;
                case 3: setMotor4(final_pwm); break;
            }
        }

        // Clean values for readable monitoring telemetry
        double current_angle_deg = imu_yaw_rad * (180.0 / PI);
        Serial.printf("Glob_X:%.2fm | Glob_Y:%.2fm | Ang:%.1f° | Cmd_Vx:%.2f | Cmd_Vy:%.2f | Cmd_W:%.2f\n", 
                      robot_pose_x, robot_pose_y, current_angle_deg, target_robot_vx, target_robot_vy, target_robot_omega);
    } 
}
