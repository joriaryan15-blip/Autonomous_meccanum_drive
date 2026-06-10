


///////////////////////////////////_____________________WAY_POINT___________________///////////////////////////////////////////////////////////////

#include <Arduino.h>
#include <ESP32Encoder.h> 
#include <Adafruit_BNO08x.h>
#include <math.h>

// ─────────────────────────────────────────────────────────────────────────────
// 0. GLOBAL WAYPOINT TRACKING SYSTEM
// ─────────────────────────────────────────────────────────────────────────────
struct Waypoint {
    double x;
    double y;
    double angle_deg;
};

const int trig = 3;

// Define your list of global targets here
const Waypoint WAYPOINT_QUEUE[] = {
    {0.50, 0.80, 90.0}  // Waypoint 1 (Your original target)
     // Waypoint 2 
};
const int TOTAL_WAYPOINTS = sizeof(WAYPOINT_QUEUE) / sizeof(Waypoint);
int current_waypoint_index = 0;
bool path_completed = false;

// Robot Velocity States
double actual_robot_vx = 0.0, actual_robot_vy = 0.0, actual_robot_omega = 0.0;
double target_robot_vx = 0.0, target_robot_vy = 0.0, target_robot_omega = 0.0;

// Active targets updated dynamically from the queue
double TARGET_DISTANCE_X = 0.0;
double TARGET_DISTANCE_Y = 0.0;
double TARGET_ANGLE_DEG  = 0.0;

// Settle Timer Configurations for Arrival Verification
unsigned long zone_entry_time = 0;
bool inside_target_zone = false;
const unsigned long SETTLE_DURATION_MS = 1000; 

// ─────────────────────────────────────────────────────────────────────────────
// ANTI-STICTION MINIMUM VELOCITY THRESHOLDS (ROBOT-LEVEL)
// ─────────────────────────────────────────────────────────────────────────────
// Adjust these empirical limits based on your chassis weight and floor surface
const double MIN_VX_FLOOR    = 0.05;  // Min forward/backward output (m/s)
const double MIN_VY_FLOOR    = 0.05;  // Min sideways strafe output (m/s)
const double MIN_OMEGA_FLOOR = 0.15;  // Min turning/yaw twist output (rad/s)

// Absolute target deadzones where the floors turn off completely to stop drift
const double STOP_TOLERANCE_DIST  = 0.02; // 2 cm positional threshold
const double STOP_TOLERANCE_ANGLE = 0.04; // ~2.3 degrees rotational threshold

// BNO08x Global Setup
#define BNO08X_RESET -1
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
double imu_yaw_rad = 0.0; 

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
const double WHEEL_RADIUS = 0.05; 
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
double KP_INNER_VEL = 600.0; 
double KI_INNER_VEL = 0.0;  
double actual_wheel_speeds[4] = {0.0, 0.0, 0.0, 0.0};
double target_wheel_speeds[4] = {0.0, 0.0, 0.0, 0.0};
double wheel_integrals[4]     = {0.0, 0.0, 0.0, 0.0};

double KP_DIST = 2.5; 
double KP_ANGLE = 1.0; 

const double MAX_ALLOWED_SPEED_MS = 0.5;   
const double MAX_ALLOWED_OMEGA_RAD = 2.0;  

// Real-Time Odometry States (Absolute positions)
double robot_pose_x = 0.0;
double robot_pose_y = 0.0;

// ─────────────────────────────────────────────────────────────────────────────
// 3. LOW LEVEL DRIVERS & KINEMATICS MATRIX FUNCTIONS
// ─────────────────────────────────────────────────────────────────────────────
void setMotor1(int pwm) { int p = constrain((int)abs(pwm), 0, 255); if (pwm >= 0) { ledcWrite(CH_FL_RPWM, p); ledcWrite(CH_FL_LPWM, 0); } else { ledcWrite(CH_FL_RPWM, 0); ledcWrite(CH_FL_LPWM, p); } }
void setMotor2(int pwm) { int p = constrain((int)abs(pwm), 0, 255); if (pwm >= 0) { ledcWrite(CH_FR_RPWM, p); ledcWrite(CH_FR_LPWM, 0); } else { ledcWrite(CH_FR_RPWM, 0); ledcWrite(CH_FR_LPWM, p); } }
void setMotor3(int pwm) { int p = constrain((int)abs(pwm), 0, 255); if (pwm >= 0) { ledcWrite(CH_RL_RPWM, p); ledcWrite(CH_RL_LPWM, 0); } else { ledcWrite(CH_RL_RPWM, 0); ledcWrite(CH_RL_LPWM, p); } }
void setMotor4(int pwm) { int p = constrain((int)abs(pwm), 0, 255); if (pwm >= 0) { ledcWrite(CH_RR_RPWM, p); ledcWrite(CH_RR_LPWM, 0); } else { ledcWrite(CH_RR_RPWM, 0); ledcWrite(CH_RR_LPWM, p); } }

void stopAllMotors() {
    setMotor1(0); setMotor2(0); setMotor3(0); setMotor4(0);
    for(int i=0; i<4; i++) wheel_integrals[i] = 0.0;
}

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

    pinMode(trig, OUTPUT);

    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    enc_FL.attachFullQuad(PIN_FL_ENC_A, PIN_FL_ENC_B); enc_FR.attachFullQuad(PIN_FR_ENC_A, PIN_FR_ENC_B);
    enc_RL.attachFullQuad(PIN_RL_ENC_A, PIN_RL_ENC_B); enc_RR.attachFullQuad(PIN_RR_ENC_A, PIN_RR_ENC_B);

    if (!bno08x.begin_I2C(0x4B)) {
        Serial.println("Failed to find BNO08x chip");
        while (1) { delay(10); }
    }
    Serial.println("BNO08x Found!");
    setReports();
    delay(100);
    
    Serial.println("--- WAYPOINT SYSTEM ONLINE ---");
}

void loop() {
    unsigned long now = millis();

    if (bno08x.wasReset()) {
        setReports();
    }

    if (bno08x.getSensorEvent(&sensorValue)) {
        if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
            float r = sensorValue.un.gameRotationVector.real;
            float i = sensorValue.un.gameRotationVector.i;
            float j = sensorValue.un.gameRotationVector.j;
            float k = sensorValue.un.gameRotationVector.k;

            imu_yaw_rad = atan2(2.0f * (r * k + i * j), 1.0f - 2.0f * (j * j + k * k));
        }
    }

    if (now - last_inner_time >= INNER_LOOP_MS) {
        double dt = (now - last_inner_time) / 1000.0;
        last_inner_time = now;

        // Update current targets from global waypoint queue
        if (!path_completed) {
            TARGET_DISTANCE_X = WAYPOINT_QUEUE[current_waypoint_index].x;
            TARGET_DISTANCE_Y = WAYPOINT_QUEUE[current_waypoint_index].y;
            TARGET_ANGLE_DEG  = WAYPOINT_QUEUE[current_waypoint_index].angle_deg;
        } else {
            stopAllMotors();
            Serial.println(">>> ALL GLOBAL WAYPOINTS ACCOMPLISHED! RUN COMPLETE. <<<");
            delay(1000); 
            return;
        }

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

        mecanumForwardKinematics(actual_wheel_speeds, actual_robot_vx, actual_robot_vy, actual_robot_omega);
        
        double delta_x = actual_robot_vx * dt;
        double delta_y = actual_robot_vy * dt;

        robot_pose_x += delta_x * cos(imu_yaw_rad) - delta_y * sin(imu_yaw_rad);
        robot_pose_y += delta_x * sin(imu_yaw_rad) + delta_y * cos(imu_yaw_rad);

        double global_error_x = TARGET_DISTANCE_X - robot_pose_x;
        double global_error_y = TARGET_DISTANCE_Y - robot_pose_y;
        
        double target_angle_rad = TARGET_ANGLE_DEG * (PI / 180.0);
        double error_theta = target_angle_rad - imu_yaw_rad; 

        while (error_theta > PI)  error_theta -= 2.0 * PI;
        while (error_theta < -PI) error_theta += 2.0 * PI;

        double local_error_x = global_error_x * cos(imu_yaw_rad) + global_error_y * sin(imu_yaw_rad);
        double local_error_y = -global_error_x * sin(imu_yaw_rad) + global_error_y * cos(imu_yaw_rad);

        double linear_error_distance = sqrt(global_error_x * global_error_x + global_error_y * global_error_y);

        // ─────────────────────────────────────────────────────────────────────
        // ARRIVAL CONDITION & WAYPOINT TRANSITION LOGIC
        // ─────────────────────────────────────────────────────────────────────
        if (linear_error_distance < STOP_TOLERANCE_DIST && abs(error_theta) < STOP_TOLERANCE_ANGLE) {
            if (!inside_target_zone) {
                inside_target_zone = true;
                zone_entry_time = now; 
            } else if (now - zone_entry_time >= SETTLE_DURATION_MS) {
                Serial.printf("\n>>> TARGET %d ACCOMPLISHED! <<<\n", current_waypoint_index + 1);
                
                current_waypoint_index++;
                inside_target_zone = false; 
                
                for(int i = 0; i < 4; i++) wheel_integrals[i] = 0.0;

                if (current_waypoint_index >= TOTAL_WAYPOINTS) {
                    path_completed = true;
                    digitalWrite(trig, HIGH);
                }
            }
        } else {
            inside_target_zone = false;
        }

        // 1. ROTATIONAL LOOP DRIVER
        if (abs(error_theta) < STOP_TOLERANCE_ANGLE) { 
            target_robot_omega = 0.0;
        } else {
            target_robot_omega = error_theta * KP_ANGLE;
        }

        // 2. THE DYNAMIC GOVERNOR 
        double translation_scale = 1.0 - (abs(error_theta) / (PI / 2.0));
        translation_scale = constrain(translation_scale, 0.1, 1.0); 

        // 3. LINEAR LOOP DRIVER
        if (linear_error_distance < STOP_TOLERANCE_DIST) { 
            target_robot_vx = 0.0;
            target_robot_vy = 0.0;

        } else {
            target_robot_vx = local_error_x * KP_DIST * translation_scale;
            target_robot_vy = local_error_y * KP_DIST * translation_scale;
        }

        // ─────────────────────────────────────────────────────────────────────
        // UNIVERSAL ANTI-STICTION FILTER BLOCK
        // ─────────────────────────────────────────────────────────────────────
        // If the robot is outside the positional target zone, enforce a minimum floor 
        // to prevent motor stalling due to chassis stiction.
        if (linear_error_distance > STOP_TOLERANCE_DIST) {
            if (abs(target_robot_vx) > 0.001 && abs(target_robot_vx) < MIN_VX_FLOOR) {
                target_robot_vx = (target_robot_vx > 0) ? MIN_VX_FLOOR : -MIN_VX_FLOOR;
            }
            if (abs(target_robot_vy) > 0.001 && abs(target_robot_vy) < MIN_VY_FLOOR) {
                target_robot_vy = (target_robot_vy > 0) ? MIN_VY_FLOOR : -MIN_VY_FLOOR;
            }
        }

        // Enforce rotational floor if pointing outside the angular tolerance window
        if (abs(error_theta) > STOP_TOLERANCE_ANGLE) {
            if (abs(target_robot_omega) > 0.001 && abs(target_robot_omega) < MIN_OMEGA_FLOOR) {
                target_robot_omega = (target_robot_omega > 0) ? MIN_OMEGA_FLOOR : -MIN_OMEGA_FLOOR;
            }
        }

        // Limit maximum velocities
        target_robot_vx = constrain(target_robot_vx, -MAX_ALLOWED_SPEED_MS, MAX_ALLOWED_SPEED_MS);
        target_robot_vy = constrain(target_robot_vy, -MAX_ALLOWED_SPEED_MS, MAX_ALLOWED_SPEED_MS);
        target_robot_omega = constrain(target_robot_omega, -MAX_ALLOWED_OMEGA_RAD, MAX_ALLOWED_OMEGA_RAD);

        mecanumInverseKinematics(target_robot_vx, target_robot_vy, target_robot_omega, target_wheel_speeds);

        for (int i = 0; i < 4; i++) {
            double error = target_wheel_speeds[i] - actual_wheel_speeds[i];
            wheel_integrals[i] = constrain(wheel_integrals[i] + error * dt, -10.0, 10.0);

            double ff_term = target_wheel_speeds[i] * (118.5 / 1.5);
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

        double current_angle_deg = imu_yaw_rad * (180.0 / PI);
        Serial.printf("WP:%d/%d | X:%.2f->%.2f | Y:%.2f->%.2f | Ang:%.1f°->%.1f° | DistErr:%.3fm | Cmd_Vx:%.3f | Cmd_Vy:%.3f\n", 
                      current_waypoint_index + 1, TOTAL_WAYPOINTS,
                      robot_pose_x, TARGET_DISTANCE_X, 
                      robot_pose_y, TARGET_DISTANCE_Y, 
                      current_angle_deg, TARGET_ANGLE_DEG,
                      linear_error_distance, target_robot_vx, target_robot_vy);
    } 
}




