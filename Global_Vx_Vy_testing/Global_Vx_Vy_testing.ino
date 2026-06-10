///////////////////////////////////////////Global_vx and Global_vy/////////////////////////////////////////////////////////////////////



// =============================================================================
// ROBOCON TARGET LOCKING FIRMWARE: FIELD-CENTRIC GLOBAL VELOCITY VECTORING
// =============================================================================
#include <Arduino.h>
#include <ESP32Encoder.h> 
#include <Adafruit_BNO08x.h>
#include <math.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// ─────────────────────────────────────────────────────────────────────────────
// 1. STATE MACHINE & GLOBAL TRACKING CONFIGURATIONS
// ─────────────────────────────────────────────────────────────────────────────
enum ControlState { APPROACH, VISUAL_ALIGN, FORWARD_DOCK, COMPLETED };
ControlState current_state = APPROACH;

bool path_completed = false;

// Dynamic tracking targets (Defined in absolute global coordinates)
double target_x = 0.70;
double target_y = 0.80;
double target_yaw_deg = 90.0;

// Kinematic Outputs
double target_robot_vx = 0.0, target_robot_vy = 0.0, target_robot_omega = 0.0;
double actual_robot_vx = 0.0, actual_robot_vy = 0.0, actual_robot_omega = 0.0;

// Momentum Bridge Filters
static double filtered_vx = 0.0;
static double filtered_vy = 0.0;
static double filtered_omega = 0.0;

// ─────────────────────────────────────────────────────────────────────────────
// 2. NETWORK CONFIGURATION (NON-BLOCKING FALLBACK)
// ─────────────────────────────────────────────────────────────────────────────
const char* ssid = "Realme 8";
const char* password = "123456780";

WiFiUDP udp;
const int UDP_PORT = 1234;
char packetBuffer[255];

int udp_x = 457; 
unsigned long last_packet_time = 0;
const unsigned long UDP_TIMEOUT_MS = 400; 

double KP_UDP_STRAFE = 0.0025;     
double UDP_ERROR_TOLERANCE = 8.0;  

// ─────────────────────────────────────────────────────────────────────────────
// 3. HARDWARE CONSTANTS & ANTI-STICTION FLOORS
// ─────────────────────────────────────────────────────────────────────────────
const double MIN_VX_FLOOR    = 0.06; 
const double MIN_VY_FLOOR    = 0.07; 
const double MIN_OMEGA_FLOOR = 0.12; 

const double STOP_TOLERANCE_DIST  = 0.02; 
const double STOP_TOLERANCE_ANGLE = 0.03; 

#define BNO08X_RESET -1
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
double imu_yaw_rad = 0.0; 

const int PIN_FL_RPWM = 16; const int PIN_FL_LPWM = 17; const int CH_FL_RPWM = 0; const int CH_FL_LPWM = 1;
const int PIN_FR_RPWM = 18; const int PIN_FR_LPWM = 19; const int CH_FR_RPWM = 2; const int CH_FR_LPWM = 3;
const int PIN_RL_RPWM = 23; const int PIN_RL_LPWM = 4;  const int CH_RL_RPWM = 4; const int CH_RL_LPWM = 5;
const int PIN_RR_RPWM = 13; const int PIN_RR_LPWM = 14; const int CH_RR_RPWM = 6; const int CH_RR_LPWM = 7;

const int PIN_FL_ENC_A = 39; const int PIN_FL_ENC_B = 36;
const int PIN_FR_ENC_A = 34; const int PIN_FR_ENC_B = 35;
const int PIN_RL_ENC_A = 26; const int PIN_RL_ENC_B = 25;
const int PIN_RR_ENC_A = 32; const int PIN_RR_ENC_B = 33;

const int PWM_FREQ = 5000; const int PWM_RES = 8;     
const double WHEEL_RADIUS = 0.05; 
const double LX = 0.30; const double LY = 0.30;           
const int TICKS_PER_REV = 1300; 

const unsigned long INNER_LOOP_MS = 10; 
unsigned long last_inner_time = 0;

ESP32Encoder enc_FL, enc_FR, enc_RL, enc_RR;
double smoothed_wheel_speeds[4] = {0.0, 0.0, 0.0, 0.0};
const double ALPHA = 0.3; 
long prev_counts[4] = {0, 0, 0, 0};

double KP_INNER_VEL = 600.0; 
double actual_wheel_speeds[4] = {0.0, 0.0, 0.0, 0.0};
double target_wheel_speeds[4] = {0.0, 0.0, 0.0, 0.0};

double KP_DIST = 2.5; 

double robot_pose_x = 0.0;
double robot_pose_y = 0.0;

// ─────────────────────────────────────────────────────────────────────────────
// 4. KINEMATICS & MOTOR FUNCTIONS
// ─────────────────────────────────────────────────────────────────────────────
void setMotor1(int pwm) { int p = constrain((int)abs(pwm), 0, 255); if (pwm >= 0) { ledcWrite(CH_FL_RPWM, p); ledcWrite(CH_FL_LPWM, 0); } else { ledcWrite(CH_FL_RPWM, 0); ledcWrite(CH_FL_LPWM, p); } }
void setMotor2(int pwm) { int p = constrain((int)abs(pwm), 0, 255); if (pwm >= 0) { ledcWrite(CH_FR_RPWM, p); ledcWrite(CH_FR_LPWM, 0); } else { ledcWrite(CH_FR_RPWM, 0); ledcWrite(CH_FR_LPWM, p); } }
void setMotor3(int pwm) { int p = constrain((int)abs(pwm), 0, 255); if (pwm >= 0) { ledcWrite(CH_RL_RPWM, p); ledcWrite(CH_RL_LPWM, 0); } else { ledcWrite(CH_RL_RPWM, 0); ledcWrite(CH_RL_LPWM, p); } }
void setMotor4(int pwm) { int p = constrain((int)abs(pwm), 0, 255); if (pwm >= 0) { ledcWrite(CH_RR_RPWM, p); ledcWrite(CH_RR_LPWM, 0); } else { ledcWrite(CH_RR_RPWM, 0); ledcWrite(CH_RR_LPWM, p); } }

void stopAllMotors() { setMotor1(0); setMotor2(0); setMotor3(0); setMotor4(0); }

void mecanumInverseKinematics(double vx, double vy, double omega, double* speeds) {
    speeds[0] = vx - vy - (LX + LY) * omega; 
    speeds[1] = vx + vy + (LX + LY) * omega; 
    speeds[2] = vx + vy - (LX + LY) * omega; 
    speeds[3] = vx - vy + (LX + LY) * omega; 
}

void mecanumForwardKinematics(double* w_speeds, double& vx, double& vy, double& omega) {
    vx = (w_speeds[0] + w_speeds[1] + w_speeds[2] + w_speeds[3]) / 4.0;
    vy = (-w_speeds[0] + w_speeds[1] + w_speeds[2] - w_speeds[3]) / 4.0;
    omega = (-w_speeds[0] + w_speeds[1] - w_speeds[2] + w_speeds[3]) / (4.0 * (LX + LY));
}

// ─────────────────────────────────────────────────────────────────────────────
// 5. SETUP
// ─────────────────────────────────────────────────────────────────────────────
void setup_reports() { bno08x.enableReport(SH2_GAME_ROTATION_VECTOR); }

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

    if (!bno08x.begin_I2C(0x4B)) { while (1) { delay(10); } }
    setup_reports();
    delay(100);

    // Non-Blocking Wi-Fi Gateway Initialization (3 second cut-off limit)
    Serial.println("Initialising Wi-Fi Broadcast System...");
    WiFi.begin(ssid, password);
    unsigned long wifi_start_attempt = millis();
    while (WiFi.status() != WL_CONNECTED) {
        delay(100); Serial.print(".");
        if (millis() - wifi_start_attempt >= 3000) {
            Serial.println("\n⚠️ Network Timeout. Running in PURE ODOMETRY mode.");
            break; 
        }
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n✅ Link Active. UDP Socket Mounted.");
        udp.begin(UDP_PORT);
    }
    last_packet_time = millis(); 
}

// ─────────────────────────────────────────────────────────────────────────────
// 6. EXECUTION LOOP
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
    unsigned long now = millis();

    if (bno08x.wasReset()) { setup_reports(); }

    if (bno08x.getSensorEvent(&sensorValue) && sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
        float r = sensorValue.un.gameRotationVector.real; float i = sensorValue.un.gameRotationVector.i;
        float j = sensorValue.un.gameRotationVector.j; float k = sensorValue.un.gameRotationVector.k;
        imu_yaw_rad = atan2(2.0f * (r * k + i * j), 1.0f - 2.0f * (j * j + k * k));
    }

    if (udp.parsePacket()) {
        int len = udp.read(packetBuffer, sizeof(packetBuffer) - 1);
        if (len > 0) {
            packetBuffer[len] = '\0'; int parsed_x, parsed_y;
            if (sscanf(packetBuffer, "%d,%d", &parsed_x, &parsed_y) == 2) { udp_x = parsed_x; last_packet_time = now; }
        }
    }

    if (now - last_inner_time >= INNER_LOOP_MS) {
        double dt = (now - last_inner_time) / 1000.0; last_inner_time = now;

        // Fetch Odometry Speeds
        long current_counts[4] = {enc_FL.getCount(), enc_FR.getCount(), enc_RL.getCount(), enc_RR.getCount()};
        for (int i = 0; i < 4; i++) {
            double raw_speed = (((double)(current_counts[i] - prev_counts[i]) / TICKS_PER_REV) * (2.0 * PI * WHEEL_RADIUS)) / dt;
            prev_counts[i] = current_counts[i];
            smoothed_wheel_speeds[i] = (ALPHA * raw_speed) + ((1.0 - ALPHA) * smoothed_wheel_speeds[i]);
            actual_wheel_speeds[i] = smoothed_wheel_speeds[i];
        }

        mecanumForwardKinematics(actual_wheel_speeds, actual_robot_vx, actual_robot_vy, actual_robot_omega);
        robot_pose_x += (actual_robot_vx * cos(imu_yaw_rad) - actual_robot_vy * sin(imu_yaw_rad)) * dt;
        robot_pose_y += (actual_robot_vx * sin(imu_yaw_rad) + actual_robot_vy * cos(imu_yaw_rad)) * dt;

        // ─────────────────────────────────────────────────────────────────────
        // FIELD-CENTRIC CALCULATIONS (Errors Computed Globally)
        // ─────────────────────────────────────────────────────────────────────
        double global_error_x = target_x - robot_pose_x;
        double global_error_y = target_y - robot_pose_y;
        double linear_error_dist = sqrt(global_error_x * global_error_x + global_error_y * global_error_y);

        double target_angle_rad = target_yaw_deg * (PI / 180.0);
        double error_theta = target_angle_rad - imu_yaw_rad; 
        while (error_theta > PI)  error_theta -= 2.0 * PI;
        while (error_theta < -PI) error_theta += 2.0 * PI;

        // Raw Demand Variables mapped in GLOBAL Space
        double global_vx = 0.0;
        double global_vy = 0.0;
        double raw_omega = error_theta * 1.5; // Reduced from 3.5 to prevent initial power-on slam

        bool udp_stream_active = (now - last_packet_time <= UDP_TIMEOUT_MS);
        double udp_error_x = 160.0 - (double)udp_x; 

        // ─────────────────────────────────────────────────────────────────────
        // SEQUENTIAL STATE CONTROLLER (Operating on Global Error Vectors)
        // ─────────────────────────────────────────────────────────────────────
        switch (current_state) {
            
            case APPROACH:
                // Global velocities look directly at target coordinates regardless of where the nose points
                global_vx = global_error_x * KP_DIST;
                global_vy = global_error_y * KP_DIST;

                if (linear_error_dist < 0.04 && abs(error_theta) < 0.05) { 
                    current_state = VISUAL_ALIGN;
                    Serial.println("🔄 Handover: Transitioning to FIELD-CENTRIC VISUAL_ALIGN...");
                }
                break;

            case VISUAL_ALIGN:
                global_vx = 0.0; 

                if (WiFi.status() == WL_CONNECTED && udp_stream_active && udp_x != 0) {
                    if (abs(udp_error_x) > UDP_ERROR_TOLERANCE) {
                        // Because the robot is rotated to 90 degrees global, the camera line-of-sight
                        // matches the global X axis. Therefore, camera adjustments apply to global_vx!
                        global_vx = udp_error_x * KP_UDP_STRAFE; 
                        global_vy = 0.0;
                    } else {
                        global_vx = 0.0; global_vy = 0.0;
                        
                        // Capture and lock field coordinate
                        target_x = robot_pose_x; 
                        target_y = 0.90; // Final depth
                        current_state = FORWARD_DOCK;
                        Serial.printf("🎯 Lock Acclaimed! Base X frozen at: %.3fm.\n", target_x);
                    }
                } else {
                    // Fallback to absolute global coordinates if camera goes dark
                    global_vx = global_error_x * KP_DIST;
                    global_vy = global_error_y * KP_DIST;
                    if (linear_error_dist < 0.03) {
                        target_x = robot_pose_x; target_y = 0.90; current_state = FORWARD_DOCK;
                    }
                }
                break;

            case FORWARD_DOCK:
                // Move globally straight down the field lines to target depth
                global_vx = global_error_x * KP_DIST;
                global_vy = global_error_y * KP_DIST;

                if (linear_error_dist <= STOP_TOLERANCE_DIST && abs(error_theta) <= STOP_TOLERANCE_ANGLE) {
                    current_state = COMPLETED;
                }
                break;

            case COMPLETED:
                global_vx = 0.0; global_vy = 0.0; raw_omega = 0.0;
                stopAllMotors();
                path_completed = true;
                break;
        }

        // ─────────────────────────────────────────────────────────────────────
        // THE COORDINATE TRANSPORT TRANSFORM (Global -> Local Localisation Space)
        // ─────────────────────────────────────────────────────────────────────
        double raw_local_vx = global_vx * cos(imu_yaw_rad) + global_vy * sin(imu_yaw_rad);
        double raw_local_vy = -global_vx * sin(imu_yaw_rad) + global_vy * cos(imu_yaw_rad);

        // ─────────────────────────────────────────────────────────────────────
        // THE MOMENTUM BRIDGE (Slew Filters applied on Local vectors)
        // ─────────────────────────────────────────────────────────────────────
        if (!path_completed) {
            const double MAX_ACCEL = 2.2; 
            double max_step = MAX_ACCEL * dt;

            filtered_vx += constrain(raw_local_vx - filtered_vx, -max_step, max_step);
            filtered_vy += constrain(raw_local_vy - filtered_vy, -max_step, max_step);
            filtered_omega += constrain(raw_omega - filtered_omega, -max_step, max_step);

            if (abs(filtered_vx) > 0.005 && abs(filtered_vx) < MIN_VX_FLOOR) filtered_vx = (filtered_vx > 0) ? MIN_VX_FLOOR : -MIN_VX_FLOOR;
            if (abs(filtered_vy) > 0.005 && abs(filtered_vy) < MIN_VY_FLOOR) filtered_vy = (filtered_vy > 0) ? MIN_VY_FLOOR : -MIN_VY_FLOOR;
            if (abs(filtered_omega) > 0.005 && abs(filtered_omega) < MIN_OMEGA_FLOOR) filtered_omega = (filtered_omega > 0) ? MIN_OMEGA_FLOOR : -MIN_OMEGA_FLOOR;

            target_robot_vx = constrain(filtered_vx, -0.5, 0.5);
            target_robot_vy = constrain(filtered_vy, -0.5, 0.5);
            target_robot_omega = constrain(filtered_omega, -2.0, 2.0);
        }

        // Output configuration matrix mapping to structural hardware pins
        mecanumInverseKinematics(target_robot_vx, target_robot_vy, target_robot_omega, target_wheel_speeds);

        for (int i = 0; i < 4; i++) {
            double final_pwm = (target_wheel_speeds[i] * 77.6) + ((target_wheel_speeds[i] - actual_wheel_speeds[i]) * KP_INNER_VEL);
            int pwm_out = constrain((int)final_pwm, -255, 255);
            switch(i) {
                case 0: setMotor1(pwm_out); break;
                case 1: setMotor2(pwm_out); break;
                case 2: setMotor3(pwm_out); break;
                case 3: setMotor4(pwm_out); break;
            }
        }
    } 
}


















