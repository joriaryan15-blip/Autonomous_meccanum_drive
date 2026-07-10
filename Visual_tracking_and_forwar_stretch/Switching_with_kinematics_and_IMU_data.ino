#include <Arduino.h>
#include <ESP32Encoder.h> 
#include <Adafruit_BNO08x.h>
#include <math.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h> 

// ─────────────────────────────────────────────────────────────────────────────
// 0. CONTROL MODE, STATE MACHINE & TRACKING SYSTEM
// ─────────────────────────────────────────────────────────────────────────────
enum DriveMode { 
    WAYPOINT_FOLLOWING, 
    UDP_TRACKING,      
    FORWARD_STRETCH,
    FINISHED_BRAKE
};

DriveMode current_mode = WAYPOINT_FOLLOWING; 
int sequence_step = 0;          

const int PIX_CENTER = 435;
int udp_x = 435; 
int udp_y = 0;
bool new_packet_received = false;

// Target System Parameters
double TARGET_DISTANCE_X = 0.80;  
double TARGET_DISTANCE_Y = -0.70; //only change this
double TARGET_ANGLE_DEG  = 0.0; 

const double FINAL_TARGET_X = 1.05;
double lock_x = 0.0;
double lock_y = -0.70; // only change this

// Trackers for Trajectory Anchoring
double start_pose_x = 0.0;
double start_pose_y = 0.0;
double start_pose_theta = 0.0;
double persistent_locked_heading = 999.0;

unsigned long zone_entry_time = 0;
bool inside_target_zone = false;
const unsigned long SETTLE_DURATION_MS = 500; 

double intended_vy = 0.20;       

// ─────────────────────────────────────────────────────────────────────────────
// UDP & NETWORKING CONFIGURATIONS
// ─────────────────────────────────────────────────────────────────────────────
const char* ssid = "Aryan";
const char* password = "12345678";

WiFiUDP udp;
const int UDP_PORT = 1234;
char packetBuffer[64]; 

double UDP_ERROR_TOLERANCE = 15.0; 
double UDP_SLOWDOWN_THRESHOLD = 25.0; 

unsigned long last_udp_packet_time_ms = 0;
const unsigned long UDP_PACKET_TIMEOUT_MS = 1500; 

#define BNO08X_RESET -1
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
double imu_yaw_rad = 0.0; 
double odom_yaw_rad = 0.0;        // Cumulative heading calculated from forward kinematics
double imu_odom_offset_rad = 0.0; // Tracks the exact variance between absolute IMU and wheel tracking

unsigned long last_successful_imu_time_ms = 0;
bool imu_is_faulty = false;
const unsigned long IMU_TIMEOUT_THRESHOLD_MS = 200; 

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

const int PIN_HANDSHAKE_OUT = 5;

const int PWM_FREQ = 5000;  const int PWM_RES   = 8;     
const double WHEEL_RADIUS = 0.05; 
const double LX = 0.30; const double LY = 0.30;           
const int TICKS_PER_REV = 1300;   

const unsigned long INNER_LOOP_MS = 10; 
unsigned long last_inner_time_micros = 0;
unsigned long last_log_time_ms = 0;

ESP32Encoder enc_FL; ESP32Encoder enc_FR; ESP32Encoder enc_RL; ESP32Encoder enc_RR;
double smoothed_wheel_speeds[4] = {0.0, 0.0, 0.0, 0.0};
const double ALPHA = 0.3; 
long prev_counts[4] = {0, 0, 0, 0};

// ─────────────────────────────────────────────────────────────────────────────
// 2. CASCADED PID TUNING VARIABLES
// ─────────────────────────────────────────────────────────────────────────────
double KP_INNER_VEL = 400.0; 
double KI_INNER_VEL = 0.0;  
double actual_wheel_speeds[4] = {0.0, 0.0, 0.0, 0.0};
double target_wheel_speeds[4] = {0.0, 0.0, 0.0, 0.0};
double wheel_integrals[4]     = {0.0, 0.0, 0.0, 0.0};

double KP_DIST = 2.5; 
double KP_ANGLE = 1.8; 

const double current_battery_voltage = 15.8;  
const double MAX_SAFE_SPEED_MS = 1.57;  
const double MAX_ALLOWED_SPEED_MS = 0.5;  

double robot_pose_x = 0.0;
double robot_pose_y = 0.0;

double actual_robot_vx = 0.0, actual_robot_vy = 0.0, actual_robot_omega = 0.0;
double target_robot_vx = 0.0, target_robot_vy = 0.0, target_robot_omega = 0.0;

// ─────────────────────────────────────────────────────────────────────────────
// 3. LOW LEVEL DRIVERS & KINEMATICS MATRIX FUNCTIONS
// ─────────────────────────────────────────────────────────────────────────────
void setMotor1(int pwm) { int p = constrain(abs(pwm), 0, 255); if (pwm >= 0) { ledcWrite(CH_FL_RPWM, p); ledcWrite(CH_FL_LPWM, 0); } else { ledcWrite(CH_FL_RPWM, 0); ledcWrite(CH_FL_LPWM, p); } }
void setMotor2(int pwm) { int p = constrain(abs(pwm), 0, 255); if (pwm >= 0) { ledcWrite(CH_FR_RPWM, p); ledcWrite(CH_FR_LPWM, 0); } else { ledcWrite(CH_FR_RPWM, 0); ledcWrite(CH_FR_LPWM, p); } }
void setMotor3(int pwm) { int p = constrain(abs(pwm), 0, 255); if (pwm >= 0) { ledcWrite(CH_RL_RPWM, p); ledcWrite(CH_RL_LPWM, 0); } else { ledcWrite(CH_RL_RPWM, 0); ledcWrite(CH_RL_LPWM, p); } }
void setMotor4(int pwm) { int p = constrain(abs(pwm), 0, 255); if (pwm >= 0) { ledcWrite(CH_RR_RPWM, p); ledcWrite(CH_RR_LPWM, 0); } else { ledcWrite(CH_RR_RPWM, 0); ledcWrite(CH_RR_LPWM, p); } }

void stopAllMotors() {
    ledcWrite(CH_FL_RPWM, 0); ledcWrite(CH_FL_LPWM, 0);
    ledcWrite(CH_FR_RPWM, 0); ledcWrite(CH_FR_LPWM, 0);
    ledcWrite(CH_RL_RPWM, 0); ledcWrite(CH_RL_LPWM, 0);
    ledcWrite(CH_RR_RPWM, 0); ledcWrite(CH_RR_LPWM, 0);
    for(int i = 0; i < 4; i++) wheel_integrals[i] = 0.0;
}

inline void mecanumInverseKinematics(double vx, double vy, double omega, double* speeds) {
    speeds[0] = vx - vy - (LX + LY) * omega; 
    speeds[1] = vx + vy + (LX + LY) * omega; 
    speeds[2] = vx + vy - (LX + LY) * omega; 
    speeds[3] = vx - vy + (LX + LY) * omega; 
}

inline void mecanumForwardKinematics(double* w_speeds, double& vx, double& vy, double& omega) {
    vx = (w_speeds[0] + w_speeds[1] + w_speeds[2] + w_speeds[3]) / 4.0;
    vy = (-w_speeds[0] + w_speeds[1] + w_speeds[2] - w_speeds[3]) / 4.0;
    omega = (-w_speeds[0] + w_speeds[1] - w_speeds[2] + w_speeds[3]) / (4.0 * (LX + LY));
}

void setReports(void) {
    if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
        Serial.println("Could not enable game vector");
    }
}

// Helper routine to normalize target angles down to (-PI, PI) boundary
inline double normalizeAngle(double angle) {
    while (angle > PI)  angle -= 2.0 * PI;
    while (angle < -PI) angle += 2.0 * PI;
    return angle;
}

// ─────────────────────────────────────────────────────────────────────────────
// 4. SETUP & MAIN LOOP
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    
    pinMode(PIN_HANDSHAKE_OUT, OUTPUT);
    digitalWrite(PIN_HANDSHAKE_OUT, LOW); 

    pinMode(PIN_FL_RPWM, OUTPUT); digitalWrite(PIN_FL_RPWM, LOW);
    pinMode(PIN_FL_LPWM, OUTPUT); digitalWrite(PIN_FL_LPWM, LOW);
    pinMode(PIN_FR_RPWM, OUTPUT); digitalWrite(PIN_FR_RPWM, LOW);
    pinMode(PIN_FR_LPWM, OUTPUT); digitalWrite(PIN_FR_LPWM, LOW);
    pinMode(PIN_RL_RPWM, OUTPUT); digitalWrite(PIN_RL_RPWM, LOW);
    pinMode(PIN_RL_LPWM, OUTPUT); digitalWrite(PIN_RL_LPWM, LOW);
    pinMode(PIN_RR_RPWM, OUTPUT); digitalWrite(PIN_RR_RPWM, LOW);
    pinMode(PIN_RR_LPWM, OUTPUT); digitalWrite(PIN_RR_LPWM, LOW);

    Wire.begin(); 
    Wire.setTimeOut(50); 

    // FIX 1: Non-blocking boot sequence handling
    if (!bno08x.begin_I2C(0x4A, &Wire)) {
        Serial.println("\n[WARNING]: BNO08x IMU failed to initialize! Forcing ODOMETRY fallback mode.");
        imu_is_faulty = true; 
    } else {
        Serial.println("\n[SUCCESS]: BNO08x IMU initialized successfully.");
        Wire.setClock(400000); 
        setReports();
        imu_is_faulty = false;
    }
    delay(100);

    // Retained legacy LEDC setup configurations
    ledcSetup(CH_FL_RPWM, PWM_FREQ, PWM_RES); ledcSetup(CH_FL_LPWM, PWM_FREQ, PWM_RES);
    ledcSetup(CH_FR_RPWM, PWM_FREQ, PWM_RES); ledcSetup(CH_FR_LPWM, PWM_FREQ, PWM_RES);
    ledcSetup(CH_RL_RPWM, PWM_FREQ, PWM_RES); ledcSetup(CH_RL_LPWM, PWM_FREQ, PWM_RES);
    ledcSetup(CH_RR_RPWM, PWM_FREQ, PWM_RES); ledcSetup(CH_RR_LPWM, PWM_FREQ, PWM_RES);

    ledcAttachPin(PIN_FL_RPWM, CH_FL_RPWM); ledcAttachPin(PIN_FL_LPWM, CH_FL_LPWM);
    ledcAttachPin(PIN_FR_RPWM, CH_FR_RPWM); ledcAttachPin(PIN_FR_LPWM, CH_FR_LPWM);
    ledcAttachPin(PIN_RL_RPWM, CH_RL_RPWM); ledcAttachPin(PIN_RL_LPWM, CH_RL_LPWM);
    ledcAttachPin(PIN_RR_RPWM, CH_RR_RPWM); ledcAttachPin(PIN_RR_LPWM, CH_RR_LPWM);

    stopAllMotors();

    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    enc_FL.attachFullQuad(PIN_FL_ENC_A, PIN_FL_ENC_B); enc_FR.attachFullQuad(PIN_FR_ENC_A, PIN_FR_ENC_B);
    enc_RL.attachFullQuad(PIN_RL_ENC_A, PIN_RL_ENC_B); enc_RR.attachFullQuad(PIN_RR_ENC_A, PIN_RR_ENC_B);

    WiFi.begin(ssid, password);
    unsigned long startWifiAttempt = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startWifiAttempt < 2000) {
        delay(250);
        Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi Infrastructure Linked!");
        udp.begin(UDP_PORT);

        udp.beginPacket("255.255.255.255", UDP_PORT);
        udp.print("ESP_HERE");
        udp.endPacket();
        Serial.println("ESP_HERE sent.");
    } else {
        Serial.println("\n[NOTICE]: WiFi not connected at boot. Will skip vision segment.");
    }

    last_successful_imu_time_ms = millis();
    last_inner_time_micros = micros();

    Serial.println("--- SYSTEM ARMED: WAYPOINT CONTROL SENDER ARMED ---");
}

void loop() {
    unsigned long now_ms = millis();

    if (!imu_is_faulty && bno08x.wasReset()) {
        setReports();
    }

    if (bno08x.getSensorEvent(&sensorValue)) {
        if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
            float r = sensorValue.un.gameRotationVector.real;
            float i = sensorValue.un.gameRotationVector.i;
            float j = sensorValue.un.gameRotationVector.j;
            float k = sensorValue.un.gameRotationVector.k;

            // Grab absolute incoming angle data
            double fresh_imu_yaw = atan2(2.0f * (r * k + i * j), 1.0f - 2.0f * (j * j + k * k));
            
            // FIX 4: Self-healing alignment calculation upon warm recovery sequence
            if (imu_is_faulty) {
                Serial.println("[SYSTEM]: IMU data stream re-established! Recalculating sensor variance offsets...");
                imu_odom_offset_rad = normalizeAngle(fresh_imu_yaw - odom_yaw_rad);
            }

            imu_yaw_rad = fresh_imu_yaw;
            last_successful_imu_time_ms = now_ms;
            imu_is_faulty = false; 

            // Continuous running background variance tracking
            imu_odom_offset_rad = normalizeAngle(imu_yaw_rad - odom_yaw_rad);
        }
    }

    if (now_ms - last_successful_imu_time_ms > IMU_TIMEOUT_THRESHOLD_MS) {
        imu_is_faulty = true;
    }

    // UDP Packet Handler for Stage 1
    if (sequence_step == 1 && WiFi.status() == WL_CONNECTED) {
       int packetSize = udp.parsePacket();

        if (packetSize) {
            int len = udp.read(packetBuffer, sizeof(packetBuffer) - 1);
            if (len > 0) {
                packetBuffer[len] = '\0';
                int parsed_x, parsed_y;

                if (sscanf(packetBuffer, "%d,%d", &parsed_x, &parsed_y) == 2) {
                    udp_x = parsed_x;
                    udp_y = parsed_y;
                    new_packet_received = true;
                    last_udp_packet_time_ms = now_ms;
                }
            }
        }
        
        if (now_ms - last_udp_packet_time_ms > UDP_PACKET_TIMEOUT_MS) {
            stopAllMotors();
            inside_target_zone = false;
            for(int i = 0; i < 4; i++) wheel_integrals[i] = 0.0;
        }
    }

    unsigned long current_micros = micros();

    if (current_micros - last_inner_time_micros >= (INNER_LOOP_MS * 1000)) {
        double dt = (double)(current_micros - last_inner_time_micros) / 1000000.0;
        last_inner_time_micros = current_micros;

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

        // Kinematics updates
        mecanumForwardKinematics(actual_wheel_speeds, actual_robot_vx, actual_robot_vy, actual_robot_omega);
        
        // FIX 2: Continuous background integration of kinematics odometry heading
        odom_yaw_rad += actual_robot_omega * dt;
        odom_yaw_rad = normalizeAngle(odom_yaw_rad);

        // FIX 3: Dynamic State Selection with offset protection to kill coordinate jumps
        double active_yaw;
        if (imu_is_faulty) {
            active_yaw = normalizeAngle(odom_yaw_rad + imu_odom_offset_rad);
        } else {
            active_yaw = imu_yaw_rad;
        }

        double delta_x = actual_robot_vx * dt;
        double delta_y = actual_robot_vy * dt;

        robot_pose_x += delta_x * cos(active_yaw) - delta_y * sin(active_yaw);
        robot_pose_y += delta_x * sin(active_yaw) + delta_y * cos(active_yaw);

        // Initialization anchor checkpoint updates
        if (sequence_step == 0 && start_pose_x == 0.0 && start_pose_y == 0.0) {
            start_pose_x = robot_pose_x;
            start_pose_y = robot_pose_y;
            start_pose_theta = active_yaw;
        }

        // Global Error Vectors evaluated relative to the chosen active state heading
        double global_error_x = TARGET_DISTANCE_X - robot_pose_x;
        double global_error_y = TARGET_DISTANCE_Y - robot_pose_y;
        double target_angle_rad = TARGET_ANGLE_DEG * (PI / 180.0);
        double error_theta = normalizeAngle(target_angle_rad - active_yaw); 

        // Coordinate Transformations
        double local_error_x = global_error_x * cos(active_yaw) + global_error_y * sin(active_yaw);
        double local_error_y = -global_error_x * sin(active_yaw) + global_error_y * cos(active_yaw);
        double linear_error_distance = sqrt(global_error_x * global_error_x + global_error_y * global_error_y);
        double udp_error_x = PIX_CENTER - (double)udp_x;
        double error_stretch_x = FINAL_TARGET_X - robot_pose_x;

        // Global Configuration Floors
        double min_vel = 0.15;
        double omega_floor = 0.15; 

        // ─────────────────────────────────────────────────────────────────
        // STATE MACHINE EXECUTION LOOPS
        // ─────────────────────────────────────────────────────────────────
        if (sequence_step == 0) {
            current_mode = WAYPOINT_FOLLOWING;
            if (linear_error_distance < 0.03 && abs(error_theta) < 0.04) {
                if (!inside_target_zone) {
                    inside_target_zone = true;
                    zone_entry_time = now_ms; 
                } else if (now_ms - zone_entry_time >= SETTLE_DURATION_MS) {
                    if (WiFi.status() == WL_CONNECTED) {
                        sequence_step = 1; 
                        lock_x = robot_pose_x; 
                        lock_y = robot_pose_y;
                        last_udp_packet_time_ms = millis(); 
                    } else {
                        sequence_step = 2; 
                        lock_y = robot_pose_y;
                    }
                    inside_target_zone = false;
                    persistent_locked_heading = active_yaw; 
                    for(int i = 0; i < 4; i++) wheel_integrals[i] = 0.0;
                }
            } else {
                inside_target_zone = false;
            }
        }
        else if (sequence_step == 1) {
            current_mode = UDP_TRACKING;
            if (abs(udp_error_x) <= UDP_ERROR_TOLERANCE) {
                if (!inside_target_zone) {
                    inside_target_zone = true;
                    zone_entry_time = now_ms;
                } else if (now_ms - zone_entry_time >= SETTLE_DURATION_MS) {
                    sequence_step = 2;
                    lock_y = robot_pose_y; 
                    inside_target_zone = false;
                    persistent_locked_heading = active_yaw; // Maintain alignment to active state
                }
            } else {
                inside_target_zone = false;
            }
        }
        else if (sequence_step == 2) {
            current_mode = FORWARD_STRETCH;
            if (abs(error_stretch_x) < 0.03) {
                if (!inside_target_zone) {
                    inside_target_zone = true;
                    zone_entry_time = now_ms;
                } else if (now_ms - zone_entry_time >= SETTLE_DURATION_MS) {
                    sequence_step = 3; 
                    inside_target_zone = false;
                }
            } else {
                inside_target_zone = false;
            }
        }
        else if (sequence_step == 3) {
            current_mode = FINISHED_BRAKE;
            target_robot_vx = 0.0; target_robot_vy = 0.0; target_robot_omega = 0.0;
            stopAllMotors(); 
            digitalWrite(PIN_HANDSHAKE_OUT, HIGH); 
        }

        // ─────────────────────────────────────────────────────────────────
        // MOTION CONTROLLERS FOR ACTIVE STATES
        // ─────────────────────────────────────────────────────────────────
        if (sequence_step < 3) { 
            if (current_mode == WAYPOINT_FOLLOWING) {
                double P = sqrt(pow(TARGET_DISTANCE_X - start_pose_x, 2) + pow(TARGET_DISTANCE_Y - start_pose_y, 2));
                double x = sqrt(pow(robot_pose_x - start_pose_x, 2) + pow(robot_pose_y - start_pose_y, 2));
                
                if (P < 0.001) P = 0.001;
                double progress_ratio = constrain(x / P, 0.0, 1.0);

                double total_delta_theta = normalizeAngle(target_angle_rad - start_pose_theta);
                double target_theta_path = start_pose_theta + (progress_ratio * total_delta_theta);
                double ratio_error_theta = normalizeAngle(target_theta_path - active_yaw);

                if (linear_error_distance < 0.005) {
                    target_robot_vx = 0.0; target_robot_vy = 0.0;
                } else {
                    target_robot_vx = local_error_x * KP_DIST; 
                    target_robot_vy = local_error_y * KP_DIST; 
                }
                target_robot_omega = ratio_error_theta * KP_ANGLE;p

                if (abs(target_robot_vx) > 0.01 && abs(target_robot_vx) < min_vel) {
                    target_robot_vx = (target_robot_vx > 0) ? min_vel : -min_vel;
                }
                if (abs(target_robot_vy) > 0.01 && abs(target_robot_vy) < min_vel) {
                    target_robot_vy = (target_robot_vy > 0) ? min_vel : -min_vel;
                }
            }
            else if (current_mode == UDP_TRACKING) {
                double global_lock_err_x = lock_x - robot_pose_x;
                double global_lock_err_y = lock_y - robot_pose_y; 

                double local_lock_err_x = global_lock_err_x * cos(active_yaw) + global_lock_err_y * sin(active_yaw);
                target_robot_vx = local_lock_err_x * KP_DIST; 

                if (abs(udp_error_x) <= UDP_ERROR_TOLERANCE) {
                    target_robot_vy = 0.0; 
                } else {
                    double target_speed = (abs(udp_error_x) < UDP_SLOWDOWN_THRESHOLD) ? (intended_vy * 0.40) : intended_vy;
                    target_robot_vy = (udp_error_x > 0) ? target_speed : -target_speed;
                }

                double heading_error = normalizeAngle(persistent_locked_heading - active_yaw);
                target_robot_omega = heading_error * KP_ANGLE;
            }
            else if (current_mode == FORWARD_STRETCH) {
                double stretch_err_x = (FINAL_TARGET_X - robot_pose_x) * cos(active_yaw) + (lock_y - robot_pose_y) * sin(active_yaw);
                double stretch_err_y = -(FINAL_TARGET_X - robot_pose_x) * sin(active_yaw) + (lock_y - robot_pose_y) * cos(active_yaw);

                target_robot_vx = stretch_err_x * KP_DIST;
                target_robot_vy = stretch_err_y * KP_DIST; 

                double stretch_heading_error = normalizeAngle(persistent_locked_heading - active_yaw);
                target_robot_omega = stretch_heading_error * KP_ANGLE;
            }
            
            // Unified global omega-floor clamp applied post-calculation for stages 0, 1, and 2
            if (abs(target_robot_omega) > 0.01 && abs(target_robot_omega) < omega_floor) {
                target_robot_omega = (target_robot_omega > 0) ? omega_floor : -omega_floor;
            }

            double linear_contribution = abs(target_robot_vx) + abs(target_robot_vy);
            double dynamic_max_omega = (MAX_SAFE_SPEED_MS - linear_contribution) / (LX + LY);
            dynamic_max_omega = constrain(dynamic_max_omega, 0.18, 1.2);

            target_robot_vx    = constrain(target_robot_vx,    -MAX_ALLOWED_SPEED_MS,  MAX_ALLOWED_SPEED_MS);
            target_robot_vy    = constrain(target_robot_vy,    -MAX_ALLOWED_SPEED_MS,  MAX_ALLOWED_SPEED_MS);
            target_robot_omega = constrain(target_robot_omega, -dynamic_max_omega,     dynamic_max_omega);
        }

        // ─────────────────────────────────────────────────────────────────────
        // MOTOR VELOCITY ENGINE & DRIVE OUTPUTS
        // ─────────────────────────────────────────────────────────────────────
        mecanumInverseKinematics(target_robot_vx, target_robot_vy, target_robot_omega, target_wheel_speeds);

        double max_requested = 0.0;
        for(int i = 0; i < 4; i++) {
            if(abs(target_wheel_speeds[i]) > max_requested) max_requested = abs(target_wheel_speeds[i]);
        }
        if (max_requested > MAX_SAFE_SPEED_MS) {
            double scale = MAX_SAFE_SPEED_MS / max_requested;
            for(int i = 0; i < 4; i++) target_wheel_speeds[i] *= scale;
        }

        int dynamic_pwm_limit = (int)(1836.0 / current_battery_voltage);

        for (int i = 0; i < 4; i++) {
            double error = target_wheel_speeds[i] - actual_wheel_speeds[i];
            wheel_integrals[i] = constrain(wheel_integrals[i] + error * dt, -0.5, 0.5); 

            double ff_term = target_wheel_speeds[i] * ((double)dynamic_pwm_limit / MAX_SAFE_SPEED_MS);
            double p_term  = error * KP_INNER_VEL;
            double i_term  = wheel_integrals[i] * KI_INNER_VEL;

            int final_pwm = constrain((int)(ff_term + p_term + i_term), -dynamic_pwm_limit, dynamic_pwm_limit);

            switch(i) {
                case 0: setMotor1(final_pwm); break;
                case 1: setMotor2(final_pwm); break;
                case 2: setMotor3(final_pwm); break;
                case 3: setMotor4(final_pwm); break;
            }
        }

        // Serial Logging Utility Output
        if (now_ms - last_log_time_ms >= 100) {
            last_log_time_ms = now_ms;
            const char* mode_string = imu_is_faulty ? "[FALLBACK_ODOM]" : "[ACTIVE_IMU]";
            
            if (sequence_step == 0) {
                Serial.printf("%s | ST:0_WAY | X:%.2f->%.2f | Y:%.2f->%.2f | Yaw:%.2f | w:%.2f\n", mode_string, robot_pose_x, TARGET_DISTANCE_X, robot_pose_y, TARGET_DISTANCE_Y, active_yaw, target_robot_omega);
            } else if (sequence_step == 1) {
                Serial.printf("%s | ST:1_CAM | PixErrX:%.1f | X_Locked:%.2f | Yaw:%.2f | w:%.2f\n", mode_string, udp_x - PIX_CENTER, robot_pose_x, active_yaw, target_robot_omega);
            } else if (sequence_step == 2) {
                Serial.printf("%s | ST:2_STRCH | CurrX:%.2fm -> TargetX:%.2fm | Yaw:%.2f | w:%.2f\n", mode_string, robot_pose_x, FINAL_TARGET_X, active_yaw, target_robot_omega);
            } else {
                Serial.printf("ST:3_BRK | Handshake Active (GPIO 5 = HIGH)\n");
            }
        }
    } 
}
