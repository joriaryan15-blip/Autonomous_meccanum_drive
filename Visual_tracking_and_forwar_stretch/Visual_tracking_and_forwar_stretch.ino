/////////////////////////////////////////////////////////////optimised code with failsafe watchdogs///////////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include <ESP32Encoder.h> 
#include <Adafruit_BNO08x.h>
#include <math.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h> // Explicitly included for advanced I2C configuration

const int trig = 15;

// ─────────────────────────────────────────────────────────────────────────────
// 0. CONTROL MODE, STATE MACHINE & TRACKING SYSTEM
// ─────────────────────────────────────────────────────────────────────────────
enum DriveMode { 
    WAYPOINT_FOLLOWING, // Drives to an absolute coordinate location (meters)
    UDP_TRACKING,       // Dynamically steers using incoming UDP (X, Y) packets
    FORWARD_STRETCH     // Mode: Locks orientation and drives straight ahead to Y target
};

DriveMode current_mode = WAYPOINT_FOLLOWING; 

const int PIX_CENTER = 435;
int sequence_step = 0;          

struct Waypoint {
    double x;
    double y;
    double angle_deg;
};

// Target coordinate before initiating visual tracking
const Waypoint WAYPOINT_QUEUE[] = {
    {0.50, 0.80, 90.0}
};

const int TOTAL_WAYPOINTS = sizeof(WAYPOINT_QUEUE) / sizeof(Waypoint);
int current_waypoint_index = 0;
bool path_completed = false;

// Robot Velocity States
double actual_robot_vx = 0.0, actual_robot_vy = 0.0, actual_robot_omega = 0.0;
double target_robot_vx = 0.0, target_robot_vy = 0.0, target_robot_omega = 0.0;

// BASE SPEEDS FOR UDP PHASE
double intended_vx = 0.00;       
double intended_vy = 0.20;       
double intended_omega = 0.0;

// Step 2 Target parameters
const double FINAL_TARGET_Y = 1.00; // Target absolute Y distance in meters

// Active targets updated dynamically from the coordinate queue
double TARGET_DISTANCE_X = 0.0;
double TARGET_DISTANCE_Y = 0.0;
double TARGET_ANGLE_DEG  = 0.0;

// Persistent Heading Lock Memory across states
double persistent_locked_heading = 999.0;

// Settle Timer Configurations for Arrival Verification
unsigned long zone_entry_time = 0;
bool inside_target_zone = false;
const unsigned long SETTLE_DURATION_MS = 1000; 

// GLOBAL VARIABLES FOR ALIGNMENT AND RUNWAY STATUS (Moved for global scope visibility)
double strafe_lock_pose_x = 0.0;
double error_stretch_y = 0.0;

// ─────────────────────────────────────────────────────────────────────────────
// UDP & NETWORKING CONFIGURATIONS
// ─────────────────────────────────────────────────────────────────────────────
const char* ssid = "Aryan";
const char* password = "12345678";

WiFiUDP udp;
const int UDP_PORT = 1234;
char packetBuffer[64]; // Optimized buffer size for "X,Y" strings

// Dynamic Tracking Variables updated by incoming UDP packets
int udp_x = 320; 
int udp_y = 0;
bool new_packet_received = false;

// Steering Controller Gains for UDP Tracking
double UDP_ERROR_TOLERANCE = 5.0; 
double UDP_SLOWDOWN_THRESHOLD = 40.0; 

// BNO08x Global Setup
#define BNO08X_RESET -1
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
double imu_yaw_rad = 0.0; 

// CRITICAL SAFETY WATCHDOG VARIABLES
unsigned long last_successful_imu_time = 0;
bool imu_is_faulty = false;
const unsigned long IMU_TIMEOUT_THRESHOLD_MS = 200; // Intercept window

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
const double DISTANCE_PER_TICK = (2.0 * PI * WHEEL_RADIUS) / TICKS_PER_REV;

// Timing Loops
const unsigned long INNER_LOOP_MS = 10; 
unsigned long last_inner_time = 0;
unsigned long last_log_time = 0;

// Encoders & Wheel Filtering
ESP32Encoder enc_FL; ESP32Encoder enc_FR; ESP32Encoder enc_RL; ESP32Encoder enc_RR;
double smoothed_wheel_speeds[4] = {0.0, 0.0, 0.0, 0.0};
const double ALPHA = 0.3; 
long prev_counts[4] = {0, 0, 0, 0};

// ─────────────────────────────────────────────────────────────────────────────
// 2. CASCADED PID TUNING VARIABLES
// ─────────────────────────────────────────────────────────────────────────────
double KP_INNER_VEL = 600.0; 
double KI_INNER_VEL = 0.0;  // Active Integral Loop Gain to kill oscillation deadlocks/drift
double actual_wheel_speeds[4] = {0.0, 0.0, 0.0, 0.0};
double target_wheel_speeds[4] = {0.0, 0.0, 0.0, 0.0};
double wheel_integrals[4]     = {0.0, 0.0, 0.0, 0.0};

double KP_DIST = 2.5; 
double KP_ANGLE = 1.0; 

const double MAX_ALLOWED_SPEED_MS = 0.5;   
const double MAX_ALLOWED_OMEGA_RAD = 2.5;  

// Real-Time Odometry States
double robot_pose_x = 0.0;
double robot_pose_y = 0.0;

// ─────────────────────────────────────────────────────────────────────────────
// 3. LOW LEVEL DRIVERS & KINEMATICS MATRIX FUNCTIONS
// ─────────────────────────────────────────────────────────────────────────────
inline void setMotorOutput(int r_ch, int l_ch, int pwm) {
    int p = constrain(abs(pwm), 0, 255);
    if (pwm >= 0) {
        ledcWrite(r_ch, p);
        ledcWrite(l_ch, 0);
    } else {
        ledcWrite(r_ch, 0);
        ledcWrite(l_ch, p);
    }
}

void stopAllMotors() {
    ledcWrite(CH_FL_RPWM, 0); ledcWrite(CH_FL_LPWM, 0);
    ledcWrite(CH_FR_RPWM, 0); ledcWrite(CH_FR_LPWM, 0);
    ledcWrite(CH_RL_RPWM, 0); ledcWrite(CH_RL_LPWM, 0);
    ledcWrite(CH_RR_RPWM, 0); ledcWrite(CH_RR_LPWM, 0);
    for(int i = 0; i < 4; i++) wheel_integrals[i] = 0.0;
}

inline void mecanumInverseKinematics(double vx, double vy, double omega, double* speeds) {
    double arm = (LX + LY) * omega;
    speeds[0] = vx - vy - arm; // FL
    speeds[1] = vx + vy + arm; // FR
    speeds[2] = vx + vy - arm; // RL
    speeds[3] = vx - vy + arm; // RR
}

inline void mecanumForwardKinematics(const double* w_speeds, double& vx, double& vy, double& omega) {
    vx = (w_speeds[0] + w_speeds[1] + w_speeds[2] + w_speeds[3]) * 0.25;
    vy = (-w_speeds[0] + w_speeds[1] + w_speeds[2] - w_speeds[3]) * 0.25;
    omega = (-w_speeds[0] + w_speeds[1] - w_speeds[2] + w_speeds[3]) / (4.0 * (LX + LY));
}

void setReports(void) {
    if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
        Serial.println("Could not enable game vector");
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 4. SETUP & MAIN LOOP
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    pinMode(trig, OUTPUT);
    digitalWrite(trig, LOW);
    
    // HARDWARE SUPPRESSION: Enforce hardware constraints directly onto the I2C peripheral
    Wire.begin(); 
    Wire.setTimeOut(50); // Set bus blocking timeout limit to 50ms to bypass hangs

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

    // Pass specific Wire handler to ensure advanced configuration rules hold true
    if (!bno08x.begin_I2C(0x4B, &Wire)) {
        Serial.println("FATAL: BNO08x IMU failed to initialize on I2C bus.");
        while (1) { delay(10); }
    }
    setReports();
    delay(100);

    // Initialize baseline freshness tracking time
    last_successful_imu_time = millis();

    // NON-BLOCKING WI-FI INITIALIZATION WITH TIMEOUT
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    unsigned long startWifiAttempt = millis();
    
    while (WiFi.status() != WL_CONNECTED && millis() - startWifiAttempt < 5000) {
        delay(250);
        Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi Connected!");
        Serial.print("ESP IP Address: "); Serial.println(WiFi.localIP());
        udp.begin(UDP_PORT);
        Serial.printf("Listening on UDP port %d\n", UDP_PORT);
    } else {
        Serial.println("\n[WARNING]: WiFi initialization timed out. System operating in STANDALONE OFFLINE mode.");
    }
    
    Serial.println("--- DRIVER ONLINE & NETWORK ARMED ---");
}

void loop() {
    unsigned long now = millis();

    if (bno08x.wasReset()) {
        setReports();
    }

    // Attempt to handle data transactions from IMU sensor
    if (bno08x.getSensorEvent(&sensorValue)) {
        if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
            float r = sensorValue.un.gameRotationVector.real;
            float i = sensorValue.un.gameRotationVector.i;
            float j = sensorValue.un.gameRotationVector.j;
            float k = sensorValue.un.gameRotationVector.k;
            imu_yaw_rad = atan2(2.0f * (r * k + i * j), 1.0f - 2.0f * (j * j + k * k));
            
            // WATCHDOG RESET: Data frame is fresh, sync baseline clock
            last_successful_imu_time = now;
            imu_is_faulty = false;
        }
    }

    // EVALUATE WATCHDOG WINDOW: Intercept stagnant inputs
    if (now - last_successful_imu_time > IMU_TIMEOUT_THRESHOLD_MS) {
        if (!imu_is_faulty) {
            Serial.println("[CRITICAL ERROR]: IMU data stream frozen! Safetying chassis to prevent rotation.");
            imu_is_faulty = true;
        }
    }

    // Run network processes only if Wi-Fi linkage is valid
    if (WiFi.status() == WL_CONNECTED) {
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
                }
            }
        }
    } else if (sequence_step == 1) {
        Serial.println("[CRITICAL]: WiFi connection severed during tracking! Bypassing to step 2 stretch.");
        sequence_step = 2; 
        persistent_locked_heading = imu_yaw_rad;
    }

    // Fixed-interval 10ms execution block
    if (now - last_inner_time >= INNER_LOOP_MS) {
        double dt = (now - last_inner_time) / 1000.0;
        last_inner_time = now;

        long current_counts[4] = {enc_FL.getCount(), enc_FR.getCount(), enc_RL.getCount(), enc_RR.getCount()};

        for (int i = 0; i < 4; i++) {
            long delta_ticks = current_counts[i] - prev_counts[i]; 
            prev_counts[i] = current_counts[i];

            double raw_speed = (delta_ticks * DISTANCE_PER_TICK) / dt; 
            smoothed_wheel_speeds[i] = (ALPHA * raw_speed) + ((1.0 - ALPHA) * smoothed_wheel_speeds[i]);
            actual_wheel_speeds[i] = smoothed_wheel_speeds[i];
        }

        mecanumForwardKinematics(actual_wheel_speeds, actual_robot_vx, actual_robot_vy, actual_robot_omega);
        
        double delta_x = actual_robot_vx * dt;
        double delta_y = actual_robot_vy * dt;
        double sin_yaw = sin(imu_yaw_rad);
        double cos_yaw = cos(imu_yaw_rad);

        // Core tracking odometry updates
        robot_pose_x += delta_x * cos_yaw - delta_y * sin_yaw;
        robot_pose_y += delta_x * sin_yaw + delta_y * cos_yaw;

        // ─────────────────────────────────────────────────────────────────────
        // SAFETY ROUTINE: IF INTERFERENCE CRASHED SENSOR, BYPASS DRIVE MATRIX
        // ─────────────────────────────────────────────────────────────────────
        if (imu_is_faulty) {
            target_robot_vx = 0.0; 
            target_robot_vy = 0.0; 
            target_robot_omega = 0.0;
            stopAllMotors();
        } 
        else {
            // EXECUTE MAIN ROBOT KINEMATICS CONTROL MATRIX PROFILE
            if (current_mode == WAYPOINT_FOLLOWING && !path_completed) {
                TARGET_DISTANCE_X = WAYPOINT_QUEUE[current_waypoint_index].x;
                TARGET_DISTANCE_Y = WAYPOINT_QUEUE[current_waypoint_index].y;
                TARGET_ANGLE_DEG  = WAYPOINT_QUEUE[current_waypoint_index].angle_deg;
            }

            double global_error_x = TARGET_DISTANCE_X - robot_pose_x;
            double global_error_y = TARGET_DISTANCE_Y - robot_pose_y;
            
            double error_theta = (TARGET_ANGLE_DEG * (PI / 180.0)) - imu_yaw_rad; 
            while (error_theta > PI)  error_theta -= 2.0 * PI;
            while (error_theta < -PI) error_theta += 2.0 * PI;

            double local_error_x = global_error_x * cos_yaw + global_error_y * sin_yaw;
            double local_error_y = -global_error_x * sin_yaw + global_error_y * cos_yaw;

            double linear_error_distance = sqrt(global_error_x * global_error_x + global_error_y * global_error_y);
            double udp_error_x = PIX_CENTER - (double)udp_x;
            error_stretch_y = FINAL_TARGET_Y - robot_pose_y;

            // Automated State Machine Switch Rules
            if (sequence_step == 0) {
                current_mode = WAYPOINT_FOLLOWING;
                if (linear_error_distance < 0.03 && abs(error_theta) < 0.04) {
                    if (!inside_target_zone) {
                        inside_target_zone = true;
                        zone_entry_time = now; 
                    } else if (now - zone_entry_time >= SETTLE_DURATION_MS) {
                        Serial.println("\n>>> ARRIVED AT COORD TARGET <<<");
                        strafe_lock_pose_x = robot_pose_x; 
                        
                        if (WiFi.status() == WL_CONNECTED) {
                            sequence_step = 1;
                        } else {
                            Serial.println("[OFFLINE]: Bypassing UDP camera alignment, jumping to stretch.");
                            sequence_step = 2;
                        }
                        inside_target_zone = false;
                        persistent_locked_heading = imu_yaw_rad; 
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
                        zone_entry_time = now;
                    } else if (now - zone_entry_time >= SETTLE_DURATION_MS) {
                        sequence_step = 2; 
                        inside_target_zone = false;
                        robot_pose_x = strafe_lock_pose_x; 
                        Serial.println("\n>>> ALIGNED! CHARGING STRAIGHT TO FINAL Y GOAL <<<");
                    }
                } else {
                    inside_target_zone = false;
                }
            }
            else if (sequence_step == 2) {
                current_mode = FORWARD_STRETCH;
                if (abs(error_stretch_y) < 0.03) {
                    if (!inside_target_zone) {
                        inside_target_zone = true;
                        zone_entry_time = now;
                    } else if (now - zone_entry_time >= SETTLE_DURATION_MS) {
                        sequence_step = 3; 
                        inside_target_zone = false;
                        Serial.println("\n>>> TERMINAL GOAL HIT SUCCESSFULLY! BRAKES ENGAGED <<<");
                    }
                } else {
                    inside_target_zone = false;
                }
            }
            else if (sequence_step == 3) {
                target_robot_vx = 0.0; target_robot_vy = 0.0; target_robot_omega = 0.0;
                digitalWrite(trig, HIGH);
            }

            // Trajectory Processing Velocity Outputs
            if (sequence_step < 3) { 
                if (current_mode == WAYPOINT_FOLLOWING) {
                    if (abs(error_theta) < 0.04) { 
                        target_robot_omega = 0.0;
                    } else {
                        double p_omega = error_theta * KP_ANGLE;
                        target_robot_omega = (p_omega > 0) ? max(p_omega, 0.08) : min(p_omega, -0.08);
                    }

                    double translation_scale = constrain(1.0 - (abs(error_theta) / (PI / 2.0)), 0.1, 1.0); 

                    if (linear_error_distance < 0.03) {
                        target_robot_vx = 0.0; target_robot_vy = 0.0;
                    } else {
                        double p_vx = local_error_x * KP_DIST * translation_scale;
                        double p_vy = local_error_y * KP_DIST * translation_scale;
                        double calculated_speed = sqrt(p_vx * p_vx + p_vy * p_vy);

                        if (calculated_speed < 0.05 && calculated_speed > 0.001) {
                            double boost_factor = 0.05 / calculated_speed;
                            target_robot_vx = p_vx * boost_factor;
                            target_robot_vy = p_vy * boost_factor;
                        } else {
                            target_robot_vx = p_vx; target_robot_vy = p_vy;
                        }
                    }
                } 
                else if (current_mode == UDP_TRACKING) {
                    double drift_error_x = strafe_lock_pose_x - robot_pose_x;
                    double local_drift_x = drift_error_x * cos_yaw; 
                    target_robot_vx = local_drift_x * KP_DIST; 

                    if (abs(udp_error_x) <= UDP_ERROR_TOLERANCE) {
                        target_robot_vy = 0.0; 
                    } else {
                        double target_speed = (abs(udp_error_x) < UDP_SLOWDOWN_THRESHOLD) ? (intended_vy * 0.40) : intended_vy;
                        target_robot_vy = (udp_error_x > 0) ? target_speed : -target_speed;
                    }

                    double heading_error = persistent_locked_heading - imu_yaw_rad;
                    while (heading_error > PI)  heading_error -= 2.0 * PI;
                    while (heading_error < -PI) heading_error += 2.0 * PI;
                    target_robot_omega = heading_error * KP_ANGLE;
                }
                else if (current_mode == FORWARD_STRETCH) {
                    double global_stretch_err_x = strafe_lock_pose_x - robot_pose_x;
                    
                    double local_stretch_x = global_stretch_err_x * cos_yaw + error_stretch_y * sin_yaw;
                    double local_stretch_y = -global_stretch_err_x * sin_yaw + error_stretch_y * cos_yaw;

                    if (abs(error_stretch_y) < 0.03) {
                        target_robot_vx = 0.0; target_robot_vy = 0.0;
                    } else {
                        double p_vx = local_stretch_x * KP_DIST;
                        double p_vy = local_stretch_y * KP_DIST;
                        double calculated_speed = sqrt(p_vx * p_vx + p_vy * p_vy);

                        if (calculated_speed < 0.05 && calculated_speed > 0.001) {
                            double boost_factor = 0.05 / calculated_speed;
                            target_robot_vx = p_vx * boost_factor;
                            target_robot_vy = p_vy * boost_factor;
                        } else {
                            target_robot_vx = p_vx; target_robot_vy = p_vy;
                        }
                    }

                    double stretch_heading_error = persistent_locked_heading - imu_yaw_rad;
                    while (stretch_heading_error > PI)  stretch_heading_error -= 2.0 * PI;
                    while (stretch_heading_error < -PI) stretch_heading_error += 2.0 * PI;
                    target_robot_omega = stretch_heading_error * KP_ANGLE;
                }
                
                target_robot_vx = constrain(target_robot_vx, -MAX_ALLOWED_SPEED_MS, MAX_ALLOWED_SPEED_MS);
                target_robot_vy = constrain(target_robot_vy, -MAX_ALLOWED_SPEED_MS, MAX_ALLOWED_SPEED_MS);
                target_robot_omega = constrain(target_robot_omega, -MAX_ALLOWED_OMEGA_RAD, MAX_ALLOWED_OMEGA_RAD);
            }
        } // End of Active Trajectory Logic Block

        // ─────────────────────────────────────────────────────────────────────
        // 5. INNER INVERSE KINEMATICS & MOTOR DRIVE EXECUTION
        // ─────────────────────────────────────────────────────────────────────
        mecanumInverseKinematics(target_robot_vx, target_robot_vy, target_robot_omega, target_wheel_speeds);

        for (int i = 0; i < 4; i++) {
            // Windup prevention during failure states
            if (imu_is_faulty) wheel_integrals[i] = 0.0;

            double error = target_wheel_speeds[i] - actual_wheel_speeds[i];
            wheel_integrals[i] = constrain(wheel_integrals[i] + error * dt, -10.0, 10.0);

            double ff_term = target_wheel_speeds[i] * 79.0187; 
            double p_term  = error * KP_INNER_VEL;
            double i_term  = wheel_integrals[i] * KI_INNER_VEL;

            setMotorOutput(i == 0 ? CH_FL_RPWM : (i == 1 ? CH_FR_RPWM : (i == 2 ? CH_RL_RPWM : CH_RR_RPWM)),
                           i == 0 ? CH_FL_LPWM : (i == 1 ? CH_FR_LPWM : (i == 2 ? CH_RL_LPWM : CH_RR_LPWM)),
                           constrain((int)(ff_term + p_term + i_term), -255, 255));
        }

        // Clean Serial diagnostics throttled to 100ms
        if (now - last_log_time >= 100) {
            last_log_time = now;
            if (imu_is_faulty) {
                Serial.printf("[FAULT ACTIVE] EMERGENCY HARD HALT | Watchdog Delta: %lu ms\n", (now - last_successful_imu_time));
            } else if (sequence_step == 0) {
                Serial.printf("STATE:0_WAYPOINT | X:%.2f->%.2f | Y:%.2f->%.2f\n", robot_pose_x, TARGET_DISTANCE_X, robot_pose_y, TARGET_DISTANCE_Y);
            } else if (sequence_step == 1) {
                Serial.printf("STATE:1_UDP_TRACK | PixErrX:%.1f | ActiveCorrectionVx:%.3f | Cmd_Vy:%.2f\n", udp_x, target_robot_vx, target_robot_vy);
            } else if (sequence_step == 2) {
                Serial.printf("STATE:2_FORWARD_STRETCH | CurrentY:%.2fm | LockedX_Err:%.3fm | ErrY:%.3fm\n", robot_pose_y, (strafe_lock_pose_x - robot_pose_x), error_stretch_y);
            } else {
                Serial.printf("STATE:3_LOCKED_BRAKE | PoseX:%.2fm | PoseY:%.2fm\n", robot_pose_x, robot_pose_y);
            }
        }
    } 
}