
//////////////////////////////////////UDP_____TRACKING/////////////////////////////////////

#include <Arduino.h>
#include <ESP32Encoder.h> 
#include <Adafruit_BNO08x.h>
#include <math.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// ─────────────────────────────────────────────────────────────────────────────
// 0. CONTROL MODE, STATE MACHINE & TRACKING SYSTEM
// ─────────────────────────────────────────────────────────────────────────────
enum DriveMode { 
    WAYPOINT_FOLLOWING, // Drives to an absolute coordinate location (meters)
    UDP_TRACKING        // Dynamically steers using incoming UDP (X, Y) packets
};

DriveMode current_mode = WAYPOINT_FOLLOWING; 

// State Machine Steps:
// Step 0: Move to absolute coordinates (WAYPOINT_QUEUE)
// Step 1: Move with constant forward velocity, steering via UDP until error (320 - X) < 5
// Step 2: Sequence Termination & Active Electronic Braking
int sequence_step = 0;          

struct Waypoint {
    double x;
    double y;
    double angle_deg;
};

// Define your target coordinate here (Where the robot goes BEFORE looking for the UDP target)
const Waypoint WAYPOINT_QUEUE[] = {
    {0.60, 0.90, 90.0}
};

const int TOTAL_WAYPOINTS = sizeof(WAYPOINT_QUEUE) / sizeof(Waypoint);
int current_waypoint_index = 0;
bool path_completed = false;

// Robot Velocity States
double actual_robot_vx = 0.0, actual_robot_vy = 0.0, actual_robot_omega = 0.0;
double target_robot_vx = 0.0, target_robot_vy = 0.0, target_robot_omega = 0.0;

// BASE SPEEDS FOR UDP PHASE (Step 1)
double intended_vx = 0.00;       // Constant forward speed during visual tracking phase
double intended_vy = -0.20;
double intended_omega = 0.0;

// Active targets updated dynamically from the coordinate queue
double TARGET_DISTANCE_X = 0.0;
double TARGET_DISTANCE_Y = 0.0;
double TARGET_ANGLE_DEG  = 0.0;

// Settle Timer Configurations for Arrival Verification
unsigned long zone_entry_time = 0;
bool inside_target_zone = false;
const unsigned long SETTLE_DURATION_MS = 1000; 

// ─────────────────────────────────────────────────────────────────────────────
// UDP & NETWORKING CONFIGURATIONS
// ─────────────────────────────────────────────────────────────────────────────
const char* ssid = "Realme 8";
const char* password = "123456780";

WiFiUDP udp;
const int UDP_PORT = 1234;
char packetBuffer[255];

// Dynamic Tracking Variables updated by incoming UDP packets
int udp_x = 453; // Initialized to 320 (Zero Error default)
int udp_y = 0;
bool new_packet_received = false;

// Steering Controller Gains for UDP Tracking
double KP_UDP_STRAFE = 0.0015; // Maps pixel error to strafe velocity (m/s)
double UDP_ERROR_TOLERANCE = 5.0; 

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
const double MAX_ALLOWED_OMEGA_RAD = 2.5;  

// Real-Time Odometry States
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

    if (!bno08x.begin_I2C(0x4B)) {
        while (1) { delay(10); }
    }
    setReports();
    delay(100);

    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi Connected!");
    Serial.print("ESP IP Address: "); Serial.println(WiFi.localIP());

    udp.begin(UDP_PORT);
    Serial.printf("Listening on UDP port %d\n", UDP_PORT);
    
    Serial.println("--- DRIVER ONLINE & NETWORK ARMED ---");
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

    if (now - last_inner_time >= INNER_LOOP_MS) {
        double dt = (now - last_inner_time) / 1000.0;
        last_inner_time = now;

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

        // Background Odometry Calculations
        robot_pose_x += delta_x * cos(imu_yaw_rad) - delta_y * sin(imu_yaw_rad);
        robot_pose_y += delta_x * sin(imu_yaw_rad) + delta_y * cos(imu_yaw_rad);

        // Sync target coordinate map parameters
        if (current_mode == WAYPOINT_FOLLOWING && !path_completed) {
            TARGET_DISTANCE_X = WAYPOINT_QUEUE[current_waypoint_index].x;
            TARGET_DISTANCE_Y = WAYPOINT_QUEUE[current_waypoint_index].y;
            TARGET_ANGLE_DEG  = WAYPOINT_QUEUE[current_waypoint_index].angle_deg;
        }

        double global_error_x = TARGET_DISTANCE_X - robot_pose_x;
        double global_error_y = TARGET_DISTANCE_Y - robot_pose_y;
        
        double target_angle_rad = TARGET_ANGLE_DEG * (PI / 180.0);
        double error_theta = target_angle_rad - imu_yaw_rad; 

        while (error_theta > PI)  error_theta -= 2.0 * PI;
        while (error_theta < -PI) error_theta += 2.0 * PI;

        double local_error_x = global_error_x * cos(imu_yaw_rad) + global_error_y * sin(imu_yaw_rad);
        double local_error_y = -global_error_x * sin(imu_yaw_rad) + global_error_y * cos(imu_yaw_rad);

        double linear_error_distance = sqrt(global_error_x * global_error_x + global_error_y * global_error_y);

        // Calculate visual tracker calculation variables
        double udp_error_x = 455.0 - (double)udp_x;

        // ─────────────────────────────────────────────────────────────────────
        // AUTOMATED SEQUENTIAL STATE MACHINE SWITCH
        // ─────────────────────────────────────────────────────────────────────
        
        // --- STEP 0: DRIVE TO PHYSICAL COORDINATE TARGET ---
        if (sequence_step == 0) {
            current_mode = WAYPOINT_FOLLOWING;

            // Coordinate threshold criteria check
            if (linear_error_distance < 0.03 && abs(error_theta) < 0.04) {
                if (!inside_target_zone) {
                    inside_target_zone = true;
                    zone_entry_time = now; 
                } else if (now - zone_entry_time >= SETTLE_DURATION_MS) {
                    Serial.println("\n>>> ARRIVED AT COORD! SWITCHING TO UDP VISUAL TRACKING <<<");
                    sequence_step = 1; 
                    inside_target_zone = false;
                    for(int i = 0; i < 4; i++) wheel_integrals[i] = 0.0;
                }
            } else {
                inside_target_zone = false;
            }
        }
        // --- STEP 1: MOVE FORWARD + LIVE UDP VISUAL TRACKING ---
        else if (sequence_step == 1) {
            current_mode = UDP_TRACKING;

            // Transition condition: Check if visual error is inside your tolerance range (<= 5 pixels)
            if (abs(udp_error_x) <= UDP_ERROR_TOLERANCE) {
                if (!inside_target_zone) {
                    inside_target_zone = true;
                    zone_entry_time = now;
                } else if (now - zone_entry_time >= SETTLE_DURATION_MS) {
                    sequence_step = 2;
                    inside_target_zone = false;
                    Serial.println("\n>>> UDP TARGET ALIGNED AND TOLERANCE MET! STOPPING. <<<");
                }
            } else {
                inside_target_zone = false;
            }
        }
        // --- STEP 2: FULL TERMINAL PARKING BRAKE ---
        else if (sequence_step == 2) {
            target_robot_vx = 0.0;
            target_robot_vy = 0.0;
            target_robot_omega = 0.0;
        }

        // ─────────────────────────────────────────────────────────────────────
        // CONTROL STRATEGY ENGINE MATRIX
        // ─────────────────────────────────────────────────────────────────────
        if (sequence_step < 2) { // Skip calculations if parked in Step 2
            if (current_mode == WAYPOINT_FOLLOWING) {
                // 1. Rotational Loop Driver with Stiction Boosting
                if (abs(error_theta) < 0.04) { 
                    target_robot_omega = 0.0;
                } else {
                    double p_omega = error_theta * KP_ANGLE;
                    double min_turn_speed = 0.08; 
                    
                    if (p_omega > 0) {
                        target_robot_omega = max(p_omega, min_turn_speed);
                    } else {
                        target_robot_omega = min(p_omega, -min_turn_speed);
                    }
                }

                // 2. Dynamic Governor Cross-Coupling
                double translation_scale = 1.0 - (abs(error_theta) / (PI / 2.0));
                translation_scale = constrain(translation_scale, 0.1, 1.0); 

                // 3. Proportional Linear Loop Driver with Stiction Boosting
                if (linear_error_distance < 0.03) {
                    target_robot_vx = 0.0;
                    target_robot_vy = 0.0;
                } else {
                    double p_vx = local_error_x * KP_DIST * translation_scale;
                    double p_vy = local_error_y * KP_DIST * translation_scale;
                    
                    double calculated_speed = sqrt(p_vx * p_vx + p_vy * p_vy);
                    double min_linear_speed = 0.05; 

                    if (calculated_speed < min_linear_speed && calculated_speed > 0.001) {
                        double boost_factor = min_linear_speed / calculated_speed;
                        target_robot_vx = p_vx * boost_factor;
                        target_robot_vy = p_vy * boost_factor;
                    } else {
                        target_robot_vx = p_vx;
                        target_robot_vy = p_vy;
                    }
                }
            } 
            else if (current_mode == UDP_TRACKING) {
                // Keep the yaw orientation locked onto whatever angle the robot was at when it finished step 0
                static double track_locked_heading = 999.0;
                if (track_locked_heading > 500.0) {
                    track_locked_heading = imu_yaw_rad;
                }

                // Move forward at your designated constant baseline pace
                target_robot_vx = intended_vx; 

                // Adjust left/right strafe actively with UDP Minimum Velocity Boosting
                if (abs(udp_error_x) <= UDP_ERROR_TOLERANCE) {
                    target_robot_vy = 0.0; // Target centered inside final tolerance bounds
                } else {
                    double p_vy = udp_error_x * KP_UDP_STRAFE;
                    double min_strafe_speed = 0.02; // Minimum velocity required to physically slide sideways
                    
                    if (p_vy > 0) {
                        target_robot_vy = max(p_vy, min_strafe_speed);
                    } else {
                        target_robot_vy = min(p_vy, -min_strafe_speed);
                    }
                }

                // Rotational holding profile calculation
                double heading_error = track_locked_heading - imu_yaw_rad;
                while (heading_error > PI)  heading_error -= 2.0 * PI;
                while (heading_error < -PI) heading_error += 2.0 * PI;
                target_robot_omega = heading_error * KP_ANGLE;
            }
            
            // Limit maximum velocities bounds
            target_robot_vx = constrain(target_robot_vx, -MAX_ALLOWED_SPEED_MS, MAX_ALLOWED_SPEED_MS);
            target_robot_vy = constrain(target_robot_vy, -MAX_ALLOWED_SPEED_MS, MAX_ALLOWED_SPEED_MS);
            target_robot_omega = constrain(target_robot_omega, -MAX_ALLOWED_OMEGA_RAD, MAX_ALLOWED_OMEGA_RAD);
        }

        // ─────────────────────────────────────────────────────────────────────
        // INNER INVERSE KINEMATICS & MOTOR DRIVE EXECUTION
        // ─────────────────────────────────────────────────────────────────────
        mecanumInverseKinematics(target_robot_vx, target_robot_vy, target_robot_omega, target_wheel_speeds);

        for (int i = 0; i < 4; i++) {
            double error = target_wheel_speeds[i] - actual_wheel_speeds[i];
            wheel_integrals[i] = constrain(wheel_integrals[i] + error * dt, -10.0, 10.0);

            double ff_term = target_wheel_speeds[i] * (116.5 / 1.5);
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

        // Diagnostic Reporting Logs
        if (sequence_step == 0) {
            Serial.printf("STATE:0_WAYPOINT | X:%.2f->%.2f | Y:%.2f->%.2f | DistErr:%.3fm\n", 
                          robot_pose_x, TARGET_DISTANCE_X, robot_pose_y, TARGET_DISTANCE_Y, linear_error_distance);
        } else if (sequence_step == 1) {
            Serial.printf("STATE:1_UDP_TRACK | TargetX:%d | PixErrX:%.1f | Cmd_Vy:%.2fm/s\n", 
                          udp_x, udp_error_x, target_robot_vy);
        } else {
            Serial.printf("STATE:2_LOCKED_BRAKE | PoseX:%.2fm | PoseY:%.2fm\n", robot_pose_x, robot_pose_y);
        }
    } 
}


