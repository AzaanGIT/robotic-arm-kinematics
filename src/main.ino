/**
 * @file main.ino
 * @brief 3-DOF Robotic Arm Controller with Kinematics, Serial Command & Manual Modes
 * @author K Azaan Ahamed (23BEC0362)
 *
 * Pin Map:
 *   - Servo Base (Yaw)     : Digital Pin 3 (PWM)
 *   - Servo Shoulder       : Digital Pin 5 (PWM)
 *   - Servo Elbow          : Digital Pin 6 (PWM)
 *   - Servo Gripper        : Digital Pin 9 (PWM)
 *   - Button 1 (Step / Mode): Pin 2 (External Interrupt 0 / Pullup)
 *   - Button 2 (Teach/Save): Pin 4 (Pullup)
 *   - Button 3 (Playback)  : Pin 7 (Pullup)
 *   - Status LED           : Pin 13
 */

#include <Arduino.h>
#include "kinematics.h"
#include "servo_driver.h"

// Hardware Geometry (in millimeters)
const float L_BASE    = 65.0f;  // Base pivot to shoulder joint height
const float L_UPPER   = 105.0f; // Shoulder joint to elbow joint length
const float L_FOREARM = 120.0f; // Elbow joint to gripper tip length

// Pin Definitions
const int PIN_SERVO_BASE     = 3;
const int PIN_SERVO_SHOULDER = 5;
const int PIN_SERVO_ELBOW    = 6;
const int PIN_SERVO_GRIPPER  = 9;

const int PIN_BTN_MODE     = 2;
const int PIN_BTN_TEACH    = 4;
const int PIN_BTN_PLAYBACK = 7;
const int PIN_LED_STATUS   = 13;

// Global Subsystems
ArmKinematics kinematics(L_BASE, L_UPPER, L_FOREARM);
SmoothServo servoBase;
SmoothServo servoShoulder;
SmoothServo servoElbow;
SmoothServo servoGripper;

// Control Modes
enum OperatingMode {
    MODE_CALIBRATION,
    MODE_CARTESIAN_IK,
    MODE_MANUAL_BUTTONS,
    MODE_AUTO_PICK_AND_PLACE
};

OperatingMode currentMode = MODE_AUTO_PICK_AND_PLACE;
unsigned long lastUpdateMicros = 0;

// Trajectory Waypoint structure for pick & place sequence
struct Waypoint {
    float x;
    float y;
    float z;
    float grip; // 0 = closed, 90 = open
    int dwellMs;
};

const int NUM_WAYPOINTS = 6;
Waypoint pickAndPlaceRoutine[NUM_WAYPOINTS] = {
    {120.0f,   0.0f, 130.0f, 90.0f,  800}, // Approach above pickup spot (open grip)
    {120.0f,   0.0f,  45.0f, 90.0f,  600}, // Lower to object
    {120.0f,   0.0f,  45.0f, 15.0f, 1000}, // Close gripper firmly
    {120.0f,   0.0f, 140.0f, 15.0f,  800}, // Lift object clear of obstacles
    {  0.0f, 140.0f, 140.0f, 15.0f, 1000}, // Swing to deposit location
    {  0.0f, 140.0f,  50.0f, 90.0f,  800}  // Lower and release object
};

int currentWaypointIdx = 0;
unsigned long waypointHoldStart = 0;
bool isHoldingWaypoint = false;

void moveToCartesian(float x, float y, float z, float grip, float speed = 50.0f) {
    JointAngles ja = kinematics.inverseKinematics(x, y, z);
    if (ja.reachable) {
        servoBase.setTargetAngle(ja.theta1, speed);
        servoShoulder.setTargetAngle(ja.theta2, speed);
        servoElbow.setTargetAngle(ja.theta3, speed);
        servoGripper.setTargetAngle(grip, speed * 1.5f);
        Serial.print(F("ACK MOVE -> Joint Angles (deg): Base="));
        Serial.print(ja.theta1);
        Serial.print(F(" Shoulder="));
        Serial.print(ja.theta2);
        Serial.print(F(" Elbow="));
        Serial.println(ja.theta3);
    } else {
        Serial.println(F("ERR: Target point outside reachable workspace boundary!"));
    }
}

void processSerialCommands() {
    if (!Serial.available()) return;
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.startsWith("G0") || cmd.startsWith("MOVE")) {
        // e.g. "MOVE 120 0 100 45"
        float x = 100, y = 0, z = 100, g = 45;
        int parsed = sscanf(cmd.c_str(), "%*s %f %f %f %f", &x, &y, &z, &g);
        if (parsed >= 3) {
            currentMode = MODE_CARTESIAN_IK;
            moveToCartesian(x, y, z, g);
        } else {
            Serial.println(F("Usage: MOVE <x_mm> <y_mm> <z_mm> [grip_deg]"));
        }
    } else if (cmd.equalsIgnoreCase("HOME")) {
        currentMode = MODE_CARTESIAN_IK;
        servoBase.setTargetAngle(90.0f, 40.0f);
        servoShoulder.setTargetAngle(90.0f, 40.0f);
        servoElbow.setTargetAngle(90.0f, 40.0f);
        servoGripper.setTargetAngle(45.0f, 40.0f);
        Serial.println(F("ACK: Returning to Home Position"));
    } else if (cmd.equalsIgnoreCase("AUTO")) {
        currentMode = MODE_AUTO_PICK_AND_PLACE;
        currentWaypointIdx = 0;
        isHoldingWaypoint = false;
        Serial.println(F("ACK: Switched to Autonomous Pick-and-Place Routine"));
    } else if (cmd.equalsIgnoreCase("STATUS")) {
        CartesianCoord pos = kinematics.forwardKinematics(
            servoBase.getCurrentAngle(),
            servoShoulder.getCurrentAngle(),
            servoElbow.getCurrentAngle()
        );
        Serial.print(F("STATUS: X=")); Serial.print(pos.x);
        Serial.print(F(" Y=")); Serial.print(pos.y);
        Serial.print(F(" Z=")); Serial.print(pos.z);
        Serial.print(F(" Grip=")); Serial.println(servoGripper.getCurrentAngle());
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(PIN_LED_STATUS, OUTPUT);
    pinMode(PIN_BTN_MODE, INPUT_PULLUP);
    pinMode(PIN_BTN_TEACH, INPUT_PULLUP);
    pinMode(PIN_BTN_PLAYBACK, INPUT_PULLUP);

    servoBase.attach(PIN_SERVO_BASE);
    servoShoulder.attach(PIN_SERVO_SHOULDER);
    servoElbow.attach(PIN_SERVO_ELBOW);
    servoGripper.attach(PIN_SERVO_GRIPPER);

    // Initial safe startup position
    servoBase.forceAngle(90.0f);
    servoShoulder.forceAngle(90.0f);
    servoElbow.forceAngle(90.0f);
    servoGripper.forceAngle(45.0f);

    lastUpdateMicros = micros();
    Serial.println(F("=== 3-DOF Robotic Arm Kinematics Controller Online ==="));
    Serial.println(F("Ready. Type AUTO, MOVE <x> <y> <z> <grip>, or HOME"));
}

void loop() {
    unsigned long now = micros();
    float dt = (now - lastUpdateMicros) / 1000000.0f;
    lastUpdateMicros = now;

    // Handle host communications
    processSerialCommands();

    // Autonomous routine sequencer
    if (currentMode == MODE_AUTO_PICK_AND_PLACE) {
        bool inMotion = servoBase.isMoving() || servoShoulder.isMoving() ||
                        servoElbow.isMoving() || servoGripper.isMoving();

        if (!inMotion) {
            if (!isHoldingWaypoint) {
                isHoldingWaypoint = true;
                waypointHoldStart = millis();
            } else if (millis() - waypointHoldStart >= (unsigned long)pickAndPlaceRoutine[currentWaypointIdx].dwellMs) {
                isHoldingWaypoint = false;
                currentWaypointIdx = (currentWaypointIdx + 1) % NUM_WAYPOINTS;
                Waypoint wp = pickAndPlaceRoutine[currentWaypointIdx];
                moveToCartesian(wp.x, wp.y, wp.z, wp.grip, 45.0f);
            }
        }
    }

    // Step motion interpolators
    servoBase.update(dt);
    servoShoulder.update(dt);
    servoElbow.update(dt);
    servoGripper.update(dt);

    digitalWrite(PIN_LED_STATUS, (servoBase.isMoving() || servoShoulder.isMoving()) ? HIGH : LOW);
    delay(5);
}
