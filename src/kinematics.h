/**
 * @file kinematics.h
 * @brief Forward and Inverse Kinematics Engine for 3-DOF Robotic Arm
 * @author K Azaan Ahamed (23BEC0362)
 *
 * Link parameters:
 *  - Base joint (theta1): Yaw rotation (azimuth), 0 to 180 degrees
 *  - Shoulder joint (theta2): Pitch rotation, 15 to 165 degrees
 *  - Elbow joint (theta3): Pitch rotation, 15 to 165 degrees
 *  - Gripper: End-effector servo, 0 (closed) to 90 (open) degrees
 */

#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Arduino.h>

struct CartesianCoord {
    float x; // mm relative to base origin
    float y; // mm relative to base origin
    float z; // mm height above shoulder pivot
};

struct JointAngles {
    float theta1; // Base yaw (degrees: 0 - 180)
    float theta2; // Shoulder pitch (degrees: 15 - 165)
    float theta3; // Elbow pitch (degrees: 15 - 165)
    bool reachable; // True if target lies within reachable workspace
};

class ArmKinematics {
public:
    ArmKinematics(float l_base, float l_upper, float l_forearm);

    // Compute end-effector Cartesian position from joint angles
    CartesianCoord forwardKinematics(float theta1_deg, float theta2_deg, float theta3_deg);

    // Compute joint angles required to reach target Cartesian position
    JointAngles inverseKinematics(float target_x, float target_y, float target_z);

    // Validate if angles obey physical mechanical limits
    bool checkJointLimits(const JointAngles& angles);

    float getLBase() const { return L_BASE; }
    float getLUpper() const { return L_UPPER; }
    float getLForearm() const { return L_FOREARM; }

private:
    float L_BASE;    // Height of shoulder pivot from mounting surface (mm)
    float L_UPPER;   // Length of upper arm / humerus (mm)
    float L_FOREARM; // Length of forearm to gripper tip (mm)
};

#endif // KINEMATICS_H
