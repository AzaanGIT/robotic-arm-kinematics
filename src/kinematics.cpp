#include "kinematics.h"
#include <math.h>

#define DEG_TO_RAD (3.14159265358979323846 / 180.0)
#define RAD_TO_DEG (180.0 / 3.14159265358979323846)

ArmKinematics::ArmKinematics(float l_base, float l_upper, float l_forearm)
    : L_BASE(l_base), L_UPPER(l_upper), L_FOREARM(l_forearm) {}

CartesianCoord ArmKinematics::forwardKinematics(float theta1_deg, float theta2_deg, float theta3_deg) {
    CartesianCoord pos;
    float t1 = theta1_deg * DEG_TO_RAD;
    float t2 = theta2_deg * DEG_TO_RAD;
    float t3 = theta3_deg * DEG_TO_RAD;

    // Planar projection along radial arm
    float r = L_UPPER * cos(t2) + L_FOREARM * cos(t2 + t3);
    pos.x = r * cos(t1);
    pos.y = r * sin(t1);
    pos.z = L_BASE + L_UPPER * sin(t2) + L_FOREARM * sin(t2 + t3);

    return pos;
}

JointAngles ArmKinematics::inverseKinematics(float target_x, float target_y, float target_z) {
    JointAngles ja = {90.0f, 90.0f, 90.0f, false};

    // 1. Base yaw rotation (azimuth angle)
    float theta1 = atan2(target_y, target_x) * RAD_TO_DEG;
    if (theta1 < 0.0f) {
        theta1 += 180.0f; // Constrain to forward semicircle
    }
    ja.theta1 = theta1;

    // 2. Planar projection in vertical plane
    float r = sqrt(target_x * target_x + target_y * target_y);
    float z_rel = target_z - L_BASE;

    // Distance from shoulder joint to end effector
    float d = sqrt(r * r + z_rel * z_rel);

    // Workspace reachability boundary verification
    if (d > (L_UPPER + L_FOREARM) || d < fabs(L_UPPER - L_FOREARM) || d == 0.0f) {
        ja.reachable = false;
        return ja;
    }

    // Law of Cosines for elbow angle (theta3)
    float cos_t3 = (d * d - L_UPPER * L_UPPER - L_FOREARM * L_FOREARM) / (2.0f * L_UPPER * L_FOREARM);
    if (cos_t3 > 1.0f) cos_t3 = 1.0f;
    if (cos_t3 < -1.0f) cos_t3 = -1.0f;

    // Negative sign represents elbow-up configuration
    float t3_rad = acos(cos_t3);
    ja.theta3 = t3_rad * RAD_TO_DEG;

    // Angle of target vector relative to horizontal
    float alpha = atan2(z_rel, r);

    // Angle between target vector and upper arm
    float cos_beta = (L_UPPER * L_UPPER + d * d - L_FOREARM * L_FOREARM) / (2.0f * L_UPPER * d);
    if (cos_beta > 1.0f) cos_beta = 1.0f;
    if (cos_beta < -1.0f) cos_beta = -1.0f;
    float beta = acos(cos_beta);

    // Shoulder angle
    float t2_rad = alpha + beta;
    ja.theta2 = t2_rad * RAD_TO_DEG;

    // Check physical servo angular limits
    ja.reachable = checkJointLimits(ja);
    return ja;
}

bool ArmKinematics::checkJointLimits(const JointAngles& angles) {
    if (angles.theta1 < 0.0f || angles.theta1 > 180.0f) return false;
    if (angles.theta2 < 10.0f || angles.theta2 > 170.0f) return false;
    if (angles.theta3 < 10.0f || angles.theta3 > 170.0f) return false;
    return true;
}
