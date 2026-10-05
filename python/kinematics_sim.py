#!/usr/bin/env python3
"""
Kinematics Simulation & 3D Workspace Analysis for 3-DOF Robotic Arm
Author: K Azaan Ahamed (23BEC0362)
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Robot geometric parameters (mm)
L_BASE = 65.0
L_UPPER = 105.0
L_FOREARM = 120.0

def forward_kinematics(theta1_deg, theta2_deg, theta3_deg):
    t1 = np.radians(theta1_deg)
    t2 = np.radians(theta2_deg)
    t3 = np.radians(theta3_deg)

    # Joint positions
    p0 = np.array([0.0, 0.0, 0.0])
    p1 = np.array([0.0, 0.0, L_BASE])

    r2 = L_UPPER * np.cos(t2)
    p2 = np.array([
        r2 * np.cos(t1),
        r2 * np.sin(t1),
        L_BASE + L_UPPER * np.sin(t2)
    ])

    r3 = L_UPPER * np.cos(t2) + L_FOREARM * np.cos(t2 + t3)
    p3 = np.array([
        r3 * np.cos(t1),
        r3 * np.sin(t1),
        L_BASE + L_UPPER * np.sin(t2) + L_FOREARM * np.sin(t2 + t3)
    ])

    return p0, p1, p2, p3

def generate_workspace_cloud(num_samples=2500):
    t1 = np.random.uniform(0, 180, num_samples)
    t2 = np.random.uniform(20, 160, num_samples)
    t3 = np.random.uniform(20, 160, num_samples)

    xs, ys, zs = [], [], []
    for a, b, c in zip(t1, t2, t3):
        _, _, _, p3 = forward_kinematics(a, b, c)
        xs.append(p3[0])
        ys.append(p3[1])
        zs.append(p3[2])
    return np.array(xs), np.array(ys), np.array(zs)

def main():
    os.environ['MPLCONFIGDIR'] = '/tmp/mpl'
    script_dir = os.path.dirname(os.path.abspath(__file__))
    assets_dir = os.path.join(script_dir, '..', 'assets')
    os.makedirs(assets_dir, exist_ok=True)

    fig = plt.figure(figsize=(10, 8), dpi=150)
    ax = fig.add_subplot(111, projection='3d')

    # Workspace point cloud
    xs, ys, zs = generate_workspace_cloud(3000)
    sc = ax.scatter(xs, ys, zs, c=zs, cmap='viridis', alpha=0.15, s=4, label='Reachable Workspace')

    # Example Arm configuration pose
    p0, p1, p2, p3 = forward_kinematics(45, 60, 45)
    arm_x = [p0[0], p1[0], p2[0], p3[0]]
    arm_y = [p0[1], p1[1], p2[1], p3[1]]
    arm_z = [p0[2], p1[2], p2[2], p3[2]]

    ax.plot(arm_x, arm_y, arm_z, '-o', color='#ff3366', linewidth=4, markersize=8, label='Arm Posture Pose')
    ax.scatter([p3[0]], [p3[1]], [p3[2]], color='#00ffcc', s=100, label='End-Effector Tip')

    ax.set_title('3-DOF Robotic Arm: 3D Reachable Workspace & Kinematics', fontsize=14, fontweight='bold', pad=15)
    ax.set_xlabel('X Position (mm)', fontsize=11, labelpad=10)
    ax.set_ylabel('Y Position (mm)', fontsize=11, labelpad=10)
    ax.set_zlabel('Z Position (mm)', fontsize=11, labelpad=10)
    ax.set_xlim([-150, 250])
    ax.set_ylim([-150, 250])
    ax.set_zlim([0, 300])
    ax.legend(loc='upper right')

    out_path = os.path.join(assets_dir, 'workspace_simulation.png')
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    plt.close()
    print(f"Generated workspace plot at: {out_path}")

if __name__ == '__main__':
    main()
