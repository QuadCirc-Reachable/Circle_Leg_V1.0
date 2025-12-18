#pragma once

#ifndef PI
#define PI 3.1415926535f
#endif

#ifndef TWO_PI
#define TWO_PI (2.0f * PI)
#endif

#define LEG_MOTOR_NUM 4
#define WHEEL_MOTOR_NUM 4

/*
                    WHEEL_TRACK BACK
    3508 ID4 ------------------------- 3508 ID2
    6020 ID3 ------------------------- 6020 ID1
              |                     |
              |                     |
              |                     |
              |                     |
              |                     |
              |                     |
3508 ID3 ------------------------------------ 3508 ID1
6020 ID2 ------------------------------------ 6020 ID4
                    WHEEL_TRACK FRONT


*/

// ==========================================
//        Leg Motor Configuration
// ==========================================

#define USE_HT_LEG_MOTOR 1    // 1: use HT8115 as leg motor;
#define USE_6020_LEG_MOTOR 0  // 1: use DJI GM6020 as leg motor;

#if USE_6020_LEG_MOTOR
#define GM6020_ID1_OFFSET -2.050f
#define GM6020_ID2_OFFSET -0.070f
#define GM6020_ID3_OFFSET -0.040f
#define GM6020_ID4_OFFSET 1.000f

#elif USE_HT_LEG_MOTOR
#define HT8115_ID1_OFFSET 0.0f
#define HT8115_ID2_OFFSET 0.0f
#define HT8115_ID3_OFFSET 0.0f
#define HT8115_ID4_OFFSET 0.0f
#endif
// ==========================================
//        Mechanical Parameters
// ==========================================

// Wheel-Leg Geometry
#define ECCENTRIC_OFFSET_r 65.0f
#define WHEEL_RADIUS_R 320.0f

// Chassis Dimensions
#define WHEEL_TRACK_FRONT 531.0f  // Distance between Front Left and Front Right
#define WHEEL_TRACK_BACK 395.0f   // Distance between Back Left and Back Right
#define WHEEL_BASE 270.0f         // Distance between Front Axle and Back Axle

// IMU Mounting Position (Relative to Chassis Geometric Center)
// Positive X: Forward, Positive Y: Left, Positive Z: Up
#define IMU_MOUNT_X 0.0f
#define IMU_MOUNT_Y 0.0f
#define IMU_MOUNT_Z 0.0f

// IMU Mounting Orientation (Relative to Chassis Frame)
// Unit: Degrees
#define IMU_MOUNT_ROLL_DEG 180.0f
#define IMU_MOUNT_PITCH_DEG 0.0f
#define IMU_MOUNT_YAW_DEG 0.0f

// Robot Physical Properties
#define ROBOT_MASS_kg 10.0f  // Total mass of the robot in kg
#define LEG_MASS_kg 0.5f     // Mass of the moving part of the leg (Motor + Wheel)
#define GRAVITY_g 9.81f      // Gravity acceleration in m/s^2

// Active Suspension Parameters
#define DLS_LAMBDA 0.1f  // Damping factor for Damped Least Squares

// Distances from IMU to Wheel Centers (Calculated or Measured)
// X-axis (Longitudinal)
#define DIST_IMU_TO_FRONT_AXLE (WHEEL_BASE / 2.0f - IMU_MOUNT_X)
#define DIST_IMU_TO_BACK_AXLE (WHEEL_BASE / 2.0f + IMU_MOUNT_X)

// Y-axis (Lateral)
#define DIST_IMU_TO_LEFT_WHEEL (WHEEL_TRACK_FRONT / 2.0f - IMU_MOUNT_Y)  // Assuming symmetric track for now
#define DIST_IMU_TO_RIGHT_WHEEL (WHEEL_TRACK_FRONT / 2.0f + IMU_MOUNT_Y)

// ==========================================
//           Control Parameters
// ==========================================

// Maximum wheel RPM
#define MAX_WHEEL_RPM 200.0f
#define MAX_FORWARD_RPM 100.0f
#define MAX_TURN_RPM 100.0f

// Maximum Leg Speed (Degrees per second)
// Used for Slew Rate Limiter to prevent violent movements
#define LEG_MAX_SPEED 400.0f

// Safety Thresholds
#define MAX_TILT_ANGLE 40.0f          // Max allowed Roll/Pitch in degrees
#define FORCE_LIFT_THRESHOLD 15.0f    // Force threshold (Newtons) for ground detection
#define LEG_EXTENDED_THRESHOLD 45.0f  // Max allowed deviation from BASE_LEG_POS before detecting "Lifted"

// Initial Configuration
#define INITIAL_LEG_ANGLE 90.0f  // Nominal operating angle (Degrees). 0=Extended, 180=Retracted.

// Joystick Parameters
#define JOYSTICK_MAX_R 1000.0f
#define JOYSTICK_DEADZONE 50.0f

// Direction Angle Ranges
// Forward range: [30, 150]
#define ANGLE_FWD_MIN 30.0f
#define ANGLE_FWD_MAX 150.0f

// Backward range: [210, 330]
#define ANGLE_BWD_MIN 210.0f
#define ANGLE_BWD_MAX 330.0f

// Trigger Control Angle Range
#define LEG_TRIGGER_CTRL_MIN_ANGLE 200.0f
#define LEG_TRIGGER_CTRL_MAX_ANGLE 360.0f