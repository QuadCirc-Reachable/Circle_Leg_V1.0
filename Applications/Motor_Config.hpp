#pragma once

#define LEG_MOTOR_NUM 4
#define WHEEL_MOTOR_NUM 4

/*

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



*/

#define GM6020_ID1_OFFSET -2.050f
#define GM6020_ID2_OFFSET -0.070f
#define GM6020_ID3_OFFSET -0.040f
#define GM6020_ID4_OFFSET 1.000f

// ==========================================
//        Mechanical Parameters
// ==========================================

#define ECCENTRIC_OFFSET_r 65.0f
#define WHEEL_RADIUS_R 320.0f

// ==========================================
//           Control Parameters
// ==========================================

// Maximum wheel RPM
#define MAX_WHEEL_RPM 60.0f

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