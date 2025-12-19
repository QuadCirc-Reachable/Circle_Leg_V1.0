#include "Robot_Config.hpp"

namespace Applications
{
using namespace Core::Drivers;
using namespace Core::Control;

// ==========================================
//              Motor Definitions
// ==========================================

// --- Wheel Motors (M3508) ---
// FL: ID4, FR: ID2, BL: ID3, BR: ID1
Motors::M3508 M3508_FL(2, 1, 12, 1, true);   // ID 2, CAN 2, Reduction 12:1 (Example)
Motors::M3508 M3508_FR(3, 1, 12, 1, false);  // ID 3, CAN 2
Motors::M3508 M3508_BL(1, 1, 12, 1, true);   // ID 1, CAN 2
Motors::M3508 M3508_BR(4, 1, 12, 1, false);  // ID 4, CAN 2

// --- Leg Motors ---
#if USE_6020_LEG_MOTOR
// FL: ID3, FR: ID1, BL: ID2, BR: ID4
Motors::GM6020 Leg_Motor_FL(3, 0, Motors::GM6020::ControlMode::CURRENT, false);  // ID 3, CAN 1
Motors::GM6020 Leg_Motor_FR(1, 0, Motors::GM6020::ControlMode::CURRENT, true);   // ID 1, CAN 1
Motors::GM6020 Leg_Motor_BL(2, 0, Motors::GM6020::ControlMode::CURRENT, true);   // ID 2, CAN 1
Motors::GM6020 Leg_Motor_BR(4, 0, Motors::GM6020::ControlMode::CURRENT, false);  // ID 4, CAN 1
#elif USE_HT_LEG_MOTOR
// FL: ID2, FR: ID3, BL: ID1, BR: ID4 (Based on code below)
// Note: Ref/RM2024-Balanced-Infantry uses CAN2 (index 1) and IDs: FL=1, BL=2, FR=3, BR=4.
// If using the same hardware, verify CAN bus and IDs.
Motors::HT8115 Leg_Motor_FL(2, 0, false);  // ID 2, CAN 3
Motors::HT8115 Leg_Motor_FR(3, 0, true);   // ID 3, CAN 3
Motors::HT8115 Leg_Motor_BL(1, 0, true);   // ID 1, CAN 3
Motors::HT8115 Leg_Motor_BR(4, 0, false);  // ID 4, CAN 3
#endif

// ==========================================
//               PID Definitions
// ==========================================

// --- Wheel Velocity PIDs ---
PID::Param WHEEL_VEL_PID_PARAM_FL(200.0f, 20.0f, 1.0f, 15000.0f, 16000.0f, 0.0f, 0.0f, 0.2f, pdMS_TO_TICKS(2));
PID::Param WHEEL_VEL_PID_PARAM_FR(200.0f, 20.0f, 1.0f, 15000.0f, 16000.0f, 0.0f, 0.0f, 0.2f, pdMS_TO_TICKS(2));
PID::Param WHEEL_VEL_PID_PARAM_BL(200.0f, 20.0f, 1.0f, 15000.0f, 16000.0f, 0.0f, 0.0f, 0.2f, pdMS_TO_TICKS(2));
PID::Param WHEEL_VEL_PID_PARAM_BR(200.0f, 20.0f, 1.0f, 15000.0f, 16000.0f, 0.0f, 0.0f, 0.2f, pdMS_TO_TICKS(2));

PID Wheel_PID_FL(WHEEL_VEL_PID_PARAM_FL);
PID Wheel_PID_FR(WHEEL_VEL_PID_PARAM_FR);
PID Wheel_PID_BL(WHEEL_VEL_PID_PARAM_BL);
PID Wheel_PID_BR(WHEEL_VEL_PID_PARAM_BR);

// --- Leg PIDs (For GM6020) ---
#if USE_6020_LEG_MOTOR
PID::Param LEG_VEL_PID_PARAM(18.0f, 1.0f, 0.0f, 5000.0f, 15000.0f, 0.0f, 0.0f, 0.2f, pdMS_TO_TICKS(2));
PID::Param LEG_POS_PID_PARAM(2500.0f, 0.0f, 120.0f, 15000.0f, 15000.0f, 0.0f, 0.0f, 0.2f, pdMS_TO_TICKS(2));

PID Leg_Vel_PID_FL(LEG_VEL_PID_PARAM);
PID Leg_Pos_PID_FL(LEG_POS_PID_PARAM);

PID Leg_Vel_PID_FR(LEG_VEL_PID_PARAM);
PID Leg_Pos_PID_FR(LEG_POS_PID_PARAM);

PID Leg_Vel_PID_BL(LEG_VEL_PID_PARAM);
PID Leg_Pos_PID_BL(LEG_POS_PID_PARAM);

PID Leg_Vel_PID_BR(LEG_VEL_PID_PARAM);
PID Leg_Pos_PID_BR(LEG_POS_PID_PARAM);
#elif USE_HT_LEG_MOTOR
// MIT Params for HT Motor
// Reduced KP to reduce stiffness, increased KD for damping
MIT_Params Leg_MIT_Param_FL = {.Position = 0, .Velocity = 0, .Pos_KP = 35.0f, .Vel_KD = 1.5f, .FFW_Current = 0};
MIT_Params Leg_MIT_Param_FR = {.Position = 0, .Velocity = 0, .Pos_KP = 35.0f, .Vel_KD = 1.5f, .FFW_Current = 0};
MIT_Params Leg_MIT_Param_BL = {.Position = 0, .Velocity = 0, .Pos_KP = 35.0f, .Vel_KD = 1.5f, .FFW_Current = 0};
MIT_Params Leg_MIT_Param_BR = {.Position = 0, .Velocity = 0, .Pos_KP = 35.0f, .Vel_KD = 1.5f, .FFW_Current = 0};
#endif

// ==========================================
//           Wheel_Leg Instantiation
// ==========================================

#if USE_6020_LEG_MOTOR
Wheel_Leg FL_Leg(&M3508_FL, &Wheel_PID_FL, &Leg_Motor_FL, &Leg_Vel_PID_FL, &Leg_Pos_PID_FL, GM6020_ID3_OFFSET, 1, -1.0f);
Wheel_Leg FR_Leg(&M3508_FR, &Wheel_PID_FR, &Leg_Motor_FR, &Leg_Vel_PID_FR, &Leg_Pos_PID_FR, GM6020_ID1_OFFSET, 1, -1.0f);
Wheel_Leg BL_Leg(&M3508_BL, &Wheel_PID_BL, &Leg_Motor_BL, &Leg_Vel_PID_BL, &Leg_Pos_PID_BL, GM6020_ID2_OFFSET, -1, 1.0f);
Wheel_Leg BR_Leg(&M3508_BR, &Wheel_PID_BR, &Leg_Motor_BR, &Leg_Vel_PID_BR, &Leg_Pos_PID_BR, GM6020_ID4_OFFSET, -1, 1.0f);
#elif USE_HT_LEG_MOTOR
Wheel_Leg FL_Leg(&M3508_FL, &Wheel_PID_FL, &Leg_Motor_FL, Leg_MIT_Param_FL, HT8115_ID3_OFFSET, 1, -1.0f);
Wheel_Leg FR_Leg(&M3508_FR, &Wheel_PID_FR, &Leg_Motor_FR, Leg_MIT_Param_FR, HT8115_ID1_OFFSET, 1, -1.0f);
Wheel_Leg BL_Leg(&M3508_BL, &Wheel_PID_BL, &Leg_Motor_BL, Leg_MIT_Param_BL, HT8115_ID2_OFFSET, 1, 1.0f);
Wheel_Leg BR_Leg(&M3508_BR, &Wheel_PID_BR, &Leg_Motor_BR, Leg_MIT_Param_BR, HT8115_ID4_OFFSET, 1, 1.0f);
#endif

// ==========================================
//           Chassis Instantiation
// ==========================================

Chassis chassis(&FL_Leg, &FR_Leg, &BL_Leg, &BR_Leg);

}  // namespace Applications
