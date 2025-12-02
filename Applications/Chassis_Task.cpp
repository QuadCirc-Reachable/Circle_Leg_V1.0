#include "Chassis_Task.hpp"

namespace Applications::Chassis_Task
{
using namespace Core::Drivers;
using namespace Core::Control;
using namespace Applications::Command_Task;

// --- Motor Definitions  ---
Motors::GM6020 GM6020_id1(1, 0, Motors::GM6020::ControlMode::CURRENT, false);
Motors::GM6020 GM6020_id2(2, 0, Motors::GM6020::ControlMode::CURRENT, true);
Motors::GM6020 GM6020_id3(3, 0, Motors::GM6020::ControlMode::CURRENT, true);
Motors::GM6020 GM6020_id4(4, 0, Motors::GM6020::ControlMode::CURRENT, false);

Motors::M3508 M3508_id1(1, 1, 6, 1, false);
Motors::M3508 M3508_id2(2, 1, 6, 1, false);
Motors::M3508 M3508_id3(3, 1, 6, 1, true);
Motors::M3508 M3508_id4(4, 1, 6, 1, true);

// --- PID Definitions  ---
// Leg Motor PID Params
PID::Param LEG_MOTOR_VEL_PID_PARAM(18.0f, 1.0f, 0.0f, 5000.0f, 15000.0f, 0.0f, 0.0f, 0.2f, pdMS_TO_TICKS(2));
PID::Param LEG_MOTOR_POS_PID_PARAM(2500.0f, 0.0f, 120.0f, 15000.0f, 15000.0f, 0.0f, 0.0f, 0.2f, pdMS_TO_TICKS(2));

// Wheel Motor PID Params
PID::Param WHEEL_MOTOR_ID1_VEL_PID_PARAM(200.0f, 20.0f, 1.0f, 14000.0f, 16000.0f, 0.0f, 0.0f, 0.2f, pdMS_TO_TICKS(2));

PID::Param WHEEL_MOTOR_ID2_VEL_PID_PARAM(200.0f, 20.0f, 1.0f, 15000.0f, 15000.0f, 0.0f, 0.0f, 0.2f, pdMS_TO_TICKS(2));

PID::Param WHEEL_MOTOR_ID3_VEL_PID_PARAM(200.0f, 25.0f, 1.0f, 15000.0f, 15000.0f, 0.0f, 0.0f, 0.2f, pdMS_TO_TICKS(2));

PID::Param WHEEL_MOTOR_ID4_VEL_PID_PARAM(200.0f, 20.0f, 1.0f, 15000.0f, 15000.0f, 0.0f, 0.0f, 0.2f, pdMS_TO_TICKS(2));

PID LEG_MOTOR_ID1_VEL_PID(LEG_MOTOR_VEL_PID_PARAM);
PID LEG_MOTOR_ID2_VEL_PID(LEG_MOTOR_VEL_PID_PARAM);
PID LEG_MOTOR_ID3_VEL_PID(LEG_MOTOR_VEL_PID_PARAM);
PID LEG_MOTOR_ID4_VEL_PID(LEG_MOTOR_VEL_PID_PARAM);

PID LEG_MOTOR_ID1_POS_PID(LEG_MOTOR_POS_PID_PARAM);
PID LEG_MOTOR_ID2_POS_PID(LEG_MOTOR_POS_PID_PARAM);
PID LEG_MOTOR_ID3_POS_PID(LEG_MOTOR_POS_PID_PARAM);
PID LEG_MOTOR_ID4_POS_PID(LEG_MOTOR_POS_PID_PARAM);

PID WHEEL_MOTOR_ID1_VEL_PID(WHEEL_MOTOR_ID1_VEL_PID_PARAM);
PID WHEEL_MOTOR_ID2_VEL_PID(WHEEL_MOTOR_ID2_VEL_PID_PARAM);
PID WHEEL_MOTOR_ID3_VEL_PID(WHEEL_MOTOR_ID3_VEL_PID_PARAM);
PID WHEEL_MOTOR_ID4_VEL_PID(WHEEL_MOTOR_ID4_VEL_PID_PARAM);

// Arrays
Motors::GM6020 *LEG_Motors[4] = {&GM6020_id1, &GM6020_id2, &GM6020_id3, &GM6020_id4};
PID *LEG_Motor_vel_PIDs[4]    = {&LEG_MOTOR_ID1_VEL_PID, &LEG_MOTOR_ID2_VEL_PID, &LEG_MOTOR_ID3_VEL_PID, &LEG_MOTOR_ID4_VEL_PID};
PID *LEG_Motor_pos_PIDs[4]    = {&LEG_MOTOR_ID1_POS_PID, &LEG_MOTOR_ID2_POS_PID, &LEG_MOTOR_ID3_POS_PID, &LEG_MOTOR_ID4_POS_PID};

Motors::M3508 *WHEEL_Motors[4] = {&M3508_id1, &M3508_id2, &M3508_id3, &M3508_id4};
PID *WHEEL_Motor_vel_PIDs[4]   = {&WHEEL_MOTOR_ID1_VEL_PID, &WHEEL_MOTOR_ID2_VEL_PID, &WHEEL_MOTOR_ID3_VEL_PID, &WHEEL_MOTOR_ID4_VEL_PID};

// Controllers
Motor_Controller::GM6020_Controller LEG_Motor_Controller(LEG_Motors, LEG_Motor_vel_PIDs, LEG_Motor_pos_PIDs);
Motor_Controller::M3508_Controller WHEEL_Motor_Controller(WHEEL_Motors, WHEEL_Motor_vel_PIDs);

// Global Message Buffers
Protocol::Reachable_Msg reachable_msg_chassis = {};
Protocol::PC_Msg pc_msg_chassis               = {};

// State
static bool is_free_mode          = false;
static uint8_t last_button_status = 0;

StackType_t uxChassisTaskStack[2048];
StaticTask_t xChassisTaskTCB;

void Chassis_Task(void *pvPara)
{
    static float LEG_Set_Pos[4]       = {0.0f, 0.0f, 0.0f, 0.0f};
    static float LEG_Current_Pos[4]   = {0.0f, 0.0f, 0.0f, 0.0f};
    static float WHEEL_Set_RPM[4]     = {0.0f, 0.0f, 0.0f, 0.0f};
    static float WHEEL_Current_RPM[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    // Enable Motors
    GM6020_id1.enable();
    GM6020_id2.enable();
    GM6020_id3.enable();
    GM6020_id4.enable();
    M3508_id1.enable();
    M3508_id2.enable();
    M3508_id3.enable();
    M3508_id4.enable();

    while (true)
    {
        // Get PC Command
        Get_PC_Msg(&pc_msg_chassis);

        // Decode PC Command to Motor Setpoints
        Set_Leg_Pos_by_Buttons(&pc_msg_chassis, LEG_Set_Pos, is_free_mode, last_button_status);

        // --- Leg Control (GM6020) ---
        if (is_free_mode)
        {
            // Free Mode - Leg Position Control by Triggers

            // Range Span
            float angle_range = LEG_TRIGGER_CTRL_MAX_ANGLE - LEG_TRIGGER_CTRL_MIN_ANGLE;

            // Left_trigger -> Front Legs (ID2, ID4)
            // Range: 0 (0.0) -> 1000 (1.0) mapped to MIN_ANGLE -> MAX_ANGLE
            float L_trigger_val = (float)pc_msg_chassis.Left_trigger_x1000_msg / 1000.0f;  // 0.0 ~ 1.0
            // Min + (Ratio * Range)
            float target_F_leg_angle = LEG_TRIGGER_CTRL_MIN_ANGLE + (L_trigger_val * angle_range);

            LEG_Set_Pos[1] = target_F_leg_angle;  // ID2
            LEG_Set_Pos[3] = target_F_leg_angle;  // ID4

            // Right_trigger -> Back Legs (ID1, ID3)
            // Range: 0 (0.0) -> 1000 (1.0) mapped to MIN_ANGLE -> MAX_ANGLE
            float R_trigger_val = (float)pc_msg_chassis.Right_trigger_x1000_msg / 1000.0f;  // 0.0 ~ 1.0
            // Min + (Ratio * Range)
            float target_B_leg_angle = LEG_TRIGGER_CTRL_MIN_ANGLE + (R_trigger_val * angle_range);

            LEG_Set_Pos[0] = target_B_leg_angle;  // ID1
            LEG_Set_Pos[2] = target_B_leg_angle;  // ID3
        }

        // --- Wheel (M3508) ---
        // Left Joystick r - > Wheel RPM
        // Range: 0 (0.0) -> 1000 (1.0) mapped to 0 -> 200 RPM
        WHEEL_Set_RPM[0] = Calculate_Wheel_RPM(pc_msg_chassis.left_joystick);   // Left Front Wheel (ID1)
        WHEEL_Set_RPM[1] = Calculate_Wheel_RPM(pc_msg_chassis.left_joystick);   // Left Back Wheel (ID3)
        WHEEL_Set_RPM[2] = Calculate_Wheel_RPM(pc_msg_chassis.right_joystick);  // Right Front Wheel (ID2)
        WHEEL_Set_RPM[3] = Calculate_Wheel_RPM(pc_msg_chassis.right_joystick);  // Right Back Wheel (ID4)

        // Compansation
        WHEEL_Motor_Controller.updateCompensation(LEG_Motors, LEG_Current_Pos, WHEEL_Set_RPM);

        // Control Motors
        LEG_Motor_Controller.setTargetPosition(LEG_Set_Pos, LEG_Current_Pos);
        WHEEL_Motor_Controller.setTargetRPM(WHEEL_Set_RPM, WHEEL_Current_RPM);

        // Transmit CAN Messages
        Motors::DJIMotor::transmit(3, 0);  // Transmit GM6020 group 1 (CAN1)
        Motors::DJIMotor::transmit(0, 1);  // Transmit M3508 group 1 (CAN2)

        // Update Feedback Messages (Reachable_Msg)
        // Fill current motor status into reachable_msg_chassis
        LEG_Motor_Controller.updateReachableMsg(&reachable_msg_chassis);
        WHEEL_Motor_Controller.updateReachableMsg(&reachable_msg_chassis);

        // Send feedback to PC
        Set_Reachable_Msg(&reachable_msg_chassis);

        vTaskDelay(1);
    }
}

void init() { xTaskCreateStatic(Chassis_Task, "Chassis_Task", 2048, NULL, 0, uxChassisTaskStack, &xChassisTaskTCB); }
}  // namespace Applications::Chassis_Task