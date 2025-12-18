#pragma once
#include "DJIMotor.hpp"
#include "FreeRTOS.h"
#include "GM6020.hpp"
#include "Helper.hpp"
#include "M3508.hpp"
#include "Math.hpp"
#include "PC_Comm.hpp"
#include "PID.hpp"
#include "Robot_Params.hpp"
#include "main.h"
#include "task.h"

namespace Motor_Controller
{
using namespace Core::Drivers;
using namespace Core::Control;

class GM6020_Controller
{
   public:
    GM6020_Controller(Motors::GM6020 **Motors, PID **Vel_PIDs, PID **Pos_PIDs);
    GM6020_Controller() = delete;

    /**
     * @brief Set target position for all motors in degrees
     * @param set_position Array of target positions in degrees
     * @param current_position Array to store current positions in degrees (for debugging/logging)
     * @param is_rad Whether the input positions are in radians
     */
    void setTargetPosition(float *set_position, float *current_position, bool is_rad = false);

    /**
     * @brief Update the Reachable_Msg with current motor status
     * @param msg Pointer to the Reachable_Msg to be filled
     */
    void updateReachableMsg(Protocol::Reachable_Msg *msg);

   private:
    Motors::GM6020 **motors;
    PID **vel_PIDs;
    PID **pos_PIDs;
    float target_accumulated_positions_rad[LEG_MOTOR_NUM];
    float current_accumulated_positions_rad[LEG_MOTOR_NUM];
    float target_pos_delta_rad[LEG_MOTOR_NUM];
    float current_positions_deg[LEG_MOTOR_NUM];
    float current_positions_rad[LEG_MOTOR_NUM];

    // For debugging/logging
    float last_set_positions_deg[LEG_MOTOR_NUM] = {0};

#if USE_6020_LEG_MOTOR
    float motor_offsets_rad[LEG_MOTOR_NUM] = {GM6020_ID1_OFFSET, GM6020_ID2_OFFSET, GM6020_ID3_OFFSET, GM6020_ID4_OFFSET};
#else
    float motor_offsets_rad[LEG_MOTOR_NUM] = {0};
#endif
};

class M3508_Controller
{
   public:
    M3508_Controller(Motors::M3508 **Motors, PID **Vel_PIDs);
    M3508_Controller() = delete;

    void setTargetForce(float *set_force);

    /*

    */
    void updateCompensation(Motors::GM6020 **leg_motors, float *leg_current_pos, float *set_rpm);

    /**
     * @brief Set target RPM for all motors
     * @param set_rpm Array of target RPMs
     * @param current_rpm Array to store current RPMs (for debugging/logging)
     */
    void setTargetRPM(float *set_rpm, float *current_rpm);

    /**
     * @brief Update the Reachable_Msg with current motor status
     * @param msg Pointer to the Reachable_Msg to be filled
     */
    void updateReachableMsg(Protocol::Reachable_Msg *msg);

   private:
    Motors::M3508 **motors;
    PID **vel_PIDs;

    // For debugging/logging
    float last_set_rpm[WHEEL_MOTOR_NUM] = {0};
};
}  // namespace Motor_Controller