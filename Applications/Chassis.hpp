#pragma once
#include "Comm_Msg.hpp"
#include "Controller.hpp"
#include "Cust_Types.hpp"
#include "IMU.hpp"
#include "Math.hpp"
#include "Wheel_Leg.hpp"

namespace Applications
{
using namespace Core::Drivers;
class Chassis
{
   private:
    Chassis_State current_state_ = Chassis_State::CALIBRATION;
    Controller controller_;
    Wheel_Leg *FL_WheelLegs_;
    Wheel_Leg *FR_WheelLegs_;
    Wheel_Leg *BL_WheelLegs_;
    Wheel_Leg *BR_WheelLegs_;

    float target_chassis_height_ = 0.0f;  // Nominal height in meters

    // Debug Variables (Private members for Ozone observation)
    float imu_pitch_pv = 0.0f;
    float imu_roll_pv  = 0.0f;
    float h_fl_pv      = 0.0f;
    float h_fr_pv      = 0.0f;
    float h_bl_pv      = 0.0f;
    float h_br_pv      = 0.0f;

    // Debug Command (Set this in Ozone to switch state safely)
    // -1: No Command
    // 0: IDLE, 1: ENERGY_SAVING, 2: COMFORT, 3: CLIMBING, 4: FREE_CONTROL
    int debug_state_cmd = -1;

    uint8_t last_button_status_ = 0;

   public:
    Chassis() = delete;
    /**
     * @brief Chassis
     * @param fl Front-left Wheel_Leg pointer
     * @param fr Front-right Wheel_Leg pointer
     * @param bl Back-left Wheel_Leg pointer
     * @param br Back-right Wheel_Leg pointer
     */
    Chassis(Wheel_Leg *fl, Wheel_Leg *fr, Wheel_Leg *bl, Wheel_Leg *br);

    //===================//
    //==== User API =====//
    //===================//

    /**
     * @brief Initialize the chassis subsystem
     */
    void Init();

    /**
     * @brief Main update loop
     * @param cmd Command from PC or remote controller
     */
    void Update(const Protocol::PC_Msg &cmd);

    /**
     * @brief Get the Reachable_Msg for feedback
     * @param msg Pointer to the Reachable_Msg to be filled
     */
    void Get_Msg(Protocol::Reachable_Msg *msg);
    //---------------------------------------------------------------------------------------------//

    //======================//
    //==== Internal API ====//
    //======================//
    /**
     * @brief Set the Chassis Mode
     * @param new_state New chassis state
     * */
    void Set_Mode(Chassis_State new_state);

    /**
     * @brief Get the current Chassis Mode
     * @return Current chassis state
     */
    Chassis_State Get_Mode() const { return current_state_; }

    //---------------------------------------------------------------------------------------------//

    //===

   private:
    // --- 各种模式的处理函数 ---
    void handleCalibrationMode();
    void handleEnergySaving(const Protocol::PC_Msg &cmd);
    void handleFreeControl(const Protocol::PC_Msg &cmd);

    // 舒适模式：核心是主动悬挂算法
    // 输入：IMU数据 (Roll, Pitch, Z-accel)
    // 输出：调整腿的角度和轮子的力矩
    void handleComfortMode(const Protocol::PC_Msg &cmd);

    // 攀爬模式：可能涉及到重心调整或特殊的步态
    void handleClimbingMode(const Protocol::PC_Msg &cmd);

    // 运动学解算：将底盘整体速度(Vx, Vy, Wz)分解为4个轮子的速度
    void inverseKinematics(float vx, float vy, float wz, float *out_wheel_rpms);
};

}  // namespace Applications