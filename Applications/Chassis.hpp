#pragma once
#include "Climbing_Dynamics.hpp"
#include "Comm_Msg.hpp"
#include "Controller.hpp"
#include "Cust_Types.hpp"
#include "Ground_Contact.hpp"
#include "IMU.hpp"
#include "Impedance_Controller.hpp"
#include "Math.hpp"
#include "Wheel_Leg.hpp"

namespace Applications
{
using namespace Core::Drivers;

// =========================================================================
// Debug Global Variables — for direct Ozone watch
// =========================================================================

struct DbgIMU
{
    float pitch   = 0.0f;
    float roll    = 0.0f;
    float accel_z = 0.0f;
};

struct DbgLeveling
{
    float h_fl = 0.0f;
    float h_fr = 0.0f;
    float h_bl = 0.0f;
    float h_br = 0.0f;
};

struct DbgClimbing
{
    float dh_fl      = 0.0f;
    float dh_fr      = 0.0f;
    float dh_bl      = 0.0f;
    float dh_br      = 0.0f;
    uint8_t phase_fl = 0;
    uint8_t phase_fr = 0;
    uint8_t phase_bl = 0;
    uint8_t phase_br = 0;
    // Spike detection debug
    float spike_fl = 0.0f, spike_fr = 0.0f, spike_bl = 0.0f, spike_br = 0.0f;  // |I - baseline|
    float base_fl = 0.0f, base_fr = 0.0f, base_bl = 0.0f, base_br = 0.0f;      // LPF baseline
};

struct DbgControl
{
    int state_cmd        = -1;
    float step_height_mm = 10.0f;
};

// 接地补偿 (warp mode)
struct DbgGroundContact
{
    float warp_error = 0.0f;  // 对角电流差 (A)
    float warp_dh    = 0.0f;  // 补偿量 (m)
    float dh_fl      = 0.0f;
    float dh_fr      = 0.0f;
    float dh_bl      = 0.0f;
    float dh_br      = 0.0f;
};

// 阻抗控制器 (variable impedance)
struct DbgImpedance
{
    float kp_fl = 0.0f, kp_fr = 0.0f, kp_bl = 0.0f, kp_br = 0.0f;
    float kd_fl = 0.0f, kd_fr = 0.0f, kd_bl = 0.0f, kd_br = 0.0f;
    float ffw_fl = 0.0f, ffw_fr = 0.0f, ffw_bl = 0.0f, ffw_br = 0.0f;
    float mass_est   = 0.0f;
    float warp_error = 0.0f;
    float vz         = 0.0f;
};

extern DbgIMU dbg_imu;
extern DbgLeveling dbg_leveling;
extern DbgClimbing dbg_climb;
extern DbgControl dbg_ctrl;
extern DbgGroundContact dbg_gc;
extern DbgImpedance dbg_imp;

// =========================================================================

class Chassis
{
   private:
    Chassis_State current_state_ = Chassis_State::CALIBRATION;
    Controller controller_;
    Wheel_Leg *FL_WheelLegs_;
    Wheel_Leg *FR_WheelLegs_;
    Wheel_Leg *BL_WheelLegs_;
    Wheel_Leg *BR_WheelLegs_;

    // Precomputed geometry (meters), init in constructor
    float R_m_;            // wheel radius
    float r_m_;            // eccentric offset (leg length)
    float wb_m_;           // wheelbase
    float wt_f_m_;         // front track width
    float max_pitch_deg_;  // max PID input clamp for pitch
    float max_roll_deg_;   // max PID input clamp for roll
    float h_max_;          // height upper bound
    float h_min_;          // height lower bound

    float target_chassis_height_ = 0.0f;  // Nominal height in meters

    // Ground Contact Warp Compensator (COMFORT / CLIMBING modes)
    GroundContact ground_contact_;

    // Impedance Controller (COMFORT mode — alternative to position-based suspension)
    Impedance_Controller impedance_;

    // Climbing Dynamics Controller (CLIMBING mode)
    Climbing_Dynamics climbing_;

    // IMU-derived values (updated each cycle by readAndTransformIMU)
    float chassis_pitch_      = 0.0f;  // deg
    float chassis_roll_       = 0.0f;  // deg
    float chassis_accel_z_    = 0.0f;  // m/s² (gravity removed)
    float chassis_pitch_rate_ = 0.0f;  // rad/s
    float chassis_roll_rate_  = 0.0f;  // rad/s

    uint8_t last_button_status_ = 0;

    // Mode transition: smooth Kp ramp when leaving COMFORT
    static constexpr int TRANSITION_FRAMES = 200;  // 0.4s @500Hz
    int mode_transition_timer_             = 0;
    float exit_kp_[4]                      = {35.0f, 35.0f, 35.0f, 35.0f};  // Last impedance Kp per leg
    float exit_kd_[4]                      = {1.5f, 1.5f, 1.5f, 1.5f};      // Last impedance Kd per leg

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
    void handleDebugMode(const Protocol::PC_Msg &cmd);

    // 舒适模式：核心是主动悬挂算法
    // 输入：IMU数据 (Roll, Pitch, Z-accel)
    // 输出：调整腿的角度和轮子的力矩
    void handleComfortMode(const Protocol::PC_Msg &cmd);

    // 攀爬模式：可能涉及到重心调整或特殊的步态
    void handleClimbingMode(const Protocol::PC_Msg &cmd);

    // ===== 共享管线 (Shared Pipeline) =====
    // 读取IMU并转换到底盘坐标系
    void readAndTransformIMU();
    // 按钮 → 目标高度映射 (可在各模式中覆盖)
    void handleHeightButtons(const Protocol::PC_Msg &cmd);
    // 共享的车身控制管线：自动平衡PID + 高度分配 + 轮速控制
    // mode_dh[4]: 由模式特定算法提供的额外高度补偿 (FL, FR, BL, BR)
    void executeBodyControl(const Protocol::PC_Msg &cmd, const float mode_dh[4]);
    // Impedance variant: uses per-leg Kp/Kd/FFW from Impedance_Controller
    void executeBodyControlImpedance(const Protocol::PC_Msg &cmd);
    void executeMotorCommands();
    float clampHeight(float h) const { return h < h_min_ ? h_min_ : (h > h_max_ ? h_max_ : h); }

    // 运动学解算：将底盘整体速度(Vx, Vy, Wz)分解为4个轮子的速度
    void inverseKinematics(float vx, float vy, float wz, float *out_wheel_rpms);

    // Helper functions
    float CalculateHeightFromAngle(float angle_deg);
    void SetBendingDirection(int fl, int fr, int bl, int br);
};

}  // namespace Applications