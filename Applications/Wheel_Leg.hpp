#pragma once
#include "Cust_Types.hpp"
#include "DJIMotor.hpp"
#include "Helper.hpp"
#include "M3508.hpp"
#include "PID.hpp"
#include "Robot_Params.hpp"

#ifndef USE_6020_LEG_MOTOR
#define USE_6020_LEG_MOTOR 0
#endif

#ifndef USE_HT_LEG_MOTOR
#define USE_HT_LEG_MOTOR 0
#endif

#if (USE_6020_LEG_MOTOR && USE_HT_LEG_MOTOR)
#error "Cannot enable both GM6020 and HT8115 for the same leg!"
#endif
#if (!USE_6020_LEG_MOTOR && !USE_HT_LEG_MOTOR)
#error "Must enable at least one leg motor type!"
#endif

#if USE_6020_LEG_MOTOR
#include "GM6020.hpp"
#elif USE_HT_LEG_MOTOR
#include "HT8115.hpp"
#endif

namespace Applications
{

using namespace Core::Drivers;
using namespace Core::Control;
class Wheel_Leg
{
   private:
    //===Wheel Motor Selection===
    Motors::M3508 *wheel_motor;
    PID *Wheel_Vel_PIDs;
    //===Leg Motor Selection===
#if USE_6020_LEG_MOTOR
    Motors::GM6020 *leg_motor;
    PID *Leg_Vel_PIDs;
    PID *Leg_Pos_PIDs;
#elif USE_HT_LEG_MOTOR
    Motors::HT8115 *leg_motor;
    MIT_Params mit_set;
    MIT_Params default_mit_set;
#endif
    //=== Info Struct===
    Wheel_Leg_Params info;
    float leg_offset = 0.0f;

    //=== Pipeline Variables ===
    float target_wheel_rpm       = 0.0f;
    float wheel_compensation_rpm = 0.0f;
    float final_wheel_rpm        = 0.0f;

    // Leg Control (P-V-F)
    float target_leg_pos   = 0.0f;
    float target_leg_vel   = 0.0f;
    float target_leg_force = 0.0f;

    // Leg Impedance (Stiffness & Damping) - For MIT Mode
    float target_leg_kp = 0.0f;
    float target_leg_kd = 0.0f;

    // Leg Compensation
    float leg_compensation_pos   = 0.0f;
    float leg_compensation_vel   = 0.0f;
    float leg_compensation_force = 0.0f;

    // Final Execution
    float final_leg_pos   = 0.0f;
    float final_leg_vel   = 0.0f;
    float final_leg_force = 0.0f;
    float final_leg_kp    = 0.0f;
    float final_leg_kd    = 0.0f;

    // Slew Rate Limiter State
    float prev_leg_pos_cmd = 0.0f;

    // Configuration
    int bending_direction_     = 1;  // 1 for Positive Angle solution, -1 for Negative Angle solution
    float wheel_coupling_sign_ = 1.0f;

   public:
#if USE_6020_LEG_MOTOR
    Wheel_Leg(Motors::M3508 *wheel_motor_,
              PID *Wheel_Vel_PIDs_,
              Motors::GM6020 *leg_motor_,
              PID *Leg_Vel_PIDs_,
              PID *Leg_Pos_PIDs_,
              float leg_offset_         = 0.0f,
              int bending_direction     = 1,
              float wheel_coupling_sign = 1.0f);
#elif USE_HT_LEG_MOTOR
    Wheel_Leg(Motors::M3508 *wheel_motor_,
              PID *Wheel_Vel_PIDs_,
              Motors::HT8115 *leg_motor_,
              MIT_Params mit_pid_,
              float leg_offset_         = 0.0f,
              int bending_direction     = 1,
              float wheel_coupling_sign = 1.0f);
#endif

    //====================//
    //==== Initialize ====//
    //====================//
    /**
     * @brief Initialize Wheel_Leg controller
     */
    void Init();
    //---------------------------------------------------------------------------------------------//

    //==================//
    //====Calculate ====//
    //==================//
    /**
     * @brief Calculate wheel compensation based on leg position and velocity
     * @param leg_current_rpm Current leg motor RPM
     * @param leg_current_pos Current leg motor position in radians
     * @return Compensation rpm to be added to wheel motor control
     */
    float Wheel_Compensation();

    /**
     * @brief Calculate motor torque required for a given vertical force (VMC)
     * @param F_z Vertical force in Newtons
     * @return Required motor torque in N-m
     */
    float VMC_Calculation(float F_z);

    //---------------------------------------------------------------------------------------------//

    //=================//
    //==== Getters ====//
    //=================//
    /**
     * @brief Get wheel RPM command
     * @return Wheel RPM command
     */
    float Get_WheelRPM();
    /**
     * @brief Get leg position command in degrees
     * @return Leg position command in degrees
     */
    float Get_LegPosition();

    /**
     * @brief Get leg torque feedback from motor
     * @return Leg torque in N-m
     */
    float Get_LegTorqueFeedback();

    /**
     * @brief Get the torque required to compensate for leg gravity
     * @return Gravity compensation torque in N-m (usually negative if holding leg up)
     */
    float Get_LegGravityTorque();

    /**
     * @brief Get leg force command in N-m
     * @return Leg force command in N-m
     */
    float Get_LegForce();
    /**
     * @brief Get leg velocity command in rad/s
     * @return Leg velocity command in rad/s
     */
    float Get_LegVelocity();
    /**
     * @brief Get both wheel and leg commands
     * @return Struct containing wheel and leg commands
     */
    Wheel_Leg_Params Get_Info();

    //---------------------------------------------------------------------------------------------//

    //=================//
    //==== Setters ====//
    //=================//
    /**
     * @brief Set target wheel RPM (Stage 1 of Pipeline)
     * @param rpm_cmd Target RPM
     */
    void Set_Wheel_Target(float rpm_cmd);

    /**
     * @brief Add compensation to wheel RPM (Stage 2 of Pipeline)
     * @param comp_rpm Compensation RPM to add
     */
    void Add_Wheel_Compensation(float comp_rpm);

    /**
     * @brief Execute wheel control loop (Stage 3 of Pipeline)
     */
    void Execute_Wheel_Control();

#if USE_6020_LEG_MOTOR
    /**
     * @brief Set leg position command in degrees (Stage 1 of Pipeline)
     * @param pos_cmd Target position in degrees
     * @param vel_cmd Target velocity in rad/s (Feedforward)
     * @param for_cmd Target force in N-m (Feedforward)
     */
    void Set_Leg_Target(float pos_cmd, float vel_cmd = 0.0f, float for_cmd = 0.0f);

    /**
     * @brief Set leg height (Active Suspension)
     * @param h_meters Target height in meters (relative to BASE_LEG_POS level)
     *                 Positive = Up (Extend), Negative = Down (Retract)
     * @param v_meters_s Target vertical velocity in m/s (Feedforward)
     */
    void Set_Leg_Height(float h_meters, float v_meters_s = 0.0f);
#elif USE_HT_LEG_MOTOR
    /**
     * @brief Set leg MIT command (Stage 1 of Pipeline)
     * @param pos_cmd Target position in degrees
     * @param vel_cmd Target velocity in rad/s
     * @param for_cmd Target force in N-m
     * @param kp Position Gain
     * @param kd Velocity Gain
     */
    void Set_Leg_Target(float pos_cmd, float vel_cmd, float for_cmd, float kp = 0.0f, float kd = 0.0f);

    /**
     * @brief Set leg height (Active Suspension)
     * @param h_meters Target height in meters (relative to BASE_LEG_POS level)
     *                 Positive = Up (Extend), Negative = Down (Retract)
     * @param v_meters_s Target vertical velocity in m/s (Feedforward)
     */
    void Set_Leg_Height(float h_meters, float v_meters_s = 0.0f);

    /**
     * @brief Set current position as zero
     */
    void SetZero();
#endif

    /**
     * @brief Add compensation to leg command (Stage 2 of Pipeline)
     * @param comp_pos Position compensation in degrees
     * @param comp_vel Velocity compensation in rad/s
     * @param comp_force Force compensation in N-m
     */
    void Add_Leg_Compensation(float comp_pos, float comp_vel, float comp_force);

    /**
     * @brief Execute leg control loop (Stage 3 of Pipeline)
     */
    void Execute_Leg_Control();

    /**
     * @brief Set both wheel and leg commands and execute immediately (Legacy/Convenience)
     * @param cmd Struct containing wheel and leg commands
     */
    void Set_Wheel_Leg(Wheel_Leg_Params cmd);

    void Set_Bending_Direction(int dir) { bending_direction_ = dir; }

    //---------------------------------------------------------------------------------------------//
};
}  // namespace Applications
