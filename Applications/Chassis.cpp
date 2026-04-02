#include "Chassis.hpp"

#include "Matrixf.hpp"
#include "PC_Comm.hpp"
#include "PID.hpp"
#include "Quaternion.hpp"

namespace Applications
{
using namespace Core::Drivers;

// PID Parameters for Active Suspension (Comfort Mode)
// Adjust these based on actual tuning
// Output limit is now in METERS.
// Max travel is 2*r = 130mm = 0.13m. Set limit to 0.15m to allow full range.
static Core::Control::PID::Param roll_pid_param(0.015f, 0.0002f, 0.00012f, 1000.0f, 0.08f);
static Core::Control::PID::Param pitch_pid_param(0.015f, 0.00015f, 0.00012f, 1000.0f, 0.08f);

static Core::Control::PID roll_pid(roll_pid_param);
static Core::Control::PID pitch_pid(pitch_pid_param);

static inline float clampSym(float v, float lim) { return v > lim ? lim : (v < -lim ? -lim : v); }

// =====================================================================
// Bending Direction Convention
// =====================================================================
// bending_direction determines which angular solution Set_Leg_Height uses:
//   +1 → positive angle θ (0..+180°)
//   -1 → negative angle θ (0..-180°)
//
// "Standard" stance: all legs same direction (matches HT motor defaults)
// "Inward" stance:  all legs reversed → wheels face each other (收缩)
//
// Adjust these if physical motor mounting or bending convention changes.
static constexpr int BEND_STD_FL = 1, BEND_STD_FR = 1, BEND_STD_BL = 1, BEND_STD_BR = 1;
static constexpr int BEND_INV_FL = -1, BEND_INV_FR = -1, BEND_INV_BL = -1, BEND_INV_BR = -1;

// =====================================================================
// Global debug variables — watch these directly in Ozone
// =====================================================================
DbgIMU dbg_imu;
DbgLeveling dbg_leveling;
DbgClimbing dbg_climb;
DbgControl dbg_ctrl;
DbgGroundContact dbg_gc;
DbgImpedance dbg_imp;

Chassis::Chassis(Wheel_Leg *fl, Wheel_Leg *fr, Wheel_Leg *bl, Wheel_Leg *br)
    : FL_WheelLegs_(fl),
      FR_WheelLegs_(fr),
      BL_WheelLegs_(bl),
      BR_WheelLegs_(br),
      R_m_(WHEEL_RADIUS_R / 1000.0f),
      r_m_(ECCENTRIC_OFFSET_r / 1000.0f),
      wb_m_(WHEEL_BASE / 1000.0f),
      wt_f_m_(WHEEL_TRACK_FRONT / 1000.0f)
{
    max_pitch_deg_  = rad2deg(atan2f(2.0f * r_m_, wb_m_));
    max_roll_deg_   = rad2deg(atan2f(2.0f * r_m_, wt_f_m_));
    float limit_cos = cosf(deg2rad(10.0f));
    h_max_          = R_m_ + r_m_ * limit_cos;
    h_min_          = R_m_ - r_m_ * limit_cos;
    // H = R + r * cos(theta). At 135 deg ≈ 0.114m.
    target_chassis_height_ = R_m_ + r_m_ * cosf(deg2rad(INITIAL_LEG_ANGLE));
}

void Chassis::Init()
{
    if (FL_WheelLegs_)
        FL_WheelLegs_->Init();
    if (FR_WheelLegs_)
        FR_WheelLegs_->Init();
    if (BL_WheelLegs_)
        BL_WheelLegs_->Init();
    if (BR_WheelLegs_)
        BR_WheelLegs_->Init();
}

void Chassis::Set_Mode(Chassis_State new_state)
{
    // --- Capture exit state when LEAVING COMFORT (for smooth Kp ramp-out) ---
    if (current_state_ == Chassis_State::COMFORT && new_state != Chassis_State::COMFORT)
    {
        for (int i = 0; i < 4; i++)
        {
            exit_kp_[i] = impedance_.getLegOutput(i).kp;
            exit_kd_[i] = impedance_.getLegOutput(i).kd;
        }
        mode_transition_timer_ = TRANSITION_FRAMES;
    }

    current_state_ = new_state;
    // Reset PIDs when entering modes that use body leveling
    if (new_state == Chassis_State::COMFORT || new_state == Chassis_State::CLIMBING)
    {
        roll_pid.reset();
        pitch_pid.reset();
        ground_contact_.reset();
    }
    // Reset mode-specific compensators
    if (new_state == Chassis_State::COMFORT)
    {
        impedance_.reset();  // Starts ramp from entry_kp → impedance values
    }
    if (new_state == Chassis_State::CLIMBING)
    {
        climbing_.reset();
        climbing_.startClimbAll();
        // Don't jump to h_max_ — ramp height smoothly in handleClimbingMode PREP phase
        // target_chassis_height_ stays at current value
        // Keep current bending direction from COMFORT — switching direction mid-transition
        // causes the slew-rate limiter to route through θ=180° (h_min), dropping the robot.
    }
}

void Chassis::Update(const Protocol::PC_Msg &cmd)
{
    // Check connection first
    if (!Applications::Command_Task::Is_PC_Connected())
    {
        if (current_state_ != Chassis_State::IDLE)
        {
            Set_Mode(Chassis_State::IDLE);
        }

        // Force update of last button to prevent immediate jump on reconnect
        // Setting to 0xFF means all buttons are considered "previously pressed"
        // So a held button on reconnect (1) won't trigger a rising edge (1->1 no change, or 0->1 rising)
        // logic: rising = (current ^ last) & current
        // If held on reconnect: current=1, last=0xFF. changed=0xFE. rising=0. SAFE.
        last_button_status_ = 0xFF;

        // Stop motors immediately for safety
        Wheel_Leg_Params stop_params = {0};
        stop_params.state            = Chassis_State::IDLE;
        if (FL_WheelLegs_)
            FL_WheelLegs_->Set_Wheel_Leg(stop_params);
        if (FR_WheelLegs_)
            FR_WheelLegs_->Set_Wheel_Leg(stop_params);
        if (BL_WheelLegs_)
            BL_WheelLegs_->Set_Wheel_Leg(stop_params);
        if (BR_WheelLegs_)
            BR_WheelLegs_->Set_Wheel_Leg(stop_params);

        return;
    }

    // Extract buttons
    uint8_t current_buttons = cmd.button_status;
    uint8_t changing_edges  = current_buttons ^ last_button_status_;
    uint8_t rising_edges    = changing_edges & current_buttons;
    last_button_status_     = current_buttons;

    bool ml_pressed = (current_buttons & BTN_ML);
    bool mr_pressed = (current_buttons & BTN_MR);

    // Check for Calibration (Both ML and MR pressed)
    if (ml_pressed && mr_pressed)
    {
        if (current_state_ != Chassis_State::CALIBRATION)
        {
            Set_Mode(Chassis_State::CALIBRATION);
        }
    }
    else
    {
        // Single button presses for state switching
        // Range [1, 6]: IDLE(1) ... DEBUG(6)
        int state_idx = static_cast<int>(current_state_);

        // If current state is out of range (e.g. CALIBRATION=0 or ERROR), default to IDLE(1) for next switch
        if (state_idx < 1 || state_idx > 6)
            state_idx = 1;

        if (rising_edges & BTN_ML)
        {
            // ML: Up Switch (Previous)
            state_idx--;
            if (state_idx < 1)
                state_idx = 6;
            Set_Mode(static_cast<Chassis_State>(state_idx));
        }
        else if (rising_edges & BTN_MR)
        {
            // MR: Down Switch (Next)
            state_idx++;
            if (state_idx > 6)
                state_idx = 1;
            Set_Mode(static_cast<Chassis_State>(state_idx));
        }
    }

    last_button_status_ = current_buttons;

    switch (current_state_)
    {
    case Chassis_State::CALIBRATION:
        handleCalibrationMode();
        break;
    case Chassis_State::IDLE:
    {
        Wheel_Leg_Params stop_params = {0};
        stop_params.state            = Chassis_State::IDLE;
        if (FL_WheelLegs_)
            FL_WheelLegs_->Set_Wheel_Leg(stop_params);
        if (FR_WheelLegs_)
            FR_WheelLegs_->Set_Wheel_Leg(stop_params);
        if (BL_WheelLegs_)
            BL_WheelLegs_->Set_Wheel_Leg(stop_params);
        if (BR_WheelLegs_)
            BR_WheelLegs_->Set_Wheel_Leg(stop_params);
        break;
    }
    case Chassis_State::ENERGY_SAVING:
        handleEnergySaving(cmd);
        break;
    case Chassis_State::COMFORT:
        handleComfortMode(cmd);
        break;
    case Chassis_State::CLIMBING:
        handleClimbingMode(cmd);
        break;
    case Chassis_State::FREE_CONTROL:
        handleFreeControl(cmd);
        break;
    case Chassis_State::DEBUG:
        handleDebugMode(cmd);
        break;
    default:
        break;
    }
}

void Chassis::handleCalibrationMode()
{
#if USE_HT_LEG_MOTOR
    // Re-enter motor mode (safe if already entered; recovers from lost ENTER_MOTOR)
    if (FL_WheelLegs_)
        FL_WheelLegs_->EnterMotorMode();
    if (FR_WheelLegs_)
        FR_WheelLegs_->EnterMotorMode();
    if (BL_WheelLegs_)
        BL_WheelLegs_->EnterMotorMode();
    if (BR_WheelLegs_)
        BR_WheelLegs_->EnterMotorMode();

    if (FL_WheelLegs_)
        FL_WheelLegs_->SetZero();
    if (FR_WheelLegs_)
        FR_WheelLegs_->SetZero();
    if (BL_WheelLegs_)
        BL_WheelLegs_->SetZero();
    if (BR_WheelLegs_)
        BR_WheelLegs_->SetZero();
#endif
    Set_Mode(Chassis_State::IDLE);
}

void Chassis::readAndTransformIMU()
{
    // 1. Read Euler Angles
    float euler[3];  // Z, Y, X (Yaw, Pitch, Roll)
    Core::Drivers::IMU::getEulerZYX(euler);

    float current_pitch = rad2deg(euler[1]);
    float current_roll  = rad2deg(euler[2]);

    // 2. Transform to Chassis Frame (correct for IMU mounting offset)
    float raw_roll  = current_roll - IMU_MOUNT_ROLL_DEG;
    float raw_pitch = current_pitch;

    // Normalize Roll to [-180, 180]
    if (raw_roll > 180.0f)
        raw_roll -= 360.0f;
    if (raw_roll < -180.0f)
        raw_roll += 360.0f;

    // Low-pass filter angles (cutoff ≈ 4 Hz @ 500 Hz)
    // Prevents motor vibration from feeding back through IMU into PID
    constexpr float imu_alpha = 0.02f;
    chassis_roll_             = imu_alpha * raw_roll + (1.0f - imu_alpha) * chassis_roll_;
    chassis_pitch_            = imu_alpha * raw_pitch + (1.0f - imu_alpha) * chassis_pitch_;

    // Update debug variables
    dbg_imu.pitch = chassis_pitch_;
    dbg_imu.roll  = chassis_roll_;

    // 3. Earth-frame linear acceleration (gravity-removed, AHRS-fused)
    float earth_accel[3];
    Core::Drivers::IMU::getEarthLinearAccel(earth_accel);
    chassis_accel_z_ = earth_accel[2];  // Z-up in earth frame

    // 4. Read & Transform Gyro
    float gyro[3];
    Core::Drivers::IMU::getCalibratedGyroTransformed(gyro);
    // IMU Roll=180 -> X=X, Y=-Y, Z=-Z
    chassis_pitch_rate_ = -gyro[1];
    chassis_roll_rate_  = gyro[0];

    dbg_imu.accel_z = chassis_accel_z_;
}

void Chassis::handleHeightButtons(const Protocol::PC_Msg &cmd)
{
    //==========================================================================================
    //============ Button Mappings for Target Heights + Bending Direction =======================
    //
    // Standard (BEND_STD): all legs same direction (outward)
    // Inverted (BEND_INV): front reversed, back normal (inward / 收缩)
    //==========================================================================================

    // X: 90 deg — standard stance
    if (cmd.button_status & BTN_X)
    {
        target_chassis_height_ = CalculateHeightFromAngle(90.0f);
        SetBendingDirection(BEND_STD_FL, BEND_STD_FR, BEND_STD_BL, BEND_STD_BR);
    }
    // Y: 145 deg — standard stance
    else if (cmd.button_status & BTN_Y)
    {
        target_chassis_height_ = CalculateHeightFromAngle(145.0f);
        SetBendingDirection(BEND_STD_FL, BEND_STD_FR, BEND_STD_BL, BEND_STD_BR);
    }
    // A: 45 deg — standard stance
    else if (cmd.button_status & BTN_A)
    {
        target_chassis_height_ = CalculateHeightFromAngle(45.0f);
        SetBendingDirection(BEND_STD_FL, BEND_STD_FR, BEND_STD_BL, BEND_STD_BR);
    }
    // B: 165 deg — standard stance
    else if (cmd.button_status & BTN_B)
    {
        target_chassis_height_ = CalculateHeightFromAngle(165.0f);
        SetBendingDirection(BEND_STD_FL, BEND_STD_FR, BEND_STD_BL, BEND_STD_BR);
    }
    // RB: 135 deg — inward stance (front reversed, back normal)
    else if (cmd.button_status & BTN_RB)
    {
        target_chassis_height_ = CalculateHeightFromAngle(135.0f);
        SetBendingDirection(BEND_INV_FL, BEND_INV_FR, BEND_INV_BL, BEND_INV_BR);
    }
    // LB: 90 deg — inward stance (front reversed, back normal)
    else if (cmd.button_status & BTN_LB)
    {
        target_chassis_height_ = CalculateHeightFromAngle(90.0f);
        SetBendingDirection(BEND_INV_FL, BEND_INV_FR, BEND_INV_BL, BEND_INV_BR);
    }
}

void Chassis::executeBodyControl(const Protocol::PC_Msg &cmd, const float mode_dh[4])
{
    // 1. Body Leveling PID
    float roll_h_adj  = roll_pid(0.0f, clampSym(chassis_roll_, max_roll_deg_));
    float pitch_h_adj = pitch_pid(0.0f, clampSym(chassis_pitch_, max_pitch_deg_));

    // 2. Gyro Feedforward (disabled — set ff_gain > 0 to re-enable after tuning)
    static float filt_pitch_rate = 0.0f, filt_roll_rate = 0.0f;
    const float alpha = 0.05f, ff_gain = 0.0f;
    filt_pitch_rate  = alpha * chassis_pitch_rate_ + (1.0f - alpha) * filt_pitch_rate;
    filt_roll_rate   = alpha * chassis_roll_rate_ + (1.0f - alpha) * filt_roll_rate;
    float v_pitch_ff = (wb_m_ / 2.0f) * filt_pitch_rate * ff_gain;
    float v_roll_ff  = (wt_f_m_ / 2.0f) * filt_roll_rate * ff_gain;

    // 3. Per-leg height distribution: base + leveling + mode ΔH → clamp → send
    // FL (Front-Left): +Pitch, -Roll
    float h_fl = clampHeight(target_chassis_height_ + pitch_h_adj - roll_h_adj + mode_dh[0]);
    float v_fl = v_pitch_ff - v_roll_ff;
    if (FL_WheelLegs_)
        FL_WheelLegs_->Set_Leg_Height(h_fl, v_fl);

    // FR (Front-Right): +Pitch, +Roll
    float h_fr = clampHeight(target_chassis_height_ + pitch_h_adj + roll_h_adj + mode_dh[1]);
    float v_fr = v_pitch_ff + v_roll_ff;
    if (FR_WheelLegs_)
        FR_WheelLegs_->Set_Leg_Height(h_fr, v_fr);

    // BL (Back-Left): -Pitch, -Roll
    float h_bl = clampHeight(target_chassis_height_ - pitch_h_adj - roll_h_adj + mode_dh[2]);
    float v_bl = -v_pitch_ff - v_roll_ff;
    if (BL_WheelLegs_)
        BL_WheelLegs_->Set_Leg_Height(h_bl, v_bl);

    // BR (Back-Right): -Pitch, +Roll
    float h_br = clampHeight(target_chassis_height_ - pitch_h_adj + roll_h_adj + mode_dh[3]);
    float v_br = -v_pitch_ff + v_roll_ff;
    if (BR_WheelLegs_)
        BR_WheelLegs_->Set_Leg_Height(h_br, v_br);

    dbg_leveling.h_fl = h_fl;
    dbg_leveling.h_fr = h_fr;
    dbg_leveling.h_bl = h_bl;
    dbg_leveling.h_br = h_br;

    // 4. Wheel Velocity Control
    float wheel_rpms[4], vx = 0.0f, wz = 0.0f;
    controller_.Map_Joystick_To_Velocity(cmd, vx, wz);
    inverseKinematics(vx, 0, wz, wheel_rpms);

    if (FL_WheelLegs_)
    {
        FL_WheelLegs_->Set_Wheel_Target(wheel_rpms[0]);
        FL_WheelLegs_->Add_Wheel_Compensation(FL_WheelLegs_->Wheel_Compensation());
    }
    if (FR_WheelLegs_)
    {
        FR_WheelLegs_->Set_Wheel_Target(wheel_rpms[1]);
        FR_WheelLegs_->Add_Wheel_Compensation(FR_WheelLegs_->Wheel_Compensation());
    }
    if (BL_WheelLegs_)
    {
        BL_WheelLegs_->Set_Wheel_Target(wheel_rpms[2]);
        BL_WheelLegs_->Add_Wheel_Compensation(BL_WheelLegs_->Wheel_Compensation());
    }
    if (BR_WheelLegs_)
    {
        BR_WheelLegs_->Set_Wheel_Target(wheel_rpms[3]);
        BR_WheelLegs_->Add_Wheel_Compensation(BR_WheelLegs_->Wheel_Compensation());
    }

    // 5. Execute
    executeMotorCommands();
}

// =====================================================================
// Impedance Body Control: PID leveling + per-leg Kp/Kd/FFW override
// =====================================================================
void Chassis::executeBodyControlImpedance(const Protocol::PC_Msg &cmd)
{
    // 1. Body Leveling PID (same as position-based path)
    float roll_h_adj  = roll_pid(0.0f, clampSym(chassis_roll_, max_roll_deg_));
    float pitch_h_adj = pitch_pid(0.0f, clampSym(chassis_pitch_, max_pitch_deg_));

    // 2. Gyro Feedforward (disabled — set ff_gain > 0 to re-enable after tuning)
    static float filt_pitch_rate_i = 0.0f, filt_roll_rate_i = 0.0f;
    const float alpha = 0.05f, ff_gain = 0.0f;
    filt_pitch_rate_i = alpha * chassis_pitch_rate_ + (1.0f - alpha) * filt_pitch_rate_i;
    filt_roll_rate_i  = alpha * chassis_roll_rate_ + (1.0f - alpha) * filt_roll_rate_i;
    float v_pitch_ff  = (wb_m_ / 2.0f) * filt_pitch_rate_i * ff_gain;
    float v_roll_ff   = (wt_f_m_ / 2.0f) * filt_roll_rate_i * ff_gain;

    // 3. Per-leg height + impedance params → Set_Leg_Height(h, v, kp, kd, ffw)
    //    PID offsets for pitch/roll leveling are position commands; Kp/Kd/FFW handle compliance.
    auto &out_fl = impedance_.getLegOutput(0);
    auto &out_fr = impedance_.getLegOutput(1);
    auto &out_bl = impedance_.getLegOutput(2);
    auto &out_br = impedance_.getLegOutput(3);

    // FL (Front-Left): +Pitch, -Roll
    float h_fl = clampHeight(target_chassis_height_ + pitch_h_adj - roll_h_adj);
    float v_fl = v_pitch_ff - v_roll_ff;
    if (FL_WheelLegs_)
        FL_WheelLegs_->Set_Leg_Height(h_fl, v_fl, out_fl.kp, out_fl.kd, out_fl.ffw_torque);

    // FR (Front-Right): +Pitch, +Roll
    float h_fr = clampHeight(target_chassis_height_ + pitch_h_adj + roll_h_adj);
    float v_fr = v_pitch_ff + v_roll_ff;
    if (FR_WheelLegs_)
        FR_WheelLegs_->Set_Leg_Height(h_fr, v_fr, out_fr.kp, out_fr.kd, out_fr.ffw_torque);

    // BL (Back-Left): -Pitch, -Roll
    float h_bl = clampHeight(target_chassis_height_ - pitch_h_adj - roll_h_adj);
    float v_bl = -v_pitch_ff - v_roll_ff;
    if (BL_WheelLegs_)
        BL_WheelLegs_->Set_Leg_Height(h_bl, v_bl, out_bl.kp, out_bl.kd, out_bl.ffw_torque);

    // BR (Back-Right): -Pitch, +Roll
    float h_br = clampHeight(target_chassis_height_ - pitch_h_adj + roll_h_adj);
    float v_br = -v_pitch_ff + v_roll_ff;
    if (BR_WheelLegs_)
        BR_WheelLegs_->Set_Leg_Height(h_br, v_br, out_br.kp, out_br.kd, out_br.ffw_torque);

    dbg_leveling.h_fl = h_fl;
    dbg_leveling.h_fr = h_fr;
    dbg_leveling.h_bl = h_bl;
    dbg_leveling.h_br = h_br;

    // 4. Wheel Velocity Control (same as position-based path)
    float wheel_rpms[4], vx = 0.0f, wz = 0.0f;
    controller_.Map_Joystick_To_Velocity(cmd, vx, wz);
    inverseKinematics(vx, 0, wz, wheel_rpms);

    if (FL_WheelLegs_)
    {
        FL_WheelLegs_->Set_Wheel_Target(wheel_rpms[0]);
        FL_WheelLegs_->Add_Wheel_Compensation(FL_WheelLegs_->Wheel_Compensation());
    }
    if (FR_WheelLegs_)
    {
        FR_WheelLegs_->Set_Wheel_Target(wheel_rpms[1]);
        FR_WheelLegs_->Add_Wheel_Compensation(FR_WheelLegs_->Wheel_Compensation());
    }
    if (BL_WheelLegs_)
    {
        BL_WheelLegs_->Set_Wheel_Target(wheel_rpms[2]);
        BL_WheelLegs_->Add_Wheel_Compensation(BL_WheelLegs_->Wheel_Compensation());
    }
    if (BR_WheelLegs_)
    {
        BR_WheelLegs_->Set_Wheel_Target(wheel_rpms[3]);
        BR_WheelLegs_->Add_Wheel_Compensation(BR_WheelLegs_->Wheel_Compensation());
    }

    // 5. Execute
    executeMotorCommands();
}

// =====================================================================
// COMFORT MODE: Variable Impedance Control (Kp/Kd/FFW modulation)
// =====================================================================
void Chassis::handleComfortMode(const Protocol::PC_Msg &cmd)
{
    // 1. Read IMU (populates chassis_pitch_, chassis_roll_, chassis_accel_z_, rates)
    readAndTransformIMU();

    // 2. Button → Target Height
    handleHeightButtons(cmd);

    // 3. Gather per-leg data
    float leg_currents[4] = {FL_WheelLegs_ ? FL_WheelLegs_->Get_LegCurrentFeedback() : 0.0f,
                             FR_WheelLegs_ ? FR_WheelLegs_->Get_LegCurrentFeedback() : 0.0f,
                             BL_WheelLegs_ ? BL_WheelLegs_->Get_LegCurrentFeedback() : 0.0f,
                             BR_WheelLegs_ ? BR_WheelLegs_->Get_LegCurrentFeedback() : 0.0f};

    float leg_angles[4] = {FL_WheelLegs_ ? FL_WheelLegs_->Get_LegPosition() : 90.0f,
                           FR_WheelLegs_ ? FR_WheelLegs_->Get_LegPosition() : 90.0f,
                           BL_WheelLegs_ ? BL_WheelLegs_->Get_LegPosition() : 90.0f,
                           BR_WheelLegs_ ? BR_WheelLegs_->Get_LegPosition() : 90.0f};

    const float dt = 0.002f;  // 500 Hz

    // 4. Update Impedance Controller → per-leg Kp, Kd, FFW
    impedance_.update(leg_currents, leg_angles, chassis_accel_z_, chassis_roll_rate_, chassis_pitch_rate_, dt);

    // 5. Update debug variables
    dbg_imp.kp_fl      = impedance_.getLegOutput(0).kp;
    dbg_imp.kp_fr      = impedance_.getLegOutput(1).kp;
    dbg_imp.kp_bl      = impedance_.getLegOutput(2).kp;
    dbg_imp.kp_br      = impedance_.getLegOutput(3).kp;
    dbg_imp.kd_fl      = impedance_.getLegOutput(0).kd;
    dbg_imp.kd_fr      = impedance_.getLegOutput(1).kd;
    dbg_imp.kd_bl      = impedance_.getLegOutput(2).kd;
    dbg_imp.kd_br      = impedance_.getLegOutput(3).kd;
    dbg_imp.ffw_fl     = impedance_.getLegOutput(0).ffw_torque;
    dbg_imp.ffw_fr     = impedance_.getLegOutput(1).ffw_torque;
    dbg_imp.ffw_bl     = impedance_.getLegOutput(2).ffw_torque;
    dbg_imp.ffw_br     = impedance_.getLegOutput(3).ffw_torque;
    dbg_imp.mass_est   = impedance_.getEstimatedMass();
    dbg_imp.warp_error = impedance_.getWarpError();
    dbg_imp.vz         = impedance_.getBodyVelZ();

    // 6. Execute body control with impedance parameters
    executeBodyControlImpedance(cmd);
}

void Chassis::handleFreeControl(const Protocol::PC_Msg &cmd)
{
    static float folded_angle = 180.0f;

    // Triggers Control Angle Interpolation
    // Left Trigger: FL & FR
    // Right Trigger: BL & BR
    // 0 (Released) -> folded_angle
    // 1000 (Pressed) -> 0 deg (Extended)
    float l_ratio = (float)cmd.Left_trigger_x1000_msg / 1000.0f;
    float r_ratio = (float)cmd.Right_trigger_x1000_msg / 1000.0f;

    float fl_fr_angle = -folded_angle * (1.0f - l_ratio);
    float bl_br_angle = folded_angle * (1.0f - r_ratio);

    Wheel_Leg_Params params;
    params.state     = Chassis_State::FREE_CONTROL;
    params.Leg_Force = 0;
    params.Leg_RPM   = 0;
    // Use default stiff parameters for position control
    params.Leg_Kp = 50.0f;
    params.Leg_Kd = 1.0f;

    // Wheel control
    float wheel_rpms[4], vx = 0.0f, wz = 0.0f;
    controller_.Map_Joystick_To_Velocity(cmd, vx, wz);
    inverseKinematics(vx, 0, wz, wheel_rpms);

    if (FL_WheelLegs_)
    {
        params.Leg_POS   = fl_fr_angle;
        params.Wheel_RPM = wheel_rpms[0];
        FL_WheelLegs_->Set_Wheel_Leg(params);
    }
    if (FR_WheelLegs_)
    {
        params.Leg_POS   = fl_fr_angle;
        params.Wheel_RPM = wheel_rpms[1];
        FR_WheelLegs_->Set_Wheel_Leg(params);
    }
    if (BL_WheelLegs_)
    {
        params.Leg_POS   = bl_br_angle;
        params.Wheel_RPM = wheel_rpms[2];
        BL_WheelLegs_->Set_Wheel_Leg(params);
    }
    if (BR_WheelLegs_)
    {
        params.Leg_POS   = bl_br_angle;
        params.Wheel_RPM = wheel_rpms[3];
        BR_WheelLegs_->Set_Wheel_Leg(params);
    }
}

// =====================================================================
// DEBUG MODE: Pure height + wheel control, NO pitch/roll leveling
// Same button decoding as COMFORT (handleHeightButtons), but no PID,
// no gyro feedforward, no active suspension.
// =====================================================================
void Chassis::handleDebugMode(const Protocol::PC_Msg &cmd)
{
    // 1. Button → Target Height + Bending Direction (shared mapping)
    handleHeightButtons(cmd);

    // 2. Direct height command — no PID leveling, no gyro FF
    if (FL_WheelLegs_)
        FL_WheelLegs_->Set_Leg_Height(target_chassis_height_);
    if (FR_WheelLegs_)
        FR_WheelLegs_->Set_Leg_Height(target_chassis_height_);
    if (BL_WheelLegs_)
        BL_WheelLegs_->Set_Leg_Height(target_chassis_height_);
    if (BR_WheelLegs_)
        BR_WheelLegs_->Set_Leg_Height(target_chassis_height_);

    dbg_leveling.h_fl = target_chassis_height_;
    dbg_leveling.h_fr = target_chassis_height_;
    dbg_leveling.h_bl = target_chassis_height_;
    dbg_leveling.h_br = target_chassis_height_;

    // 3. Wheel velocity control
    float wheel_rpms[4], vx = 0.0f, wz = 0.0f;
    controller_.Map_Joystick_To_Velocity(cmd, vx, wz);
    inverseKinematics(vx, 0, wz, wheel_rpms);

    if (FL_WheelLegs_)
    {
        FL_WheelLegs_->Set_Wheel_Target(wheel_rpms[0]);
        FL_WheelLegs_->Add_Wheel_Compensation(FL_WheelLegs_->Wheel_Compensation());
    }
    if (FR_WheelLegs_)
    {
        FR_WheelLegs_->Set_Wheel_Target(wheel_rpms[1]);
        FR_WheelLegs_->Add_Wheel_Compensation(FR_WheelLegs_->Wheel_Compensation());
    }
    if (BL_WheelLegs_)
    {
        BL_WheelLegs_->Set_Wheel_Target(wheel_rpms[2]);
        BL_WheelLegs_->Add_Wheel_Compensation(BL_WheelLegs_->Wheel_Compensation());
    }
    if (BR_WheelLegs_)
    {
        BR_WheelLegs_->Set_Wheel_Target(wheel_rpms[3]);
        BR_WheelLegs_->Add_Wheel_Compensation(BR_WheelLegs_->Wheel_Compensation());
    }

    // 4. Execute
    executeMotorCommands();
}

void Chassis::handleEnergySaving(const Protocol::PC_Msg &cmd)
{
    // --- Smooth Kp ramp when transitioning from COMFORT ---
    float target_kp = 20.0f;
    float target_kd = 1.5f;
    float kp_use, kd_use;
    if (mode_transition_timer_ > 0)
    {
        float alpha = 1.0f - (float)mode_transition_timer_ / (float)TRANSITION_FRAMES;
        // Use per-leg average of exit values for simplicity
        float avg_exit_kp = (exit_kp_[0] + exit_kp_[1] + exit_kp_[2] + exit_kp_[3]) * 0.25f;
        float avg_exit_kd = (exit_kd_[0] + exit_kd_[1] + exit_kd_[2] + exit_kd_[3]) * 0.25f;
        kp_use            = avg_exit_kp + alpha * (target_kp - avg_exit_kp);
        kd_use            = avg_exit_kd + alpha * (target_kd - avg_exit_kd);
        mode_transition_timer_--;
    }
    else
    {
        kp_use = target_kp;
        kd_use = target_kd;
    }

    Wheel_Leg_Params params;
    params.state     = Chassis_State::ENERGY_SAVING;
    params.Leg_POS   = 0.0f;
    params.Leg_Force = 0;
    params.Leg_RPM   = 0;
    params.Leg_Kp    = kp_use;
    params.Leg_Kd    = kd_use;

    // Wheel control
    float wheel_rpms[4], vx = 0.0f, wz = 0.0f;
    controller_.Map_Joystick_To_Velocity(cmd, vx, wz);
    inverseKinematics(vx, 0, wz, wheel_rpms);

    if (FL_WheelLegs_)
    {
        params.Wheel_RPM = wheel_rpms[0];
        FL_WheelLegs_->Set_Wheel_Leg(params);
    }
    if (FR_WheelLegs_)
    {
        params.Wheel_RPM = wheel_rpms[1];
        FR_WheelLegs_->Set_Wheel_Leg(params);
    }
    if (BL_WheelLegs_)
    {
        params.Wheel_RPM = wheel_rpms[2];
        BL_WheelLegs_->Set_Wheel_Leg(params);
    }
    if (BR_WheelLegs_)
    {
        params.Wheel_RPM = wheel_rpms[3];
        BR_WheelLegs_->Set_Wheel_Leg(params);
    }
}

// =====================================================================
// CLIMBING MODE: Per-leg kinematic step climbing + body leveling
// =====================================================================
void Chassis::handleClimbingMode(const Protocol::PC_Msg &cmd)
{
    readAndTransformIMU();
    // NOTE: No handleHeightButtons() here — CLIMBING manages its own height ramp

    // --- Smooth Kp/Kd ramp when transitioning from COMFORT ---
    float climb_target_kp = 12.0f;
    float climb_target_kd = 2.5f;
    float kp_use, kd_use;
    if (mode_transition_timer_ > 0)
    {
        float alpha       = 1.0f - (float)mode_transition_timer_ / (float)TRANSITION_FRAMES;
        float avg_exit_kp = (exit_kp_[0] + exit_kp_[1] + exit_kp_[2] + exit_kp_[3]) * 0.25f;
        float avg_exit_kd = (exit_kd_[0] + exit_kd_[1] + exit_kd_[2] + exit_kd_[3]) * 0.25f;
        kp_use            = avg_exit_kp + alpha * (climb_target_kp - avg_exit_kp);
        kd_use            = avg_exit_kd + alpha * (climb_target_kd - avg_exit_kd);
        mode_transition_timer_--;
    }
    else
    {
        kp_use = climb_target_kp;
        kd_use = climb_target_kd;
    }

    // Gather per-leg feedback
    LegClimbFeedback fb[4] = {};
    if (FL_WheelLegs_)
        fb[0] = {FL_WheelLegs_->Get_LegPosition(), FL_WheelLegs_->Get_WheelRPM(), FL_WheelLegs_->Get_WheelCurrentFeedback()};
    if (FR_WheelLegs_)
        fb[1] = {FR_WheelLegs_->Get_LegPosition(), FR_WheelLegs_->Get_WheelRPM(), FR_WheelLegs_->Get_WheelCurrentFeedback()};
    if (BL_WheelLegs_)
        fb[2] = {BL_WheelLegs_->Get_LegPosition(), BL_WheelLegs_->Get_WheelRPM(), BL_WheelLegs_->Get_WheelCurrentFeedback()};
    if (BR_WheelLegs_)
        fb[3] = {BR_WheelLegs_->Get_LegPosition(), BR_WheelLegs_->Get_WheelRPM(), BR_WheelLegs_->Get_WheelCurrentFeedback()};

    climbing_.config().step_height_m = dbg_ctrl.step_height_mm / 1000.0f;
    const float dt                   = 0.002f;

    // --- PREP phase: smoothly ramp height toward h_max_ ---
    // Rate: 0.3 m/s → full travel (0.11m) in ~0.37s
    bool any_prep = false;
    for (int i = 0; i < 4; i++)
        if (climbing_.getPhase(i) == LegClimbPhase::PREP)
            any_prep = true;
    if (any_prep && target_chassis_height_ < h_max_)
    {
        target_chassis_height_ += 0.3f * dt;  // 0.0006 m/frame
        if (target_chassis_height_ > h_max_)
            target_chassis_height_ = h_max_;
    }

    climbing_.update(fb, dt);

    // Ground Contact Warp Compensation (reuse leg currents from feedback)
    float leg_currents_c[4] = {FL_WheelLegs_ ? FL_WheelLegs_->Get_LegCurrentFeedback() : 0.0f,
                               FR_WheelLegs_ ? FR_WheelLegs_->Get_LegCurrentFeedback() : 0.0f,
                               BL_WheelLegs_ ? BL_WheelLegs_->Get_LegCurrentFeedback() : 0.0f,
                               BR_WheelLegs_ ? BR_WheelLegs_->Get_LegCurrentFeedback() : 0.0f};
    ground_contact_.update(leg_currents_c, dt);

    dbg_gc.warp_error = ground_contact_.getWarpError();
    dbg_gc.warp_dh    = ground_contact_.getWarpDH();
    dbg_gc.dh_fl      = ground_contact_.getDeltaH(0);
    dbg_gc.dh_fr      = ground_contact_.getDeltaH(1);
    dbg_gc.dh_bl      = ground_contact_.getDeltaH(2);
    dbg_gc.dh_br      = ground_contact_.getDeltaH(3);

    // Debug variables
    dbg_climb.dh_fl    = climbing_.getTargetHeight(0);
    dbg_climb.dh_fr    = climbing_.getTargetHeight(1);
    dbg_climb.dh_bl    = climbing_.getTargetHeight(2);
    dbg_climb.dh_br    = climbing_.getTargetHeight(3);
    dbg_climb.phase_fl = static_cast<uint8_t>(climbing_.getPhase(0));
    dbg_climb.phase_fr = static_cast<uint8_t>(climbing_.getPhase(1));
    dbg_climb.phase_bl = static_cast<uint8_t>(climbing_.getPhase(2));
    dbg_climb.phase_br = static_cast<uint8_t>(climbing_.getPhase(3));
    // Spike detection debug: |I_wheel - baseline| and baseline per leg
    float wc[4]        = {FL_WheelLegs_ ? FL_WheelLegs_->Get_WheelCurrentFeedback() : 0.0f,
                          FR_WheelLegs_ ? FR_WheelLegs_->Get_WheelCurrentFeedback() : 0.0f,
                          BL_WheelLegs_ ? BL_WheelLegs_->Get_WheelCurrentFeedback() : 0.0f,
                          BR_WheelLegs_ ? BR_WheelLegs_->Get_WheelCurrentFeedback() : 0.0f};
    dbg_climb.base_fl  = climbing_.getBaseline(0);
    dbg_climb.base_fr  = climbing_.getBaseline(1);
    dbg_climb.base_bl  = climbing_.getBaseline(2);
    dbg_climb.base_br  = climbing_.getBaseline(3);
    dbg_climb.spike_fl = fabsf(wc[0] - dbg_climb.base_fl);
    dbg_climb.spike_fr = fabsf(wc[1] - dbg_climb.base_fr);
    dbg_climb.spike_bl = fabsf(wc[2] - dbg_climb.base_bl);
    dbg_climb.spike_br = fabsf(wc[3] - dbg_climb.base_br);

    // Body Leveling PID
    float roll_h_adj  = roll_pid(0.0f, clampSym(chassis_roll_, max_roll_deg_));
    float pitch_h_adj = pitch_pid(0.0f, clampSym(chassis_pitch_, max_pitch_deg_));

    // Gyro Feedforward (disabled — set ff_gain_c > 0 to re-enable after tuning)
    static float filt_pitch_rate_c = 0.0f, filt_roll_rate_c = 0.0f;
    const float alpha_c = 0.05f, ff_gain_c = 0.0f;
    filt_pitch_rate_c = alpha_c * chassis_pitch_rate_ + (1.0f - alpha_c) * filt_pitch_rate_c;
    filt_roll_rate_c  = alpha_c * chassis_roll_rate_ + (1.0f - alpha_c) * filt_roll_rate_c;
    float v_pitch_ff  = (wb_m_ / 2.0f) * filt_pitch_rate_c * ff_gain_c;
    float v_roll_ff   = (wt_f_m_ / 2.0f) * filt_roll_rate_c * ff_gain_c;

    // Per-Leg Height Distribution (climbing-aware + warp compensation)
    // Near full extension (θ≈15°), dh/dθ is very small, so height corrections
    // get amplified into large angle changes. Scale down PID leveling to prevent oscillation.
    constexpr float lev_scale = 0.25f;

    // FL (Front-Left): +Pitch, -Roll
    if (FL_WheelLegs_)
    {
        float lev = lev_scale * (pitch_h_adj - roll_h_adj) + ground_contact_.getDeltaH(0);
        float gv  = v_pitch_ff - v_roll_ff;
        float h   = climbing_.isDirectControl(0) ? climbing_.getTargetHeight(0) + lev : target_chassis_height_ + lev;
        float v   = climbing_.isDirectControl(0) ? climbing_.getTargetVelocity(0) + gv : gv;
        FL_WheelLegs_->Set_Leg_Height(clampHeight(h), v, kp_use, kd_use, FL_WheelLegs_->Get_LegGravityTorque());
    }
    // FR (Front-Right): +Pitch, +Roll
    if (FR_WheelLegs_)
    {
        float lev = lev_scale * (pitch_h_adj + roll_h_adj) + ground_contact_.getDeltaH(1);
        float gv  = v_pitch_ff + v_roll_ff;
        float h   = climbing_.isDirectControl(1) ? climbing_.getTargetHeight(1) + lev : target_chassis_height_ + lev;
        float v   = climbing_.isDirectControl(1) ? climbing_.getTargetVelocity(1) + gv : gv;
        FR_WheelLegs_->Set_Leg_Height(clampHeight(h), v, kp_use, kd_use, FR_WheelLegs_->Get_LegGravityTorque());
    }
    // BL (Back-Left): -Pitch, -Roll
    if (BL_WheelLegs_)
    {
        float lev = lev_scale * (-pitch_h_adj - roll_h_adj) + ground_contact_.getDeltaH(2);
        float gv  = -v_pitch_ff - v_roll_ff;
        float h   = climbing_.isDirectControl(2) ? climbing_.getTargetHeight(2) + lev : target_chassis_height_ + lev;
        float v   = climbing_.isDirectControl(2) ? climbing_.getTargetVelocity(2) + gv : gv;
        BL_WheelLegs_->Set_Leg_Height(clampHeight(h), v, kp_use, kd_use, BL_WheelLegs_->Get_LegGravityTorque());
    }
    // BR (Back-Right): -Pitch, +Roll
    if (BR_WheelLegs_)
    {
        float lev = lev_scale * (-pitch_h_adj + roll_h_adj) + ground_contact_.getDeltaH(3);
        float gv  = -v_pitch_ff + v_roll_ff;
        float h   = climbing_.isDirectControl(3) ? climbing_.getTargetHeight(3) + lev : target_chassis_height_ + lev;
        float v   = climbing_.isDirectControl(3) ? climbing_.getTargetVelocity(3) + gv : gv;
        BR_WheelLegs_->Set_Leg_Height(clampHeight(h), v, kp_use, kd_use, BR_WheelLegs_->Get_LegGravityTorque());
    }

    dbg_leveling.h_fl = FL_WheelLegs_ ? (climbing_.isDirectControl(0) ? climbing_.getTargetHeight(0) : target_chassis_height_) : 0.0f;
    dbg_leveling.h_fr = FR_WheelLegs_ ? (climbing_.isDirectControl(1) ? climbing_.getTargetHeight(1) : target_chassis_height_) : 0.0f;
    dbg_leveling.h_bl = BL_WheelLegs_ ? (climbing_.isDirectControl(2) ? climbing_.getTargetHeight(2) : target_chassis_height_) : 0.0f;
    dbg_leveling.h_br = BR_WheelLegs_ ? (climbing_.isDirectControl(3) ? climbing_.getTargetHeight(3) : target_chassis_height_) : 0.0f;

    // Wheel velocity + execute
    float wheel_rpms[4], vx = 0.0f, wz = 0.0f;
    controller_.Map_Joystick_To_Velocity(cmd, vx, wz);
    inverseKinematics(vx, 0, wz, wheel_rpms);

    if (FL_WheelLegs_)
    {
        FL_WheelLegs_->Set_Wheel_Target(wheel_rpms[0]);
        FL_WheelLegs_->Add_Wheel_Compensation(FL_WheelLegs_->Wheel_Compensation());
    }
    if (FR_WheelLegs_)
    {
        FR_WheelLegs_->Set_Wheel_Target(wheel_rpms[1]);
        FR_WheelLegs_->Add_Wheel_Compensation(FR_WheelLegs_->Wheel_Compensation());
    }
    if (BL_WheelLegs_)
    {
        BL_WheelLegs_->Set_Wheel_Target(wheel_rpms[2]);
        BL_WheelLegs_->Add_Wheel_Compensation(BL_WheelLegs_->Wheel_Compensation());
    }
    if (BR_WheelLegs_)
    {
        BR_WheelLegs_->Set_Wheel_Target(wheel_rpms[3]);
        BR_WheelLegs_->Add_Wheel_Compensation(BR_WheelLegs_->Wheel_Compensation());
    }

    executeMotorCommands();
}

void Chassis::executeMotorCommands()
{
    if (FL_WheelLegs_)
    {
        FL_WheelLegs_->Execute_Wheel_Control();
        FL_WheelLegs_->Execute_Leg_Control();
    }
    if (FR_WheelLegs_)
    {
        FR_WheelLegs_->Execute_Wheel_Control();
        FR_WheelLegs_->Execute_Leg_Control();
    }
    if (BL_WheelLegs_)
    {
        BL_WheelLegs_->Execute_Wheel_Control();
        BL_WheelLegs_->Execute_Leg_Control();
    }
    if (BR_WheelLegs_)
    {
        BR_WheelLegs_->Execute_Wheel_Control();
        BR_WheelLegs_->Execute_Leg_Control();
    }
}

void Chassis::inverseKinematics(float vx, float vy, float wz, float *out_wheel_rpms)
{
    // =====================================================================
    // Skid-Steer Inverse Kinematics (asymmetric track: W_F ≠ W_B)
    // =====================================================================
    //
    // Model: Vehicle rotates around ICR at (0, R_turn) with angular vel ω.
    //   V_{x,i} = ω · (R_turn − y_i)
    //   V_center = ω · R_turn = Vx
    //
    // Each wheel:
    //   V_FL = Vx − ω · W_F/2     V_FR = Vx + ω · W_F/2
    //   V_BL = Vx − ω · W_B/2     V_BR = Vx + ω · W_B/2
    //
    // The controller outputs wz as differential RPM scaled to a reference.
    // We normalize by average track width so front/back get correct ratio.
    // =====================================================================

    (void)vy;

    // Track widths in mm (from Robot_Params.hpp)
    constexpr float W_F   = WHEEL_TRACK_FRONT;   // 531 mm
    constexpr float W_B   = WHEEL_TRACK_BACK;    // 395 mm
    constexpr float W_avg = (W_F + W_B) / 2.0f;  // Reference: average track

    // Differential RPM gain for each axle (wz is scaled to W_avg)
    constexpr float k_F = W_F / W_avg;  // > 1.0 (front wider → more differential)
    constexpr float k_B = W_B / W_avg;  // < 1.0 (back narrower → less differential)

    out_wheel_rpms[0] = vx - wz * k_F;  // FL
    out_wheel_rpms[1] = vx + wz * k_F;  // FR
    out_wheel_rpms[2] = vx - wz * k_B;  // BL
    out_wheel_rpms[3] = vx + wz * k_B;  // BR

    // Normalize: if any wheel exceeds MAX_WHEEL_RPM, scale all proportionally
    float max_rpm = 0.0f;
    for (int i = 0; i < 4; i++)
    {
        float abs_rpm = fabsf(out_wheel_rpms[i]);
        if (abs_rpm > max_rpm)
            max_rpm = abs_rpm;
    }
    if (max_rpm > MAX_WHEEL_RPM)
    {
        float scale = MAX_WHEEL_RPM / max_rpm;
        for (int i = 0; i < 4; i++)
            out_wheel_rpms[i] *= scale;
    }
}

float Chassis::CalculateHeightFromAngle(float angle_deg) { return R_m_ + r_m_ * cosf(deg2rad(angle_deg)); }

void Chassis::SetBendingDirection(int fl, int fr, int bl, int br)
{
    if (FL_WheelLegs_)
        FL_WheelLegs_->Set_Bending_Direction(fl);
    if (FR_WheelLegs_)
        FR_WheelLegs_->Set_Bending_Direction(fr);
    if (BL_WheelLegs_)
        BL_WheelLegs_->Set_Bending_Direction(bl);
    if (BR_WheelLegs_)
        BR_WheelLegs_->Set_Bending_Direction(br);
}

void Chassis::Get_Msg(Protocol::Reachable_Msg *msg)
{
    if (!msg)
        return;

    Wheel_Leg_Params info;
    if (FL_WheelLegs_)
    {
        info                               = FL_WheelLegs_->Get_Info();
        msg->GM6020_Current_Pos_x10_msg[0] = (int16_t)(info.Leg_POS * 10);
        msg->M3508_Current_RPM_msg[0]      = (int16_t)info.Wheel_RPM;
    }
    if (FR_WheelLegs_)
    {
        info                               = FR_WheelLegs_->Get_Info();
        msg->GM6020_Current_Pos_x10_msg[1] = (int16_t)(info.Leg_POS * 10);
        msg->M3508_Current_RPM_msg[1]      = (int16_t)info.Wheel_RPM;
    }
    if (BL_WheelLegs_)
    {
        info                               = BL_WheelLegs_->Get_Info();
        msg->GM6020_Current_Pos_x10_msg[2] = (int16_t)(info.Leg_POS * 10);
        msg->M3508_Current_RPM_msg[2]      = (int16_t)info.Wheel_RPM;
    }
    if (BR_WheelLegs_)
    {
        info                               = BR_WheelLegs_->Get_Info();
        msg->GM6020_Current_Pos_x10_msg[3] = (int16_t)(info.Leg_POS * 10);
        msg->M3508_Current_RPM_msg[3]      = (int16_t)info.Wheel_RPM;
    }
}

}  // namespace Applications
