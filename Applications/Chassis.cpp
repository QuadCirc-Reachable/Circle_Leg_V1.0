#include "Chassis.hpp"

#include "Matrixf.hpp"
#include "PID.hpp"
#include "Quaternion.hpp"

namespace Applications
{
using namespace Core::Drivers;

// PID Parameters for Active Suspension (Comfort Mode)
// Adjust these based on actual tuning
// Output limit is now in METERS.
// Max travel is 2*r = 130mm = 0.13m. Set limit to 0.15m to allow full range.
static Core::Control::PID::Param roll_pid_param(0.005f, 0.0f, 0.0f, 1000.0f, 0.15f);
static Core::Control::PID::Param pitch_pid_param(0.005f, 0.0f, 0.0f, 1000.0f, 0.15f);

static Core::Control::PID roll_pid(roll_pid_param);
static Core::Control::PID pitch_pid(pitch_pid_param);

Chassis::Chassis(Wheel_Leg *fl, Wheel_Leg *fr, Wheel_Leg *bl, Wheel_Leg *br)
    : FL_WheelLegs_(fl), FR_WheelLegs_(fr), BL_WheelLegs_(bl), BR_WheelLegs_(br)
{
    // Calculate Nominal Height based on INITIAL_LEG_ANGLE
    float R         = WHEEL_RADIUS_R / 1000.0f;
    float r         = ECCENTRIC_OFFSET_r / 1000.0f;
    float theta_rad = deg2rad(INITIAL_LEG_ANGLE);
    // Note: cos(135) is negative.
    // If 0 deg is fully extended (H = R+r), and 180 deg is fully retracted (H = R-r).
    // Then H = R + r * cos(theta).
    // At 135 deg, cos(135) = -0.707.
    // H = R - 0.707 * r.
    // This is correct for "Retracted" stance.
    target_chassis_height_ = R + r * cosf(theta_rad);
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
    current_state_ = new_state;
    // Reset PIDs when entering Comfort mode
    if (new_state == Chassis_State::COMFORT)
    {
        roll_pid.reset();
        pitch_pid.reset();
    }
}

void Chassis::Update(const Protocol::PC_Msg &cmd)
{
    // Check for Debug State Command from Ozone
    if (debug_state_cmd >= 0)
    {
        Set_Mode(static_cast<Chassis_State>(debug_state_cmd));
        debug_state_cmd = -1;  // Reset command
    }

    // --- State Switching Logic ---
    uint8_t current_buttons = cmd.button_status;
    uint8_t changed_buttons = current_buttons ^ last_button_status_;
    uint8_t rising_edges    = changed_buttons & current_buttons;

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
        // Range [1, 5]: IDLE(1) ... FREE_CONTROL(5)
        int state_idx = static_cast<int>(current_state_);

        // If current state is out of range (e.g. CALIBRATION=0 or ERROR), default to IDLE(1) for next switch
        if (state_idx < 1 || state_idx > 5)
            state_idx = 1;

        if (rising_edges & BTN_ML)
        {
            // ML: Up Switch (Previous)
            state_idx--;
            if (state_idx < 1)
                state_idx = 5;
            Set_Mode(static_cast<Chassis_State>(state_idx));
        }
        else if (rising_edges & BTN_MR)
        {
            // MR: Down Switch (Next)
            state_idx++;
            if (state_idx > 5)
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
        // Stop all motors
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
    case Chassis_State::COMFORT:
        handleComfortMode(cmd);
        break;
    case Chassis_State::FREE_CONTROL:
        handleFreeControl(cmd);
        break;
    case Chassis_State::ENERGY_SAVING:
        handleEnergySaving(cmd);
        break;
    case Chassis_State::CLIMBING:
        handleClimbingMode(cmd);
        break;
    default:
        break;
    }
}

void Chassis::handleCalibrationMode()
{
#if USE_HT_LEG_MOTOR
    if (FL_WheelLegs_)
        FL_WheelLegs_->SetZero();
    if (FR_WheelLegs_)
        FR_WheelLegs_->SetZero();
    if (BL_WheelLegs_)
        BL_WheelLegs_->SetZero();
    if (BR_WheelLegs_)
        BR_WheelLegs_->SetZero();
#endif
    // Automatically switch to IDLE after calibration
    Set_Mode(Chassis_State::IDLE);
}

void Chassis::handleComfortMode(const Protocol::PC_Msg &cmd)
{
    // 1. Get IMU Data
    float euler[3];  // Z, Y, X (Yaw, Pitch, Roll)
    float accel[3];  // X, Y, Z
    Core::Drivers::IMU::getEulerZYX(euler);
    Core::Drivers::IMU::getLinearAccel(accel);

    float current_pitch = rad2deg(euler[1]);
    float current_roll  = rad2deg(euler[2]);
    float accel_z       = accel[2];  // Vertical acceleration

    // 2. Transform IMU Data to Chassis Geometric Center
    // Use Fused Euler Angles (Stable) instead of Raw Accel (Noisy)

    // Calculate Chassis Angles correcting for Mounting Offset
    // IMU is mounted with Roll = 180 degrees.
    // Chassis Roll = IMU Roll - 180
    // Chassis Pitch = IMU Pitch (X-axis is aligned, Y-axis is inverted by Roll 180? No, X is aligned.)
    // If Roll=180 (Rotation around X), Y becomes -Y, Z becomes -Z.
    // Pitch is rotation around Y.
    // If Chassis pitches up (+Y rotation), IMU sees rotation around its -Y axis?
    // Let's assume Pitch sign is preserved or inverted.
    // Based on user test: Pitch Up -> FL/FR Retract (-Pitch).
    // If FL/FR decreased, then -Pitch was negative -> Pitch was Positive.
    // So IMU Pitch matches Chassis Pitch sign.

    float chassis_roll  = current_roll - IMU_MOUNT_ROLL_DEG;
    float chassis_pitch = current_pitch;  // - IMU_MOUNT_PITCH_DEG (usually 0)

    // Normalize Roll to [-180, 180]
    if (chassis_roll > 180.0f)
        chassis_roll -= 360.0f;
    if (chassis_roll < -180.0f)
        chassis_roll += 360.0f;

    // Update Member Variables for Debugging
    imu_pitch_pv = chassis_pitch;
    imu_roll_pv  = chassis_roll;

    // Button Control for Target Height
    float R_val = WHEEL_RADIUS_R / 1000.0f;
    float r_val = ECCENTRIC_OFFSET_r / 1000.0f;

    auto angle_to_height = [&](float angle_deg) { return R_val + r_val * cosf(deg2rad(angle_deg)); };

    auto set_bending_dir = [&](int dir)
    {
        if (FL_WheelLegs_)
            FL_WheelLegs_->Set_Bending_Direction(dir);
        if (FR_WheelLegs_)
            FR_WheelLegs_->Set_Bending_Direction(dir);
        if (BL_WheelLegs_)
            BL_WheelLegs_->Set_Bending_Direction(dir);
        if (BR_WheelLegs_)
            BR_WheelLegs_->Set_Bending_Direction(dir);
    };

    if (cmd.button_status & BTN_X)
    {
        target_chassis_height_ = angle_to_height(90.0f);
        set_bending_dir(1);
    }
    else if (cmd.button_status & BTN_Y)
    {
        target_chassis_height_ = angle_to_height(145.0f);
        set_bending_dir(1);
    }
    else if (cmd.button_status & BTN_A)
    {
        target_chassis_height_ = angle_to_height(45.0f);
        set_bending_dir(1);
    }
    else if (cmd.button_status & BTN_B)
    {
        target_chassis_height_ = angle_to_height(165.0f);
        set_bending_dir(1);
    }
    else if (cmd.button_status & BTN_RB)
    {
        target_chassis_height_ = angle_to_height(135.0f);
        set_bending_dir(-1);
    }
    else if (cmd.button_status & BTN_LB)
    {
        target_chassis_height_ = angle_to_height(90.0f);
        set_bending_dir(-1);
    }

    // Transform Acceleration (Linear)
    // Use Rotation Matrix T for acceleration transformation
    float r_rad = deg2rad(IMU_MOUNT_ROLL_DEG);
    float p_rad = deg2rad(IMU_MOUNT_PITCH_DEG);
    float y_rad = deg2rad(IMU_MOUNT_YAW_DEG);

    float cr = cosf(r_rad), sr = sinf(r_rad);
    float cp = cosf(p_rad), sp = sinf(p_rad);
    float cy = cosf(y_rad), sy = sinf(y_rad);

    float T_data[9] = {
        cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, -sp, cp * sr, cp * cr};
    Matrixf<3, 3> T(T_data);

    float accel_vec_data[3] = {accel[0], accel[1], accel[2]};
    Matrixf<3, 1> accel_vec(accel_vec_data);
    Matrixf<3, 1> chassis_accel = T.trans() * accel_vec;
    float chassis_accel_z       = chassis_accel[2][0];

    // 3. Calculate PID for Active Suspension (using Chassis Frame angles)

    // Calculate Max Adjustable Pitch and Roll based on Geometry
    // tan(max_angle) = (2 * r) / L
    // r = ECCENTRIC_OFFSET_r
    // L_pitch = WHEEL_BASE
    // L_roll = WHEEL_TRACK_FRONT (Wider track limits the angle more for the same vertical travel? No.)
    // h = (w/2) * tan(theta). Max h = r.
    // tan(theta_max) = r / (w/2) = 2*r / w.
    // Larger w -> Smaller theta_max. So use the LARGER track width (Front) to be safe.

    float r_m  = ECCENTRIC_OFFSET_r / 1000.0f;
    float wb_m = WHEEL_BASE / 1000.0f;
    float wt_m = WHEEL_TRACK_FRONT / 1000.0f;

    float max_pitch_rad = atan2f(2.0f * r_m, wb_m);
    float max_roll_rad  = atan2f(2.0f * r_m, wt_m);

    float max_pitch_deg = rad2deg(max_pitch_rad);
    float max_roll_deg  = rad2deg(max_roll_rad);

    // Clamp the input angles to the PID
    // If the chassis is tilted beyond what we can compensate, we only compensate up to the limit.
    // This prevents the PID from trying to reach an impossible target (0 deg) and saturating the legs.

    auto clamp_angle = [](float val, float max_val)
    {
        if (val > max_val)
            return max_val;
        if (val < -max_val)
            return -max_val;
        return val;
    };

    float pid_input_roll  = clamp_angle(chassis_roll, max_roll_deg);
    float pid_input_pitch = clamp_angle(chassis_pitch, max_pitch_deg);

    // Target is 0 degrees (flat)
    // PID Output is now Height Adjustment (Meters)
    float roll_h_adj  = roll_pid(0.0f, pid_input_roll);
    float pitch_h_adj = pitch_pid(0.0f, pid_input_pitch);

    // Velocity Feedforward (Gyro)
    // V_z = Distance * Omega
    // Pitch rotates around Y axis. Front/Back distance is WHEEL_BASE / 2.
    // Roll rotates around X axis. Left/Right distance is WHEEL_TRACK / 2.

    // Get Gyro Data (rad/s)
    float gyro[3];
    Core::Drivers::IMU::getCalibratedGyroTransformed(gyro);
    // Transform Gyro to Chassis Frame (Assuming simple mounting for now, or use T)
    // IMU Roll=180 -> X=X, Y=-Y, Z=-Z.
    // Pitch Rate (Y-axis) -> -gyro[1]
    // Roll Rate (X-axis) -> gyro[0]

    float chassis_pitch_rate = -gyro[1];
    float chassis_roll_rate  = gyro[0];

    // Filter Gyro Data (Low Pass Filter) to reduce jitter
    static float filtered_pitch_rate = 0.0f;
    static float filtered_roll_rate  = 0.0f;
    const float alpha                = 0.1f;  // Filter coefficient (0.0 - 1.0). Smaller = Smoother.

    filtered_pitch_rate = alpha * chassis_pitch_rate + (1.0f - alpha) * filtered_pitch_rate;
    filtered_roll_rate  = alpha * chassis_roll_rate + (1.0f - alpha) * filtered_roll_rate;

    // Use Filtered Rates for Feedforward with a Gain Factor
    const float ff_gain = 0.6f;  // Reduce feedforward aggressiveness

    float v_pitch_ff = (WHEEL_BASE / 2.0f / 1000.0f) * filtered_pitch_rate * ff_gain;        // m/s
    float v_roll_ff  = (WHEEL_TRACK_FRONT / 2.0f / 1000.0f) * filtered_roll_rate * ff_gain;  // m/s

    // 4. Distribute Height Adjustments to Legs
    // Feedforward: target_chassis_height_
    // PID Output is Negative when Error is Negative (Current > Target).
    // Example: Pitch Up (+). Error (-). Output (-).
    // We want Front to Retract (-). So we ADD the Negative Output. (+ Output)
    // We want Back to Extend (+). So we SUBTRACT the Negative Output. (- Output)

    // FL (Front Left): +Pitch, -Roll
    float h_fl = target_chassis_height_ + pitch_h_adj - roll_h_adj;
    float v_fl = v_pitch_ff - v_roll_ff;

    // FR (Front Right): +Pitch, +Roll
    float h_fr = target_chassis_height_ + pitch_h_adj + roll_h_adj;
    float v_fr = v_pitch_ff + v_roll_ff;

    // BL (Back Left): -Pitch, -Roll
    float h_bl = target_chassis_height_ - pitch_h_adj - roll_h_adj;
    float v_bl = -v_pitch_ff - v_roll_ff;

    // BR (Back Right): -Pitch, +Roll
    float h_br = target_chassis_height_ - pitch_h_adj + roll_h_adj;
    float v_br = -v_pitch_ff + v_roll_ff;

    // Limit Height to Physical Bounds [R-r, R+r]
    // Reduce buffer to allow reaching closer to 180 degrees (Singularity)
    float R = WHEEL_RADIUS_R / 1000.0f;
    float r = ECCENTRIC_OFFSET_r / 1000.0f;

    // Avoid Singularity at 0 and 180 degrees
    // Restrict range to [10, 170] degrees
    // cos(10) = 0.9848
    float limit_angle_deg = 10.0f;
    float limit_cos       = cosf(deg2rad(limit_angle_deg));

    float h_max = R + r * limit_cos;
    float h_min = R - r * limit_cos;

    auto clamp_h = [&](float h)
    {
        if (h < h_min)
            return h_min;
        if (h > h_max)
            return h_max;
        return h;
    };

    h_fl = clamp_h(h_fl);
    h_fr = clamp_h(h_fr);
    h_bl = clamp_h(h_bl);
    h_br = clamp_h(h_br);

    // Update Member Variables for Debugging
    h_fl_pv = h_fl;
    h_fr_pv = h_fr;
    h_bl_pv = h_bl;
    h_br_pv = h_br;

    // 5. Send Height Commands to Wheel Legs
    if (FL_WheelLegs_)
        FL_WheelLegs_->Set_Leg_Height(h_fl, v_fl);
    if (FR_WheelLegs_)
        FR_WheelLegs_->Set_Leg_Height(h_fr, v_fr);
    if (BL_WheelLegs_)
        BL_WheelLegs_->Set_Leg_Height(h_bl, v_bl);
    if (BR_WheelLegs_)
        BR_WheelLegs_->Set_Leg_Height(h_br, v_br);

    // 6. Wheel Control (Velocity)
    float wheel_rpms[4];

    // Use Controller to decode joystick commands
    float vx = 0.0f;
    float wz = 0.0f;
    controller_.Map_Joystick_To_Velocity(cmd, vx, wz);

    inverseKinematics(vx, 0, wz, wheel_rpms);

    // 7. Send Wheel Commands (Leg Height is already sent in Step 5)
    if (FL_WheelLegs_)
        FL_WheelLegs_->Set_Wheel_Target(wheel_rpms[0]);
    if (FR_WheelLegs_)
        FR_WheelLegs_->Set_Wheel_Target(wheel_rpms[1]);
    if (BL_WheelLegs_)
        BL_WheelLegs_->Set_Wheel_Target(wheel_rpms[2]);
    if (BR_WheelLegs_)
        BR_WheelLegs_->Set_Wheel_Target(wheel_rpms[3]);

    if (FL_WheelLegs_)
        FL_WheelLegs_->Execute_Wheel_Control();
    if (FR_WheelLegs_)
        FR_WheelLegs_->Execute_Wheel_Control();
    if (BL_WheelLegs_)
        BL_WheelLegs_->Execute_Wheel_Control();
    if (BR_WheelLegs_)
        BR_WheelLegs_->Execute_Wheel_Control();

    if (FL_WheelLegs_)
        FL_WheelLegs_->Execute_Leg_Control();
    if (FR_WheelLegs_)
        FR_WheelLegs_->Execute_Leg_Control();
    if (BL_WheelLegs_)
        BL_WheelLegs_->Execute_Leg_Control();
    if (BR_WheelLegs_)
        BR_WheelLegs_->Execute_Leg_Control();
}

void Chassis::handleFreeControl(const Protocol::PC_Msg &cmd)
{
    // Direct control of leg height via Triggers
    float leg_pos = INITIAL_LEG_ANGLE;
    // Example: LT lowers, RT raises
    float trigger_val = (float)cmd.Right_trigger_x1000_msg / 1000.0f - (float)cmd.Left_trigger_x1000_msg / 1000.0f;
    leg_pos += trigger_val * 45.0f;  // +/- 45 degrees range

    // Wheel control same as Comfort
    float wheel_rpms[4];
    float vx = (float)cmd.left_joystick.r_x1000_msg / 1000.0f * MAX_WHEEL_RPM;
    if (cmd.left_joystick.angle_x10_msg > 900 && cmd.left_joystick.angle_x10_msg < 2700)
        vx = -vx;

    float wz = (float)cmd.right_joystick.r_x1000_msg / 1000.0f * MAX_WHEEL_RPM * 0.5f;
    if (cmd.right_joystick.angle_x10_msg > 1800)
        wz = -wz;

    inverseKinematics(vx, 0, wz, wheel_rpms);

    Wheel_Leg_Params params;
    params.state     = Chassis_State::FREE_CONTROL;
    params.Leg_POS   = leg_pos;
    params.Leg_Force = 0;
    params.Leg_RPM   = 0;
    // Use default stiff parameters for position control
    params.Leg_Kp = 50.0f;
    params.Leg_Kd = 1.0f;

    for (int i = 0; i < 4; i++)
    {
        params.Wheel_RPM = wheel_rpms[i];
        if (i == 0 && FL_WheelLegs_)
            FL_WheelLegs_->Set_Wheel_Leg(params);
        if (i == 1 && FR_WheelLegs_)
            FR_WheelLegs_->Set_Wheel_Leg(params);
        if (i == 2 && BL_WheelLegs_)
            BL_WheelLegs_->Set_Wheel_Leg(params);
        if (i == 3 && BR_WheelLegs_)
            BR_WheelLegs_->Set_Wheel_Leg(params);
    }
}

void Chassis::handleEnergySaving(const Protocol::PC_Msg &cmd)
{
    // Wheel control same as Comfort
    float wheel_rpms[4];
    float vx = (float)cmd.left_joystick.r_x1000_msg / 1000.0f * MAX_WHEEL_RPM;
    if (cmd.left_joystick.angle_x10_msg > 900 && cmd.left_joystick.angle_x10_msg < 2700)
        vx = -vx;

    float wz = (float)cmd.right_joystick.r_x1000_msg / 1000.0f * MAX_WHEEL_RPM * 0.5f;
    if (cmd.right_joystick.angle_x10_msg > 1800)
        wz = -wz;

    inverseKinematics(vx, 0, wz, wheel_rpms);

    Wheel_Leg_Params params;
    params.state     = Chassis_State::ENERGY_SAVING;  // Use Energy Saving State
    params.Leg_POS   = 0;                             // Lower legs to minimum height
    params.Leg_Force = 0;                             // No force
    params.Leg_RPM   = 0;
    params.Leg_Kp    = 50.0f;
    params.Leg_Kd    = 1.0f;

    for (int i = 0; i < 4; i++)
    {
        params.Wheel_RPM = wheel_rpms[i];
        if (i == 0 && FL_WheelLegs_)
            FL_WheelLegs_->Set_Wheel_Leg(params);
        if (i == 1 && FR_WheelLegs_)
            FR_WheelLegs_->Set_Wheel_Leg(params);
        if (i == 2 && BL_WheelLegs_)
            BL_WheelLegs_->Set_Wheel_Leg(params);
        if (i == 3 && BR_WheelLegs_)
            BR_WheelLegs_->Set_Wheel_Leg(params);
    }
}

void Chassis::handleClimbingMode(const Protocol::PC_Msg &cmd)
{
    // Placeholder for climbing logic
    // Maybe raise front legs, push with back legs?
    handleComfortMode(cmd);  // Fallback to comfort for now
}

void Chassis::inverseKinematics(float vx, float vy, float wz, float *out_wheel_rpms)
{
    // Skid Steer / Differential Drive Kinematics
    // FL (0) = Vx - Wz
    // FR (1) = Vx + Wz
    // BL (2) = Vx - Wz
    // BR (3) = Vx + Wz

    // Note: Check motor directions. Usually Left and Right are mirrored.
    // Assuming positive RPM moves robot forward for all wheels if configured correctly.

    out_wheel_rpms[0] = vx - wz;  // FL
    out_wheel_rpms[1] = vx + wz;  // FR
    out_wheel_rpms[2] = vx - wz;  // BL
    out_wheel_rpms[3] = vx + wz;  // BR
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
