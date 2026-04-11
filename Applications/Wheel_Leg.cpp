#include "Wheel_Leg.hpp"

#include <cmath>

namespace Applications
{
#if USE_6020_LEG_MOTOR
Wheel_Leg::Wheel_Leg(Motors::M3508 *wheel_motor_,
                     PID *Wheel_Vel_PIDs_,
                     Motors::GM6020 *leg_motor_,
                     PID *Leg_Vel_PIDs_,
                     PID *Leg_Pos_PIDs_,
                     float leg_offset_,
                     int bending_direction,
                     float wheel_coupling_sign)
    : wheel_motor(wheel_motor_),
      Wheel_Vel_PIDs(Wheel_Vel_PIDs_),
      leg_motor(leg_motor_),
      Leg_Vel_PIDs(Leg_Vel_PIDs_),
      Leg_Pos_PIDs(Leg_Pos_PIDs_),
      leg_offset(leg_offset_),
      bending_direction_(bending_direction),
      wheel_coupling_sign_(wheel_coupling_sign)
{
    info.Wheel_RPM = 0.0f;
    info.Leg_POS   = 0.0f;
    info.Leg_Force = 0.0f;
    info.Leg_RPM   = 0.0f;
}
#elif USE_HT_LEG_MOTOR
Wheel_Leg::Wheel_Leg(Motors::M3508 *wheel_motor_,
                     PID *Wheel_Vel_PIDs_,
                     Motors::HT8115 *leg_motor_,
                     MIT_Params mit_pid_,
                     float leg_offset_,
                     int bending_direction,
                     float wheel_coupling_sign)
    : wheel_motor(wheel_motor_),
      Wheel_Vel_PIDs(Wheel_Vel_PIDs_),
      leg_motor(leg_motor_),
      leg_offset(leg_offset_),
      bending_direction_(bending_direction),
      wheel_coupling_sign_(wheel_coupling_sign)
{
    mit_set.Position    = 0.0f;
    mit_set.Velocity    = 0.0f;
    mit_set.Pos_KP      = mit_pid_.Pos_KP;
    mit_set.Vel_KD      = mit_pid_.Vel_KD;
    mit_set.FFW_Current = 0.0f;

    default_mit_set = mit_set;

    info.Wheel_RPM = 0.0f;
    info.Leg_POS   = 0.0f;
    info.Leg_Force = 0.0f;
    info.Leg_RPM   = 0.0f;
}
#endif

void Wheel_Leg::Init()
{
    if (wheel_motor)
        wheel_motor->enable();

#if USE_6020_LEG_MOTOR
    if (leg_motor)
        leg_motor->enable();
#elif USE_HT_LEG_MOTOR
    if (leg_motor)
    {
        leg_motor->enable();

        // Send ENTER_MOTOR multiple times to ensure reception
        // HT motors may need several attempts after power-on
        for (int i = 0; i < 20; i++)
        {
            leg_motor->sendCommand(Motors::HT8115::SpecialCommands::ENTER_MOTOR);
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
#endif

    // Initialize Slew Rate Limiter to current position
    if (leg_motor)
    {
        // Wait a bit for feedback to update
        vTaskDelay(pdMS_TO_TICKS(10));
        prev_leg_pos_cmd = Get_LegPosition();
    }
}

float Wheel_Leg::Wheel_Compensation()
{
    float leg_rpm = 0.0f;
    float leg_pos = 0.0f;

    /**
     * @brief Decoupling Compensation Formula
     *
     *                               r_offset
     * n_Wheel = n_Leg * ( 1.0 +  -------------- * cos(theta_Leg) )
     *                              R_Wheel
     */
#if USE_6020_LEG_MOTOR
    if (leg_motor)
    {
        leg_rpm = leg_motor->getRPMFeedback();
        leg_pos = rad2deg(normalizeAngle(leg_motor->getPositionFeedback() - leg_offset));
    }
#elif USE_HT_LEG_MOTOR
    if (leg_motor)
    {
        leg_rpm = leg_motor->getRPMFeedback();
        leg_pos = rad2deg(normalizeAngle(leg_motor->getPositionFeedback() - leg_offset));
    }
#endif

    float compensation_rpm = leg_rpm * (1.0f + (ECCENTRIC_OFFSET_r / WHEEL_RADIUS_R) * cosf(deg2rad(leg_pos)));
    return compensation_rpm * wheel_coupling_sign_;
}

float Wheel_Leg::VMC_Calculation(float F_z)
{
    float leg_pos   = Get_LegPosition();  // Degrees
    float theta_rad = deg2rad(leg_pos);

    // Jacobian: Torque = F_z * r * cos(theta)
    // ECCENTRIC_OFFSET_r is in mm, convert to m
    float torque = F_z * (ECCENTRIC_OFFSET_r / 1000.0f) * cosf(theta_rad);

    return torque;
}

float Wheel_Leg::Get_WheelRPM()
{
    if (wheel_motor)
        return wheel_motor->getRPMFeedback();
    return 0.0f;
}

float Wheel_Leg::Get_LegPosition()
{
#if USE_6020_LEG_MOTOR
    if (leg_motor)
        return rad2deg(normalizeAngle(leg_motor->getPositionFeedback() - leg_offset));
#elif USE_HT_LEG_MOTOR
    if (leg_motor)
        return rad2deg(normalizeAngle(leg_motor->getPositionFeedback() - leg_offset));
#endif
    return 0.0f;
}

float Wheel_Leg::Get_WheelCurrentFeedback()
{
    if (wheel_motor)
        return wheel_motor->getCurrentFeedback();
    return 0.0f;
}

float Wheel_Leg::Get_WheelTemperature()
{
    if (wheel_motor)
        return wheel_motor->getTemperatureFeedback();
    return 0.0f;
}

float Wheel_Leg::Get_WheelOutput()
{
    if (wheel_motor)
        return wheel_motor->getOutput();
    return 0.0f;
}

float Wheel_Leg::Get_LegCurrentFeedback()
{
    if (leg_motor)
        return leg_motor->getCurrentFeedback();
    return 0.0f;
}

float Wheel_Leg::Get_LegTorqueFeedback()
{
#if USE_6020_LEG_MOTOR
    if (leg_motor)
        return leg_motor->getTorqueFeedback();
#elif USE_HT_LEG_MOTOR
    if (leg_motor)
        return leg_motor->getTorqueFeedback();
#endif
    return 0.0f;
}

float Wheel_Leg::Get_LegGravityTorque()
{
    // Calculate torque needed to hold the leg against gravity
    // Torque = - (m * g * r * cos(theta))
    // Assuming Positive Torque pushes DOWN (extends).
    // Gravity pushes DOWN.
    // So to hold it UP, we need Negative Torque.

    float pos_deg   = Get_LegPosition();
    float theta_rad = deg2rad(pos_deg);
    float r_meter   = ECCENTRIC_OFFSET_r / 1000.0f;

    // Gravity Torque (Effect of gravity) = m * g * r * cos(theta)
    // This is Positive (pushes down).
    // Compensation Torque (Motor Output) = - Gravity Torque

    return -(LEG_MASS_kg * GRAVITY_g * r_meter * cosf(theta_rad));
}

float Wheel_Leg::Get_LegForce()
{
    if (leg_motor)
        return leg_motor->getTorqueFeedback();
    return 0.0f;
}

float Wheel_Leg::Get_LegVelocity()
{
    if (leg_motor)
        return leg_motor->getRPMFeedback() * 2.0f * M_PI / 60.0f;
    return 0.0f;
}

Wheel_Leg_Params Wheel_Leg::Get_Info()
{
    info.Wheel_RPM = Get_WheelRPM();
    info.Leg_POS   = Get_LegPosition();
    info.Leg_Force = Get_LegForce();
    info.Leg_RPM   = Get_LegVelocity();
    return info;
}

// === Wheel Pipeline ===

void Wheel_Leg::Set_Wheel_Target(float rpm_cmd)
{
    target_wheel_rpm       = rpm_cmd;
    wheel_compensation_rpm = 0.0f;  // Reset compensation for new cycle
}

void Wheel_Leg::Add_Wheel_Compensation(float comp_rpm) { wheel_compensation_rpm += comp_rpm; }

void Wheel_Leg::Execute_Wheel_Control()
{
    final_wheel_rpm = target_wheel_rpm + wheel_compensation_rpm;

    if (wheel_motor && Wheel_Vel_PIDs)
    {
        // Deadzone: when target RPM is near zero, stop outputting to prevent
        // stall current from coupling compensation / PID integral windup.
        constexpr float WHEEL_RPM_DEADZONE = 15.0f;
        if (fabsf(final_wheel_rpm) < WHEEL_RPM_DEADZONE && fabsf(wheel_motor->getRPMFeedback()) < WHEEL_RPM_DEADZONE)
        {
            wheel_motor->setOutput(0.0f);
            Wheel_Vel_PIDs->reset();
        }
        else
        {
            wheel_motor->setOutput(Wheel_Vel_PIDs->operator()(final_wheel_rpm, wheel_motor->getRPMFeedback()));
        }
    }
}

// === Leg Pipeline ===

#if USE_6020_LEG_MOTOR
void Wheel_Leg::Set_Leg_Target(float pos_cmd, float vel_cmd, float for_cmd)
{
    target_leg_pos   = pos_cmd;
    target_leg_vel   = vel_cmd;
    target_leg_force = for_cmd;

    leg_compensation_pos   = 0.0f;
    leg_compensation_vel   = 0.0f;
    leg_compensation_force = 0.0f;
}
#elif USE_HT_LEG_MOTOR
void Wheel_Leg::Set_Leg_Target(float pos_cmd, float vel_cmd, float for_cmd, float kp, float kd)
{
    target_leg_pos   = pos_cmd;
    target_leg_vel   = vel_cmd;
    target_leg_force = for_cmd;

    mit_set.Pos_KP = kp;
    mit_set.Vel_KD = kd;

    leg_compensation_pos   = 0.0f;
    leg_compensation_vel   = 0.0f;
    leg_compensation_force = 0.0f;
}
#endif

void Wheel_Leg::Set_Leg_Height(float h_meters, float v_meters_s)
{
    // Convert Height (m) to Angle (deg)
    // Model: H = R + r * cos(theta)
    // H: Total height from ground to motor center
    // R: Wheel Radius
    // r: Eccentric Offset
    // theta: Leg Angle (0 = Fully Extended/Highest, 180 = Fully Retracted/Lowest)

    float R = WHEEL_RADIUS_R / 1000.0f;      // m
    float r = ECCENTRIC_OFFSET_r / 1000.0f;  // m

    // Clamp height to reachable physical limits
    // Max Height = R + r
    // Min Height = R - r
    // Apply Safety Margin to avoid Singularity (0 and 180 degrees)
    // Limit to [10, 170] degrees
    float limit_angle_deg = 10.0f;
    float limit_cos       = cosf(deg2rad(limit_angle_deg));

    float max_h = R + r * limit_cos;
    float min_h = R - r * limit_cos;

    bool is_clamped_max = false;
    bool is_clamped_min = false;

    if (h_meters > max_h)
    {
        h_meters       = max_h;
        is_clamped_max = true;
    }
    if (h_meters < min_h)
    {
        h_meters       = min_h;
        is_clamped_min = true;
    }

    // Safety: Zero out velocity feedforward if pushing against the limits
    if (is_clamped_max && v_meters_s > 0.0f)
    {
        v_meters_s = 0.0f;  // Trying to extend further than max
    }
    if (is_clamped_min && v_meters_s < 0.0f)
    {
        v_meters_s = 0.0f;  // Trying to retract further than min
    }

    // Solve for theta: cos(theta) = (H - R) / r
    float cos_theta = (h_meters - R) / r;

    // Safety clamp for acos domain [-1, 1]
    if (cos_theta > 1.0f)
        cos_theta = 1.0f;
    if (cos_theta < -1.0f)
        cos_theta = -1.0f;

    float theta_rad = acosf(cos_theta);
    float theta_deg = rad2deg(theta_rad);

    // Calculate Angular Velocity Feedforward
    // H = R + r * cos(theta)
    // dH/dt = -r * sin(theta) * dtheta/dt
    // dtheta/dt = -dH/dt / (r * sin(theta))

    float sin_theta = sinf(theta_rad);

    // Add a small value to denominator to avoid division by zero and dampen the singularity
    // This replaces the hard threshold cut-off which caused shaking
    float damping_val      = 0.1f;
    float target_vel_rad_s = -v_meters_s / (r * (sin_theta + damping_val));

    // acos returns [0, 180].
    // 0 deg -> cos=1 -> H=R+r (Max)
    // 180 deg -> cos=-1 -> H=R-r (Min)
    // This matches the standard definition where 0 is down (extended).

    // Apply Bending Direction Preference
    // If bending_direction_ is 1, we use +theta (0 to 180)
    // If bending_direction_ is -1, we use -theta (0 to -180)
    float target_angle = theta_deg * (float)bending_direction_;
    target_vel_rad_s *= (float)bending_direction_;

    float gravity_comp = Get_LegGravityTorque();

#if USE_6020_LEG_MOTOR
    Set_Leg_Target(target_angle, target_vel_rad_s, gravity_comp);
#elif USE_HT_LEG_MOTOR
    Set_Leg_Target(target_angle, target_vel_rad_s, gravity_comp, default_mit_set.Pos_KP, default_mit_set.Vel_KD);
#endif
}

#if USE_HT_LEG_MOTOR
void Wheel_Leg::Set_Leg_Height(float h_meters, float v_meters_s, float kp, float kd, float ffw_torque)
{
    // Same height→angle conversion as the default overload,
    // but uses caller-provided Kp, Kd, and FFW instead of defaults.

    float R = WHEEL_RADIUS_R / 1000.0f;
    float r = ECCENTRIC_OFFSET_r / 1000.0f;

    float limit_angle_deg = 10.0f;
    float limit_cos       = cosf(deg2rad(limit_angle_deg));
    float max_h           = R + r * limit_cos;
    float min_h           = R - r * limit_cos;

    if (h_meters > max_h)
    {
        h_meters = max_h;
        if (v_meters_s > 0.0f)
            v_meters_s = 0.0f;
    }
    if (h_meters < min_h)
    {
        h_meters = min_h;
        if (v_meters_s < 0.0f)
            v_meters_s = 0.0f;
    }

    float cos_theta = (h_meters - R) / r;
    if (cos_theta > 1.0f)
        cos_theta = 1.0f;
    if (cos_theta < -1.0f)
        cos_theta = -1.0f;

    float theta_rad = acosf(cos_theta);
    float theta_deg = rad2deg(theta_rad);
    float sin_theta = sinf(theta_rad);

    float damping_val      = 0.1f;
    float target_vel_rad_s = -v_meters_s / (r * (sin_theta + damping_val));

    float target_angle = theta_deg * (float)bending_direction_;
    target_vel_rad_s *= (float)bending_direction_;

    Set_Leg_Target(target_angle, target_vel_rad_s, ffw_torque, kp, kd);
}
#endif

void Wheel_Leg::Add_Leg_Compensation(float comp_pos, float comp_vel, float comp_force)
{
    leg_compensation_pos += comp_pos;
    leg_compensation_vel += comp_vel;
    leg_compensation_force += comp_force;
}

void Wheel_Leg::Execute_Leg_Control()
{
    float raw_target_pos = target_leg_pos + leg_compensation_pos;

    // Unwrap target position relative to previous command to find shortest path
    // and avoid jumps for the Slew Rate Limiter
    float diff = raw_target_pos - prev_leg_pos_cmd;
    // Normalize diff to [-180, 180]
    while (diff > 180.0f)
        diff -= 360.0f;
    while (diff < -180.0f)
        diff += 360.0f;

    raw_target_pos = prev_leg_pos_cmd + diff;

    // Slew Rate Limiter (Ramp)
    // Limit the change in position per update to prevent violent movements
    // Assuming 2ms update rate (500Hz)
    float dt        = 0.002f;
    float max_delta = LEG_MAX_SPEED * dt;

    if (raw_target_pos > prev_leg_pos_cmd + max_delta)
        raw_target_pos = prev_leg_pos_cmd + max_delta;
    else if (raw_target_pos < prev_leg_pos_cmd - max_delta)
        raw_target_pos = prev_leg_pos_cmd - max_delta;

    prev_leg_pos_cmd = raw_target_pos;
    final_leg_pos    = raw_target_pos;

    final_leg_vel   = target_leg_vel + leg_compensation_vel;
    final_leg_force = target_leg_force + leg_compensation_force;

#if USE_6020_LEG_MOTOR
    if (leg_motor && Leg_Vel_PIDs && Leg_Pos_PIDs)
    {
        float target_pos_rad  = deg2rad(final_leg_pos);
        float current_pos_rad = normalizeAngle(leg_motor->getPositionFeedback() - leg_offset);

        float target_pos_delta_rad = normalizeAngle(target_pos_rad - current_pos_rad);

        float current_accumulated_pos_rad = leg_motor->getAccumulatedPosition();
        float target_accumulated_pos_rad  = current_accumulated_pos_rad + target_pos_delta_rad;

        // Position Loop -> Velocity Loop
        float vel_cmd = Leg_Pos_PIDs->operator()(target_accumulated_pos_rad, current_accumulated_pos_rad);

        // Add velocity compensation here if needed, or treat it as feedforward
        vel_cmd += final_leg_vel;

        float current_cmd = Leg_Vel_PIDs->operator()(vel_cmd, leg_motor->getRPMFeedback());

        // Add force (current) compensation
        current_cmd += final_leg_force;  // Assuming force is current for GM6020 or needs conversion

        leg_motor->setOutput(current_cmd);
    }
#elif USE_HT_LEG_MOTOR
    if (leg_motor)
    {
        // Calculate base target in radians
        float target_pos_rad_base = deg2rad(final_leg_pos) + leg_offset;

        // Get current position
        float current_pos_rad = leg_motor->getPositionFeedback();

        // Find closest 2k*pi equivalent to avoid multi-turn unwinding
        float diff             = current_pos_rad - target_pos_rad_base;
        float k                = roundf(diff / TWO_PI);
        float final_target_rad = target_pos_rad_base + k * TWO_PI;

        mit_set.Position = final_target_rad;
        mit_set.Velocity = final_leg_vel;

        // Convert Torque (Nm) to Current (A)
        mit_set.FFW_Current = final_leg_force / leg_motor->getKA();

        leg_motor->setMIT(mit_set.Position, mit_set.Velocity, mit_set.Pos_KP, mit_set.Vel_KD, mit_set.FFW_Current);
        leg_motor->transmit();
    }
#endif
}

#if USE_HT_LEG_MOTOR
void Wheel_Leg::SetZero()
{
    if (leg_motor)
    {
        leg_motor->setZeroPosition(0.0f);
    }
}

void Wheel_Leg::EnterMotorMode()
{
    if (leg_motor)
    {
        for (int i = 0; i < 5; i++)
        {
            leg_motor->sendCommand(Motors::HT8115::SpecialCommands::ENTER_MOTOR);
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }
}
#endif

void Wheel_Leg::Set_Wheel_Leg(Wheel_Leg_Params cmd)
{
    if (cmd.state == Chassis_State::IDLE)
    {
        cmd.Wheel_RPM = 0.0f;
        cmd.Leg_POS   = 0.0f;
        cmd.Leg_RPM   = 0.0f;
        cmd.Leg_Force = 0.0f;
        cmd.Leg_Kp    = 0.0f;
        cmd.Leg_Kd    = 0.0f;
    }

    // 1. Set Targets
    Set_Wheel_Target(cmd.Wheel_RPM);
#if USE_6020_LEG_MOTOR
    Set_Leg_Target(cmd.Leg_POS, cmd.Leg_RPM, cmd.Leg_Force);
#elif USE_HT_LEG_MOTOR
    Set_Leg_Target(cmd.Leg_POS, cmd.Leg_RPM, cmd.Leg_Force, cmd.Leg_Kp, cmd.Leg_Kd);
#endif

    // 2. Add Compensations
    // Internal Decoupling Compensation
    float decouple_comp = Wheel_Compensation();
    Add_Wheel_Compensation(decouple_comp);

    // 3. Execute Control
    Execute_Wheel_Control();
    Execute_Leg_Control();
}
}  // namespace Applications