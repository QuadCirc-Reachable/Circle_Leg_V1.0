#include "Climbing_Dynamics.hpp"

namespace Applications
{

Climbing_Dynamics::Climbing_Dynamics(const Config &cfg) : cfg_(cfg) { reset(); }

void Climbing_Dynamics::init(const Config &cfg)
{
    cfg_ = cfg;
    reset();
}

void Climbing_Dynamics::reset()
{
    for (int i = 0; i < 4; i++)
    {
        legs_[i]     = LegState{};
        target_h_[i] = 0.0f;
        target_v_[i] = 0.0f;
    }
}

void Climbing_Dynamics::startClimb(int idx)
{
    if (idx < 0 || idx > 3)
        return;
    legs_[idx].phase          = LegClimbPhase::PREP;
    legs_[idx].beta           = 0.0f;
    legs_[idx].detect_timer_s = 0.0f;
}

void Climbing_Dynamics::startClimbAll()
{
    for (int i = 0; i < 4; i++)
        startClimb(i);
}

bool Climbing_Dynamics::isDirectControl(int idx) const
{
    LegClimbPhase p = legs_[idx].phase;
    // PREP uses normal height pipeline (smooth ramp), not direct climbing control
    return p == LegClimbPhase::DETECT || p == LegClimbPhase::CLIMBING;
}

// =====================================================================
// Kinematic helpers
// =====================================================================

float Climbing_Dynamics::computeBeta0() const
{
    // β₀ = arcsin((R - h) / R)
    float arg = (cfg_.wheel_radius_m - cfg_.step_height_m) / cfg_.wheel_radius_m;
    if (arg > 1.0f)
        arg = 1.0f;
    if (arg < -1.0f)
        arg = -1.0f;
    return asinf(arg);
}

float Climbing_Dynamics::computeThetaEnd() const
{
    // θ_end = arccos((L - h) / L)
    float arg = (cfg_.leg_length_m - cfg_.step_height_m) / cfg_.leg_length_m;
    if (arg > 1.0f)
        arg = 1.0f;
    if (arg < -1.0f)
        arg = -1.0f;
    return acosf(arg);
}

float Climbing_Dynamics::computeThetaDot(float theta_rad, float phi_w) const
{
    //            sqrt(R² - (R - h + L·(1 - cosθ))²)
    // θ̇ =  ─────────────────────────────────────────── · φ_w
    //                    L · sinθ

    float R = cfg_.wheel_radius_m;
    float L = cfg_.leg_length_m;
    float h = cfg_.step_height_m;

    float inner = R - h + L * (1.0f - cosf(theta_rad));
    float disc  = R * R - inner * inner;
    if (disc < 0.0f)
        disc = 0.0f;

    float numerator = sqrtf(disc);

    // Singularity protection: clamp sin(θ) away from zero
    float sin_theta = sinf(theta_rad);
    float min_sin   = sinf(cfg_.theta_min_deg * PI / 180.0f);
    if (min_sin < 1e-4f)
        min_sin = 1e-4f;
    if (sin_theta < min_sin)
        sin_theta = min_sin;

    float denominator = L * sin_theta;
    return (numerator / denominator) * phi_w;
}

float Climbing_Dynamics::heightFromTheta(float theta_rad) const
{
    // Pipeline model: h = R + L·cos(θ)
    return cfg_.wheel_radius_m + cfg_.leg_length_m * cosf(theta_rad);
}

// =====================================================================
// Main update — per-leg state machine
// =====================================================================

void Climbing_Dynamics::update(const LegClimbFeedback feedback[4], float dt)
{
    float theta_end = computeThetaEnd();
    float prep_rad  = cfg_.prep_theta_deg * PI / 180.0f;
    float h_prep    = heightFromTheta(prep_rad);

    for (int i = 0; i < 4; i++)
    {
        LegState &leg              = legs_[i];
        const LegClimbFeedback &fb = feedback[i];

        switch (leg.phase)
        {
        // ---------------------------------------------------------
        case LegClimbPhase::IDLE:
            target_h_[i] = 0.0f;
            target_v_[i] = 0.0f;
            break;

        // ---------------------------------------------------------
        case LegClimbPhase::PREP:
        {
            // Command leg to prep angle (near-extended) to avoid singularity
            target_h_[i] = h_prep;
            target_v_[i] = 0.0f;

            // Check if leg has reached prep angle (within tolerance)
            float current_theta = fabsf(fb.leg_pos_deg);
            if (fabsf(current_theta - cfg_.prep_theta_deg) < cfg_.prep_tolerance_deg)
            {
                leg.phase            = LegClimbPhase::DETECT;
                leg.detect_timer_s   = 0.0f;
                leg.current_baseline = fb.wheel_current;  // Initialize baseline to current value
            }
            break;
        }

        // ---------------------------------------------------------
        case LegClimbPhase::DETECT:
        {
            // Hold prep angle while monitoring wheel current for step contact
            target_h_[i] = h_prep;
            target_v_[i] = 0.0f;

            // Update baseline with slow LPF
            leg.current_baseline = cfg_.baseline_alpha * fb.wheel_current + (1.0f - cfg_.baseline_alpha) * leg.current_baseline;

            // Spike detection: deviation from baseline
            float deviation = fabsf(fb.wheel_current - leg.current_baseline);
            if (deviation > cfg_.spike_threshold)
            {
                leg.detect_timer_s += dt;
                if (leg.detect_timer_s >= cfg_.detect_confirm_s)
                {
                    // Step confirmed — begin climbing
                    leg.phase = LegClimbPhase::CLIMBING;
                    leg.beta  = computeBeta0();
                }
            }
            else
            {
                leg.detect_timer_s = 0.0f;  // Reset if spike subsides
            }
            break;
        }

        // ---------------------------------------------------------
        case LegClimbPhase::CLIMBING:
        {
            // Integrate β:  dβ/dt = φ_w
            float phi_w = fb.wheel_rpm * 2.0f * PI / 60.0f;  // RPM → rad/s
            leg.beta += phi_w * dt;

            // Compute target θ from constraint:
            //   cos(θ) = (R + L − h − R·sin(β)) / L
            float R   = cfg_.wheel_radius_m;
            float L   = cfg_.leg_length_m;
            float h   = cfg_.step_height_m;
            float cth = (R + L - h - R * sinf(leg.beta)) / L;
            if (cth > 1.0f)
                cth = 1.0f;
            if (cth < -1.0f)
                cth = -1.0f;
            float theta_rad = acosf(cth);

            // Compute θ̇ from kinematic equation
            float theta_dot = computeThetaDot(theta_rad, phi_w);

            // Convert to height & velocity for the pipeline
            target_h_[i] = heightFromTheta(theta_rad);
            target_v_[i] = -L * sinf(theta_rad) * theta_dot;  // dh/dt = −L·sinθ·θ̇

            // End condition: β ≥ 90° or θ ≥ θ_end
            if (leg.beta >= PI / 2.0f || theta_rad >= theta_end)
            {
                leg.phase = LegClimbPhase::COMPLETE;
            }
            break;
        }

        // ---------------------------------------------------------
        case LegClimbPhase::COMPLETE:
            target_h_[i] = 0.0f;
            target_v_[i] = 0.0f;
            leg.phase    = LegClimbPhase::IDLE;
            break;
        }

        // Clamp velocity output
        if (target_v_[i] > cfg_.max_target_v)
            target_v_[i] = cfg_.max_target_v;
        if (target_v_[i] < -cfg_.max_target_v)
            target_v_[i] = -cfg_.max_target_v;
    }
}

}  // namespace Applications
