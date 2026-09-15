/**
 * @file    Robot_Config.hpp
 * @brief   Hardware instantiation: M3508 wheel motors, HT8115 leg motors (GM6020
 *          optional), CAN IDs, per-leg PID / MIT parameters and Wheel_Leg pairing.
 *
 * Circle_Leg_V1 - REACHABLE (QuadCirc) half-size prototype firmware.
 *
 * @author  LIU Hualin
 */

#pragma once
#include "Chassis.hpp"
#include "GM6020.hpp"
#include "HT8115.hpp"
#include "M3508.hpp"
#include "PID.hpp"
#include "Robot_Params.hpp"
#include "Wheel_Leg.hpp"

namespace Applications
{
// Expose the Chassis instance
extern Chassis chassis;

// Expose individual components if needed for debugging
extern Wheel_Leg FL_Leg;
extern Wheel_Leg FR_Leg;
extern Wheel_Leg BL_Leg;
extern Wheel_Leg BR_Leg;

}  // namespace Applications
