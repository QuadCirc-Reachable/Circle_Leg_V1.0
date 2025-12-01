#pragma once
#include "Comm_Msg.hpp"
#include "DJIMotor.hpp"
#include "FreeRTOS.h"
#include "GM6020.hpp"
#include "M3508.hpp"
#include "Helper.hpp"
#include "IMU.hpp"
// #include "Motor_Controller.hpp"
#include "PC_Comm.hpp"
#include "PID.hpp"
#include "main.h"
#include "task.h"

/*

    Index_3:                                     Index_2:
    RearLeftWheelMotor ------------------------- RearRightWheelMotor
    RearLeftJointMotor ------------------------- RearRightJointMotor
                        |                     |
                        |                     |
                        |                     |
                        |                     |
                        |                     |
Index_0:                |                     |     Index_1:
FrontLeftWheelMotor ------------------------------- FrontRightWheelMotor
FrontLeftJointMotor ------------------------------- FrontRightJointMotor

*/
namespace Reachable_Controller
{
using namespace Core::Drivers;
using namespace Core::Control;

class WheelModule_t
{
private:
    Motors::M3508 *wheelMotor;
    Motors::GM6020 *jointMotor;
    PID wheelVelPID;
    PID jointVelPID;
    PID jointPosPID;
    
    const float jointMotorOffsetRad;

public:
    WheelModule_t(Motors::M3508 *wheelMotor, Motors::GM6020 *jointMotor);
    WheelModule_t() = delete;

    /**
     * @brief Set PID parameters for wheel and joint motors
     * @param param PID parameters for wheel motor
     * @param velParam PID parameters for joint motor velocity control
     * @param posParam PID parameters for joint motor position control
     */
    void setWheelPIDParams(const PID::Param &param);
    void setJointPIDParams(const PID::Param &velParam, const PID::Param &posParam);

    /**
     * @brief Set target wheel RPM and joint position
     * @param targetWheelRPM Target wheel RPM
     * @param targetJointPos Target joint position in degrees
     */
    void setTargets(float targetWheelRPM, float targetJointPos);

    /**
     * @brief Update the Reachable_Msg with current motor status
     * @param msg Pointer to the Reachable_Msg to be filled
     * @param wheelIndex Index of the wheel module (0-3)
     */
    void updateReachableMsg(Protocol::Reachable_Msg *msg, uint8_t wheelIndex);
};


}