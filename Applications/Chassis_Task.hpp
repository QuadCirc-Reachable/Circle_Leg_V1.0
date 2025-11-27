#pragma once
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "DJIMotor.hpp"
#include "GM6020.hpp"
#include "M3508.hpp"
#include "IMU.hpp"
#include "PID.hpp"
#include "Helper.hpp"
#include "Motor_Controller.hpp"
#include "PC_Comm.hpp"
#include "Comm_Msg.hpp"

namespace Applications::Chassis_Task{
    void Chassis_Task(void *pvPara);
    void init();
}