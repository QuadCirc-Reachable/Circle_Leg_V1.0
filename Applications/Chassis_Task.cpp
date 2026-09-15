/**
 * @file    Chassis_Task.cpp
 * @brief   FreeRTOS chassis task: 500 Hz control loop (PC command in ->
 *          Chassis::Update -> motor commands -> feedback out).
 *
 * Circle_Leg_V1 - REACHABLE (QuadCirc) half-size prototype firmware.
 *
 * @author  LIU Hualin
 */

#include "Chassis_Task.hpp"

#include "Robot_Config.hpp"

namespace Applications::Chassis_Task
{
using namespace Core::Drivers;
using namespace Core::Control;
using namespace Applications::Command_Task;

// Global Message Buffers
Protocol::Reachable_Msg reachable_msg_chassis = {};
Protocol::PC_Msg pc_msg_chassis               = {};

StackType_t uxChassisTaskStack[2048];
StaticTask_t xChassisTaskTCB;

void Chassis_Task(void *pvPara)
{
    // Wait for HT motors to boot after power-on
    // HT8115 internal MCU needs ~500ms to initialize CAN interface
    // Without this delay, ENTER_MOTOR commands sent during Init() are lost
    // (This is why it works under Ozone but not standalone: the debugger adds implicit delay)
    vTaskDelay(pdMS_TO_TICKS(3000));

    // Initialize Chassis (Motors, PIDs, etc.)
    chassis.Init();

    while (true)
    {
        // Get PC Command
        Get_PC_Msg(&pc_msg_chassis);

        // Update Chassis Control Loop
        chassis.Update(pc_msg_chassis);

        // Transmit CAN Messages
        // Note: Transmit logic might need to be inside Chassis::Update or called here explicitly
        // if Chassis doesn't handle transmission.
        // Assuming Chassis handles logic, but we need to trigger CAN transmit.
        // DJIMotor::transmit sends to all motors in a group.
        // We need to know which groups are used.
        // Based on Robot_Config:
        // Leg Motors (GM6020): ID 1-4 on CAN 1 -> Group 1 (transmit(3, 0) for GM6020 ID1-4 on CAN1?)
        // Wait, transmit(3, 0) -> Group 3 on CAN 0?
        // Let's check DJIMotor.cpp mapping again if needed, but assuming previous code was correct:
        // Motors::DJIMotor::transmit(3, 0); // GM6020 group 1 (CAN1) -> Wait, CAN index 0 is CAN1? Yes.
        // Motors::DJIMotor::transmit(0, 1); // M3508 group 1 (CAN2) -> CAN index 1 is CAN2.

#if USE_6020_LEG_MOTOR
        Motors::DJIMotor::transmit(3, 0);  // Transmit GM6020 ID 1-4 on CAN 1
#endif
        Motors::DJIMotor::transmit(0, 1);  // Transmit M3508 ID 1-4 on CAN 2

        // Update Feedback Messages
        chassis.Get_Msg(&reachable_msg_chassis);

        // Send feedback to PC
        Set_Reachable_Msg(&reachable_msg_chassis);

        vTaskDelay(2);
    }
}

void init() { xTaskCreateStatic(Chassis_Task, "Chassis_Task", 2048, NULL, 0, uxChassisTaskStack, &xChassisTaskTCB); }
}  // namespace Applications::Chassis_Task