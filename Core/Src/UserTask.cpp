/**
 * @file UserTask.cpp
 * @author JIANG Yicheng  RM2023 (EthenJ@outlook.sg)
 * @brief Create user tasks with cpp support
 * @version 0.1
 * @date 2022-08-20
 *
 * @copyright Copyright (c) 2022
 */

#include "FreeRTOS.h"
#include "gpio.h"
#include "usart.h"
#include "main.h"
#include "task.h"

#include "CANManager.hpp"
#include "DJIMotor.hpp"
#include "IMU.hpp"
#include "Chassis_Task.hpp"
#include "RosComm.hpp"
#include "PC_Comm.hpp"

StackType_t uxBlinkTaskStack[configMINIMAL_STACK_SIZE];
StaticTask_t xBlinkTaskTCB;

void blink(void *pvPara)
{
    HAL_GPIO_WritePin(LED_ACT_GPIO_Port, LED_ACT_Pin, GPIO_PIN_RESET);

    while (true)
    {
        HAL_GPIO_TogglePin(LED_ACT_GPIO_Port, LED_ACT_Pin);
        HAL_GPIO_TogglePin(LASER_GPIO_Port, LASER_Pin);
        vTaskDelay(500);
    }
}

/**
 * @brief Create user tasks
 */
void startUserTasks() {    
        xTaskCreateStatic(blink, "blink", configMINIMAL_STACK_SIZE, NULL, 0, uxBlinkTaskStack, &xBlinkTaskTCB); 
        Core::Drivers::CANManager::managers[0].init(&hfdcan1);
        Core::Drivers::CANManager::managers[1].init(&hfdcan2);
        Core::Drivers::Motors::DJIMotor::init();
        Core::Drivers::IMU::init();
        Core::Communication::RosComm::RosManager::managers[0].init(&huart2);
        Applications::Command_Task::init();
        Applications::Chassis_Task::init();
    }
