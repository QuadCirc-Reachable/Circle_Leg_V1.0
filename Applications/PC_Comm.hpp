#pragma once
#include "AppConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "RosComm.hpp"
#include "Comm_Msg.hpp"

namespace Applications::Command_Task{
    class PC_Comm{
        public:
            PC_Comm(Protocol::PC_Msg& pc_msg,  Protocol::Reachable_Msg& reachable_msg):
                pv_pc_msg(pc_msg),
                pv_reachable_msg(reachable_msg)
            {}
            /** 
             * @brief Get the PC Msg object
             * @param pc_msg Pointer to store the received PC_Msg
             */
            void Fetch_PC_Msg(Protocol::PC_Msg* pc_msg);

            /** 
             * @brief Set the Reachable Msg object
             * @param reachable_msg Pointer to the Reachable_Msg to be sent
             */
            void Update_Reachable_Msg(Protocol::Reachable_Msg* reachable_msg);
            /** 
             * @brief PC_Comm Task
             */
            static void PC_CommTask(void *pvPara);
            

        private:
            bool is_connected = false;
            uint16_t tx_freq = 100; // in ms
            Protocol::PC_Msg& pv_pc_msg;
            Protocol::Reachable_Msg& pv_reachable_msg;
    };
    
    /** 
     * @brief Callback function to handle received PC_Msg
     */
    void PC_RxCallback(uint8_t* data, uint16_t len, UART_HandleTypeDef* handle);
    /** 
     * @brief Getter for PC_Msg
     */
    void Get_PC_Msg(Protocol::PC_Msg* pc_msg);
    /** 
     * @brief Setter for Reachable_Msg
     */
    void Set_Reachable_Msg(Protocol::Reachable_Msg* reachable_msg);
    /** 
     * @brief Initialize the PC_Comm task
     */
    void init();

}