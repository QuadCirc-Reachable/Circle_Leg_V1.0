#include "Motor_Controller.hpp"

namespace Motor_Controller
{ 
    GM6020_Controller::GM6020_Controller(Motors::GM6020 **Motors,
                                         PID **Vel_PIDs,
                                         PID **Pos_PIDs)
    {
        this->motors    = Motors;
        this->vel_PIDs  = Vel_PIDs;
        this->pos_PIDs  = Pos_PIDs;
        //Initialize target positions and deltas
        for (int i = 0; i < LEG_MOTOR_NUM; i++)
        {
            target_accumulated_positions_rad[i]     = 0.0f;
            current_accumulated_positions_rad[i] = 0.0f;
            target_pos_delta_rad[i]     = 0.0f;
            current_positions_deg[i] = 0.0f;
            last_set_positions_deg[i] = 0.0f;
        }
    }

    void GM6020_Controller::setTargetPosition(float *set_position, float* current_position, bool is_rad)
    {
        for(int i = 0; i < LEG_MOTOR_NUM; i++){
            // Record last set position for debugging/logging
            last_set_positions_deg[i] = is_rad ? rad2deg(set_position[i]) : set_position[i];

            //Get current position in rad
            current_positions_rad[i]= normalizeAngle(motors[i]->getPositionFeedback() - motor_offsets_rad[i]);
            current_positions_deg[i]= rad2deg(current_positions_rad[i]); //For debug purpose
            
            //Calculate target position delta in rad
            //Calculate the shortest path
            if(is_rad){
                target_pos_delta_rad[i] = normalizeAngle(set_position[i] - current_positions_rad[i]);
            }
            else{
                target_pos_delta_rad[i] = normalizeAngle(deg2rad(set_position[i]) - current_positions_rad[i]);
            }
            
            //Calculate target accumulated position in rad
            //Add the shortest path to current accumulated position
            current_accumulated_positions_rad[i] = motors[i]->getAccumulatedPosition();
            target_accumulated_positions_rad[i] = current_accumulated_positions_rad[i] + target_pos_delta_rad[i];
            
            //Set motor output using position PID and velocity PID
            motors[i]->setOutput(
                    vel_PIDs[i]->operator()(
                        pos_PIDs[i]->operator()(
                            target_accumulated_positions_rad[i], 
                            current_accumulated_positions_rad[i]), 
                        motors[i]->getRPMFeedback()
                ));
            current_position[i] = current_positions_deg[i];
        }
    }

    void GM6020_Controller::updateReachableMsg(Protocol::Reachable_Msg* msg)
    {
        if(msg == nullptr) return;
        for(int i=0; i<LEG_MOTOR_NUM; i++)
        {
            msg->GM6020_Current_Pos_x10_msg[i] = (int16_t)(current_positions_deg[i] * 10.0f);
            msg->GM6020_Target_Pos_x10_msg[i]  = (int16_t)(last_set_positions_deg[i] * 10.0f);
            msg->GM6020_Current_Tem_msg[i]     = (int8_t)motors[i]->getTemperatureFeedback();
        }
    }

    M3508_Controller::M3508_Controller(Motors::M3508 **Motors,
                                         PID **Vel_PIDs)
    {
        this->motors   = Motors;
        this->vel_PIDs = Vel_PIDs;
        for(int i=0; i<WHEEL_MOTOR_NUM; i++) {
            last_set_rpm[i] = 0.0f;
        }
    }

    void  M3508_Controller::setTargetForce(float *set_force)
    {
        //To be implemented if needed
    }

    void M3508_Controller::updateCompensation(Motors::GM6020 **leg_motors, float* leg_current_pos, float *set_rpm)
    {
        uint8_t j = 0;
        for(uint8_t i = 0; i < WHEEL_MOTOR_NUM; i++) {
            switch (i)
            {
            //3508 ID 1 -> Leg Motor ID 4
            case 0:
                j = 3;
                break;
            //3508 ID 2 -> Leg Motor ID 1
            case 1:
                j = 0;
                break;
            //3508 ID 3 -> Leg Motor ID 2
            case 2:
                j = 1;
                break;
            //3508 ID 4 -> Leg Motor ID 3
            case 3:
                j = 2;
                break;
            default:
                break;
            }
            float leg_rpm = leg_motors[j]->getRPMFeedback();
            float compensation_rpm = leg_rpm * (1.0f + (ECCENTRIC_OFFSET_r / WHEEL_RADIUS_R) * cosf(deg2rad(leg_current_pos[j])));
            set_rpm[i] -= compensation_rpm;
        }
    }

    void M3508_Controller::setTargetRPM(float *set_rpm, float* current_rpm)
    {
        for(uint8_t i = 0; i < WHEEL_MOTOR_NUM; i++){
            last_set_rpm[i] = set_rpm[i]; // Record for debugging/logging

            //Set motor output using velocity PID
            motors[i]->setOutput(
                vel_PIDs[i]->operator()(
                    set_rpm[i], 
                    motors[i]->getRPMFeedback()
            ));
            current_rpm[i] = motors[i]->getRPMFeedback();
        }
    }

    void M3508_Controller::updateReachableMsg(Protocol::Reachable_Msg* msg)
    {
        if(msg == nullptr) return;
        for(int i=0; i<WHEEL_MOTOR_NUM; i++)
        {
            msg->M3508_Current_RPM_msg[i] = (int16_t)motors[i]->getRPMFeedback();
            msg->M3508_Target_RPM_msg[i]  = (int16_t)last_set_rpm[i];
            msg->M3508_Current_Tem_msg[i] = (int8_t)motors[i]->getTemperatureFeedback();
        }
    }

} // namespace Motor_Controller