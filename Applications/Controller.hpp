#pragma once
#include "Comm_Msg.hpp"
#include "Cust_Types.hpp"

namespace Applications
{

class Controller
{
   public:
    Controller() = default;

    /**
     * @brief Map Joystick to Velocity
     * @param msg The received PC message
     * @param vx Reference to output linear velocity
     * @param wz Reference to output angular velocity
     */
    void Map_Joystick_To_Velocity(const Protocol::PC_Msg &msg, float &vx, float &wz);
};

}  // namespace Applications
