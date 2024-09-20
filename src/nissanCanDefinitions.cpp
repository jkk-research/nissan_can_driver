#include "nissan_can_driver/nissanCanDefinitions.hpp"


float crp::vil::NissanCanDefinitions::decodeVehicleSpeed(const can_msgs::msg::Frame &frame)
{
    double measured_speed =
        ((double)((frame.data[0] << 8) | frame.data[1])) * 1.0/8.570637;
    return measured_speed * 1.0/3.6; // to m/s
}


float crp::vil::NissanCanDefinitions::decodeVehicleSteering(const can_msgs::msg::Frame &frame)
{
    float raw_steer_ang = ((int16_t)((frame.data[1] << 8) | frame.data[0])) / 10.0;
    return raw_steer_ang;
}


uint8_t crp::vil::NissanCanDefinitions::decodeVehicleShifter(const can_msgs::msg::Frame &frame)
{
    unsigned int gear_state = frame.data[0];
    switch (gear_state)
    {
        case 0x20: // D
        {
            return 1;
        }
        case 0x10: // R
        {
            return -1;
        }
        case 0x08: // P
        {
            return 0;
        }
        case 0x18: // N
        {
            return 0;
        }
    }
    return 0;
}
