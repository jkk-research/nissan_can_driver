#include "nissan_can_driver/nissanCanDefinitions.hpp"


can_msgs::msg::Frame crp::vil::NissanCanDefinitions::encodeHlcAutonomous(const float speed, const float tireAngle)
{
    can_msgs::msg::Frame frame;
    frame.id = m_HLC_AUTONOMOUS_ID;
    frame.is_extended = false;
    frame.is_rtr = false;
    frame.is_error = false;
    frame.dlc = 8;

    int16_t tireAngle_deg = static_cast<int16_t>(-tireAngle * 180.0 / 3.14 * 100); // to deg, scale

    frame.data[0] = 0x01;
    frame.data[1] = uint8_t(speed * 3.6); // to km/h
    frame.data[2] = tireAngle_deg >> 8;
    frame.data[3] = tireAngle_deg & 0xFF;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    return frame;
}

can_msgs::msg::Frame crp::vil::NissanCanDefinitions::encodeHlcSpeedRefKmph(const float speed)
{
    can_msgs::msg::Frame frame;
    frame.id = m_HLC_SPEED_REF_KMPH_ID;
    frame.is_extended = false;
    frame.is_rtr = false;
    frame.is_error = false;
    frame.dlc = 2;

    uint16_t speed_kmph = static_cast<uint16_t>(speed * 3.6); // to km/h
    frame.data[0] = speed_kmph >> 8;
    frame.data[1] = speed_kmph & 0xFF;

    return frame;
}


float crp::vil::NissanCanDefinitions::decodeVehicleSpeed(const can_msgs::msg::Frame &frame)
{
    double measured_speed =
        ((double)((frame.data[0] << 8) | frame.data[1])) * 1.0/8.570637;
    return measured_speed * 1.0/3.6 * -1; // to m/s, right is negative
}


float crp::vil::NissanCanDefinitions::decodeVehicleSteering(const can_msgs::msg::Frame &frame)
{
    float raw_steer_ang = ((int16_t)((frame.data[1] << 8) | frame.data[0])) / 10.0;
    return raw_steer_ang * 3.14 / 180.0; // to rad
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
