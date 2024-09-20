#ifndef CRP_VIL_NISSAN_CAN_DEFINITIONS_NISSANCANDRIVER_HPP
#define CRP_VIL_NISSAN_CAN_DEFINITIONS_NISSANCANDRIVER_HPP

#include <can_msgs/msg/frame.hpp>

namespace crp
{
namespace vil
{

class NissanCanDefinitions
{
public:
    NissanCanDefinitions() {};

    can_msgs::msg::Frame encodeHlcAutonomous(const float speed, const float tireAngle);
    can_msgs::msg::Frame encodeHlcSpeedRefKmph(const float speed);

    float   decodeVehicleSpeed(const can_msgs::msg::Frame &frame);
    float   decodeVehicleSteering(const can_msgs::msg::Frame &frame);
    uint8_t decodeVehicleShifter(const can_msgs::msg::Frame &frame);

    const uint16_t m_HLC_AUTONOMOUS_ID     = 0x01;
    const uint16_t m_HLC_SPEED_REF_KMPH_ID = 0x05;

    const uint16_t m_VEHICLE_SPEED_ID    = 0x176;
    const uint16_t m_VEHICLE_STEERING_ID = 0x2;
    const uint16_t m_VEHICLE_SHIFTER_ID  = 0x421;
};

} // namespace vil
} // namespace crp
#endif //CRP_VIL_NISSAN_CAN_DEFINITIONS_NISSANCANDRIVER_HPP