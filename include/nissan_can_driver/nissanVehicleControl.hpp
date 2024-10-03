#ifndef CRP_VIL_NISSAN_VEHICLE_CONTROL_NISSANCANDRIVER_HPP
#define CRP_VIL_NISSAN_VEHICLE_CONTROL_NISSANCANDRIVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>
#include <std_msgs/msg/float32.hpp>
#include <autoware_control_msgs/msg/control.hpp>

#include "nissan_can_driver/nissanCanDefinitions.hpp"

namespace crp
{
namespace vil
{

class NissanVehicleControl : public rclcpp::Node
{
public:
    NissanVehicleControl();

private:
    void cmdSpeedCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void cmdSteeringCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void cmdControlCallback(const autoware_control_msgs::msg::Control::SharedPtr msg);
    void timerCallback();

    rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr sub_cmdControl_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_cmdSpeed_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_cmdSteering_;

    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr m_pub_can_;

    rclcpp::TimerBase::SharedPtr m_timer_;

    crp::vil::NissanCanDefinitions m_nissanCanDefinitions;
    float m_vehicleSpeed{0.0};
    float m_vehicleSteering{0.0};
    float m_lastVehicleSpeedTime{0.0};
    float m_lastVehicleSteeringTime{0.0};
};

} // namespace vil
} // namespace crp
#endif // CRP_VIL_NISSAN_VEHICLE_CONTROL_NISSANCANDRIVER_HPP