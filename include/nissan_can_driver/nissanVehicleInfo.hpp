#ifndef CRP_VIL_NISSAN_VEHICLE_INFO_NISSANCANDRIVER_HPP
#define CRP_VIL_NISSAN_VEHICLE_INFO_NISSANCANDRIVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "nissan_can_driver/nissanCanDefinitions.hpp"


namespace crp
{
namespace vil
{

class NissanVehicleInfo : public rclcpp::Node
{
public:
    NissanVehicleInfo();

private:
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr    m_pub_vehicleSpeed_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr    m_pub_vehicleSteering_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr    m_pub_vehicleTireAngle_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_pub_vehicleTwist_;

    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr m_sub_can_;

    void canCallback(const can_msgs::msg::Frame::SharedPtr msg);
    void timerCallback();

    NissanCanDefinitions m_nissanCanDefinitions;
    rclcpp::TimerBase::SharedPtr m_timer_;

    int8_t m_vehicleDirection{0};
    float m_vehicleSpeed{0.0f};
    float m_vehicleSteering{0.0f};
};

} // namespace vil
} // namespace crp
#endif //CRP_VIL_NISSAN_VEHICLE_INFO_NISSANCANDRIVER_HPP