#include "nissan_can_driver/nissanVehicleControl.hpp"


crp::vil::NissanVehicleControl::NissanVehicleControl() : Node("nissan_vehicle_control")
{
    m_sub_cmdSpeed_ = this->create_subscription<std_msgs::msg::Float32>(
        "speed_cmd", 10, std::bind(&NissanVehicleControl::cmdSpeedCallback, this, std::placeholders::_1));
    m_sub_cmdSteering_ = this->create_subscription<std_msgs::msg::Float32>(
        "steering_cmd", 10, std::bind(&NissanVehicleControl::cmdSteeringCallback, this, std::placeholders::_1));

    m_pub_can_ = this->create_publisher<can_msgs::msg::Frame>("can_rx", 10);

    m_timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&NissanVehicleControl::timerCallback, this));
}


void crp::vil::NissanVehicleControl::cmdSpeedCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    m_vehicleSpeed = msg->data;
}


void crp::vil::NissanVehicleControl::cmdSteeringCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    m_vehicleSteering = msg->data;
}


void crp::vil::NissanVehicleControl::timerCallback()
{
    can_msgs::msg::Frame frameSpeedRef    = m_nissanCanDefinitions.encodeHlcSpeedRefKmph(m_vehicleSpeed);
    can_msgs::msg::Frame frameHlcAutonom = m_nissanCanDefinitions.encodeHlcAutonomous(m_vehicleSpeed, m_vehicleSteering);

    m_pub_can_->publish(frameSpeedRef);
    m_pub_can_->publish(frameHlcAutonom);
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<crp::vil::NissanVehicleControl>());
    rclcpp::shutdown();
    return 0;
}