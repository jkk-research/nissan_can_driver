#include "nissan_can_driver/nissanVehicleControl.hpp"


crp::vil::NissanVehicleControl::NissanVehicleControl() : Node("nissan_vehicle_control")
{
    bool isAutowareControlInput;
    this->declare_parameter<bool>("autoware_control_input", true);
    this->get_parameter<bool>("autoware_control_input", isAutowareControlInput);
    
    if (isAutowareControlInput)
    {
        RCLCPP_INFO(this->get_logger(), "Using autoware_control_input");
        sub_cmdControl_ = this->create_subscription<autoware_control_msgs::msg::Control>(
            "control_cmd", 10, std::bind(&NissanVehicleControl::cmdControlCallback, this, std::placeholders::_1));
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Using speed_cmd and steering_cmd");
        sub_cmdSpeed_ = this->create_subscription<std_msgs::msg::Float32>(
            "speed_cmd", 10, std::bind(&NissanVehicleControl::cmdSpeedCallback, this, std::placeholders::_1));
        sub_cmdSteering_ = this->create_subscription<std_msgs::msg::Float32>(
            "steering_cmd", 10, std::bind(&NissanVehicleControl::cmdSteeringCallback, this, std::placeholders::_1));
    }

    m_pub_can_ = this->create_publisher<can_msgs::msg::Frame>("can_rx", 10);

    m_timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&NissanVehicleControl::timerCallback, this));
}


void crp::vil::NissanVehicleControl::cmdSpeedCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    m_vehicleSpeed = msg->data;
    m_lastVehicleSpeedTime = this->now().seconds();
}


void crp::vil::NissanVehicleControl::cmdSteeringCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    m_vehicleSteering = msg->data;
    m_lastVehicleSteeringTime = this->now().seconds();
}


void crp::vil::NissanVehicleControl::cmdControlCallback(const autoware_control_msgs::msg::Control::SharedPtr msg)
{
    m_vehicleSpeed = msg->longitudinal.velocity;
    m_vehicleSteering = msg->lateral.steering_tire_angle;

    m_lastVehicleSpeedTime = this->now().seconds();
    m_lastVehicleSteeringTime = this->now().seconds();
}


void crp::vil::NissanVehicleControl::timerCallback()
{
    if (this->now().seconds() - m_lastVehicleSpeedTime > 0.2)
    {
        RCLCPP_WARN(this->get_logger(), "No speed command received for more than 200 ms! Control is disabled.");
        return;
    }
    if (this->now().seconds() - m_lastVehicleSteeringTime > 0.2)
    {
        RCLCPP_WARN(this->get_logger(), "No steering command received for more than 200 ms! Control is disabled.");
        return;
    }

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