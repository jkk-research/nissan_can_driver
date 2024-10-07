#include "nissan_can_driver/nissanVehicleInfo.hpp"

crp::vil::NissanVehicleInfo::NissanVehicleInfo() : Node("nissan_vehicle_info")
{
    m_pub_vehicleSpeed_     = this->create_publisher<std_msgs::msg::Float32>("vehicle_speed", 10);
    m_pub_vehicleSteering_  = this->create_publisher<std_msgs::msg::Float32>("vehicle_steering", 10);
    m_pub_vehicleTireAngle_ = this->create_publisher<std_msgs::msg::Float32>("vehicle_tire_angle", 10);
    m_pub_vehicleTwist_     = this->create_publisher<geometry_msgs::msg::Twist>("/sensing/vehicle/twist", 10);

    m_sub_can_ = this->create_subscription<can_msgs::msg::Frame>(
        "can_tx", 10, std::bind(&NissanVehicleInfo::canCallback, this, std::placeholders::_1));
    
    m_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50), 
        std::bind(&NissanVehicleInfo::timerCallback, this));
}


void crp::vil::NissanVehicleInfo::timerCallback()
{
    // Publish vehicle speed
    std_msgs::msg::Float32 vehicleSpeed;
    vehicleSpeed.data = m_vehicleSpeed;
    m_pub_vehicleSpeed_->publish(vehicleSpeed);

    // Publish vehicle steering
    std_msgs::msg::Float32 vehicleSteering;
    vehicleSteering.data = m_vehicleSteering;
    m_pub_vehicleSteering_->publish(vehicleSteering);

    // Publish vehicle tire angle
    std_msgs::msg::Float32 tireAngle;
    tireAngle.data = m_vehicleSteering * 0.0547944; // to tire angle
    m_pub_vehicleTireAngle_->publish(tireAngle);

    // Publish twist
    geometry_msgs::msg::Twist vehicleTwist;
    vehicleTwist.linear.x  = m_vehicleSpeed;
    vehicleTwist.angular.z = (m_vehicleSpeed * tan(tireAngle.data)) / 2700.0;
    m_pub_vehicleTwist_->publish(vehicleTwist);
}


void crp::vil::NissanVehicleInfo::canCallback(const can_msgs::msg::Frame::SharedPtr msg)
{
    if (msg->id == m_nissanCanDefinitions.m_VEHICLE_SPEED_ID)
    {
        m_vehicleSpeed = m_nissanCanDefinitions.decodeVehicleSpeed(*msg) * m_vehicleDirection;
    }
    else if (msg->id == m_nissanCanDefinitions.m_VEHICLE_STEERING_ID)
    {
        m_vehicleSteering = m_nissanCanDefinitions.decodeVehicleSteering(*msg);
    }
    else if (msg->id == m_nissanCanDefinitions.m_VEHICLE_SHIFTER_ID)
    {
        m_vehicleDirection = m_nissanCanDefinitions.decodeVehicleShifter(*msg);
    }
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<crp::vil::NissanVehicleInfo>());
    rclcpp::shutdown();
    return 0;
}