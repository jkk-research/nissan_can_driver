#include "nissan_can_driver/nissanVehicleInfo.hpp"

crp::vil::NissanVehicleInfo::NissanVehicleInfo() : Node("nissan_vehicle_info")
{
    m_pub_vehicleSpeed_     = this->create_publisher<std_msgs::msg::Float32>("vehicle_speed", 10);
    m_pub_vehicleSteering_  = this->create_publisher<std_msgs::msg::Float32>("vehicle_steering", 10);
    m_pub_vehicleTireAngle_ = this->create_publisher<std_msgs::msg::Float32>("vehicle_tire_angle", 10);

    m_sub_can_ = this->create_subscription<can_msgs::msg::Frame>(
        "can_tx", 10, std::bind(&NissanVehicleInfo::canCallback, this, std::placeholders::_1));
}


void crp::vil::NissanVehicleInfo::canCallback(const can_msgs::msg::Frame::SharedPtr msg)
{
    if (msg->id == m_nissanCanDefinitions.m_VEHICLE_SPEED_ID)
    {
        std_msgs::msg::Float32 vehicleSpeed;
        vehicleSpeed.data = m_nissanCanDefinitions.decodeVehicleSpeed(*msg);
        m_pub_vehicleSpeed_->publish(vehicleSpeed);
    }
    else if (msg->id == m_nissanCanDefinitions.m_VEHICLE_STEERING_ID)
    {
        std_msgs::msg::Float32 vehicleSteering;
        vehicleSteering.data = m_nissanCanDefinitions.decodeVehicleSteering(*msg) * 3.14 / 180.0; // to rad
        m_pub_vehicleSteering_->publish(vehicleSteering);

        std_msgs::msg::Float32 tireAngle;
        tireAngle.data = (-vehicleSteering.data * 0.00547944); // to tire angle
        m_pub_vehicleTireAngle_->publish(tireAngle);
        
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