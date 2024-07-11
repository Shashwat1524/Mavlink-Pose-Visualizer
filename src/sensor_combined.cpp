#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/node.hpp"
class VehicleOdometryToImu : public rclcpp::Node
{
public:
    VehicleOdometryToImu()
        : Node("vehicle_odometry_to_imu")
    {
   
        auto qos = rclcpp::QoS(rclcpp::KeepLast(50))
                        .reliability(rclcpp::ReliabilityPolicy::BestEffort)
                        .durability(rclcpp::DurabilityPolicy::Volatile)
                        .history(rclcpp::HistoryPolicy::KeepLast);

        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", qos);
        vehicle_odometry_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", qos,
            std::bind(&VehicleOdometryToImu::vehicleOdometryCallback, this, std::placeholders::_1));
    }

private:
    void vehicleOdometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        auto imu_msg = sensor_msgs::msg::Imu();

        imu_msg.header.stamp = this->get_clock()->now();
        imu_msg.header.frame_id = "base_link"; 


        imu_msg.linear_acceleration.x = msg->velocity[0];
        imu_msg.linear_acceleration.y = msg->velocity[1];
        imu_msg.linear_acceleration.z = msg->velocity[2];

        imu_msg.angular_velocity.x = msg->angular_velocity[0];
        imu_msg.angular_velocity.y = msg->angular_velocity[1];
        imu_msg.angular_velocity.z = msg->angular_velocity[2];

        imu_msg.orientation.x = 0.0;
        imu_msg.orientation.y = 0.0;
        imu_msg.orientation.z = 0.0;
        imu_msg.orientation.w = 1.0;

   
        imu_msg.orientation_covariance[0] = -1.0; 
        imu_msg.angular_velocity_covariance[0] = 0.1; 
        imu_msg.linear_acceleration_covariance[0] = 0.1; 


        imu_publisher_->publish(imu_msg);

        RCLCPP_INFO(this->get_logger(), "Linear Accel: [%.2f, %.2f, %.2f], Angular Vel: [%.2f, %.2f, %.2f]",
                    imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z,
                    imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z);
    }

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_subscriber_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VehicleOdometryToImu>());
    rclcpp::shutdown();
    return 0;
}
