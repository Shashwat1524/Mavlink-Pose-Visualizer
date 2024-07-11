#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/node.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"


class visualizer_node : public rclcpp::Node 
{
public:
    visualizer_node() : Node("visualizer_node") 
    {

        auto qos =rclcpp::QoS(rclcpp::KeepLast(1))
                        .reliability(rclcpp::ReliabilityPolicy::BestEffort);

    
        
        vehicle_attitude_sub=this->create_subscription<px4_msgs::msg::VehicleAttitude>
            ("/fmu/out/vehicle_attitude",qos,std::bind(&visualizer_node::vehicle_attitude_callback, this,std::placeholders::_1));

        pose_pub=this->create_publisher<geometry_msgs::msg::PoseStamped>("vehicle_pose",10);
        RCLCPP_INFO(this->get_logger(),"Check: Publishing Data for Visualization");

    }

private:

    void vehicle_attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(),"Vehicle Atitude [%.2f, %.2f, %.2f, %.2f]",msg->q[0],msg->q[1],msg->q[2],msg->q[3]);

        auto pose_msg =geometry_msgs::msg::PoseStamped();
        pose_msg.header.frame_id="map";
        pose_msg.pose.orientation.w=msg->q[0];
        pose_msg.pose.orientation.x=msg->q[1];
        pose_msg.pose.orientation.y=msg->q[2];
        pose_msg.pose.orientation.z=msg->q[3];
        pose_pub->publish(pose_msg);

    }
    
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_sub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<visualizer_node>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}