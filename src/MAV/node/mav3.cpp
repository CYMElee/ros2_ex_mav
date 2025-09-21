#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include <px4_msgs/msg/vehicle_command.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <Eigen/Dense>
#include <string>
#include <cmath>

enum MAV_mod {
    DISARM, //1
    IDLE, // 2
    TAKEOFF, // 3
    LAND, // 4
    SET_HOME, // 5
};

class MAVControl_Node : public rclcpp::Node {
public:
    MAVControl_Node() : Node("mav_control_node") {
        rmw_qos_profile_t qos_profile_pub = rmw_qos_profile_sensor_data;
        auto qos_pub = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_pub.history, 5), qos_profile_pub);
        rmw_qos_profile_t qos_profile_sub = rmw_qos_profile_sensor_data;
        qos_profile_sub.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
        auto qos_sub = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_sub.history, 5), qos_profile_sub);
        // Publishers
        T_pub_ = this->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
            "/MAV3/fmu/in/vehicle_attitude_setpoint", qos_pub);
        T_pub_debug_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/MAV3/fmu/in/vehicle_attitude_setpoint_euler", qos_pub);
        offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/MAV3/fmu/in/offboard_control_mode", qos_pub);
        vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/MAV3/fmu/in/vehicle_command", qos_pub);

        // Subscribers
        platform_pose_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/platform/measure_attitude", qos_pub,
            std::bind(&MAVControl_Node::platform_pose_cb, this, std::placeholders::_1));
        mav_pose_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
            "/MAV3/fmu/out/vehicle_attitude", qos_pub,
            std::bind(&MAVControl_Node::mav_pose, this, std::placeholders::_1));
        T_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/MAV3/cmd", qos_sub,
            std::bind(&MAVControl_Node::T_sub, this, std::placeholders::_1));
        takeoff_signal_sub_ = this->create_subscription<std_msgs::msg::Int16>(
            "/ground_station/set_mode", qos_pub,
            std::bind(&MAVControl_Node::mode_cb, this, std::placeholders::_1));
        offboard_setpoint_counter_ = 0;
        Change_Mode_Trigger_.data = MAV_mod::IDLE;
        
        timer2_ = this->create_wall_timer(std::chrono::milliseconds(10),std::bind(&MAVControl_Node::timer2_callback, this));
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&MAVControl_Node::timer_callback, this));
    }

private:
    geometry_msgs::msg::PoseStamped pose_;
    std_msgs::msg::Float64MultiArray platform_pose_;
    px4_msgs::msg::VehicleAttitude mav_pose_;
    std_msgs::msg::Float64MultiArray T_cmd_;
    std_msgs::msg::Float32MultiArray Eul_cmd_;
    std_msgs::msg::Int16 Change_Mode_Trigger_;
    px4_msgs::msg::VehicleAttitudeSetpoint T_;
    px4_msgs::msg::VehicleAttitudeSetpoint T_PREARM_;

    uint64_t offboard_setpoint_counter_;

    rclcpp::TimerBase::SharedPtr timer_, timer2_;
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr T_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr T_pub_debug_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr platform_pose_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr mav_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr T_sub_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr takeoff_signal_sub_;

    void initialize() {
        T_cmd_.data.resize(3);
        T_PREARM_.q_d[0] = 0; // w
        T_PREARM_.q_d[1] = 0; // x
        T_PREARM_.q_d[2] = 0; // y
        T_PREARM_.q_d[3] = 1; // z
        T_PREARM_.thrust_body[0] = 0.0;
        T_PREARM_.thrust_body[1] = 0.0;
        T_PREARM_.thrust_body[2] = -0.1; // Negative for upward thrust
        Eul_cmd_.data.resize(3);
        T_pub_->publish(T_PREARM_);
     
        Change_Mode_Trigger_.data = MAV_mod::IDLE;
    }

    void mode_cb(const std_msgs::msg::Int16::SharedPtr msg) {
        Change_Mode_Trigger_ = *msg;
    }

    void platform_pose_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        platform_pose_ = *msg;
    }



    void mav_pose(const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
        // Store original frd quaternion
        Eigen::Quaterniond q_frd(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
       

        Eigen::Quaterniond q_nwu;
        q_nwu.w() = q_frd.w();
        q_nwu.x() = q_frd.x();
        q_nwu.y() = -q_frd.y();
        q_nwu.z() = -q_frd.z();
    
        mav_pose_.q[0] = q_nwu.w();
        mav_pose_.q[1] = q_nwu.x();
        mav_pose_.q[2] = q_nwu.y();
        mav_pose_.q[3] = q_nwu.z();
    }

    void T_sub(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        T_cmd_ = *msg;
    }

    void T_cmd_calculate() {

       double alpha = T_cmd_.data[1];
       double beta = T_cmd_.data[2];
       double thrust = T_cmd_.data[0];

       // For testing ,the alpha beta we set as user input


        
       

        Eigen::Quaterniond MAV_pose_cmd;
        MAV_pose_cmd = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()) *
                       Eigen::AngleAxisd(beta, Eigen::Vector3d::UnitY()) *
                       Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitX());
        MAV_pose_cmd.normalize(); // Ensure normalization
        Eigen::Matrix3d rotationMatrix_mav_des_i2b = MAV_pose_cmd.toRotationMatrix(); // R_i2B desire

        Eigen::Quaterniond quaternion_platform(
            platform_pose_.data[0],
            platform_pose_.data[1],
            platform_pose_.data[2],
            platform_pose_.data[3]);
        quaternion_platform.normalize();
        Eigen::Matrix3d rotationMatrix_platform = quaternion_platform.toRotationMatrix(); // R_B2W 

        Eigen::Matrix3d rotationMatrix_mav_des_i2w = rotationMatrix_platform * rotationMatrix_mav_des_i2b;
        Eigen::Vector3d eulerAngles_mav_des = rotationMatrix_mav_des_i2w.eulerAngles(2, 1, 0);

        Eigen::Quaterniond mav_pose_desire(rotationMatrix_mav_des_i2w);
      
        mav_pose_desire.normalize(); 


        Eigen::Quaterniond mav_pose_desire_ned;

        mav_pose_desire_ned.w() = mav_pose_desire.w(); // trans the fram from nwu to ned
        mav_pose_desire_ned.x() = mav_pose_desire.x();
        mav_pose_desire_ned.y() = -mav_pose_desire.y();
        mav_pose_desire_ned.z() = -mav_pose_desire.z();



        T_.q_d[0] = mav_pose_desire_ned.w();
        T_.q_d[1] = mav_pose_desire_ned.x();
        T_.q_d[2] = mav_pose_desire_ned.y();
        T_.q_d[3] = mav_pose_desire_ned.z();
        T_.thrust_body[0] = 0.0;
        T_.thrust_body[1] = 0.0;
        //T_.thrust_body[2] = -thrust; // Negative for upward thrust
        T_.thrust_body[2] = -0.4;
        T_.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        T_pub_->publish(T_);

        //the  eulerAngles_mav_des is use to debug
        Eul_cmd_.data[0] = eulerAngles_mav_des(0);
        Eul_cmd_.data[1] = eulerAngles_mav_des(1);
        Eul_cmd_.data[2] = eulerAngles_mav_des(2);
        T_pub_debug_->publish(Eul_cmd_);
    }

    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0) {
        px4_msgs::msg::VehicleCommand msg{};
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = 3;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        vehicle_command_publisher_->publish(msg);
    }

    void publish_offboard_control_mode() {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.position = false;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = true;
        msg.body_rate = false;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_publisher_->publish(msg);
    }

    void arm() {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0);
        RCLCPP_INFO(this->get_logger(), "Arm command send");
    }

    void timer_callback() {
        if (offboard_setpoint_counter_ == 10) {
            // Change to Offboard mode after 10 setpoints
            this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            // Arm the vehicle
            this->arm();
            offboard_setpoint_counter_ = 12;
        }

        publish_offboard_control_mode();
        // stop the counter after reaching 11
        if (offboard_setpoint_counter_ < 11) {
            offboard_setpoint_counter_++;
        }
    }

    void timer2_callback() {
        if (offboard_setpoint_counter_ == 12) {
            if (Change_Mode_Trigger_.data == MAV_mod::IDLE) {
                initialize();
            }
            if (Change_Mode_Trigger_.data == MAV_mod::TAKEOFF) {
                T_cmd_calculate();
            }
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MAVControl_Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}