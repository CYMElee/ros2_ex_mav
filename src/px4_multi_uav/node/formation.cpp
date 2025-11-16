#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <string>
#include <vector>

class Formation_Node : public rclcpp::Node {
public:
    Formation_Node() : Node("formation_node") {
        // ---------- QoS ----------
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        rmw_qos_profile_t qos_profile_sub = rmw_qos_profile_sensor_data;
        qos_profile_sub.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
        auto qos_sub = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_sub.history, 5), qos_profile_sub);

        // ---------- Publishers ----------
        mav1_cmd_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/MAV1/cmd", qos_sub);
        mav2_cmd_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/MAV2/cmd", qos_sub);
        mav3_cmd_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/MAV3/cmd", qos_sub);
        mav4_cmd_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/MAV4/cmd", qos_sub);

        // ---------- Subscribers ----------
        virtual_pose_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/virtual_leader/state", qos,
            std::bind(&Formation_Node::virtual_pose_cb, this, std::placeholders::_1));

        mav1_pose_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/MAV1/fmu/out/vehicle_local_position_v1", qos,
            std::bind(&Formation_Node::mav1_pose_cb, this, std::placeholders::_1));

        mav2_pose_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/MAV2/fmu/out/vehicle_local_position_v1", qos,
            std::bind(&Formation_Node::mav2_pose_cb, this, std::placeholders::_1));

        mav3_pose_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/MAV3/fmu/out/vehicle_local_position_v1", qos,
            std::bind(&Formation_Node::mav3_pose_cb, this, std::placeholders::_1));

        mav4_pose_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/MAV4/fmu/out/vehicle_local_position_v1", qos,
            std::bind(&Formation_Node::mav4_pose_cb, this, std::placeholders::_1));

        // ---------- Timer ----------
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),  // 50 Hz
            std::bind(&Formation_Node::timer_callback, this));

        // 初始化 vector
        mav_z_heights_ = std::vector<float>(4, 0.0f);
        mav1_cmd.data.resize(3);
        mav2_cmd.data.resize(3);
        mav3_cmd.data.resize(3);
        mav4_cmd.data.resize(3);
    }

private:
    // ----- 狀態變數 -----
    px4_msgs::msg::VehicleLocalPosition mav1_pose_, mav2_pose_, mav3_pose_, mav4_pose_;
    float mav1_z = 0.0f, mav2_z = 0.0f, mav3_z = 0.0f, mav4_z = 0.0f;
    float mav1_vel_d = 0.0f, mav2_vel_d = 0.0f, mav3_vel_d = 0.0f, mav4_vel_d = 0.0f;
    float virtual_pose = 0.0f, virtual_vel = 0.0f;

    // 使用 vector 管理高度
    std::vector<float> mav_z_heights_;
    std_msgs::msg::Float64MultiArray mav1_cmd,mav2_cmd,mav3_cmd,mav4_cmd;


    // ----- Publishers -----
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr mav1_cmd_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr mav2_cmd_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr mav3_cmd_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr mav4_cmd_pub;

    // ----- Subscribers -----
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr virtual_pose_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr mav1_pose_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr mav2_pose_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr mav3_pose_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr mav4_pose_sub_;

    // ----- Timer -----
    rclcpp::TimerBase::SharedPtr timer_;

    // -------------------- Callbacks --------------------
    void mav1_pose_cb(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        mav1_pose_ = *msg;
        mav1_z = -(msg->z);
    }
    void mav2_pose_cb(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        mav2_pose_ = *msg;
        mav2_z = -(msg->z);
    }
    void mav3_pose_cb(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        mav3_pose_ = *msg;
        mav3_z = -(msg->z);
    }
    void mav4_pose_cb(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        mav4_pose_ = *msg;
        mav4_z = -(msg->z);
    }

    void virtual_pose_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 2) {
            virtual_pose = msg->data[0];
            virtual_vel  = msg->data[1];
        }
    }

    // -------------------- 編隊控制（公式完全不變） --------------------
    void formation() {
        // 填入 vector
        mav_z_heights_[0] = mav1_z;
        mav_z_heights_[1] = mav2_z;
        mav_z_heights_[2] = mav3_z;
        mav_z_heights_[3] = mav4_z;

        // **完全保留原始公式**
        //mav1_vel_d = 1*(-(virtual_pose - mav1_z))  - virtual_vel;
        //mav3_vel_d = 1*(-(virtual_pose - mav3_z)) - virtual_vel;

        mav1_vel_d = 1*((-(virtual_pose - mav1_z)) + ((mav1_z - mav2_z) + (mav1_z - mav3_z) + (mav1_z - mav4_z))) - virtual_vel;
        mav2_vel_d = 1*((-(virtual_pose - mav2_z)) + ((mav2_z - mav1_z) + (mav2_z - mav3_z) + (mav2_z - mav4_z))) - virtual_vel;
        mav3_vel_d = 1*((-(virtual_pose - mav3_z)) + ((mav3_z - mav1_z) + (mav3_z - mav2_z) + (mav3_z - mav4_z))) - virtual_vel;
        mav4_vel_d = 1*((-(virtual_pose - mav4_z)) + ((mav4_z - mav1_z) + (mav4_z - mav2_z) + (mav4_z - mav3_z))) - virtual_vel;
        //RCLCPP_INFO(this->get_logger(), "Receive Virtual Pose %f", virtual_pose);
        //RCLCPP_INFO(this->get_logger(), "Receive MAV Pose,MAV1: %f,MAV2: %f,MAV3: %f,MAV4: %f",mav1_z ,mav2_z,mav3_z,mav4_z);
        RCLCPP_INFO(this->get_logger(), "Z position,MAV1: %f,MAV2: %f,MAV3: %f,MAV4: %f \n",mav1_z ,mav2_z,mav3_z,mav4_z);
        RCLCPP_INFO(this->get_logger(), "v cmd,MAV1: %f,MAV2: %f,MAV3: %f,MAV4: %f \n",mav1_vel_d ,mav2_vel_d,mav3_vel_d,mav4_vel_d);   
   
    }

    // -------------------- 發布速度指令 --------------------
    void publish_velocity_commands() {

        mav1_cmd.data[0] = 0;
        mav1_cmd.data[1] = 0;
        mav1_cmd.data[2] = mav1_vel_d;

        mav2_cmd.data[0] = 0;
        mav2_cmd.data[1] = 0;
        mav2_cmd.data[2] = mav2_vel_d;   
        
        mav3_cmd.data[0] = 0;
        mav3_cmd.data[1] = 0;
        mav3_cmd.data[2] = mav3_vel_d;        

        mav4_cmd.data[0] = 0;
        mav4_cmd.data[1] = 0;
        mav4_cmd.data[2] = mav4_vel_d;

        mav1_cmd_pub->publish(mav1_cmd);
        mav2_cmd_pub->publish(mav2_cmd);
        mav3_cmd_pub->publish(mav3_cmd);
        mav4_cmd_pub->publish(mav4_cmd);
    }

    // -------------------- Timer --------------------
    void timer_callback() {
        formation();               // 計算速度
        publish_velocity_commands(); // 發布


    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<Formation_Node>());
    rclcpp::shutdown();
    return 0;
}