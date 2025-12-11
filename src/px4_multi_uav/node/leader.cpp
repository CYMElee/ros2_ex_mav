#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int16.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <chrono>

enum MAV_mod {
    DISARM, 
    IDLE, 
    TAKEOFF, 
    LAND, 
    SET_HOME, 
};

class VirtualLeaderNode : public rclcpp::Node {
public:
    VirtualLeaderNode() : Node("virtual_leader_node"){
        // ---------- QoS ----------
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        // ---------- Publisher ----------
        state_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/virtual_leader/state", qos);

        // ---------- Subscriber: Mode ----------
        mode_sub_ = this->create_subscription<std_msgs::msg::Int16>(
            "/ground_station/set_mode", qos,
            std::bind(&VirtualLeaderNode::mode_cb, this, std::placeholders::_1));


        timer_  = this->create_wall_timer(std::chrono::milliseconds(10), 
                                          std::bind(&VirtualLeaderNode::timer_callback, this));
        // ---------- 初始模式 ----------
        Change_Mode_Trigger_.data = MAV_mod::IDLE;
        }

private:
    // ----- Publishers -----
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr state_pub_;

    // ----- Subscribers -----
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr mode_sub_;

    // ----- Timer -----
    rclcpp::TimerBase::SharedPtr timer_;

    // ----- State -----
    std_msgs::msg::Int16 Change_Mode_Trigger_;
    float virtual_pos_ = 0.0f;        // 當前虛擬位置

    // -------------------- Mode Callback --------------------
    void mode_cb(const std_msgs::msg::Int16::SharedPtr msg) {
        Change_Mode_Trigger_ = *msg;
    }
    

    // -------------------- Timer Callback (100 Hz) --------------------
    void timer_callback() {
        auto msg = std_msgs::msg::Float32MultiArray();
        msg.data.resize(2);
        if (Change_Mode_Trigger_.data == MAV_mod::TAKEOFF) {
            // 使用 dt = 0.01 累計位置

            msg.data[0] = 0.6;
            msg.data[1] = 0;
            state_pub_->publish(msg);
        } 
            
        else if (Change_Mode_Trigger_.data == MAV_mod::LAND) {
            // 使用 dt = 0.01 累計位置
            msg.data[0] = 0.1;
            msg.data[1] = 0;
            state_pub_->publish(msg);
        }
        else{
            // 使用 dt = 0.01 累計位置
            msg.data[0] = 0.1;
            msg.data[1] = 0;
            state_pub_->publish(msg);
        }
        
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VirtualLeaderNode>());
    rclcpp::shutdown();
    return 0;
}