#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

#include <cmath>
#include <vector>

enum MAV_mod {
    DISARM, 
    IDLE, 
    TAKEOFF, 
    LAND, 
    SET_HOME, 
};
class MAVControl_Node : public rclcpp::Node {
public:
    MAVControl_Node() : Node("mav_control_node") {
        // ---------- QoS ----------
        rmw_qos_profile_t qos_profile_pub = rmw_qos_profile_sensor_data;
        auto qos_pub = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_pub.history, 5), qos_profile_pub);

        rmw_qos_profile_t qos_profile_sub = rmw_qos_profile_sensor_data;
        qos_profile_sub.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
        auto qos_sub = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_sub.history, 5), qos_profile_sub);

        // ---------- Publishers ----------
        offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/MAV4/fmu/in/offboard_control_mode", qos_pub);

        trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/MAV4/fmu/in/trajectory_setpoint", qos_pub);

        vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/MAV4/fmu/in/vehicle_command", qos_pub);



        // ---------- Subscribers ----------
        cmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/MAV4/cmd", qos_sub,
            std::bind(&MAVControl_Node::cmd_cb, this, std::placeholders::_1));

        mode_sub_ = this->create_subscription<std_msgs::msg::Int16>(
            "/ground_station/set_mode", qos_pub,
            std::bind(&MAVControl_Node::mode_cb, this, std::placeholders::_1));

        // ---------- Variables ----------
        offboard_setpoint_counter_ = 0;
        Change_Mode_Trigger_.data = MAV_mod::IDLE;

        // Default velocity (zero)
        velocity_cmd_ = {0.0, 0.0, 0.0};  // vx, vy, vz in NED

        // ---------- Timers ----------
        timer_  = this->create_wall_timer(std::chrono::milliseconds(100),  // 10 Hz handshake
                                          std::bind(&MAVControl_Node::timer_callback, this));
        timer2_ = this->create_wall_timer(std::chrono::milliseconds(20),   // 50 Hz setpoint
                                          std::bind(&MAVControl_Node::timer2_callback, this));
    }

private:
    // ----- Msg containers -----
    std_msgs::msg::Float64MultiArray cmd_;             // [vx, vy, vz]
    std_msgs::msg::Int16 Change_Mode_Trigger_;

    // ----- Velocity storage -----
    std::vector<double> velocity_cmd_;  // vx, vy, vz (NED)

    // ----- Publishers -----
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;

    // ----- Subscribers -----
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr mode_sub_;

    // ----- Timers -----
    rclcpp::TimerBase::SharedPtr timer_, timer2_;

    uint64_t offboard_setpoint_counter_;

    // ------------------------------------------------------------
    void cmd_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {

        cmd_ = *msg;
        velocity_cmd_[0] = cmd_.data[0];  // vx
        velocity_cmd_[1] = cmd_.data[1];  // vy
        velocity_cmd_[2] = cmd_.data[2];  // vz
    }

    void mode_cb(const std_msgs::msg::Int16::SharedPtr msg) {
        Change_Mode_Trigger_ = *msg;
    }

    // ------------------------------------------------------------
    void publish_vehicle_command(uint16_t command, float param1 = 0.0f, float param2 = 0.0f) {
        px4_msgs::msg::VehicleCommand msg{};
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = 4;
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
        msg.velocity = true;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_publisher_->publish(msg);
    }

    void arm() {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
        RCLCPP_INFO(this->get_logger(), "Arm command sent");
    }

    // ------------------------------------------------------------
    void publish_zero_velocity_setpoint() {
        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

        msg.position[0] = std::nan("");
        msg.position[1] = std::nan("");
        msg.position[2] = std::nan("");

        msg.velocity[0] = 0.0f;
        msg.velocity[1] = 0.0f;
        msg.velocity[2] = 0.0f;

        msg.acceleration[0] = std::nan("");
        msg.acceleration[1] = std::nan("");
        msg.acceleration[2] = std::nan("");

        msg.yaw = std::nan("");   // Keep initial yaw
        msg.yawspeed = 0.0f;

        trajectory_setpoint_publisher_->publish(msg);
    }

    // ------------------------------------------------------------
    void velocity_cmd_calculate() {
        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

        // Position: NaN → 不控制位置
        msg.position[0] = std::nan("");
        msg.position[1] = std::nan("");
        msg.position[2] = std::nan("");

        msg.velocity[0] = static_cast<float>(velocity_cmd_[0]);
        msg.velocity[1] = static_cast<float>(velocity_cmd_[1]);
        msg.velocity[2] = static_cast<float>(velocity_cmd_[2]);

        // Acceleration: NaN
        msg.acceleration[0] = std::nan("");
        msg.acceleration[1] = std::nan("");
        msg.acceleration[2] = std::nan("");

        // Yaw: 固定為 NaN → 保持起飛時的初始 yaw
        msg.yaw = std::nan("");
        msg.yawspeed = 0.0f;

        trajectory_setpoint_publisher_->publish(msg);


    }

    // ------------------------------------------------------------
    void timer_callback() {
        if (offboard_setpoint_counter_ == 10) {
            publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f); // Offboard
            this->arm();
            offboard_setpoint_counter_ = 12;
        }

        publish_offboard_control_mode();

        if (offboard_setpoint_counter_ < 11) {
            publish_zero_velocity_setpoint();
            offboard_setpoint_counter_++;
        }
    }

    void timer2_callback() {
        if (offboard_setpoint_counter_ != 12) return;

        if (Change_Mode_Trigger_.data == MAV_mod::IDLE) {
            publish_zero_velocity_setpoint();
        }
        else if (Change_Mode_Trigger_.data == MAV_mod::TAKEOFF) {
            velocity_cmd_calculate(); 
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MAVControl_Node>());
    rclcpp::shutdown();
    return 0;
}