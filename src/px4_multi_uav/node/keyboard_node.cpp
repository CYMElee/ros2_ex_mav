#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <getch.h>

enum MAV_mod {
    DISARM,
    IDLE, 
    TAKEOFF, 
    LAND, 
    SET_HOME, 
};

class MAV_State : public rclcpp::Node
{
private:
    // Subscribers for MAV vehicle status topics (using DDS-based /MAV{1~4}/fmu/out/vehicle_status)
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr MAV1_,MAV2_,MAV3_,MAV4_;


    // State variables for each MAV
    px4_msgs::msg::VehicleStatus MAV1_State_,MAV2_State_,MAV3_State_,MAV4_State_;

void MAV1_CB(const px4_msgs::msg::VehicleStatus::UniquePtr msg) {
        MAV1_State_ = *msg;
    }
    void MAV2_CB(const px4_msgs::msg::VehicleStatus::UniquePtr msg) {
        MAV2_State_ = *msg;
    }
    void MAV3_CB(const px4_msgs::msg::VehicleStatus::UniquePtr msg) {
        MAV3_State_ = *msg;
    }
    void MAV4_CB(const px4_msgs::msg::VehicleStatus::UniquePtr msg) {
        MAV4_State_ = *msg;
    }


public:



    MAV_State() : rclcpp::Node("mav_state_node")
    {
        // Initialize subscribers with QoS settings (reliable, keep last 10)
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
        MAV1_ = this->create_subscription<px4_msgs::msg::VehicleStatus>("/MAV1/fmu/out/vehicle_status", qos, std::bind(&MAV_State::MAV1_CB, this, std::placeholders::_1));
        MAV2_ = this->create_subscription<px4_msgs::msg::VehicleStatus>("/MAV2/fmu/out/vehicle_status", qos, std::bind(&MAV_State::MAV2_CB, this, std::placeholders::_1));
        MAV3_ = this->create_subscription<px4_msgs::msg::VehicleStatus>("/MAV3/fmu/out/vehicle_status", qos, std::bind(&MAV_State::MAV3_CB, this, std::placeholders::_1));
        MAV4_ = this->create_subscription<px4_msgs::msg::VehicleStatus>("/MAV4/fmu/out/vehicle_status", qos, std::bind(&MAV_State::MAV4_CB, this, std::placeholders::_1));
    }



    void Check_All_State(const std_msgs::msg::Int16& Mode)
    {
        // Check if all MAVs are armed using arming_state (assuming ARMING_STATE_ARMED == 2)
        if (MAV1_State_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED &&
            MAV2_State_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED &&
            MAV3_State_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED &&
            MAV4_State_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED &&
            Mode.data == (MAV_mod::IDLE)) {
            RCLCPP_INFO(this->get_logger(), "All_Mav_Get_Ready!!!");
        }
        if (Mode.data == MAV_mod::TAKEOFF) {
            RCLCPP_INFO(this->get_logger(), "All_Mav_Takeoff!!!");
            
        }
        if (Mode.data == MAV_mod::LAND) {
            RCLCPP_INFO(this->get_logger(), "All_Mav_Land!!!");
            
        }
        if (Mode.data == MAV_mod::SET_HOME) {
            RCLCPP_INFO(this->get_logger(), "SET_THE_PLATFORM_HOME_POSITION(keyboard)!!!");
           
        }
    }
};

int main(int argc, char** argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create node
    auto node = std::make_shared<rclcpp::Node>("keyboard_node");

    // Publisher for mode setting
    auto SET_MODE = node->create_publisher<std_msgs::msg::Int16>("/ground_station/set_mode", 10);

    // Create MAV_State instance
    auto mav_state = std::make_shared<MAV_State>();

    // Initialize mode
    std_msgs::msg::Int16 Mode;
    Mode.data = MAV_mod::IDLE;

    // Loop rate (100 Hz)
    rclcpp::Rate rate(100.0);
    int c_prev = EOF;

    while (rclcpp::ok())
    {
        // Check all MAV states
        mav_state->Check_All_State(Mode);

        // Get keyboard input
        int c = getch();

        if (c != EOF) {
            switch (c)
            {
            case 'T':
                Mode.data = MAV_mod::TAKEOFF;
                
                break;
            case 'L':
                Mode.data = MAV_mod::LAND;
                
                break;
            case 'S':
                Mode.data = MAV_mod::SET_HOME;
                
                break;
            }
            SET_MODE->publish(Mode);
        }
        c_prev = c;

        // Process callbacks
        rclcpp::spin_some(node);
        rate.sleep();
    }

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}