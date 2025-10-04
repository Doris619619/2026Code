#ifndef UP_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#define UP_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

#include "rm_serial_ext/crc.hpp"
#include "rm_serial_ext/packet.hpp"
#include "rm_serial_ext/serial_driver.hpp"
// ros2
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rm_decision_interfaces/msg/all_robot_hp.hpp>
#include <rm_decision_interfaces/msg/detail/all_robot_hp__struct.hpp>
#include <rm_decision_interfaces/msg/detail/game_status__struct.hpp>
#include <rm_decision_interfaces/msg/detail/robot_status__struct.hpp>
#include <rm_decision_interfaces/msg/detail/test_msg__struct.hpp>
#include <rm_decision_interfaces/msg/game_status.hpp>
#include <rm_decision_interfaces/msg/robot_status.hpp>
#include <rm_decision_interfaces/msg/test_msg.hpp>

namespace ext_serial_driver
{
    class UPSerialDriver : public rclcpp::Node
    {
    public:
        explicit UPSerialDriver(const rclcpp::NodeOptions &options);

    private:
        void receiveData();
        void sendNavData(const geometry_msgs::msg::Twist msg);
        void sendRCData(const rm_decision_interfaces::msg::TestMsg msg);
        void setParam(const rclcpp::Parameter &param);
        bool getDetectColor(uint8_t robot_id, uint8_t & color);

        // Serial port
        std::unique_ptr<Port> port_;
        // nav and RC system
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
        rclcpp::Subscription<rm_decision_interfaces::msg::TestMsg>::SharedPtr test_msg_sub_;
        // Transmit referee system
        rclcpp::Publisher<rm_decision_interfaces::msg::AllRobotHP>::SharedPtr all_robot_hp_pub_;
        rclcpp::Publisher<rm_decision_interfaces::msg::RobotStatus>::SharedPtr robot_status_pub_;
        rclcpp::Publisher<rm_decision_interfaces::msg::GameStatus>::SharedPtr game_status_pub_;
        // Send and Receive packet (Referee system)
        rm_decision_interfaces::msg::AllRobotHP all_robot_hp_;
        rm_decision_interfaces::msg::RobotStatus robot_status_;
        rm_decision_interfaces::msg::GameStatus game_status_;
    
        // Param client to set detect_color
        using ResultFuturePtr = std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;
        bool initial_set_param_ = false;
        uint8_t previous_receive_color_ = 0;
        rclcpp::AsyncParametersClient::SharedPtr detector_param_client_;
        ResultFuturePtr set_param_future_;
        std::string detector_node_name_;
    };
}

#endif // UP_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_