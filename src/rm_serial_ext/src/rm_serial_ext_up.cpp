#include "rm_serial_ext/rm_serial_ext_up.hpp"

namespace ext_serial_driver
{
    UPSerialDriver::UPSerialDriver(const rclcpp::NodeOptions &options)
        : Node("rm_serial_ext_up", options), port_{new Port(2)}
    {
        RCLCPP_INFO(get_logger(), "Start UPSerialDriver!");

        port_->getParams("/dev/ttyACM0", 115200, "none", "none", "1");

        all_robot_hp_pub_ =
            this->create_publisher<rm_decision_interfaces::msg::AllRobotHP>("/all_robot_hp", 3);
        robot_status_pub_ =
            this->create_publisher<rm_decision_interfaces::msg::RobotStatus>("/robot_status", 10);
        game_status_pub_ =
            this->create_publisher<rm_decision_interfaces::msg::GameStatus>("/game_status", 1);
        try
        {
            port_->serial_driver_->init_port(port_->device_name_, *port_->device_config_);
            if (!port_->serial_driver_->port()->is_open())
            {
                port_->serial_driver_->port()->open();
                port_->receive_thread_ = std::thread(&UPSerialDriver::receiveData, this);
                // LOG
                RCLCPP_INFO(get_logger(), "serial open OK!");
            }
        }
        catch (const std::exception &ex)
        {
            RCLCPP_ERROR(
                get_logger(), "Error creating serial port: %s - %s", port_->device_name_.c_str(), ex.what());
            throw ex;
        }

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 100,
            std::bind(&UPSerialDriver::sendNavData, this, std::placeholders::_1));
        test_msg_sub_ = this->create_subscription<rm_decision_interfaces::msg::TestMsg>(
            "/test_msg", 100,
            std::bind(&UPSerialDriver::sendRCData, this, std::placeholders::_1));
    }

    void UPSerialDriver::receiveData()
    {
        std::vector<uint8_t> header(1); // unsigned int (16)
        std::vector<uint8_t> data;
        data.reserve(sizeof(UpReceivePacket));

        while (rclcpp::ok())
        {
            try
            {
                // RCLCPP_INFO(get_logger(), "[Receive] receive_header %u!", header[0]);
                port_->serial_driver_->port()->receive(header);
                if (header[0] == 0x22)
                {
                    data.resize(sizeof(UpReceivePacket) - 1);
                    port_->serial_driver_->port()->receive(data);
                    data.insert(data.begin(), header[0]);
                    UpReceivePacket packet = fromVector<UpReceivePacket>(data);

                    bool crc_ok =
                        crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
                    if (crc_ok)
                    {
                        // RCLCPP_INFO(get_logger(), "CRC OK!");
                        // receive all robot hp
                        all_robot_hp_.red_1_robot_hp = packet.red_1_robot_hp;
                        all_robot_hp_.red_2_robot_hp = packet.red_2_robot_hp;
                        all_robot_hp_.red_3_robot_hp = packet.red_3_robot_hp;
                        all_robot_hp_.red_4_robot_hp = packet.red_4_robot_hp;
                        all_robot_hp_.red_5_robot_hp = packet.red_5_robot_hp;
                        all_robot_hp_.red_7_robot_hp = packet.red_7_robot_hp;
                        all_robot_hp_.red_base_hp = packet.red_base_hp;
                        all_robot_hp_.red_outpost_hp = packet.red_outpost_hp;
                        all_robot_hp_.blue_1_robot_hp = packet.blue_1_robot_hp;
                        all_robot_hp_.blue_2_robot_hp = packet.blue_2_robot_hp;
                        all_robot_hp_.blue_3_robot_hp = packet.blue_3_robot_hp;
                        all_robot_hp_.blue_4_robot_hp = packet.blue_4_robot_hp;
                        all_robot_hp_.blue_5_robot_hp = packet.blue_5_robot_hp;
                        all_robot_hp_.blue_7_robot_hp = packet.blue_7_robot_hp;
                        all_robot_hp_.blue_base_hp = packet.blue_base_hp;
                        all_robot_hp_.blue_outpost_hp = packet.blue_outpost_hp;

                        all_robot_hp_pub_->publish(all_robot_hp_);
                        // receive game status
                        game_status_.game_progress = packet.game_progress;
                        game_status_.stage_remain_time = packet.stage_remain_time;

                        game_status_pub_->publish(game_status_);
                        // receive robot status
                        robot_status_.robot_id = packet.robot_id;
                        robot_status_.current_hp = packet.current_hp;
                        robot_status_.shooter_heat = packet.shooter_heat;
                        robot_status_.team_color = packet.team_color;
                        robot_status_.is_attacked = packet.is_attacked;

                        robot_status_pub_->publish(robot_status_);

                        uint8_t detect_color;
                        if (getDetectColor(robot_status_.robot_id, detect_color)) {
                          if (!initial_set_param_ || detect_color != previous_receive_color_) {
                            previous_receive_color_ = detect_color;
                            setParam(rclcpp::Parameter("detect_color", detect_color));
                            std::this_thread::sleep_for(std::chrono::milliseconds(500));
                          }
                        }
                    }
                    else
                    {
                        RCLCPP_ERROR(get_logger(), "CRC error!");
                    }
                }
                else
                {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "Invalid header: %02X", header[0]);
                }
            }
            catch (const std::exception &ex)
            {
                RCLCPP_ERROR_THROTTLE(
                    get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
                port_->reopenPort();
            }
        }
    }

    void UPSerialDriver::sendNavData(const geometry_msgs::msg::Twist msg)
    {
        try
        {
            NavSendPacket packet;

            packet.linear_vx = msg.linear.x;
            packet.linear_vy = msg.linear.y;
            packet.linear_vz = msg.linear.z;
            // packet.angular_x = msg.angular.x;
            // packet.angular_y = msg.angular.y;
            // packet.angular_z = msg.angular.z;

            RCLCPP_INFO(get_logger(), "linearvx %f", msg.linear.x);
            RCLCPP_INFO(get_logger(), "linearvy %f", msg.linear.y);
            RCLCPP_INFO(get_logger(), "linearvz %f", msg.linear.z);
            // RCLCPP_INFO(get_logger(), "angularvx %f", msg.angular.x);
            // RCLCPP_INFO(get_logger(), "angularvy %f", msg.angular.y);
            // RCLCPP_INFO(get_logger(), "angularvz %f", msg.angular.z);

            crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
            std::vector<uint8_t> data = toVector(packet);
            port_->serial_driver_->port()->send(data);
        }
        catch (const std::exception &ex)
        {
            RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
            port_->reopenPort();
        }
    }

    void UPSerialDriver::sendRCData(const rm_decision_interfaces::msg::TestMsg msg)
    {
        try
        {
            RCSendPacket packet;

            packet.linear_vx = msg.linear_vx;
            packet.linear_vy = msg.linear_vy;
            packet.linear_vw = msg.linear_vw;
            packet.mode = msg.mode;

            crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
            std::vector<uint8_t> data = toVector(packet);
            port_->serial_driver_->port()->send(data);
        }
        catch (const std::exception &ex)
        {
            RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
            port_->reopenPort();
        }
    }
    void UPSerialDriver::setParam(const rclcpp::Parameter & param)
    {
      if (!initial_set_param_) {
        auto node_graph = this->get_node_graph_interface();
        auto node_names = node_graph->get_node_names();
        std::vector<std::string> possible_detectors = {
          "armor_detector_openvino", "armor_detector_opencv"};
    
        for (const auto & name : possible_detectors) {
          for (const auto & node_name : node_names) {
            if (node_name.find(name) != std::string::npos) {
              detector_node_name_ = node_name;
              break;
            }
          }
          if (!detector_node_name_.empty()) {
            break;
          }
        }
    
        if (detector_node_name_.empty()) {
          RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 1000, "No detector node found!");
          return;
        }
    
        detector_param_client_ =
          std::make_shared<rclcpp::AsyncParametersClient>(this, detector_node_name_);
        if (!detector_param_client_->service_is_ready()) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *this->get_clock(), 1000, "Service not ready, skipping parameter set");
          return;
        }
      }
    
      if (
        !set_param_future_.valid() ||
        set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        RCLCPP_INFO(get_logger(), "Setting detect_color to %ld...", param.as_int());
        set_param_future_ = detector_param_client_->set_parameters(
          {param}, [this, param](const ResultFuturePtr & results) {
            for (const auto & result : results.get()) {
              if (!result.successful) {
                RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
                return;
              }
            }
            RCLCPP_INFO(get_logger(), "Successfully set detect_color to %ld!", param.as_int());
            initial_set_param_ = true;
          });
      }
    }
    bool UPSerialDriver::getDetectColor(uint8_t robot_id, uint8_t & color)
    {
      if (robot_id == 0 || (robot_id > 11 && robot_id < 101)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *this->get_clock(), 1000, "Invalid robot ID: %d. Color not set.", robot_id);
        return false;
      }
      color = (robot_id >= 100) ? 0 : 1;
      return true;
    }

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(ext_serial_driver::UPSerialDriver)