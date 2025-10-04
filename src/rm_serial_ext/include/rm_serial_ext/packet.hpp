#ifndef EXT_SERIAL_DRIVER__PACKET_HPP_
#define EXT_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace ext_serial_driver
{
	// up packet
	struct UpReceivePacket
	{
		uint8_t header = 0x22;
		// float quat01;//IMU姿态四元数
		// float quat02;
		// float quat03;
		// float quat04;
		// float gyro01;//角加速度
		// float gyro02;
		// float gyro03;
		// float accel01;//线加速度
		// float accel02;
		// float accel03;

		uint16_t red_1_robot_hp;
		uint16_t red_2_robot_hp;
		uint16_t red_3_robot_hp;
		uint16_t red_4_robot_hp;
		uint16_t red_5_robot_hp;
		uint16_t red_7_robot_hp;
		uint16_t red_outpost_hp;
		uint16_t red_base_hp;
		uint16_t blue_1_robot_hp;
		uint16_t blue_2_robot_hp;
		uint16_t blue_3_robot_hp;
		uint16_t blue_4_robot_hp;
		uint16_t blue_5_robot_hp;
		uint16_t blue_7_robot_hp;
		uint16_t blue_outpost_hp;
		uint16_t blue_base_hp;

		uint8_t game_progress;
		uint16_t stage_remain_time;

		uint8_t robot_id;
		uint16_t current_hp;
		uint16_t shooter_heat;
		bool team_color;
		bool is_attacked;

		uint16_t chassis_power_limit;
		uint16_t shooter_barrel_heat_limit;
		uint16_t shooter_17mm_1_barrel_heat;
		float chassis_power;
		uint16_t chassis_power_buffer;

		float x;
		float y;
		float angle;

		uint32_t event_data;

		uint16_t checksum = 0;
	} __attribute__((packed));

	struct NavSendPacket
	{
		uint8_t header = 0x11;

		float linear_vx;
		float linear_vy;
		float linear_vz;

		float angular_x;
		float angular_y;
		float angular_z;

		uint16_t checksum = 0;
	} __attribute__((packed));

	struct RCSendPacket
	{
		uint8_t header = 0x12;

		float linear_vx;
		float linear_vy;
		float linear_vw;
		uint8_t mode;

		uint16_t checksum = 0;
	} __attribute__((packed));

	// inline UpReceivePacket fromUpVector(const std::vector<uint8_t> &data)
	// {
	// 	UpReceivePacket packet;
	// 	std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
	// 	return packet;
	// }

	// inline RCSendPacket fromVector(const std::vector<uint8_t> &data)
	// {
	// 	RCSendPacket packet;
	// 	std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
	// 	return packet;
	// }

	// inline std::vector<uint8_t> toVector(const NavSendPacket &data)
	// {
	// 	std::vector<uint8_t> packet(sizeof(NavSendPacket));
	// 	std::copy(
	// 		reinterpret_cast<const uint8_t *>(&data),
	// 		reinterpret_cast<const uint8_t *>(&data) + sizeof(NavSendPacket), packet.begin());
	// 	return packet;
	// }

	// inline std::vector<uint8_t> toVector(const RCSendPacket &data)
	// {
	// 	std::vector<uint8_t> packet(sizeof(RCSendPacket));
	// 	std::copy(
	// 		reinterpret_cast<const uint8_t *>(&data),
	// 		reinterpret_cast<const uint8_t *>(&data) + sizeof(RCSendPacket), packet.begin());
	// 	return packet;
	// }

	// down packet
	struct DownReceivePacket
	{
		uint8_t header = 0x5A;
		uint8_t detect_color : 1; // 0-red 1-blue
		bool reset_tracker : 1;
		uint8_t reserved : 6;

		float yaw;    // rad
		float pitch;  // rad
		float roll;   // rad
		float yaw_odom;
		float pitch_odom;
		float yaw_vel;    // rad/s
		float pitch_vel;  // rad/s
		float roll_vel;   // rad/s	

		float aim_x; // 瞄准的点的位置
		float aim_y;
		float aim_z;

		float linear_vx;
		float linear_vy;
		float linear_vw;
		
		uint8_t mode;
		uint16_t checksum = 0;
	} __attribute__((packed));

	struct SendPacket
	{
		uint8_t header = 0xA5;
		bool tracking : 1;		// 0-not detect, 1-detect target
		// uint8_t id : 3;			// 0-outpost 6-guard 7-base
		// uint8_t armors_num : 3; // 2-balance 3-outpost 4-normal
		// uint8_t reserved : 1;
		// float x; // target position
		// float y;
		// float z;
		// float yaw; // armor targeting angle
		// float vx;  // target velocity
		// float vy;
		// float vz;
		// float v_yaw;
		// float r1; // 椭圆的长轴和短轴
		// float r2;s
		// float dz;
		float pitch;
		float yaw;
		uint8_t fire;
		uint8_t fric_on;

		uint16_t checksum = 0; // CRC校验的冗余码

	} __attribute__((packed));

	// inline DownReceivePacket fromDownVector(const std::vector<uint8_t> &data)
	// {
	// 	DownReceivePacket packet;
	// 	std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
	// 	return packet;
	// }

	// inline std::vector<uint8_t> toVector(const SendPacket &data)
	// {
	// 	std::vector<uint8_t> packet(sizeof(SendPacket));
	// 	std::copy(
	// 		reinterpret_cast<const uint8_t *>(&data),
	// 		reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet.begin());
	// 	return packet;
	// }

	// ext packet
	struct EXTReceivePacket
	{
		uint8_t header = 0x68;

		uint8_t robot_id;

		uint16_t checksum = 0;
	} __attribute__((packed));

	/********************************************************/
	/* template                                             */
	/********************************************************/

	template <typename T>
	inline T fromVector(const std::vector<uint8_t> &data)
	{
		T packet;
		std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
		return packet;
	}

	template <typename T>
	inline std::vector<uint8_t> toVector(const T &data)
	{
		std::vector<uint8_t> packet(sizeof(T));
		std::copy(
			reinterpret_cast<const uint8_t *>(&data), reinterpret_cast<const uint8_t *>(&data) + sizeof(T),
			packet.begin());
		return packet;
	}

} // namespace rm_serial_driver

#endif // RM_SERIAL_DRIVER__PACKET_HPP_
