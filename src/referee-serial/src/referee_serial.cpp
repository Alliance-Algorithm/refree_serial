#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/byte.hpp"
#include "std_msgs/msg/float32.hpp"
#include <cstdint>
#include <iostream>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <serial/serial.h>
#include <std_msgs/msg/detail/bool__struct.hpp>
#include <std_msgs/msg/detail/float32_multi_array__struct.hpp>
#include <string>
/*
 * 串口数据格式
 * 帧头(5字节) + cmd id(2字节) + 数据(10字节) + CRC16(2字节)
 * 帧头: SOF(1字节) + 数据长度(2字节) + 包序号(1字节) + CRC8(1字节)
 * 数据: 目标机器人ID(2字节) + 目标机器人x坐标(4字节) + 目标机器人y坐标(4字节)
 * 帧尾: CRC16(2字节)
 */

typedef struct frame_header_t
{
	uint16_t data_lenght;
	uint16_t notcare;
	uint16_t cmdid;
} FrameHeader;

// 串口初始化
serial::Serial referee_serial =
	serial::Serial("/dev/sentry_referee_serial", 115200U, serial::Timeout::simpleTimeout(50U),
				   serial::eightbits, serial::parity_none, serial::stopbits_one,
				   serial::flowcontrol_none);
/*
 * CRC16校验
 * @param data 数据
 * @param len 数据长度
 * @return CRC16校验值
 */
uint16_t CRC16_Check(const uint8_t *data, uint8_t len)
{
	uint16_t CRC16 = 0xFFFF;
	uint8_t state, i, j;
	for (i = 0; i < len; i++)
	{
		CRC16 ^= data[i];
		for (j = 0; j < 8; j++)
		{
			state = CRC16 & 0x01;
			CRC16 >>= 1;
			if (state)
			{
				CRC16 ^= 0xA001;
			}
		}
	}
	return CRC16;
}

/*
 * CRC8校验
 * @param data 数据
 * @param len 数据长度
 * @return CRC8校验值
 */
uint8_t CRC8_Check(const uint8_t *data, uint8_t len)
{
	uint8_t CRC8 = 0;
	uint8_t i, j;
	for (i = 0; i < len; i++)
	{
		CRC8 ^= data[i];
		for (j = 0; j < 8; j++)
		{
			if (CRC8 & 0x80)
			{
				CRC8 = (CRC8 << 1) ^ 0x07;
			}
			else
			{
				CRC8 <<= 1;
			}
		}
	}
	return CRC8;
}

/*  数据订阅&串口发送类
 *  订阅数据并发送到串口
 */
// class PositionsSubscriber : public rclcpp::Node {
// public:
//   // 构造函数
//   PositionsSubscriber(std::string name) : Node(name) {
//     position_subscribe_ =
//         this->create_subscription<std_msgs::msg::Float32MultiArray>(
//             "car_positions", 10,
//             std::bind(&PositionsSubscriber::command_callback, this,
//                       std::placeholders::_1));
//   }

// private:
//   // 声明订阅者
//   rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
//       position_subscribe_;
//   // 收到数据的回调函数
//   void command_callback(const std_msgs::msg::Float32MultiArray::SharedPtr
//   msg) {
//     try {
//       // 串口数据初始化容器
//       std::vector<armor_data_t> emeny_robot_positions;

//       // 串口数据解析
//       if (msg->data.size() != 0) {
//         for (size_t i = 0; i < msg->data.size() - 2; i += 3) {
//           armor_data_t emeny_robot_position;

//           //   emeny_robot_position.target_robot_id = msg->data.data()[i];
//           //   emeny_robot_position.target_position_x = msg->data.data()[i +
//           1];
//           //   emeny_robot_position.target_position_y = msg->data.data()[i +
//           2];

//           emeny_robot_positions.push_back(emeny_robot_position);
//         }
//       }

//       for (size_t i = 0; i < emeny_robot_positions.size(); i++) {
//         // 串口数据打包
//         uint8_t serial_data[19];
//         serial_data_pack(serial_data, emeny_robot_positions[i], i);

//         // 串口数据发送
//         referee_serial.write(serial_data, sizeof(serial_data));
//       }
//     } catch (const std::exception &e) {
//     }
//   }
// };

class RefereeTransport : public rclcpp::Node
{
public:
	RefereeTransport(std::string name, bool red) : rclcpp::Node(name)
	{
		isred = red;
		// gain_point_pub_ = this->create_publisher<std_msgs::msg::Bool>(
		//     header + "gain_point_enable", 10);
		lasted_time_pub_ = this->create_publisher<std_msgs::msg::Float32>(
			header + "lasted_time", 10);
		hp_pub_ = this->create_publisher<std_msgs::msg::Float32>(header + "hp", 10);
		bullet_count_pub_ = this->create_publisher<std_msgs::msg::Float32>(
			header + "bullet_count", 10);
		attckid =
			this->create_publisher<std_msgs::msg::Byte>("referee/attack_id", 10);
	};
	void Tranport(const uint16_t cmdid, const char *data)
	{
		std::cout << std::hex << (cmdid) << "\n";
		switch (cmdid)
		{
		case 0x0001:
		{
			auto msg = std_msgs::msg::Float32();
			if ((data[0] & 0xf0) == 0x40)
			{
				msg.data = 0;
			}
			else
			{
				msg.data = (float)(*(uint16_t *)(&data[1]));
			}

			lasted_time_pub_->publish(msg);
			break;
		}
		case 0x0201:
		{
			auto msg = std_msgs::msg::Float32();
			msg.data = (float)(*(uint16_t *)&data[2]);
			hp_pub_->publish(msg);
			break;
		}
		case 0x0207:
		{
			bullet_count_++;
			auto msg = std_msgs::msg::Float32();
			msg.data = (float)bullet_count_;
			hp_pub_->publish(msg);
			break;
		}
		case 0x0206:
		{
			auto msg = std_msgs::msg::Byte();
			msg.data = data[0] & 0x0f;
			if ((data[0] & 0x0f) != 0)
				attckid->publish(msg);
		}
		default:
			break;
		}
	}

private:
	const std::string header = "/watcher/decision_maker/rmul/";
	//   rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
	//       gain_point_pub_; // I do not findout how
	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr lasted_time_pub_;
	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr hp_pub_;
	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr bullet_count_pub_;
	rclcpp::Publisher<std_msgs::msg::Byte>::SharedPtr attckid;
	bool isred;
	int bullet_count_ = 0;

	//   rclcpp::Publisher<ge>::SharedPtr total_resume_pub_;
};

// // 串口发送数据
int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	std::shared_ptr<RefereeTransport> referee =
		std::make_shared<RefereeTransport>("referee_transport", argv[0]);

	

	while (true)
	{
		try
		{
			std::string head = referee_serial.read(1);
			while ((uint8_t)head[0] != 0xA5)
			{

				std::cout << head << std::endl;
				head = referee_serial.read();
			}
			std::cout << head << std::endl;

			std::string temp = referee_serial.read(sizeof(frame_header_t));
			std::cout << temp << std::endl;
			std::string data = referee_serial.read(((FrameHeader *)(temp.c_str()))->data_lenght + 2ul);
			// std::cout << std::dec << temp << "\n";
			referee->Tranport(((FrameHeader *)(temp.c_str()))->cmdid, data.c_str());
		}
		catch (const std::exception &e)
		{
			continue;
		}
	}
	//   try {
	//     // 初始化节点
	//     rclcpp::init(argc, argv);

	//     // [创建对应节点的共享指针对象]
	//     auto positions_subscriber =
	//         std::make_shared<PositionsSubscriber>("positions_subscriber");

	//     // [运行节点，并检测退出信号]
	//     rclcpp::spin(positions_subscriber);
	//     rclcpp::shutdown();
	//   } catch (const std::exception &e) {
	//   }

	return 0;
}