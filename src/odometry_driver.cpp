#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <csignal>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <chrono>
#include <memory>
#include <string>
#include <numeric>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <fcntl.h>
#include <unistd.h>
#include <semaphore.h>
#include <termios.h>
#include <math.h>
#include <time.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <nav_msgs/msg/odometry.hpp>
#include "odometry_driver.hpp"

using namespace std::chrono_literals;

// 全局可配置参数（由节点参数覆盖）
std::string serial_port = "/dev/ttyCH9344USB1";
double warn_timeout = 0.25;
double max_timeout = 1.0;
double freq = 10.0;

int fd = -1;

double now_time = 0, last_time = 0, delay_time, header_time, print_time, task_time;

bool okay = false;

bool printf_flag = false;

// 构造里程计数据结构体，timestamp为接收话题的时间戳,used为该数据是否已被使用
struct Odometry_msg
{
	nav_msgs::msg::Odometry odometry;
	double timestamp;
};

struct Odometry_msg odometry_msg;
struct Odometry_msg odometry_cache;

enum slam_driver_state
{
	SLAM_DRIVER_INIT = 0, // 初始化
	SLAM_DRIVER_WAIT = 1, // 等待：此帧数据不发送（指定频率发送）
	SLAM_DRIVER_OKAY = 2, // 正常
	SLAM_DRIVER_WARN = 3, // 警告：时间戳超时（未到不可用阶段）
	SLAM_DRIVER_ERROR = 4 // 错误：时间戳错误（严重超时，slam退化或者停止）
};

enum slam_mode
{
	SLAM_MODE_OFF = 0,	  // 注销slam传感器
	SLAM_MODE_XY = 1,  // 仅水平定位
	SLAM_MODE_XYZ = 2,  // 三维定位
	SLAM_MODE_XYZQ = 3, // 三维定位+罗盘
	SLAM_MODE_INIT = 4	  // 初始化/重置slam
};

// 配置串口
int setup_serial(int fd)
{

	struct termios options;

	// 获取当前配置
	if (tcgetattr(fd, &options) == -1)
	{
		return -1;
	}

	// 设置波特率
	cfsetispeed(&options, B921600);
	cfsetospeed(&options, B921600);

	// 设置数据位
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;

	// 设置停止位
	options.c_cflag &= ~CSTOPB;

	// 设置奇偶校验
	options.c_cflag &= ~PARENB;
	options.c_iflag &= ~INPCK;

	// 设置原始输入和输出模式
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	options.c_oflag &= ~OPOST;

	// 设置读取超时
	options.c_cc[VTIME] = 10;
	options.c_cc[VMIN] = 0;

	// 设置串口属性
	if (tcsetattr(fd, TCSANOW, &options) == -1)
	{
		return -1;
	}

	return 0;
}

// 初始化串口
int uart_init()
{
	// 打开串口
	int status = 1;
	fd = open(serial_port.c_str(), O_RDWR | O_NOCTTY);
	if (fd == -1)
	{
		perror("Error opening serial port");
		status = -1;
	}

	// 设置串口
	if (setup_serial(fd) == -1)
	{
		perror("Error setting up serial port");
		close(fd);
		status = -1;
	}

	return status;
}

// 发送数据函数
int send_data(int fd, uint8_t *data, int len)
{
	int written_len = write(fd, data, len);
	if (written_len == -1)
	{
		return -1;
	}
	return written_len;
}

// 接收数据函数
int receive_data(int fd, uint8_t *buffer, int max_len)
{
	int received_len = read(fd, buffer, max_len);
	if (received_len == -1)
	{
		return -1;
	}
	return received_len;
}

/**CRC16校验码计算函数**/
void crc16_modbus(uint8_t *puchMsg, unsigned int usDataLen, uint8_t *puchCRCHi, uint8_t *puchCRCLo)
{
	uint8_t uchCRCHi = 0xFF;
	uint8_t uchCRCLo = 0xFF;
	unsigned int uIndex;

	while (usDataLen--)
	{
		uIndex = uchCRCHi ^ *puchMsg++;
		uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
		uchCRCLo = auchCRCLo[uIndex];
	}
	*puchCRCHi = uchCRCHi;
	*puchCRCLo = uchCRCLo;
}

void send_uart(uint8_t id, float posx, float posy, float posz, double x, double y, double z, double w)
{
	poslink_msg send_msg;
	send_msg.msg[0] = 0xfe;
	send_msg.msg[1] = 0xfb;
	send_msg.msg[2] = id;
	send_msg.pos_x.data = posx * 100;
	send_msg.pos_y.data = posy * 100;
	send_msg.pos_z.data = posz * 100;
	send_msg.q_x.data = x;
	send_msg.q_y.data = y;
	send_msg.q_z.data = z;
	send_msg.q_w.data = w;

	send_msg.msg[3] = send_msg.pos_x.bytes[3];
	send_msg.msg[4] = send_msg.pos_x.bytes[2];
	send_msg.msg[5] = send_msg.pos_x.bytes[1];
	send_msg.msg[6] = send_msg.pos_x.bytes[0];
	send_msg.msg[7] = send_msg.pos_y.bytes[3];
	send_msg.msg[8] = send_msg.pos_y.bytes[2];
	send_msg.msg[9] = send_msg.pos_y.bytes[1];
	send_msg.msg[10] = send_msg.pos_y.bytes[0];
	send_msg.msg[11] = send_msg.pos_z.bytes[3];
	send_msg.msg[12] = send_msg.pos_z.bytes[2];
	send_msg.msg[13] = send_msg.pos_z.bytes[1];
	send_msg.msg[14] = send_msg.pos_z.bytes[0];
	send_msg.msg[15] = send_msg.q_x.bytes[7];
	send_msg.msg[16] = send_msg.q_x.bytes[6];
	send_msg.msg[17] = send_msg.q_x.bytes[5];
	send_msg.msg[18] = send_msg.q_x.bytes[4];
	send_msg.msg[19] = send_msg.q_x.bytes[3];
	send_msg.msg[20] = send_msg.q_x.bytes[2];
	send_msg.msg[21] = send_msg.q_x.bytes[1];
	send_msg.msg[22] = send_msg.q_x.bytes[0];
	send_msg.msg[23] = send_msg.q_y.bytes[7];
	send_msg.msg[24] = send_msg.q_y.bytes[6];
	send_msg.msg[25] = send_msg.q_y.bytes[5];
	send_msg.msg[26] = send_msg.q_y.bytes[4];
	send_msg.msg[27] = send_msg.q_y.bytes[3];
	send_msg.msg[28] = send_msg.q_y.bytes[2];
	send_msg.msg[29] = send_msg.q_y.bytes[1];
	send_msg.msg[30] = send_msg.q_y.bytes[0];
	send_msg.msg[31] = send_msg.q_z.bytes[7];
	send_msg.msg[32] = send_msg.q_z.bytes[6];
	send_msg.msg[33] = send_msg.q_z.bytes[5];
	send_msg.msg[34] = send_msg.q_z.bytes[4];
	send_msg.msg[35] = send_msg.q_z.bytes[3];
	send_msg.msg[36] = send_msg.q_z.bytes[2];
	send_msg.msg[37] = send_msg.q_z.bytes[1];
	send_msg.msg[38] = send_msg.q_z.bytes[0];
	send_msg.msg[39] = send_msg.q_w.bytes[7];
	send_msg.msg[40] = send_msg.q_w.bytes[6];
	send_msg.msg[41] = send_msg.q_w.bytes[5];
	send_msg.msg[42] = send_msg.q_w.bytes[4];
	send_msg.msg[43] = send_msg.q_w.bytes[3];
	send_msg.msg[44] = send_msg.q_w.bytes[2];
	send_msg.msg[45] = send_msg.q_w.bytes[1];
	send_msg.msg[46] = send_msg.q_w.bytes[0];

	crc16_modbus(send_msg.msg, 47, &send_msg.crc16.bytes[0], &send_msg.crc16.bytes[1]);

	send_msg.msg[47] = send_msg.crc16.bytes[1];
	send_msg.msg[48] = send_msg.crc16.bytes[0];

	send_data(fd, send_msg.msg, 49);
}

// rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped = nh->create_publisher<nav_msgs::msg::Odometry>("/body_to_init", 1000);

class OdomeListener : public rclcpp::Node
{
public:
	OdomeListener()
		: Node("mid360_link")
	{
		std::string odom_topic;
		this->declare_parameter<std::string>("odometry_topic", "/odin1/odometry");
		this->get_parameter("odometry_topic", odom_topic);
		pubOdomAftMapped = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic, 1000, std::bind(&OdomeListener::odomecallback, this, std::placeholders::_1));
	}

private:
	void odomecallback(const nav_msgs::msg::Odometry::SharedPtr msg)
	{
		header_time = rclcpp::Time(msg->header.stamp).seconds();
		now_time = this->get_clock()->now().seconds();

		odometry_msg.odometry = *msg;
		odometry_msg.timestamp = header_time;
	}

	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped;
};

class LCTask : public rclcpp::Node
{
public:
	LCTask()
		: Node("mid360_link_lc")
	{
		timer_ = this->create_wall_timer(
			std::chrono::milliseconds(100), std::bind(&LCTask::timer, this));
	}

private:
	void timer()
	{
		task_time = this->get_clock()->now().seconds();

		if (task_time - print_time >= 1.0)
		{
			print_time = task_time;
			printf_flag = true;
		}

		if (fd == -1)
		{
			uart_init();
			if (printf_flag)
			{
				if (fd != -1)
				{
					RCLCPP_INFO(this->get_logger(), "Serial port restart open.");
				}
				else
				{
					RCLCPP_ERROR(this->get_logger(), "Serial port open failed.");
				}
			}
		}
	}

	rclcpp::TimerBase::SharedPtr timer_;
};

class push : public rclcpp::Node
{
public:
	push()
		: Node("slam_push_uart")
	{
		this->declare_parameter<std::string>("serial_port", serial_port);
		this->declare_parameter<double>("warn_timeout", warn_timeout);
		this->declare_parameter<double>("max_timeout", max_timeout);
		this->declare_parameter<double>("freq", freq);

		this->get_parameter("serial_port", serial_port);
		this->get_parameter("warn_timeout", warn_timeout);
		this->get_parameter("max_timeout", max_timeout);
		this->get_parameter("freq", freq);

		// 合法性检查（防小人++）
		if(freq <= 0){
			freq = 10.0;
		}

		timer_ = this->create_wall_timer(
			std::chrono::milliseconds(1), std::bind(&push::timer, this));
	}

private:
	void timer()
	{
		now_time = this->get_clock()->now().seconds();

		// 此帧数据已处理
		if (odometry_cache.timestamp == odometry_msg.timestamp)
		{
			now_slam_state = slam_driver_state::SLAM_DRIVER_WAIT;
			return;
		}

		// 初始化状态检测（丢弃首帧数据）
		if ( last_time == 0.0 )
		{
			last_time = now_time;
			now_slam_state = slam_driver_state::SLAM_DRIVER_INIT;
			return;
		}else{
			if(now_slam_state!=slam_driver_state::SLAM_DRIVER_INIT)
				now_slam_state = slam_driver_state::SLAM_DRIVER_OKAY;
		}

		// 串口发送频率限制
		if ((now_time - last_time) < (1.0 / freq))
		{
			return;
		}

		// 防止初始化被覆盖
		if(now_slam_state!=slam_driver_state::SLAM_DRIVER_INIT){
			double delay_time = odometry_msg.timestamp - get_time_sec(odometry_msg.odometry.header.stamp);
			if(delay_time > max_timeout){
				now_slam_state = slam_driver_state::SLAM_DRIVER_ERROR;
			}else if(delay_time > warn_timeout){
				now_slam_state = slam_driver_state::SLAM_DRIVER_WARN;
			}else{
				now_slam_state = slam_driver_state::SLAM_DRIVER_OKAY;
			}
		}
		// 不直接使用原始数据，防止数据竞争
		odometry_cache = odometry_msg;

		switch(now_slam_state)
		{
			case slam_driver_state::SLAM_DRIVER_INIT:
			{
				send_uart((uint8_t)slam_mode::SLAM_MODE_INIT, odometry_cache.odometry.pose.pose.position.x, odometry_cache.odometry.pose.pose.position.y, odometry_cache.odometry.pose.pose.position.z,
						  odometry_cache.odometry.pose.pose.orientation.x, odometry_cache.odometry.pose.pose.orientation.y,
						  odometry_cache.odometry.pose.pose.orientation.z, odometry_cache.odometry.pose.pose.orientation.w);
				RCLCPP_INFO(this->get_logger(), "SLAM Driver Init.");
				last_time = now_time;
				break;
			}
			case slam_driver_state::SLAM_DRIVER_WAIT:
			{
				break;
			}
			case slam_driver_state::SLAM_DRIVER_OKAY:
			{
				send_uart((uint8_t)slam_mode::SLAM_MODE_XYZQ, odometry_cache.odometry.pose.pose.position.x, odometry_cache.odometry.pose.pose.position.y, odometry_cache.odometry.pose.pose.position.z,
						  odometry_cache.odometry.pose.pose.orientation.x, odometry_cache.odometry.pose.pose.orientation.y,
						  odometry_cache.odometry.pose.pose.orientation.z, odometry_cache.odometry.pose.pose.orientation.w);
				if(now_time - last_send_time > 1.0){
					RCLCPP_INFO(this->get_logger(), "Position x: %.3f, y: %.3f, z: %.3f", odometry_cache.odometry.pose.pose.position.x*100, odometry_cache.odometry.pose.pose.position.y*100, odometry_cache.odometry.pose.pose.position.z*100);
					last_send_time = now_time;
				}
				last_time = now_time;
				break;
			}
			case slam_driver_state::SLAM_DRIVER_WARN:
			{
				send_uart((uint8_t)slam_mode::SLAM_MODE_XYZQ, odometry_cache.odometry.pose.pose.position.x, odometry_cache.odometry.pose.pose.position.y, odometry_cache.odometry.pose.pose.position.z,
						  odometry_cache.odometry.pose.pose.orientation.x, odometry_cache.odometry.pose.pose.orientation.y,
						  odometry_cache.odometry.pose.pose.orientation.z, odometry_cache.odometry.pose.pose.orientation.w);
				if(now_time - last_send_time > 1.0){
					RCLCPP_WARN(this->get_logger(), "SLAM Driver Warning: Time delay %.3f s.", now_time - odometry_cache.timestamp);
					last_send_time = now_time;
				}
				// RCLCPP_WARN(this->get_logger(), "SLAM Driver Warning: Time delay %.3f s.", now_time - odometry_cache.timestamp);
				last_time = now_time;
				last_send_time = now_time;
				break;
			}
			case slam_driver_state::SLAM_DRIVER_ERROR:
			{
				send_uart((uint8_t)slam_mode::SLAM_MODE_OFF, 0, 0, 0, 0, 0, 0, 0);
				last_time = 0;
				last_send_time = 0;
				break;
			}
			default:
			{
				send_uart((uint8_t)slam_mode::SLAM_MODE_OFF, 0, 0, 0, 0, 0, 0, 0);
				last_time = 0;
				last_send_time = 0;
				break;
			}
		}
	}
	rclcpp::TimerBase::SharedPtr timer_;
	double now_time = 0.0;
	double last_time = 0.0;
	double last_send_time = 0.0;
	slam_driver_state now_slam_state;
	slam_driver_state last_slam_state;
	
};

// 处理 Ctrl+C 信号
void signal_handler(int signal)
{
	if (signal == SIGINT)
	{
		// 处理退出逻辑
		std::cout << "Caught Ctrl+C, exiting..." << std::endl;
		rclcpp::shutdown();
		exit(0); // 强制退出程序
	}
}

int main(int argc, char *argv[])
{
	std::signal(SIGINT, signal_handler);
	rclcpp::init(argc, argv);
	uart_init();
	// You MUST use the MultiThreadedExecutor to use, well, multiple threads
	rclcpp::executors::MultiThreadedExecutor executor;
	auto posnode = std::make_shared<OdomeListener>();
	auto lcnode = std::make_shared<LCTask>();
	auto pushnode =  std::make_shared<push>();
	executor.add_node(posnode);
	executor.add_node(lcnode);
	executor.add_node(pushnode);
	executor.spin();
	rclcpp::shutdown();
	return 0;
}