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

// 参数默认值
static const char *kDefaultSerialPort = "/dev/ttyCH9344USB1";
static constexpr double kDefaultWarnTimeout = 0.25;
static constexpr double kDefaultMaxTimeout = 1.0;

// 全局可配置参数（由节点参数覆盖）
std::string g_serial_port = kDefaultSerialPort;
double g_warn_timeout = kDefaultWarnTimeout;
double g_max_timeout = kDefaultMaxTimeout;

int fd = -1;

double now_time, last_time, delay_time, header_time, print_time, task_time;

bool okay = false;

bool printf_flag = false;


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
	fd = open(g_serial_port.c_str(), O_RDWR | O_NOCTTY);
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



void send_uart( uint8_t id, float posx, float posy, float posz, double x, double y, double z, double w )
{
    poslink_msg send_msg;
	send_msg.msg[0] = 0xfe;
	send_msg.msg[1] = 0xfb;
	send_msg.msg[2] = id;
	send_msg.pos_x.data = posx;
	send_msg.pos_y.data = posy;
	send_msg.pos_z.data = posz;
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

	crc16_modbus(send_msg.msg,47,&send_msg.crc16.bytes[0],&send_msg.crc16.bytes[1]);

	send_msg.msg[47] = send_msg.crc16.bytes[1];
	send_msg.msg[48] = send_msg.crc16.bytes[0];

	send_data(fd,send_msg.msg,49);
}

//rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped = nh->create_publisher<nav_msgs::msg::Odometry>("/body_to_init", 1000);

class OdomeListener : public rclcpp::Node  
{  
public:  
  OdomeListener()  
  : Node("mid360_link")  
  {  
	std::string odom_topic;

	this->declare_parameter<std::string>("odometry_topic", "/odin1/odometry");
	this->declare_parameter<std::string>("serial_port", kDefaultSerialPort);
	this->declare_parameter<double>("warn_timeout", kDefaultWarnTimeout);
	this->declare_parameter<double>("max_timeout", kDefaultMaxTimeout);

	this->get_parameter("odometry_topic", odom_topic);
	this->get_parameter("serial_port", g_serial_port);
	this->get_parameter("warn_timeout", g_warn_timeout);
	this->get_parameter("max_timeout", g_max_timeout);

	pubOdomAftMapped = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic, 1000, std::bind(&OdomeListener::odomecallback, this, std::placeholders::_1));

  }  
private:  
  void odomecallback(const nav_msgs::msg::Odometry::SharedPtr msg)  
  {  
	header_time = rclcpp::Time(msg->header.stamp).seconds();
	now_time = this->get_clock()->now().seconds();

	double warn_timeout = g_warn_timeout;
	double max_timeout = g_max_timeout;
	if (max_timeout > 0.0 && warn_timeout > max_timeout) {
		warn_timeout = max_timeout;
	}
	double time_diff = fabs(header_time - now_time);

	if( last_time == 0 ){
		last_time = now_time;
		RCLCPP_INFO(this->get_logger(), "Pos_Link init.");
		send_uart( 0, msg->pose.pose.position.x * 100, msg->pose.pose.position.y * 100, msg->pose.pose.position.z * 100, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w );

	}

	if( time_diff > warn_timeout ){
		okay = false;
		if(printf_flag==true){
			RCLCPP_WARN(this->get_logger(), "TimeStamp error,now time is: %f, but pointcloud2 time is: %f",now_time,header_time);
			printf_flag = false;
		}
		if( time_diff <= max_timeout ){
			send_uart( 3, msg->pose.pose.position.x * 100, msg->pose.pose.position.y * 100, msg->pose.pose.position.z * 100, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w );
		}else{
			send_uart( 0, msg->pose.pose.position.x * 100, msg->pose.pose.position.y * 100, msg->pose.pose.position.z * 100, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w );
			okay = false;
		}
	}else{
		if( !okay ){
			send_uart( 4, msg->pose.pose.position.x * 100, msg->pose.pose.position.y * 100, msg->pose.pose.position.z * 100, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w );
			RCLCPP_INFO(this->get_logger(), "Reset slam mag.");
			okay = true;
		}
		delay_time = now_time - last_time;
		last_time = now_time;
		if(printf_flag==true){
			RCLCPP_INFO(this->get_logger(), "Hz:%d,x:%f,y:%f,z:%f",(int)(1/delay_time),msg->pose.pose.position.x * 100, msg->pose.pose.position.y * 100, msg->pose.pose.position.z * 100);
			printf_flag = false;
		}
		send_uart( 3, msg->pose.pose.position.x * 100, msg->pose.pose.position.y * 100, msg->pose.pose.position.z * 100, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w );
	}

    // RCLCPP_INFO(this->get_logger(), "Received odometry data");
    // You can access the data in msg and do something with it
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped; 
};  

class LCTask : public rclcpp::Node  
{  
public:  
LCTask()  
  : Node("mid360_link_lc")  
  {  
	this->declare_parameter<std::string>("serial_port", kDefaultSerialPort);
	this->declare_parameter<double>("warn_timeout", kDefaultWarnTimeout);
	this->declare_parameter<double>("max_timeout", kDefaultMaxTimeout);

	this->get_parameter("serial_port", g_serial_port);
	this->get_parameter("warn_timeout", g_warn_timeout);
	this->get_parameter("max_timeout", g_max_timeout);

    timer_ = this->create_wall_timer(  
		std::chrono::milliseconds(100), std::bind(&LCTask::timer, this));  
  }  
private:  
  void timer()  
  {  
	task_time = this->get_clock()->now().seconds();

	if(task_time - print_time >= 1.0){
		print_time = task_time;
		printf_flag = true;
	}

	if( fd == -1 ){
		uart_init();
		if(printf_flag){
			if( fd != -1){
				RCLCPP_INFO(this->get_logger(), "Serial port restart open.");
			}else{
				RCLCPP_ERROR(this->get_logger(), "Serial port open failed.");
			}
		}
	}
	
	if( g_max_timeout > 0.0 && task_time - now_time >= g_max_timeout ){
		if(printf_flag){
			RCLCPP_ERROR(this->get_logger(), "Slam pose msg lost!");
		}
		printf_flag = false;
		okay = false;
	}
  }

  rclcpp::TimerBase::SharedPtr timer_;  
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
	executor.add_node(posnode);
	executor.add_node(lcnode);
	executor.spin();
	rclcpp::shutdown(); 
	return 0; 
} 