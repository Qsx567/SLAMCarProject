#ifndef __ROBOT_BASE_H_
#define __ROBOT_BASE_H_

#include <iostream>
#include <string>
#include <ros/ros.h>
#include <serial/serial.h>	  // 串口头文件
#include <sensor_msgs/Imu.h>  // MPU6050消息格式
#include <ros/time.h>         // 时间相关头文件
#include <std_msgs/Float32MultiArray.h> //四个电机消息格式
#include <tf/transform_broadcaster.h>//tf需要的头文件
#include <geometry_msgs/Quaternion.h>//四元数消息类型
#include <nav_msgs/Odometry.h>//里程计消息类型

#include <dynamic_reconfigure/server.h>
#include <robot_base/pidConfig.h>

#include <geometry_msgs/Twist.h> // 速度的消息类型

#define ACCELEROMETER 		16384.0f   // ±2g
#define GYROSCOPE_RADIAN	0.001064f  // ±2000°/s 弧度每秒


//里程计协方差
const double odom_pose_covariance[36] = {1e-3, 0, 0, 0, 0, 0, 
										0, 1e-3, 0, 0, 0, 0,
										0, 0, 1e6, 0, 0, 0,
										0, 0, 0, 1e6, 0, 0,
										0, 0, 0, 0, 1e6, 0,
										0, 0, 0, 0, 0, 1e3};
const double odom_pose_covariance2[36] = {1e-9, 0, 0, 0, 0, 0, 
										0, 1e-3, 1e-9, 0, 0, 0,
										0, 0, 1e6, 0, 0, 0,
										0, 0, 0, 1e6, 0, 0,
										0, 0, 0, 0, 1e6, 0,
										0, 0, 0, 0, 0, 1e-9};
 
const double odom_twist_covariance[36] = {1e-3, 0, 0, 0, 0, 0, 
										0, 1e-3, 0, 0, 0, 0,
										0, 0, 1e6, 0, 0, 0,
										0, 0, 0, 1e6, 0, 0,
										0, 0, 0, 0, 1e6, 0,
										0, 0, 0, 0, 0, 1e3};
const double odom_twist_covariance2[36] = {1e-9, 0, 0, 0, 0, 0, 
										0, 1e-3, 1e-9, 0, 0, 0,
										0, 0, 1e6, 0, 0, 0,
										0, 0, 0, 1e6, 0, 0,
										0, 0, 0, 0, 1e6, 0,
										0, 0, 0, 0, 0, 1e-9};







// ----------------------------- 串口通信协议 ------------------------------- //

#define RECIVER_DATA_HEADER 0x5BB5 // 数据包头
#define RECIVER_DATA_END    0x5AA5 // 数据包尾

#define PROTOCOL_DATA_SIZE   68    // 数据包大小


#pragma pack(1)

// 电机的实际速度和目标速度
typedef struct _Moto_str
{
	float Moto_CurrentSpeed;
	float Moto_TargetSpeed;
}Moto_str;

// MPU6050数据
typedef struct _MPU6050_Str
{
	short X_data;
	short Y_data;
	short Z_data;
}MPU6050_Str;

// ROS和STM32串口通信的数据包
typedef union _Upload_Data
{
	unsigned char buffer[PROTOCOL_DATA_SIZE]; // 数据包buffer
	struct _Sensor_str
	{
		// 数据包头
		unsigned short int Header; 		// 5B B5
		
		// 前向运动学Vx,w
		float X_speed;	 				// 00 00 00 00 
		float Z_speed;   				// 00 00 00 00 
		
		// 加速度值和陀螺仪值
		MPU6050_Str Accelerometer;      // C4 06 F0 FD 6C 38
		MPU6050_Str Gyroscope;          // D8 FF 03 00 EB FF
		
		// 电机的实际速度值和目标速度值
		Moto_str MotoStr[4]; 		    //M1 00 00 00 00  00 00 00 00
									    //M2 00 00 00 00  00 00 00 00
									    //M3 00 00 00 00  00 00 00 00 
									    //M4 00 00 00 00  00 00 00 00
		
		// PID参数值
		float PID_Param[3];			    //P 00 00 00 00
										//I 00 00 00 00
										//D 00 00 00 00
		
		// 数据包尾
		unsigned short int End;			// 5A A5
	}Sensor_str;
}Upload_Data;

#pragma pack(4)

// ----------------------------- 串口通信协议 ------------------------------- //



class Robot_base
{
    private:
		ros::NodeHandle nh;
		ros::Publisher moto_pub, odom_pub;
		ros::Subscriber cmd_vel_sub;
		

        std::string usart_port;
		int baud_rate;
		double x,y,th,vx,vth,dt;
		float sampleFreq; // 采样频率
		bool publish_odom;
		std::string robot_frame_id;

		Upload_Data Reciver_Str, Send_Str; 
		sensor_msgs::Imu MPU6050;
		std_msgs::Float32MultiArray Moto_Str;//发布的电机速度数据
		ros::Time current_time,last_time; //用于计时
		tf::TransformBroadcaster odom_broadcaster;//创建TF广播器

		dynamic_reconfigure::Server<robot_base::pidConfig> server;
		dynamic_reconfigure::Server<robot_base::pidConfig>::CallbackType f;


		

    public:
        Robot_base();
        ~Robot_base();
	
		bool ReadUart();
		void LoopProcess();
		void PublisherMotoEncoder();//发布编码器速度值
		void PublisherOdom(); // 发布里程计
		void cmd_velCallback(const geometry_msgs::Twist &twist_aux);



		serial::Serial Robot_serial; // 串口声明

		static void dynamicParamCallback(robot_base::pidConfig &config);
		static float KP,KI,KD; // 类里面的静态变量需要全局初始化
};





















#endif
