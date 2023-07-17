#include "robot_base.h"

using namespace std;

float Robot_base::KP = 0;
float Robot_base::KI = 0;
float Robot_base::KD = 0;

// 构造函数
Robot_base::Robot_base()
{
    // 清空接发数据
    memset(&Reciver_Str,0,sizeof(Reciver_Str));
    memset(&Send_Str,0,sizeof(Send_Str));
    x = y = th = vx = vth = dt = 0.0;   

    //发送串口的包头和包尾
    Send_Str.Sensor_str.Header = RECIVER_DATA_HEADER;
    Send_Str.Sensor_str.End = RECIVER_DATA_END;
 
    // 加载串口参数
    nh.param<std::string>("serial/usart_port", this->usart_port, "/dev/ttyUSB0"); // 设置串口端口
    nh.param<int>("serial/baud_rate", this->baud_rate, 115200); // 设置波特率

    // 加载PID参数
    nh.param<float>("PID/KP", Robot_base::KP, 0.0);
    nh.param<float>("PID/KI", Robot_base::KI, 0.0);
    nh.param<float>("PID/KD", Robot_base::KD, 0.0);
    ROS_INFO("KP KI KD = [%f,%f,%f]",Robot_base::KP,Robot_base::KI,Robot_base::KD);
    // 发布话题
    this->moto_pub = nh.advertise<std_msgs::Float32MultiArray>("/robot/MotoEnder",30);
    this->odom_pub = nh.advertise<nav_msgs::Odometry>("odom",50);

    // 订阅话题
    this->cmd_vel_sub = nh.subscribe("/cmd_vel", 100, &Robot_base::cmd_velCallback, this);


    // 创建PID动态参数服务器
    // f = boost::bind(&Robot_base::dynamicParamCallback,_1);
    // server.setCallback(f);
    


    // 打开串口
    try{
        Robot_serial.setPort(this->usart_port);
        Robot_serial.setBaudrate(this->baud_rate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);//设置串口超时时间
        Robot_serial.setTimeout(to);
        Robot_serial.open();//打开串口
    }
    catch(serial::IOException &e){
        ROS_INFO_STREAM("Failed to open port ");
    }
    if(Robot_serial.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized!");
    }
    else{
        ROS_INFO_STREAM("Failed to open port ");
    }


}

Robot_base::~Robot_base()
{
    Robot_serial.close();//关闭串口
    cout << "over!!!" <<endl;
}

// 串口读取函数`
bool Robot_base::ReadUart()
{
    unsigned char CheckSumBuffer[1]; 
    Robot_serial.read(Reciver_Str.buffer,sizeof(Reciver_Str.buffer));
    if(Reciver_Str.Sensor_str.Header == RECIVER_DATA_HEADER){
        if(Reciver_Str.Sensor_str.End == RECIVER_DATA_END){
            
            // 获得机器人的速度值
            this->vx  = Reciver_Str.Sensor_str.X_speed;
            this->vth = Reciver_Str.Sensor_str.Z_speed;

            // 获得加速度和角速度值
            MPU6050.linear_acceleration.x = Reciver_Str.Sensor_str.Accelerometer.X_data / ACCELEROMETER;
            MPU6050.linear_acceleration.y = Reciver_Str.Sensor_str.Accelerometer.Y_data / ACCELEROMETER;
            MPU6050.linear_acceleration.z = Reciver_Str.Sensor_str.Accelerometer.Z_data / ACCELEROMETER;

            MPU6050.angular_velocity.x = Reciver_Str.Sensor_str.Gyroscope.X_data * GYROSCOPE_RADIAN;
            MPU6050.angular_velocity.y = Reciver_Str.Sensor_str.Gyroscope.Y_data * GYROSCOPE_RADIAN;
            MPU6050.angular_velocity.z = Reciver_Str.Sensor_str.Gyroscope.Z_data * GYROSCOPE_RADIAN;

            return true;
        }
    }
    //读取空字节防止卡死
    Robot_serial.read(CheckSumBuffer,sizeof(CheckSumBuffer));
    return false;
}

// 发布编码器速度值话题
void Robot_base::PublisherMotoEncoder()
{
    //M1
    Moto_Str.data.push_back(Reciver_Str.Sensor_str.MotoStr[0].Moto_CurrentSpeed);
    Moto_Str.data.push_back(Reciver_Str.Sensor_str.MotoStr[0].Moto_TargetSpeed);

    //M2
    Moto_Str.data.push_back(Reciver_Str.Sensor_str.MotoStr[1].Moto_CurrentSpeed);
    Moto_Str.data.push_back(Reciver_Str.Sensor_str.MotoStr[1].Moto_TargetSpeed);

    //M3
    Moto_Str.data.push_back(Reciver_Str.Sensor_str.MotoStr[2].Moto_CurrentSpeed);
    Moto_Str.data.push_back(Reciver_Str.Sensor_str.MotoStr[2].Moto_TargetSpeed);

    //M3
    Moto_Str.data.push_back(Reciver_Str.Sensor_str.MotoStr[3].Moto_CurrentSpeed);
    Moto_Str.data.push_back(Reciver_Str.Sensor_str.MotoStr[3].Moto_TargetSpeed);

    //M4
    Moto_Str.data.push_back(Reciver_Str.Sensor_str.MotoStr[4].Moto_CurrentSpeed);
    Moto_Str.data.push_back(Reciver_Str.Sensor_str.MotoStr[4].Moto_TargetSpeed);

    moto_pub.publish(Moto_Str); // 发布速度
    Moto_Str.data.clear(); // 清空数据
}

void Robot_base::PublisherOdom()
{
    //将yaw角转化为四元数
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    if (publish_odom){
        geometry_msgs::TransformStamped odom_transformStamped;
        odom_transformStamped.header.stamp = ros::Time::now();//时间戳
        odom_transformStamped.header.frame_id = "odom";
        odom_transformStamped.child_frame_id = this->robot_frame_id;//base_footprint

        //平移x,y,z
        odom_transformStamped.transform.translation.x = x;
        odom_transformStamped.transform.translation.y = y;
        odom_transformStamped.transform.translation.z = 0.0;

        //旋转w,x,y,z 四元数
        odom_transformStamped.transform.rotation = odom_quat;

        //广播TF
        odom_broadcaster.sendTransform(odom_transformStamped);
    }

    // odom的话题消息发布
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = ros::Time::now();//时间戳
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = this->robot_frame_id;//base_footprint 

    //发布位姿pose消息
    //平移x,y,z
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0.0;

    //旋转w,x,y,z
    odom_msg.pose.pose.orientation = odom_quat;

    //发布速度
    odom_msg.twist.twist.linear.x = this->vx;
    odom_msg.twist.twist.linear.y = 0;
    odom_msg.twist.twist.angular.z = this->th;

    if((this->vx == 0)&&(this->th==0))
    {//当速度为0,说明编码器的误差会比较小，认为编码器的数据更可靠
        memcpy(&odom_msg.pose.covariance,odom_pose_covariance2,sizeof(odom_pose_covariance2));
        memcpy(&odom_msg.twist.covariance,odom_twist_covariance2,sizeof(odom_pose_covariance2));
    }
    else
    {//如果小车的速度非零，考虑到运动中的编码器可能带来的滑动误差，认为imu的数据更可靠
        memcpy(&odom_msg.pose.covariance,odom_pose_covariance,sizeof(odom_pose_covariance));
        memcpy(&odom_msg.twist.covariance,odom_twist_covariance,sizeof(odom_twist_covariance));
    }

    //发布消息
    odom_pub.publish(odom_msg);
}

// 循环函数
void Robot_base::LoopProcess()
{
    this->last_time = ros::Time::now(); // 获取时间
    while(ros::ok()){
        
        //计算时间dt
        this->current_time = ros::Time::now(); // 获取当前时间
        this->dt = (current_time - last_time).toSec(); // 转化为秒为单位
        this->sampleFreq = 1.0f / dt;//采样率

        if (ReadUart()){ // 如果能读取串口成功
            
            // 计算tf和odom
            x +=  vx*cos(th) * dt;
            y +=  vx*sin(th) * dt;
            th += vth * dt;



            PublisherMotoEncoder(); // 发布速度数据


        }




        this->last_time = current_time;  
        ros::spinOnce(); 
    }
}




// 动态调参回调函数
void Robot_base::dynamicParamCallback(robot_base::pidConfig &config)
{
    Robot_base::KP = config.KP;
    Robot_base::KI = config.KI;
    Robot_base::KD = config.KD;

    ROS_INFO("KP KI KD = [%f,%f,%f]",Robot_base::KP,Robot_base::KI,Robot_base::KD);
}

// cmd_vel 话题订阅回调函数
void Robot_base::cmd_velCallback(const geometry_msgs::Twist &twist_aux)
{
    // 将手柄的速度值下发到STM32控制器
    Send_Str.Sensor_str.X_speed = twist_aux.linear.x;
    Send_Str.Sensor_str.Z_speed = twist_aux.angular.z;

    // 将PID值下发到STM32控制器
    // Send_Str.Sensor_str.PID_Param[0] = Robot_base::KP;
    // Send_Str.Sensor_str.PID_Param[1] = Robot_base::KI;
    // Send_Str.Sensor_str.PID_Param[2] = Robot_base::KD;

    Robot_serial.write(Send_Str.buffer, sizeof(Send_Str.buffer));
}








int main(int argc, char** argv)
{
    ros::init(argc,argv,"robot_base_node");
    
    ROS_INFO("robot_base_node start!"); 
    
    Robot_base robot_base;
    robot_base.LoopProcess();


    
    return 0;
}