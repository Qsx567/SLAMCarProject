#include "ps2.h"
#include "delay.h"
#include "main.h"
#include "motor.h"
#include "system.h"
#include "serial.h"
//使用模拟spi通信


#define DELAY_TIME  HAL_Delay_us(5); 
uint16_t Handkey;	                 // 定义键值读取变量，临时存储。
uint8_t Comd[2]={0x01,0x42};	         // 指令存储数组：0x01为通信开始指令。0x42为请求数据指令
uint8_t Data[9]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; // 定义数据存储数组


// 定义按键名
uint16_t MASK[]={
    PSB_SELECT,
    PSB_L3,
    PSB_R3 ,
    PSB_START,
    PSB_PAD_UP,
    PSB_PAD_RIGHT,
    PSB_PAD_DOWN,
    PSB_PAD_LEFT,
    PSB_L2,
    PSB_R2,
    PSB_L1,
    PSB_R1 ,
    PSB_GREEN,
    PSB_RED,
    PSB_BLUE,
    PSB_PINK
	};	

/************************************************************
函数名：PS2_Init
功能：Ps2接收器接口IO初始化函数
参数：无
返回值：无
使用注意：
    1. IO接口（模拟SPI）定义如下：
        DI/DATA -> PC2
        DO/COM -> PC1
        CS/ATT -> PC3
        CLK -> PA4
    2. 移植时需要修改此函数，以适配不同硬件电路    
*************************************************************/
void PS2_Init(void)
{
	PS2_SetInit();//手柄配置初始化											  
}
/************************************************************
函数名：PS2_Cmd
功能：向手柄发送命令
参数：CMD： uint8类型，指令编号
返回值：无
使用注意：
    1. 该函数是所有数据读写的基础，数据线DI和DO在时钟线CLK
       下降沿交换数据
    2. 发送指令后返回的数据存储在了全局变量Data[1]中
*************************************************************/
void PS2_Cmd(uint8_t CMD)
{
	volatile uint16_t ref=0x01;
	Data[1] = 0;
	for(ref=0x01;ref<0x0100;ref<<=1)
	{
		if(ref&CMD)
		{
			DO_H;                   //输出一位控制位
		}
		else DO_L;

		CLK_H;                      //时钟拉高
		DELAY_TIME;
		CLK_L;
		DELAY_TIME;
		CLK_H;
		if(DI)
			Data[1] = ref|Data[1];
	}
	HAL_Delay_us(16);
}
/************************************************************
函数名：PS2_RedLight
功能：判断是否为红灯模式,0x41=模拟绿灯，0x73=模拟红灯
参数：无
返回值：0，红灯模式
	    1，其他模式
使用注意：无
*************************************************************/
uint8_t PS2_RedLight(void)
{
	CS_L;
	PS2_Cmd(Comd[0]);  //开始命令
	PS2_Cmd(Comd[1]);  //请求数据
	CS_H;
	if( Data[1] == 0X73)   return 0 ;
	else return 1;

}
/************************************************************
函数名：PS2_ReadData
功能：读取手柄数据
参数：无
返回值：无
使用注意：
    1. 数据读写期间CS引脚需要拉低，才能选中该设备
    2. 读出来的数据存储在Data[1]到Data[8]中
*************************************************************/
void PS2_ReadData(void)
{
	volatile uint8_t byte=0;
	volatile uint16_t ref=0x01;
	CS_L;
	PS2_Cmd(Comd[0]);  //开始命令
	PS2_Cmd(Comd[1]);  //请求数据
	for(byte=2;byte<9;byte++)          //开始接受数据
	{
		for(ref=0x01;ref<0x100;ref<<=1)
		{
			CLK_H;
			DELAY_TIME;
			CLK_L;
			DELAY_TIME;
			CLK_H;
		    if(DI)
		      Data[byte] = ref|Data[byte];
		}
        HAL_Delay_us(16);
	}
	CS_H;
}
/************************************************************
函数名：PS2_DataKey
功能：对读出来的PS2的数据进行处理,只处理按键部分
参数：无
返回值：0：   无任何按键按下
        其他：按键序号
使用注意：根据协议手册，Data[3]和Data[4]存储了16个按键状态，其中
    Data[3]对应 SELECT、 L3 、 R3、 START 、 UP、 RIGHT、 DOWN、 LEFT 是否被按下，若被按下对应位为0
    Data[4]对应 L2 、 R2、L1 、R1、△、○、╳、□ 是否被按下，若被按下对应位为0
*************************************************************/
uint8_t PS2_DataKey()
{
	uint8_t index;

	PS2_ClearData();
	PS2_ReadData();

	Handkey=(Data[4]<<8)|Data[3];     //这是16个按键  按下为0， 未按下为1
	for(index=0;index<16;index++)
	{	    
		if((Handkey&(1<<(MASK[index]-1)))==0)
            return index+1;
	}
	return 0;          //没有任何按键按下
}


/************************************************************
函数名：S2_AnologData
功能：得到一个摇杆的模拟量	 范围0~256
参数：button 摇杆编号
返回值：摇杆模拟量数值
使用注意：只有在红灯模式下值才是有效的，波动摇杆，这些值才会
    变化，这些值分别存储在Data[5] Data[6] Data[7] Data[8]
*************************************************************/
uint8_t PS2_AnologData(uint8_t button)
{
	return Data[button];
}

/************************************************************
函数名：PS2_ClearData
功能：清除数据缓冲区，将全局变量Data数组清空
返回值：无
*************************************************************/
void PS2_ClearData(void)
{
	uint8_t a;
	for(a=0;a<9;a++)
		Data[a]=0x00;
}

/************************************************************
函数名： PS2_Vibration
功能：手柄震动函数
参数：
    motor1:右侧小震动电机 0x00关，其他开
	motor2:左侧大震动电机 0x40~0xFF 电机开，值越大 震动越大
返回值：无
*************************************************************/
void PS2_Vibration(uint8_t motor1, uint8_t motor2)
{
	CS_L;
	HAL_Delay_us(16);
  PS2_Cmd(0x01);  //开始命令
	PS2_Cmd(0x42);  //请求数据
	PS2_Cmd(0X00);
	PS2_Cmd(motor1);
	PS2_Cmd(motor2);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	HAL_Delay_us(16);  
}
/************************************************************
函数名：  PS2_ShortPoll
功能：  Ps2恢复和建立连接函数
参数：无
返回值：无
使用注意：无
*************************************************************/
void PS2_ShortPoll(void)
{
	CS_L;
	HAL_Delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x42);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0x00);
	CS_H;
	HAL_Delay_us(16);	
}

/************************************************************
函数名：  PS2_EnterConfing
功能：  Ps2 进入配置函数
参数：无
返回值：无
使用注意：只有先进入配置，发送配置指令才有效
*************************************************************/
void PS2_EnterConfing(void)
{
  CS_L;
	HAL_Delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x43);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x01);
	PS2_Cmd(0x00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	HAL_Delay_us(16);
}

/************************************************************
函数名：  PS2_TurnOnAnalogMode
功能：  发送模式设置，设置为模拟量模式
参数：无
返回值：无
使用注意：可通过修改此函数来更改遥控器的红绿灯模式（模拟和数字模式）
*************************************************************/
void PS2_TurnOnAnalogMode(void)
{
	CS_L;
	PS2_Cmd(0x01);  
	PS2_Cmd(0x44);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x01);          //analog=0x01;digital=0x00  软件设置发送模式
	PS2_Cmd(0xEE);          //Ox03锁存设置，即不可通过按键“MODE”设置模式。
                            //0xEE不锁存软件设置，可通过按键“MODE”设置模式。
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	HAL_Delay_us(16);
}

/************************************************************
函数名：  PS2_VibrationMode
功能：  打开振动模式函数
参数：无
返回值：无
使用注意：
    1. 在手柄初始化时使用
    2. 只有初始化时打开了震动模式，后面发送震动参数才有效
*************************************************************/
void PS2_VibrationMode(void)
{
	CS_L;
	HAL_Delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x4D);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0X01);
	CS_H;
	HAL_Delay_us(16);	
}

/************************************************************
函数名：  PS2_ExitConfing
功能：  完成并保存配置
参数：无
返回值：无
使用注意：无
*************************************************************/
void PS2_ExitConfing(void)
{
  CS_L;
	HAL_Delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x43);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	CS_H;
	HAL_Delay_us(16);
}

/************************************************************
函数名：  PS2_SetInit
功能： 手柄配置初始化
参数：无
返回值：无
使用注意：无
*************************************************************/
void PS2_SetInit(void)
{
	PS2_ShortPoll();        //建立恢复连接
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_EnterConfing();		//进入配置模式
	PS2_TurnOnAnalogMode();	//“红绿灯”配置模式，并选择是否保存
	PS2_VibrationMode();	//开启震动模式
	PS2_ExitConfing();		//完成并保存配置
}

/************************************************************
函数名：  PS2_Receive
功能：Ps2接收器接口IO初始化函数
参数：无
返回值：无
使用注意：无
*************************************************************/
//int PS2_LX, PS2_LY, PS2_RX, PS2_RY;

int PS2_Receive (void)
{
//	PS2_LX=PS2_AnologData(PSS_LX);
//	PS2_LY=PS2_AnologData(PSS_LY);
//	PS2_RX=PS2_AnologData(PSS_RX);
//	PS2_RY=PS2_AnologData(PSS_RY);

	int PS2_KEY;
	PS2_KEY=PS2_DataKey();
	
	return PS2_KEY;
}

// PS2 速度控制
void PS2_Pub_vel(int PS2_KEY)
{
		switch(PS2_KEY)
		{
			case 0:	{
//									Motor_Stop();//停止
//									ps2.X_speed = 0;
//									ps2.Z_speed = 0;
//									Recive_Data.Sensor_str.X_speed = 0;
//									Recive_Data.Sensor_str.Z_speed = 0;
//				Moto1.Target_Speed = 0;
//				Moto2.Target_Speed = 0;
//				Moto3.Target_Speed = 0;
//				Moto4.Target_Speed = 0;
				ps2.X_speed = 0;
									ps2.Z_speed = 0;
							}
				break;
			case 5:	{
//									Motor_Control(3000,3000,3000,3000);//上
//									Recive_Data.Sensor_str.X_speed = 0.5;
//									Recive_Data.Sensor_str.Z_speed = 0;
//				
									ps2.X_speed = 0.5;
									ps2.Z_speed = 0;
//				Moto1.Target_Speed = 0.5;
//				Moto2.Target_Speed = 0.5;
//				Moto3.Target_Speed = 0.5;
//				Moto4.Target_Speed = 0.5;
							}
				break;
			case 7: {
//									Motor_Control(-3000,-3000,-3000,-3000);//下
//									Recive_Data.Sensor_str.X_speed = -0.5;
//									Recive_Data.Sensor_str.Z_speed = 0;
									ps2.X_speed = -0.5;
									ps2.Z_speed = 0;
//				Moto1.Target_Speed = -0.5;
//				Moto2.Target_Speed = -0.5;
//				Moto3.Target_Speed = -0.5;
//				Moto4.Target_Speed = -0.5;
							}
				break;
//			case 8: {
////								PS2_X_speed = 0;
////								PS2_Y_speed = -0.5;
////								PS2_Z_speed = 0;
////				Motor_Control(-300,300,300,-300);
//				Mecanum_Kinematics_Inverse(0,-0.5,0);
//				motor_flag = 1;
//							}//左
//				break;
//			case 6: {
////								PS2_X_speed = 0;
////								PS2_Y_speed = 0.5;
////								PS2_Z_speed = 0;
////			Motor_Control(300,-300,-300,300);
//				Mecanum_Kinematics_Inverse(0,0.5,0);
//				motor_flag = 1;
//							}//右
//				break;
			case 16: {
//									Motor_Control(-3000,3000,-3000,3000);
//									Recive_Data.Sensor_str.X_speed = 0;
//									Recive_Data.Sensor_str.Z_speed = 1;
										ps2.X_speed = 0;
										ps2.Z_speed = 1;
								}//左转
				break;		
			case 14: {
//									Motor_Control(3000,-3000,3000,-3000);
//									Recive_Data.Sensor_str.X_speed = 0;
//									Recive_Data.Sensor_str.Z_speed = -1;
										ps2.X_speed = 0;
										ps2.Z_speed = -1;
								}//右转
				break;	
//			case 13: {
//									Motor_Control(3000,0,0,3000);
//			}//左自转
//				break;		
//			case 15: {
//									Motor_Control(0,-3000,-3000,0);				
//			}//右自转
//				break;	

		}
}










