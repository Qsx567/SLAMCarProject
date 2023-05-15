#include "mpu6050.h"
#include "delay.h"
#include "gpio.h"



void MPU_IIC_Delay(void)
{
    HAL_Delay_us(2);
}

//IIC SDA方向设置
//输出模式 data：1
//输入模式 data：0
void IIC_SDA_DIR(uint8_t data)
{
		GPIO_InitTypeDef GPIO_InitStruct = {0};
		if(data)//输出模式
		{
			GPIO_InitStruct.Pin = MPU6050_SDA_Pin;
			GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;//推挽输出
			GPIO_InitStruct.Pull = GPIO_NOPULL;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
			HAL_GPIO_Init(MPU6050_SDA_GPIO_Port, &GPIO_InitStruct);		
		}
		else//输入模式
		{
			GPIO_InitStruct.Pin = MPU6050_SDA_Pin;
			GPIO_InitStruct.Mode = GPIO_MODE_INPUT;//输入模式
			GPIO_InitStruct.Pull = GPIO_PULLUP;//上拉输入
			HAL_GPIO_Init(MPU6050_SDA_GPIO_Port, &GPIO_InitStruct);	
		}
}


//产生IIC起始信号
void MPU_IIC_Start(void)
{
    SDA_OUT();     //sda线输出
    SDA_H;
    SCL_H;
    MPU_IIC_Delay();
    SDA_L;//START:when CLK is high,DATA change form high to low
    MPU_IIC_Delay();
    SCL_L;//钳住I2C总线，准备发送或接收数据
}

//产生IIC停止信号
void MPU_IIC_Stop(void)
{
    SDA_OUT();//sda线输出
    SCL_L;
    SDA_L;//STOP:when CLK is high DATA change form low to high
    MPU_IIC_Delay();
		SCL_H;
    SDA_H;//发送I2C总线结束信号
    MPU_IIC_Delay();
}

//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
uint8_t MPU_IIC_Wait_Ack(void)
{
    uint8_t ucErrTime=0;
    SDA_IN();      //SDA设置为输入
    SDA_H;
    MPU_IIC_Delay();
    SCL_H;
    MPU_IIC_Delay();
    while(SDA_GetData)
    {
        ucErrTime++;
        if(ucErrTime>250)
        {
            MPU_IIC_Stop();
            return 1;
        }
    }
    SCL_L;//时钟输出0
    return 0;
}

//产生ACK应答
void MPU_IIC_Ack(void)
{
    SCL_L;
    SDA_OUT();
    SDA_L;
    MPU_IIC_Delay();
    SCL_H;
    MPU_IIC_Delay();
    SCL_L;
}

//不产生ACK应答
void MPU_IIC_NAck(void)
{
    SCL_L;
    SDA_OUT();
    SDA_H;
    MPU_IIC_Delay();
    SCL_H;
    MPU_IIC_Delay();
    SCL_L;
}

//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答
void MPU_IIC_Send_Byte(uint8_t data)
{
    uint8_t i;
    SDA_OUT();
    SCL_L;//拉低时钟开始数据传输
    for(i=0; i<8; i++)
    {
			 if(data & 0x80)	SDA_H;
			 else							SDA_L;
			 data <<= 1;
			 SCL_H;
			 MPU_IIC_Delay();
			 SCL_L;
			 MPU_IIC_Delay();	
    }
}

//读1个字节，ack=1时，发送ACK，ack=0，发送nACK
uint8_t MPU_IIC_Read_Byte(unsigned char ack)
{
    unsigned char i,receive=0;
    SDA_IN();//SDA设置为输入
    for(i=0; i<8; i++ )
    {
        SCL_L;
        MPU_IIC_Delay();
        SCL_H;
        receive<<=1;
        if(SDA_GetData)receive++;
        MPU_IIC_Delay();
    }
    if (!ack)
        MPU_IIC_NAck();//发送nACK
    else
        MPU_IIC_Ack(); //发送ACK
    return receive;
}

//IIC连续写
//addr:器件地址 
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:0,正常
//    其他,错误代码
uint8_t MPU_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
	uint8_t i; 
    MPU_IIC_Start(); 
	MPU_IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令	
	if(MPU_IIC_Wait_Ack())	//等待应答
	{
		MPU_IIC_Stop();		 
		return 1;		
	}
    MPU_IIC_Send_Byte(reg);	//写寄存器地址
    MPU_IIC_Wait_Ack();		//等待应答
	for(i=0;i<len;i++)
	{
		MPU_IIC_Send_Byte(buf[i]);	//发送数据
		if(MPU_IIC_Wait_Ack())		//等待ACK
		{
			MPU_IIC_Stop();	 
			return 1;		 
		}		
	}    
    MPU_IIC_Stop();	 
	return 0;	
} 

//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
uint8_t MPU_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{ 
 	MPU_IIC_Start(); 
	MPU_IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令	
	if(MPU_IIC_Wait_Ack())	//等待应答
	{
		MPU_IIC_Stop();		 
		return 1;		
	}
    MPU_IIC_Send_Byte(reg);	//写寄存器地址
    MPU_IIC_Wait_Ack();		//等待应答
    MPU_IIC_Start();
	MPU_IIC_Send_Byte((addr<<1)|1);//发送器件地址+读命令	
    MPU_IIC_Wait_Ack();		//等待应答 
	while(len)
	{
		if(len==1)*buf=MPU_IIC_Read_Byte(0);//读数据,发送nACK 
		else *buf=MPU_IIC_Read_Byte(1);		//读数据,发送ACK  
		len--;
		buf++; 
	}    
    MPU_IIC_Stop();	//产生一个停止条件 
	return 0;	
}

uint8_t IIC_Read_Buff(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)//IIC连续读
{
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令
    if(MPU_IIC_Wait_Ack())	//等待应答
    {
        MPU_IIC_Stop();
        return 1;
    }
    MPU_IIC_Send_Byte(reg);	//写寄存器地址
    MPU_IIC_Wait_Ack();		//等待应答
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((addr<<1)|1);//发送器件地址+读命令
    MPU_IIC_Wait_Ack();		//等待应答
    while(len)
    {
        if(len==1)*buf=MPU_IIC_Read_Byte(0);//读数据,发送nACK
        else *buf=MPU_IIC_Read_Byte(1);		//读数据,发送ACK
        len--;
        buf++;
    }
    MPU_IIC_Stop();	//产生一个停止条件
    return 0;
}

//IIC写一个字节
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码
uint8_t MPU_Write_Byte(uint8_t reg,uint8_t data)
{
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((MPU_ADDR<<1)|0);//发送器件地址+写命令
    if(MPU_IIC_Wait_Ack())	//等待应答
    {
        MPU_IIC_Stop();
        return 1;
    }
    MPU_IIC_Send_Byte(reg);	//写寄存器地址
    MPU_IIC_Wait_Ack();		//等待应答
    MPU_IIC_Send_Byte(data);//发送数据
    if(MPU_IIC_Wait_Ack())	//等待ACK
    {
        MPU_IIC_Stop();
        return 1;
    }
    MPU_IIC_Stop();
    return 0;
}

//IIC读一个字节
//reg:寄存器地址
//返回值:读到的数据
uint8_t MPU_Read_Byte(uint8_t reg)
{
    uint8_t res;
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((MPU_ADDR<<1)|0);//发送器件地址+写命令
    MPU_IIC_Wait_Ack();		//等待应答
    MPU_IIC_Send_Byte(reg);	//写寄存器地址
    MPU_IIC_Wait_Ack();		//等待应答
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((MPU_ADDR<<1)|1);//发送器件地址+读命令
    MPU_IIC_Wait_Ack();		//等待应答
    res=MPU_IIC_Read_Byte(0);//读取数据,发送nACK
    MPU_IIC_Stop();			//产生一个停止条件
    return res;
}




uint8_t MPU6050_Init(void)
{
	  uint8_t res;
    
    MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80);	//复位MPU6050
    HAL_Delay(100);
    MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);	//唤醒MPU6050
    MPU_Write_Byte(MPU_GYRO_CFG_REG,3<<3);//陀螺仪传感器，±2000dps
    MPU_Write_Byte(MPU_ACCEL_CFG_REG,0<<3);//加速度传感器，±2g
		MPU_Write_Byte(MPU_SAMPLE_RATE_REG,0x00);//设置采样率50hz
		MPU_Write_Byte(MPU_CFG_REG,0x04);
    MPU_Write_Byte(MPU_INT_EN_REG,0X00);	//关闭所有中断
    MPU_Write_Byte(MPU_USER_CTRL_REG,0X00);	//I2C主模式关闭
    MPU_Write_Byte(MPU_FIFO_EN_REG,0X00);	//关闭FIFO
    MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80);	//INT引脚低电平有效
    res=MPU_Read_Byte(MPU_DEVICE_ID_REG);
    if(res==MPU_ADDR)//器件ID正确
    {
        MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01);	//设置CLKSEL,PLL X轴为参考
        MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00);	//加速度与陀螺仪都工作
			  MPU_Write_Byte(MPU_SAMPLE_RATE_REG,0x00);
    } else return 1;
    return 0;
}

//得到温度值
//返回值:温度值(扩大了100倍)
int16_t MPU_Get_Temperature(void)
{
    uint8_t buf[2]; 
    int16_t raw;
	float temp;
	MPU_Read_Len(MPU_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
    raw=((uint16_t)buf[0]<<8)|buf[1];  
    temp=36.53+((double)raw)/340;  
    return temp*100;;
}

//得到加速度值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
uint8_t MPU_Get_Accelerometer(MPU6050_Str* Acc)
{
    uint8_t buf[6],res;  
	res=MPU_Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		Acc->X_data = ((uint16_t)buf[0]<<8)|buf[1];  
		Acc->Y_data = ((uint16_t)buf[2]<<8)|buf[3];  
		Acc->Z_data = ((uint16_t)buf[4]<<8)|buf[5];
	} 	
    return res;
}


//int16_t mpu_acc_x,mpu_acc_y,mpu_acc_z;
//int16_t mpu_gyro_x,mpu_gyro_y,mpu_gyro_z;


//void Get_AccData(void)
//{
//    uint8_t datatemp[6]={0},res;
//    res=IIC_Read_Buff(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,datatemp);
//    if(res==0)
//    {
//        mpu_acc_x = (int16_t)((datatemp[0]<<8)|datatemp[1]);
//        mpu_acc_y = (int16_t)((datatemp[2]<<8)|datatemp[3]);
//        mpu_acc_z = (int16_t)((datatemp[4]<<8)|datatemp[5]);	
//    }
//    
//}


//得到陀螺仪值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
uint8_t MPU_Get_Gyroscope(MPU6050_Str* Gyro)
{
    uint8_t buf[6],res;  
	res=MPU_Read_Len(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		Gyro->X_data = ((uint16_t)buf[0]<<8)|buf[1];  
		Gyro->Y_data = ((uint16_t)buf[2]<<8)|buf[3];  
		Gyro->Z_data = ((uint16_t)buf[4]<<8)|buf[5];
	} 	
    return res;
}

//void Get_Gyro(void)
//{
//	uint8_t datatemp[6]={0},res;
//    res=IIC_Read_Buff(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,datatemp);
//    if(res==0)
//    {
//				mpu_gyro_x = (int16_t)((datatemp[0]<<8)|datatemp[1]);
//				mpu_gyro_y = (int16_t)((datatemp[2]<<8)|datatemp[3]);
//				mpu_gyro_z = (int16_t)((datatemp[4]<<8)|datatemp[5]);
//    }
//  
//}
