#include "mpu6050.h"
#include "delay.h"
#include "gpio.h"



void MPU_IIC_Delay(void)
{
    HAL_Delay_us(2);
}

//IIC SDA��������
//���ģʽ data��1
//����ģʽ data��0
void IIC_SDA_DIR(uint8_t data)
{
		GPIO_InitTypeDef GPIO_InitStruct = {0};
		if(data)//���ģʽ
		{
			GPIO_InitStruct.Pin = MPU6050_SDA_Pin;
			GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;//�������
			GPIO_InitStruct.Pull = GPIO_NOPULL;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
			HAL_GPIO_Init(MPU6050_SDA_GPIO_Port, &GPIO_InitStruct);		
		}
		else//����ģʽ
		{
			GPIO_InitStruct.Pin = MPU6050_SDA_Pin;
			GPIO_InitStruct.Mode = GPIO_MODE_INPUT;//����ģʽ
			GPIO_InitStruct.Pull = GPIO_PULLUP;//��������
			HAL_GPIO_Init(MPU6050_SDA_GPIO_Port, &GPIO_InitStruct);	
		}
}


//����IIC��ʼ�ź�
void MPU_IIC_Start(void)
{
    SDA_OUT();     //sda�����
    SDA_H;
    SCL_H;
    MPU_IIC_Delay();
    SDA_L;//START:when CLK is high,DATA change form high to low
    MPU_IIC_Delay();
    SCL_L;//ǯסI2C���ߣ�׼�����ͻ��������
}

//����IICֹͣ�ź�
void MPU_IIC_Stop(void)
{
    SDA_OUT();//sda�����
    SCL_L;
    SDA_L;//STOP:when CLK is high DATA change form low to high
    MPU_IIC_Delay();
		SCL_H;
    SDA_H;//����I2C���߽����ź�
    MPU_IIC_Delay();
}

//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
uint8_t MPU_IIC_Wait_Ack(void)
{
    uint8_t ucErrTime=0;
    SDA_IN();      //SDA����Ϊ����
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
    SCL_L;//ʱ�����0
    return 0;
}

//����ACKӦ��
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

//������ACKӦ��
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

//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��
void MPU_IIC_Send_Byte(uint8_t data)
{
    uint8_t i;
    SDA_OUT();
    SCL_L;//����ʱ�ӿ�ʼ���ݴ���
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

//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK
uint8_t MPU_IIC_Read_Byte(unsigned char ack)
{
    unsigned char i,receive=0;
    SDA_IN();//SDA����Ϊ����
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
        MPU_IIC_NAck();//����nACK
    else
        MPU_IIC_Ack(); //����ACK
    return receive;
}

//IIC����д
//addr:������ַ 
//reg:�Ĵ�����ַ
//len:д�볤��
//buf:������
//����ֵ:0,����
//    ����,�������
uint8_t MPU_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
	uint8_t i; 
    MPU_IIC_Start(); 
	MPU_IIC_Send_Byte((addr<<1)|0);//����������ַ+д����	
	if(MPU_IIC_Wait_Ack())	//�ȴ�Ӧ��
	{
		MPU_IIC_Stop();		 
		return 1;		
	}
    MPU_IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    MPU_IIC_Wait_Ack();		//�ȴ�Ӧ��
	for(i=0;i<len;i++)
	{
		MPU_IIC_Send_Byte(buf[i]);	//��������
		if(MPU_IIC_Wait_Ack())		//�ȴ�ACK
		{
			MPU_IIC_Stop();	 
			return 1;		 
		}		
	}    
    MPU_IIC_Stop();	 
	return 0;	
} 

//IIC������
//addr:������ַ
//reg:Ҫ��ȡ�ļĴ�����ַ
//len:Ҫ��ȡ�ĳ���
//buf:��ȡ�������ݴ洢��
//����ֵ:0,����
//    ����,�������
uint8_t MPU_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{ 
 	MPU_IIC_Start(); 
	MPU_IIC_Send_Byte((addr<<1)|0);//����������ַ+д����	
	if(MPU_IIC_Wait_Ack())	//�ȴ�Ӧ��
	{
		MPU_IIC_Stop();		 
		return 1;		
	}
    MPU_IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    MPU_IIC_Wait_Ack();		//�ȴ�Ӧ��
    MPU_IIC_Start();
	MPU_IIC_Send_Byte((addr<<1)|1);//����������ַ+������	
    MPU_IIC_Wait_Ack();		//�ȴ�Ӧ�� 
	while(len)
	{
		if(len==1)*buf=MPU_IIC_Read_Byte(0);//������,����nACK 
		else *buf=MPU_IIC_Read_Byte(1);		//������,����ACK  
		len--;
		buf++; 
	}    
    MPU_IIC_Stop();	//����һ��ֹͣ���� 
	return 0;	
}

uint8_t IIC_Read_Buff(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)//IIC������
{
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((addr<<1)|0);//����������ַ+д����
    if(MPU_IIC_Wait_Ack())	//�ȴ�Ӧ��
    {
        MPU_IIC_Stop();
        return 1;
    }
    MPU_IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    MPU_IIC_Wait_Ack();		//�ȴ�Ӧ��
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((addr<<1)|1);//����������ַ+������
    MPU_IIC_Wait_Ack();		//�ȴ�Ӧ��
    while(len)
    {
        if(len==1)*buf=MPU_IIC_Read_Byte(0);//������,����nACK
        else *buf=MPU_IIC_Read_Byte(1);		//������,����ACK
        len--;
        buf++;
    }
    MPU_IIC_Stop();	//����һ��ֹͣ����
    return 0;
}

//IICдһ���ֽ�
//reg:�Ĵ�����ַ
//data:����
//����ֵ:0,����
//    ����,�������
uint8_t MPU_Write_Byte(uint8_t reg,uint8_t data)
{
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((MPU_ADDR<<1)|0);//����������ַ+д����
    if(MPU_IIC_Wait_Ack())	//�ȴ�Ӧ��
    {
        MPU_IIC_Stop();
        return 1;
    }
    MPU_IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    MPU_IIC_Wait_Ack();		//�ȴ�Ӧ��
    MPU_IIC_Send_Byte(data);//��������
    if(MPU_IIC_Wait_Ack())	//�ȴ�ACK
    {
        MPU_IIC_Stop();
        return 1;
    }
    MPU_IIC_Stop();
    return 0;
}

//IIC��һ���ֽ�
//reg:�Ĵ�����ַ
//����ֵ:����������
uint8_t MPU_Read_Byte(uint8_t reg)
{
    uint8_t res;
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((MPU_ADDR<<1)|0);//����������ַ+д����
    MPU_IIC_Wait_Ack();		//�ȴ�Ӧ��
    MPU_IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    MPU_IIC_Wait_Ack();		//�ȴ�Ӧ��
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((MPU_ADDR<<1)|1);//����������ַ+������
    MPU_IIC_Wait_Ack();		//�ȴ�Ӧ��
    res=MPU_IIC_Read_Byte(0);//��ȡ����,����nACK
    MPU_IIC_Stop();			//����һ��ֹͣ����
    return res;
}




uint8_t MPU6050_Init(void)
{
	  uint8_t res;
    
    MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80);	//��λMPU6050
    HAL_Delay(100);
    MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);	//����MPU6050
    MPU_Write_Byte(MPU_GYRO_CFG_REG,3<<3);//�����Ǵ���������2000dps
    MPU_Write_Byte(MPU_ACCEL_CFG_REG,0<<3);//���ٶȴ���������2g
		MPU_Write_Byte(MPU_SAMPLE_RATE_REG,0x00);//���ò�����50hz
		MPU_Write_Byte(MPU_CFG_REG,0x04);
    MPU_Write_Byte(MPU_INT_EN_REG,0X00);	//�ر������ж�
    MPU_Write_Byte(MPU_USER_CTRL_REG,0X00);	//I2C��ģʽ�ر�
    MPU_Write_Byte(MPU_FIFO_EN_REG,0X00);	//�ر�FIFO
    MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80);	//INT���ŵ͵�ƽ��Ч
    res=MPU_Read_Byte(MPU_DEVICE_ID_REG);
    if(res==MPU_ADDR)//����ID��ȷ
    {
        MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01);	//����CLKSEL,PLL X��Ϊ�ο�
        MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00);	//���ٶ��������Ƕ�����
			  MPU_Write_Byte(MPU_SAMPLE_RATE_REG,0x00);
    } else return 1;
    return 0;
}

//�õ��¶�ֵ
//����ֵ:�¶�ֵ(������100��)
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

//�õ����ٶ�ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
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


//�õ�������ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
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
