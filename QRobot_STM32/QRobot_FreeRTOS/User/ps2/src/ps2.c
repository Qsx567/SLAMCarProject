#include "ps2.h"
#include "delay.h"
#include "main.h"
#include "motor.h"
#include "system.h"
#include "serial.h"
//ʹ��ģ��spiͨ��


#define DELAY_TIME  HAL_Delay_us(5); 
uint16_t Handkey;	                 // �����ֵ��ȡ��������ʱ�洢��
uint8_t Comd[2]={0x01,0x42};	         // ָ��洢���飺0x01Ϊͨ�ſ�ʼָ�0x42Ϊ��������ָ��
uint8_t Data[9]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; // �������ݴ洢����


// ���尴����
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
��������PS2_Init
���ܣ�Ps2�������ӿ�IO��ʼ������
��������
����ֵ����
ʹ��ע�⣺
    1. IO�ӿڣ�ģ��SPI���������£�
        DI/DATA -> PC2
        DO/COM -> PC1
        CS/ATT -> PC3
        CLK -> PA4
    2. ��ֲʱ��Ҫ�޸Ĵ˺����������䲻ͬӲ����·    
*************************************************************/
void PS2_Init(void)
{
	PS2_SetInit();//�ֱ����ó�ʼ��											  
}
/************************************************************
��������PS2_Cmd
���ܣ����ֱ���������
������CMD�� uint8���ͣ�ָ����
����ֵ����
ʹ��ע�⣺
    1. �ú������������ݶ�д�Ļ�����������DI��DO��ʱ����CLK
       �½��ؽ�������
    2. ����ָ��󷵻ص����ݴ洢����ȫ�ֱ���Data[1]��
*************************************************************/
void PS2_Cmd(uint8_t CMD)
{
	volatile uint16_t ref=0x01;
	Data[1] = 0;
	for(ref=0x01;ref<0x0100;ref<<=1)
	{
		if(ref&CMD)
		{
			DO_H;                   //���һλ����λ
		}
		else DO_L;

		CLK_H;                      //ʱ������
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
��������PS2_RedLight
���ܣ��ж��Ƿ�Ϊ���ģʽ,0x41=ģ���̵ƣ�0x73=ģ����
��������
����ֵ��0�����ģʽ
	    1������ģʽ
ʹ��ע�⣺��
*************************************************************/
uint8_t PS2_RedLight(void)
{
	CS_L;
	PS2_Cmd(Comd[0]);  //��ʼ����
	PS2_Cmd(Comd[1]);  //��������
	CS_H;
	if( Data[1] == 0X73)   return 0 ;
	else return 1;

}
/************************************************************
��������PS2_ReadData
���ܣ���ȡ�ֱ�����
��������
����ֵ����
ʹ��ע�⣺
    1. ���ݶ�д�ڼ�CS������Ҫ���ͣ�����ѡ�и��豸
    2. �����������ݴ洢��Data[1]��Data[8]��
*************************************************************/
void PS2_ReadData(void)
{
	volatile uint8_t byte=0;
	volatile uint16_t ref=0x01;
	CS_L;
	PS2_Cmd(Comd[0]);  //��ʼ����
	PS2_Cmd(Comd[1]);  //��������
	for(byte=2;byte<9;byte++)          //��ʼ��������
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
��������PS2_DataKey
���ܣ��Զ�������PS2�����ݽ��д���,ֻ����������
��������
����ֵ��0��   ���κΰ�������
        �������������
ʹ��ע�⣺����Э���ֲᣬData[3]��Data[4]�洢��16������״̬������
    Data[3]��Ӧ SELECT�� L3 �� R3�� START �� UP�� RIGHT�� DOWN�� LEFT �Ƿ񱻰��£��������¶�ӦλΪ0
    Data[4]��Ӧ L2 �� R2��L1 ��R1�������𡢨w���� �Ƿ񱻰��£��������¶�ӦλΪ0
*************************************************************/
uint8_t PS2_DataKey()
{
	uint8_t index;

	PS2_ClearData();
	PS2_ReadData();

	Handkey=(Data[4]<<8)|Data[3];     //����16������  ����Ϊ0�� δ����Ϊ1
	for(index=0;index<16;index++)
	{	    
		if((Handkey&(1<<(MASK[index]-1)))==0)
            return index+1;
	}
	return 0;          //û���κΰ�������
}


/************************************************************
��������S2_AnologData
���ܣ��õ�һ��ҡ�˵�ģ����	 ��Χ0~256
������button ҡ�˱��
����ֵ��ҡ��ģ������ֵ
ʹ��ע�⣺ֻ���ں��ģʽ��ֵ������Ч�ģ�����ҡ�ˣ���Щֵ�Ż�
    �仯����Щֵ�ֱ�洢��Data[5] Data[6] Data[7] Data[8]
*************************************************************/
uint8_t PS2_AnologData(uint8_t button)
{
	return Data[button];
}

/************************************************************
��������PS2_ClearData
���ܣ�������ݻ���������ȫ�ֱ���Data�������
����ֵ����
*************************************************************/
void PS2_ClearData(void)
{
	uint8_t a;
	for(a=0;a<9;a++)
		Data[a]=0x00;
}

/************************************************************
�������� PS2_Vibration
���ܣ��ֱ��𶯺���
������
    motor1:�Ҳ�С�𶯵�� 0x00�أ�������
	motor2:�����𶯵�� 0x40~0xFF �������ֵԽ�� ��Խ��
����ֵ����
*************************************************************/
void PS2_Vibration(uint8_t motor1, uint8_t motor2)
{
	CS_L;
	HAL_Delay_us(16);
  PS2_Cmd(0x01);  //��ʼ����
	PS2_Cmd(0x42);  //��������
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
��������  PS2_ShortPoll
���ܣ�  Ps2�ָ��ͽ������Ӻ���
��������
����ֵ����
ʹ��ע�⣺��
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
��������  PS2_EnterConfing
���ܣ�  Ps2 �������ú���
��������
����ֵ����
ʹ��ע�⣺ֻ���Ƚ������ã���������ָ�����Ч
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
��������  PS2_TurnOnAnalogMode
���ܣ�  ����ģʽ���ã�����Ϊģ����ģʽ
��������
����ֵ����
ʹ��ע�⣺��ͨ���޸Ĵ˺���������ң�����ĺ��̵�ģʽ��ģ�������ģʽ��
*************************************************************/
void PS2_TurnOnAnalogMode(void)
{
	CS_L;
	PS2_Cmd(0x01);  
	PS2_Cmd(0x44);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x01);          //analog=0x01;digital=0x00  ������÷���ģʽ
	PS2_Cmd(0xEE);          //Ox03�������ã�������ͨ��������MODE������ģʽ��
                            //0xEE������������ã���ͨ��������MODE������ģʽ��
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	HAL_Delay_us(16);
}

/************************************************************
��������  PS2_VibrationMode
���ܣ�  ����ģʽ����
��������
����ֵ����
ʹ��ע�⣺
    1. ���ֱ���ʼ��ʱʹ��
    2. ֻ�г�ʼ��ʱ������ģʽ�����淢���𶯲�������Ч
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
��������  PS2_ExitConfing
���ܣ�  ��ɲ���������
��������
����ֵ����
ʹ��ע�⣺��
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
��������  PS2_SetInit
���ܣ� �ֱ����ó�ʼ��
��������
����ֵ����
ʹ��ע�⣺��
*************************************************************/
void PS2_SetInit(void)
{
	PS2_ShortPoll();        //�����ָ�����
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_EnterConfing();		//��������ģʽ
	PS2_TurnOnAnalogMode();	//�����̵ơ�����ģʽ����ѡ���Ƿ񱣴�
	PS2_VibrationMode();	//������ģʽ
	PS2_ExitConfing();		//��ɲ���������
}

/************************************************************
��������  PS2_Receive
���ܣ�Ps2�������ӿ�IO��ʼ������
��������
����ֵ����
ʹ��ע�⣺��
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

// PS2 �ٶȿ���
void PS2_Pub_vel(int PS2_KEY)
{
		switch(PS2_KEY)
		{
			case 0:	{
//									Motor_Stop();//ֹͣ
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
//									Motor_Control(3000,3000,3000,3000);//��
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
//									Motor_Control(-3000,-3000,-3000,-3000);//��
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
//							}//��
//				break;
//			case 6: {
////								PS2_X_speed = 0;
////								PS2_Y_speed = 0.5;
////								PS2_Z_speed = 0;
////			Motor_Control(300,-300,-300,300);
//				Mecanum_Kinematics_Inverse(0,0.5,0);
//				motor_flag = 1;
//							}//��
//				break;
			case 16: {
//									Motor_Control(-3000,3000,-3000,3000);
//									Recive_Data.Sensor_str.X_speed = 0;
//									Recive_Data.Sensor_str.Z_speed = 1;
										ps2.X_speed = 0;
										ps2.Z_speed = 1;
								}//��ת
				break;		
			case 14: {
//									Motor_Control(3000,-3000,3000,-3000);
//									Recive_Data.Sensor_str.X_speed = 0;
//									Recive_Data.Sensor_str.Z_speed = -1;
										ps2.X_speed = 0;
										ps2.Z_speed = -1;
								}//��ת
				break;	
//			case 13: {
//									Motor_Control(3000,0,0,3000);
//			}//����ת
//				break;		
//			case 15: {
//									Motor_Control(0,-3000,-3000,0);				
//			}//����ת
//				break;	

		}
}










