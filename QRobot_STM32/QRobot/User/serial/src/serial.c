#include "serial.h"
#include "system.h"
#include "motion_model.h"
#include "usart.h"
#include "pid.h"
#include "mpu6050.h"

Upload_Data Send_Data, Recive_Data;

void UART5_Send(uint8_t *buf)
{
	HAL_UART_Transmit(&huart5,buf,1,0xFF);
}




// ��STM32�����ݷ��͸�ROS
void STM32_TO_ROS(void)
{
	// �������ݰ�ͷ/��β
	Send_Data.Sensor_str.Header = PROTOCOL_HEADER;
	Send_Data.Sensor_str.End = PROTOCOL_END;
	
	// �˶�ѧ���⣬���ĸ�������ٶ�M1/M2/M3/M4 ת��Ϊ Vx��w
	FourWheel_car_Motion_Positive(Moto1.Current_Speed, Moto2.Current_Speed, Moto3.Current_Speed, Moto4.Current_Speed);
	
	// ���ĸ�����ĵ�ǰ�ٶȺ�Ŀ���ٶ�Ҳ�ϴ�����������ĵ���
	Send_Data.Sensor_str.MotoStr[0].Moto_CurrentSpeed = Moto1.Current_Speed;
	Send_Data.Sensor_str.MotoStr[0].Moto_TargetSpeed = Moto1.Target_Speed;
	
	Send_Data.Sensor_str.MotoStr[1].Moto_CurrentSpeed = Moto2.Current_Speed;
	Send_Data.Sensor_str.MotoStr[1].Moto_TargetSpeed = Moto2.Target_Speed;
	
	Send_Data.Sensor_str.MotoStr[2].Moto_CurrentSpeed = Moto3.Current_Speed;
	Send_Data.Sensor_str.MotoStr[2].Moto_TargetSpeed = Moto3.Target_Speed;
	
	Send_Data.Sensor_str.MotoStr[3].Moto_CurrentSpeed = Moto4.Current_Speed;
	Send_Data.Sensor_str.MotoStr[3].Moto_TargetSpeed = Moto4.Target_Speed;
	
	// ����MPU6050��ȡ���ٶ�ֵ�ͽ��ٶ�ֵ
	MPU_Get_Accelerometer(&Send_Data.Sensor_str.Accelerometer);
	MPU_Get_Gyroscope(&Send_Data.Sensor_str.Gyroscope);
	
	// ��������
	for (uint8_t i = 0; i < PROTOCOL_DATA_SIZE; i++){
		UART5_Send(&Send_Data.buffer[i]);
	}
}

uint8_t uart5_rxbuff;
uint8_t Rcount = 0;//���ڽ��ռ���
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)//���ڽ��ջص�����
{
	
	if(huart==&huart5)
	{
		Recive_Data.buffer[Rcount] = uart5_rxbuff;
		//�ж����ݰ�ͷ�Ƿ�Ϊ0xB5���ڽ��н��ղ���
		(Recive_Data.buffer[0] == 0xB5)?(Rcount++):(Rcount=0);
				if(Rcount == PROTOCOL_DATA_SIZE)//��֤���ݰ��Ĵ�С
				{
						if(Recive_Data.Sensor_str.Header == PROTOCOL_HEADER)//��֤���ݰ�ͷ��У����Ϣ
						{		
								if(Recive_Data.Sensor_str.End == PROTOCOL_END)//��֤���ݰ�β��У����Ϣ
								{
									HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
									//������ݮ�ɶ��·���Ŀ���ٶ�ֵ							
									FourWheel_car_Motion_Inverse(Recive_Data.Sensor_str.X_speed,Recive_Data.Sensor_str.Z_speed);
//									if((Moto1.Target_Speed==0)&&(Moto2.Target_Speed==0)&&(Moto3.Target_Speed==0)&&(Moto4.Target_Speed==0))//���ֱ�ûĿ���ٶ�ֵ������ƶ�
//									{
//											motor_flag = 0;
//									}
//									else//�����ٶ�ֵʱ��PWM����
//									{
//											motor_flag = 1;
//									}
									
						
									//����PIDֵ
										if((Recive_Data.Sensor_str.PID_Param[0] + Recive_Data.Sensor_str.PID_Param[1] + Recive_Data.Sensor_str.PID_Param[2]) != 0)
										{
												KP = Recive_Data.Sensor_str.PID_Param[0];
												KI = Recive_Data.Sensor_str.PID_Param[1];
												KD = Recive_Data.Sensor_str.PID_Param[2];
										}

								}

						}

						
					Rcount = 0;	
				}
				
				
		HAL_UART_Receive_IT(&huart5,(void *)&uart5_rxbuff,1);
	}
	
	
} 
