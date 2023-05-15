#ifndef __PS2_H__
#define __PS2_H__
#include "main.h"
#include "gpio.h"

#define DI     HAL_GPIO_ReadPin(PS2_DAT_GPIO_Port,PS2_DAT_Pin)                //����DATA

#define DO_H   HAL_GPIO_WritePin(PS2_COM_GPIO_Port,PS2_COM_Pin,GPIO_PIN_SET)    //COM����λ��
#define DO_L   HAL_GPIO_WritePin(PS2_COM_GPIO_Port,PS2_COM_Pin,GPIO_PIN_RESET)  //����λ��

#define CS_H   HAL_GPIO_WritePin(PS2_ATT_GPIO_Port,PS2_ATT_Pin,GPIO_PIN_SET)    //ATT����
#define CS_L   HAL_GPIO_WritePin(PS2_ATT_GPIO_Port,PS2_ATT_Pin,GPIO_PIN_RESET)  //ATT����

#define CLK_H  HAL_GPIO_WritePin(PS2_CLK_GPIO_Port,PS2_CLK_Pin,GPIO_PIN_SET)     //ʱ������
#define CLK_L  HAL_GPIO_WritePin(PS2_CLK_GPIO_Port,PS2_CLK_Pin,GPIO_PIN_RESET)   //ʱ������

//These are our button constants
#define PSB_SELECT      1
#define PSB_L3          2
#define PSB_R3          3
#define PSB_START       4
#define PSB_PAD_UP      5
#define PSB_PAD_RIGHT   6
#define PSB_PAD_DOWN    7
#define PSB_PAD_LEFT    8
#define PSB_L2          9
#define PSB_R2          10
#define PSB_L1          11
#define PSB_R1          12
#define PSB_GREEN       13
#define PSB_RED         14
#define PSB_BLUE        15
#define PSB_PINK        16

#define PSB_TRIANGLE    13
#define PSB_CIRCLE      14
#define PSB_CROSS       15
#define PSB_SQUARE      16

//#define WHAMMY_BAR		8

//These are stick values
#define PSS_RX 5                //��ҡ��X������
#define PSS_RY 6
#define PSS_LX 7
#define PSS_LY 8

extern uint8_t Data[9];
extern uint16_t MASK[16];
extern uint16_t Handkey;

void PS2_Init(void);
uint8_t PS2_RedLight(void);                      //�ж��Ƿ�Ϊ���ģʽ
void PS2_ReadData(void);                    //���ֱ�����
void PS2_Cmd(uint8_t CMD);		                //���ֱ���������
uint8_t PS2_DataKey(void);		                //����ֵ��ȡ
uint8_t PS2_AnologData(uint8_t button);               //�õ�һ��ҡ�˵�ģ����
void PS2_ClearData(void);	                //������ݻ�����
void PS2_Vibration(uint8_t motor1, uint8_t motor2);   //������motor1  0xFF���������أ�motor2  0x40~0xFF

void PS2_EnterConfing(void);	            //��������
void PS2_TurnOnAnalogMode(void);            //����ģ����
void PS2_VibrationMode(void);               //������
void PS2_ExitConfing(void);	                //�������
void PS2_SetInit(void);		                //���ó�ʼ��
int PS2_Receive (void);

void PS2_Pub_vel(int PS2_KEY);



#endif
