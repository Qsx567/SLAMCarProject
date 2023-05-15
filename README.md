# SLAMCarProject
### 本项目为我设计的Q-Robot，其中包括**底层控制器**（基于STM32F103RCT6),上层控制器为**树莓派4b**。

![图片](./pics/plate1.png#pic_center=60x60)



## 1、机器人底盘控制器--> STM32
- 电机控制（motor）:TIM5/TIM8
  - Motor1：TIM8_CH1/TIM8_CH2
  - Motor2：TIM5_CH3/TIM5_CH4
  - Motor3：TIM8_CH3/TIM8_CH4
  - Motor4：TIM5_CH1/TIM8_CH2
- 编码器（Encoder）：TIM1/TIM2/TIM3/TIM4
  - Encoder1:TIM1
  - Encoder2:TIM2
  - Encoder3:TIM3
  - Encoder4:TIM4
- PS2遥控器
  - PC0 ->Input    PS2_DAT
  - PC1 ->Output   PS2_COM
  - PC2 ->Output   PS2_ATT
  - PC3 ->OutPut   PS2_CLK
- 串口（UART5）
  - PD2  --> U5_RX
  - PC12 --> U5_TX

已完成情况：
- [x]  添加电机控制（PWM）相关代码
- [x]  添加编码器数据的采集以及转化为速度m/s
- [x]  添加机器人运动模型（四轮模型）运动学逆解
- [x]  添加PID控制算法（增量式） 
- [x]  添加PS2遥控器相关代码
- [x]  添加将底层控制数据通过串口发送给ROS端

