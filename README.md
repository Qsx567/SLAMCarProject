# SLAMCarProject
### 本项目为我设计的Q-Robot，其中包括**底层控制器**（基于STM32F103RCT),上层控制器为**树莓派4b**。
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

已完成情况：
- [x]  电机控制（PWM）相关代码
- [x]  编码器数据的采集以及转化为速度m/s
