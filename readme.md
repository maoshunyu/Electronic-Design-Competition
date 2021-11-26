<!--
 * @Descripttion: 
 * @version: 
 * @Author: Maoshunyu
 * @Date: 2021-10-24 15:27:27
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2021-11-26 10:10:13
-->

+ 运动部分
  + tim6 时钟
  + tim1 PA8 PA9 --- 左后轮
  + tim3 PA6 PA7 --- 右后轮
  + tim4 PB6 PB7 --- 左前轮
  + tim8 PC6 PC7 --- 右前轮
  + tim2 pwm 
    + PA0 --- 左后轮PWM -- CH1
    + PA1 --- 右后轮PWM -- CH2
    + PB10 --- 左前轮PWM -- CH3
    + PB11 --- 右前轮PWM -- CH4
  + PB4高  PB5低  ---  左后轮前进
  + PB3高 PC14低  ---  右后轮前进
  + PB14高 PB15低 --- 右前轮前进
  + PB12高 PB13低  ---  左前轮前进 
+ 通信部分
  + PA2 --- zigbee TX USART2
  + PA3 --- zigbee RX USART2
  + PC10 --- 陀螺仪TX   USART3
  + PC11 --- 陀螺仪RX  
白黄绿黑左
黑灰自白右






---------

#### pwm正常 count正常 speed突变

#### 接电池不发送

