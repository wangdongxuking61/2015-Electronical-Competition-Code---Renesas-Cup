#ifndef __INCLUDE_H__
#define __INCLUDE_H__


#include  "common.h"
#include  "define.h"

/*************************************************************************
*  模块名称：没有名称
*  功能说明：用户自定义的头文件
*  参数说明：无
*  函数返回：无
*  修改时间：2012-2-14
*************************************************************************/
#include  "gpio.h"      //IO口操作
#include  "uart.h"      //串口
#include  "adc.h"       //ADC模块
#include  "FTM.h"       //FTM模块（FTM0：电机控制 / 通用 /PWM     FTM1、2：正交解码 / 通用 /PWM ）
#include  "PIT.h"       //周期中断计时器
#include  "lptmr.h"     //低功耗定时器(延时)
#include  "exti.h"      //EXTI外部GPIO中断
#include  "arm_math.h"  //DSP库
#include  "delay.h"
#include  "OV7725.h"
#include  "dma.h"
#include  "ff.h"
#include  "flash.h"
#include  "stdio.h"

#include  "Car_init.h"
#include  "LCD.h"
#include  "IIC.h"
#include  "ISR_fun.h"
#include  "math.h"
#include  "ccd.h"

/*************************************************************************
*  模块名称：结构体和变量模块
*  功能说明：Include 用户自定义的结构体和变量
*  参数说明：无
*  函数返回：无
*  修改时间：2012-2-14
*************************************************************************/
struct CarThreeNum//三元组，好多数据都是三元组
{
    float x;
    float y;
    float z;    
};
struct CarAngle//角度的一些数据
{
    float roll;	//四轴算法的数据，最后没用到这个数据，不用看
    float pitch;//四轴算法的数据，最后没用到这个数据，不用看
    float yaw;	//四轴算法的数据，最后没用到这个数据，不用看
    
    float rotation;//四轴算法的数据，最后没用到这个数据，不用看
    float vertical;//四轴算法的数据，最后没用到这个数据，不用看
    
    float GoalH,GoalL;//H代表行，L代表列，GoalH是行方向的目标角度,单位°
    float NowH,NowL;	//NowH是行方向的当前实际角度，只是算pid用到了，其实和m_angleH是一个东西，其中有句代码是NowH=G_angleH,单位°
    float ErrorI_H,ErrorI_L;//ErrorI_H是行方向i参数的积分,有限幅的
    
    float a_sinH,a_sinL;//a_sinH是角度的sin值，取值[-1,1]之间
    float m_angleH,m_angleL;//a_sinH取反正弦得到的角度值，注意用反正弦，不用反余弦或者反正切，角度很小时反正弦比较准,单位°
    float G_angleH,G_angleL;//G_angleH是行方向的当前实际角度，算pid时把值赋给了NowH，其实两者是一个东西，也是比较匆忙没改,单位°
    float speedH,speedL;//speedH是H方向的角速度，单位°/s
    
    int Length;//摆动长度，对于基础部分来说就是长度，对于画圆来说就是直径
    float ForceH[100],ForceL[100];//ForceH，行方向的摆动长度对应的振幅大小
	
    int Period;//自然摆动的周期，T=1500ms，直接拿来用应该就可以。具体原理请看文档的总结（虽然写的也不是很清楚）
	//因为是模拟自然摆动，如果不合适的话要实测，摆动n次除以n即可
	
    int DirAngle;//基础部分第三问，要设置的偏角，取值0~35，对应0°~350°
    float DirAngleRotation;//没用到
    float DirErr[40];//角度矫正列表，原理请看文档总结
    
    float aveH,aveL;//用于发挥第三问，摄像头定位，aveH好aveL对应苍蝇拍黑色头部的位置
    int startImage;//Motor_Out()调用了，在发挥第三问的时候，不开启偏角过大保护，因为苍蝇拍可能离得位置比较远，所以把保护关掉
};
struct CarFourNum//姿态四元组，最后没用到
{
    float q1;
    float q2;
    float q3;
    float q4;    
};
struct CarMotor
{
    int speed;				//没用到
    float PWM;        	  //给电机的PWM
};
struct CarLcd	//显示屏专用
{
    int level;//当前处于第几级
    int Add;//Address，当前处于第几行
    int flushFg;//是否刷屏的Flag
    int AddMax[2];//Add的最大值，每级对应一个最大值
};
struct CarMode
{
    int mode;//代表模式几，0档位用于改变pwm测试4个电机（用于测试），1~4对应基础部分，5~7对应发挥部分
    long int TimeStart;//记录开始按键按下的时间，此模式下的startFg置1
    int I_max;//PID计算的积分限幅
    float P;//PID计算的P
    float I;//PID计算的I
    float D;//PID计算的D
    int para[10][4];//可以被按键改变的临时数组，这个数组通常存储pid这类变量，然后赋值给变量
    int startFg[10];//每个模式都有个标志位，0代表不开始，1代表开始了这个模式
};


/*************************************************************************
*  模块名称：没有名称
*  功能说明：Include 用户自定义的全局变量声明
*  参数说明：无
*  函数返回：无
*  修改时间：2012-2-14
*************************************************************************/
extern struct CarThreeNum  Acc_ADC_Data,Acc_ADC,Acc_Offset;
extern struct CarThreeNum  Gyro_dps,Gyro_ADC,Gyro_Offset;
extern struct CarAngle Angle;
extern struct CarFourNum  Q;
extern struct CarThreeNum Gravity;

extern struct CarMotor Motor[4];
extern struct CarLcd Lcd;
extern struct CarMode Mode;
extern long int    Time_1ms;


/***************** ucos 专用 *****************/
#define USOC_EN     0u      //0为禁止uC/OS，大于0则启动uC/OS
#if USOC_EN > 0u
#include  "ucos_ii.h"  		//uC/OS-II系统函数头文件
#include  "BSP.h"			//与开发板相关的函数
#include  "app.h"			//用户任务函数


#endif  //if  USOC_EN > 0


#endif  //__INCLUDE_H__
