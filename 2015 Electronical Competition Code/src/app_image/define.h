#ifndef _DEFINE_H
#define _DEFINE_H


/*************************************************************************
*  模块名称：define模块
*  功能说明：Include 用户自定义的宏
*  参数说明：无
*  函数返回：无
*  修改时间：2012-2-14
*************************************************************************/
//蜂鸣器
#define buzzer  PTC14_OUT
//测试时间的IO口
#define Test_IO PTE28_OUT
//3个LED灯
#define _LED1 PTE24_OUT
#define _LED2 PTE25_OUT
#define _LED3 PTE26_OUT
//拨码开关
#define sw1 PTC6_IN
#define sw2 PTC7_IN
#define sw3 PTC8_IN
#define sw4 PTC9_IN
#define sw5 PTC10_IN
#define sw6 PTC11_IN
#define sw7 PTC12_IN
#define sw8 PTC13_IN
#define sw_PORT ((sw8<<7u)+(sw7<<6u)+(sw6<<5u)+(sw5<<4u)+(sw4<<3u)+(sw3<<2u)+(sw2<<1u)+sw1)
//PWM
#define DIR1            PTC1_OUT
#define DIR2            PTA12_OUT
#define DIR3            PTA13_OUT
#define DIR4            PTA11_OUT



#endif