#include"include.h"


/***************************编码器初始化*****************************/


/************************************************
*  函数名称：SSI_Init
*  功能说明：绝对编码器引脚初始化，所有引脚必需为5V容忍（编码器输出信号为5V）
*  参数说明：无
*  函数返回：无
*  修改时间：2014-1-14    已经测试
*************************************************/
void SSI_Init()
{
    gpio_init(PORTB, 0, GPI, LOW);//使能DO
    gpio_init(PORTB, 1, GPO, LOW);//使能CS
    gpio_init(PORTB, 2, GPO, LOW);//使能CLK 	  
}


/************************************************
*  函数名称：SSIRead
*  功能说明：返回绝对编码器的值,T=34us
*  参数说明：无
*  函数返回：绝对编码器的值
*  修改时间：2014-1-14    已经测试
*************************************************/
unsigned short int SSIRead()
{  
    int	data=0;
    CS_1;
    SSInops();SSInops();SSInops();
    CS_0;
    SSInops();SSInops();SSInops();
    for(int i=0; i<10; i++)
    {
        CLK_1;
        SSInops();SSInops();SSInops();
        if(DataOut)
            data+=1;
        data<<=1;
        CLK_0;
        SSInops();SSInops();SSInops();
    }
    data>>=1;
    return(data);
}
