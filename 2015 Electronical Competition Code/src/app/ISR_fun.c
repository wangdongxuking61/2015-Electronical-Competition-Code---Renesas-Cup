#include "include.h"
#include "math.h"

/*************************************************************************
*  函数名称：Water_LEDs
*  功能说明：自己的4个led灯
*  参数说明：无
*  函数返回：无
*  修改时间：2012-2-14    已测试
*************************************************************************/
void Water_LEDs()
{
    unsigned char Led_num = (Time_1ms/500)%3;
    if(Led_num==0)
    {_LED1 = 0;_LED2 = 1;_LED3 = 1;}
    else if(Led_num==1)
    {_LED1 = 1;_LED2 = 0;_LED3 = 1;}
    else if(Led_num==2)
    {_LED1 = 1;_LED2 = 1;_LED3 = 0;}
}




/*************************************************************************
*  函数名称：Angle_control
*  功能说明：角度融合，计算出角度，计算角度pwm输出，T=600us
*  参数说明：无
*  函数返回：无
*  修改时间：2012-2-14    已测试
*************************************************************************/
#define dt 0.002
#define tg 2         //**跟踪时间:1~4
void Angle_Calculate()
{
    float mG_delta=0;      //和angle_speed一个级别
    
    /*********************** 横着 ************************/
    Angle.a_sinH = Acc_ADC_Data.y/300.0;	//Acc_ADC_Data取值最大约300多
    if(Angle.a_sinH > 1)         Angle.a_sinH = 1;//限幅
    else if(Angle.a_sinH < -1)   Angle.a_sinH = -1;//限幅
    
    Angle.m_angleH = asin(Angle.a_sinH)*57.295779513;//反正弦，由于m_angleH比较小，所以不用acos或者atan
    if(Angle.m_angleH > 90)      Angle.m_angleH = 90;//限幅
    else if(Angle.m_angleH<-90)  Angle.m_angleH = -90;//限幅
    
	//直立车角度融合代码
    mG_delta = (Angle.m_angleH - Angle.G_angleH) / tg;
    Angle.speedH = 0 - Gyro_ADC.x * MPU6050G_s2000dps ;//角速度，单位°/s
    Angle.G_angleH += (mG_delta + Angle.speedH) * dt;//融合后的角度G_angleH
    
    
    /*********************** 竖着 ************************/
    Angle.a_sinL = 0 - Acc_ADC_Data.x/300.0;//Acc_ADC_Data取值最大约300多
    if(Angle.a_sinL > 1)         Angle.a_sinL = 1;//限幅
    else if(Angle.a_sinL < -1)   Angle.a_sinL = -1;//限幅
    
    Angle.m_angleL = asin(Angle.a_sinL)*57.295779513;//反正弦，由于m_angleH比较小，所以不用acos或者atan
    if(Angle.m_angleL > 90)      Angle.m_angleL = 90;//限幅
    else if(Angle.m_angleL<-90)  Angle.m_angleL = -90;//限幅
    
	//直立车角度融合代码
    mG_delta = (Angle.m_angleL - Angle.G_angleL) / tg;
    Angle.speedL = 0 - Gyro_ADC.y * MPU6050G_s2000dps;//角速度，单位°/s
    Angle.G_angleL += (mG_delta + Angle.speedL) * dt;//融合后的角度G_angleL
}



/*************************************************************************
*  函数名称：Mode_PID
*  功能说明：
*  参数说明：无
*  函数返回：无
*  修改时间：2012-2-14    已测试
*************************************************************************/
void Mode_PID()
{
#define Pi 3.1415926535f
    int mode = Mode.mode;
    
    //没进入start状态就不动，是靠按下“start”按键将Mode.startFg[mode]置1
    if(Mode.startFg[mode]==0)
    {
        if(mode != 0 )//0档位是调试4个电机的档位
            Motor[0].PWM = Motor[1].PWM = Motor[2].PWM = Motor[3].PWM = 0;
        return ;
    }
    
    //参数夫赋值Mode.P，I，D是临时PID变量，每一问都是这几个变量
	// Mode.para是可以随便改变的几个变量，具体参考PORTB_IRQHandler()
    Mode.P = Mode.para[mode][0];
    Mode.I = Mode.para[mode][1];
    Mode.D = Mode.para[mode][2];
    
    if(mode == 1)       //第一问指定60cm
        Angle.Length = 60;
    else if(mode == 2) //第二问指定距离，可通过按键设置
        Angle.Length = Mode.para[mode][3];
    else if(mode == 3) //第三问指定60cm
    {
        Angle.Length = 60;
        Angle.DirAngle = Mode.para[mode][3];//可通过按键设置
    }
    else if(mode == 4) //第四问专门设置，P大，D大大大（最重要），i设成0，积分值也清零
    {
        Mode.P = Mode.para[mode][0] = 100;
        Mode.I = Mode.para[mode][1] = 0;
        Mode.D = Mode.para[mode][2] = 200;
        Angle.ErrorI_H = Angle.ErrorI_L = 0;
    }
    else if(mode == 5) //第五问，画圆，圆的直径 Angle.Length 可通过按键设置
    {
        Angle.Length = Mode.para[mode][3];
    }
    
    
    //PID control：具体原理请看总结文档，这些模式的代码都非常像，我感觉区别就是Angle.Goal不一样
	//PID control：具体原理请看总结文档，这些模式的代码都非常像，我感觉区别就是Angle.Goal不一样
	//PID control：具体原理请看总结文档，这些模式的代码都非常像，我感觉区别就是Angle.Goal不一样
	//PID control：具体原理请看总结文档，这些模式的代码都非常像，我感觉区别就是Angle.Goal不一样
	//PID control：具体原理请看总结文档，这些模式的代码都非常像，我感觉区别就是Angle.Goal不一样
    if(mode==1 || mode==2 || mode==4)//1,2,4用同一套代码，mode=4时候Angle.NowL和Angle.NowH都是0罢了
    {
        /*********************** 竖着 ************************/
        //更新Angle.Goal和Angle.Now
        if(mode==4)
            Angle.GoalL = 0;
        else
            Angle.GoalL = Angle.ForceL[Angle.Length] * sin((Time_1ms - Mode.TimeStart)*2*Pi/((float)(Angle.Period)));
        Angle.NowL  = Angle.G_angleL;
        //更新ErrorI，积分限幅
        Angle.ErrorI_L += Mode.I * (Angle.NowL - Angle.GoalL)/10.0;
        if(Angle.ErrorI_L >= Mode.I_max) Angle.ErrorI_L = Mode.I_max;
        //电机输出，对角线的电机一个推、一个吸
        Motor[0].PWM = Mode.P * (Angle.NowL - Angle.GoalL) + Angle.ErrorI_L + Mode.D * Angle.speedL;
        Motor[2].PWM = 0 - Motor[0].PWM;
        
        /*********************** 横着 ************************/
        //更新Angle.Goal和Angle.Now
        Angle.GoalH = 0;
        Angle.NowH  = Angle.G_angleH;
        //更新ErrorI，积分限幅
        Angle.ErrorI_H += Mode.I * (Angle.NowH - Angle.GoalH)/10.0;
        if(Angle.ErrorI_H >= Mode.I_max) Angle.ErrorI_H = Mode.I_max;
        else if(Angle.ErrorI_H <= 0 - Mode.I_max) Angle.ErrorI_H = 0 - Mode.I_max;
        //电机输出，对角线的电机一个推、一个吸
        Motor[1].PWM = Mode.P * (Angle.NowH - Angle.GoalH) + Angle.ErrorI_H + Mode.D * Angle.speedH;
        Motor[3].PWM = 0 - Motor[1].PWM;
    }
    else if(mode == 3)//斜线，具体原理请看总结文档，和1、2、4的区别就是Angle.Goal不一样
    {        
        static float cosAngle,sinAngle;
        cosAngle = cos( 0 - (Angle.DirAngle*10.0+Angle.DirErr[Angle.DirAngle]) * 2* Pi / 360.0);
        sinAngle = sin( 0 - (Angle.DirAngle*10.0+Angle.DirErr[Angle.DirAngle]) * 2* Pi / 360.0);
        
        /*********************** 竖着 ************************/
        //更新Angle.Goal和Angle.Now
        Angle.GoalL = cosAngle * Angle.ForceL[Angle.Length] * sin((Time_1ms - Mode.TimeStart)*2*Pi/((float)(Angle.Period)));
        Angle.NowL  = Angle.G_angleL;
        //更新ErrorI，积分限幅
        Angle.ErrorI_L += Mode.I * (Angle.NowL - Angle.GoalL)/10.0;
        if(Angle.ErrorI_L >= Mode.I_max) Angle.ErrorI_L = Mode.I_max;
        //电机输出，对角线的电机一个推、一个吸
        Motor[0].PWM = Mode.P * (Angle.NowL - Angle.GoalL) + Angle.ErrorI_L + Mode.D * Angle.speedL;
        Motor[2].PWM = 0 - Motor[0].PWM;
        
        /*********************** 横着 ************************/
        //更新Angle.Goal和Angle.Now
        Angle.GoalH = sinAngle * Angle.ForceH[Angle.Length] * sin((Time_1ms - Mode.TimeStart)*2*Pi/((float)(Angle.Period)));
        Angle.NowH  = Angle.G_angleH;
        //更新ErrorI，积分限幅
        Angle.ErrorI_H += Mode.I * (Angle.NowH - Angle.GoalH)/10.0;
        if(Angle.ErrorI_H >= Mode.I_max) Angle.ErrorI_H = Mode.I_max;
        else if(Angle.ErrorI_H <= 0 - Mode.I_max) Angle.ErrorI_H = 0 - Mode.I_max;
        //电机输出，对角线的电机一个推、一个吸
        Motor[1].PWM = Mode.P * (Angle.NowH - Angle.GoalH) + Angle.ErrorI_H + Mode.D * Angle.speedH;
        Motor[3].PWM = 0 - Motor[1].PWM;
    }
    else if(mode==5)//画圆，没有mode=6，因为根本就没做，根本不受风扇干扰。具体原理请看总结文档，和1、2、4的区别就是Angle.Goal不一样
    {        
        /*********************** 竖着 ************************/
        //更新Angle.Goal和Angle.Now
        Angle.GoalL = Angle.ForceL[Angle.Length] * cos((Time_1ms - Mode.TimeStart)*2*Pi/((float)(Angle.Period)));
        Angle.NowL  = Angle.G_angleL;
        //更新ErrorI，积分限幅
        Angle.ErrorI_L += Mode.I * (Angle.NowL - Angle.GoalL)/10.0;
        if(Angle.ErrorI_L >= Mode.I_max) Angle.ErrorI_L = Mode.I_max;
        //电机输出，对角线的电机一个推、一个吸
        Motor[0].PWM = Mode.P * (Angle.NowL - Angle.GoalL) + Angle.ErrorI_L + Mode.D * Angle.speedL;
        Motor[2].PWM = 0 - Motor[0].PWM;
        
        /*********************** 横着 ************************/
        //更新Angle.Goal和Angle.Now
        Angle.GoalH = Angle.ForceH[Angle.Length] * sin((Time_1ms - Mode.TimeStart)*2*Pi/((float)(Angle.Period)));
        Angle.NowH  = Angle.G_angleH;
        //更新ErrorI，积分限幅
        Angle.ErrorI_H += Mode.I * (Angle.NowH - Angle.GoalH)/10.0;
        if(Angle.ErrorI_H >= Mode.I_max) Angle.ErrorI_H = Mode.I_max;
        else if(Angle.ErrorI_H <= 0 - Mode.I_max) Angle.ErrorI_H = 0 - Mode.I_max;
        //电机输出，对角线的电机一个推、一个吸
        Motor[1].PWM = Mode.P * (Angle.NowH - Angle.GoalH) + Angle.ErrorI_H + Mode.D * Angle.speedH;
        Motor[3].PWM = 0 - Motor[1].PWM;
    }
    else if(mode==7)//摄像头定位，和1、2、4的区别就是Angle.Goal不一样
    {   
        /*********************** 竖着 ************************/
        //更新Angle.Goal和Angle.Now 
        //static int tmp=65;
        Angle.GoalL = 360 * atan((Angle.aveL-130)/2.0/72.5)/2.0/Pi;//H = 72.5cm，位置转换为角度的公式
        Angle.NowL  = Angle.G_angleL;
        //更新ErrorI，积分限幅
        Angle.ErrorI_L += Mode.I * (Angle.NowL - Angle.GoalL)/10.0;
        if(Angle.ErrorI_L >= Mode.I_max) Angle.ErrorI_L = Mode.I_max;
        //电机输出，对角线的电机一个推、一个吸
        Motor[0].PWM = Mode.P * (Angle.NowL - Angle.GoalL) + Angle.ErrorI_L + Mode.D * Angle.speedL;
        Motor[2].PWM = 0 - Motor[0].PWM;
        
        /*********************** 横着 ************************/
        //更新Angle.Goal和Angle.Now
        Angle.GoalH = 360 * atan(0 - (Angle.aveH-71)/2.0/72.5)/2.0/Pi;//H = 72.5cm，位置转换为角度的公式
        Angle.NowH  = Angle.G_angleH;
        //更新ErrorI，积分限幅
        Angle.ErrorI_H += Mode.I * (Angle.NowH - Angle.GoalH)/10.0;
        if(Angle.ErrorI_H >= Mode.I_max) Angle.ErrorI_H = Mode.I_max;
        else if(Angle.ErrorI_H <= 0 - Mode.I_max) Angle.ErrorI_H = 0 - Mode.I_max;
        //电机输出，对角线的电机一个推、一个吸
        Motor[1].PWM = Mode.P * (Angle.NowH - Angle.GoalH) + Angle.ErrorI_H + Mode.D * Angle.speedH;
        Motor[3].PWM = 0 - Motor[1].PWM; 
    }
}



/*************************************************************************
*  函数名称：Motor_Control
*  功能说明：pwm输出
*  参数说明：无
*  函数返回：无
*  修改时间：2012-2-14    已测试
*************************************************************************/
void Motor_Out()
{
    static int pwm_Dead[4]={0,0,0,0};//死区简单设成0
    static float pwm_Limit[4]={2400.0,2400.0,2400.0,2400.0};//4个电机输出限幅

    //偏角过大保护，第七问运行时不设保护
    if(Angle.G_angleL > 35.0f || Angle.G_angleL<-35.0f  || Angle.G_angleH > 35.0f || Angle.G_angleH<-35.0f || (Mode.mode==7 && Angle.startImage == 0))
        Motor[0].PWM = Motor[1].PWM = Motor[2].PWM = Motor[3].PWM = 0;
    
	//以下是4个电机的pwm赋值，对角线的电机一个推、一个吸
	//以下是4个电机的pwm赋值，对角线的电机一个推、一个吸
	//以下是4个电机的pwm赋值，对角线的电机一个推、一个吸
	
    //PWM1
    if(Motor[0].PWM >= 0)
    {
        Motor[0].PWM += pwm_Dead[0]; //加死区
        if(Motor[0].PWM >= pwm_Limit[0])   //限幅
            Motor[0].PWM = pwm_Limit[0];
        FTM_PWM_Duty(FTM0, CH2 ,  (unsigned int)(Motor[0].PWM));//输出
        DIR1 = 0;
    }
    else
    {
        Motor[0].PWM -= pwm_Dead[2]; //加死区
        if(Motor[0].PWM <= -pwm_Limit[0])  //限幅
            Motor[0].PWM = -pwm_Limit[0];
        FTM_PWM_Duty(FTM0, CH2 , (unsigned int)(3990.0 +  Motor[0].PWM)  );//输出
        DIR1 = 1;
    }
    
    //PWM2				
    if(Motor[1].PWM >= 0)
    {
        Motor[1].PWM += pwm_Dead[1]; //加死区
        if(Motor[1].PWM >= pwm_Limit[1])   //限幅
            Motor[1].PWM = pwm_Limit[1];
        FTM_PWM_Duty(FTM0, CH3 ,  ((unsigned int)(Motor[1].PWM)));//输出
        DIR2 = 0;
    }
    else
    {
        Motor[1].PWM -= pwm_Dead[1]; //加死区
        if(Motor[1].PWM <= -pwm_Limit[1])  //限幅
            Motor[1].PWM = -pwm_Limit[1];
        FTM_PWM_Duty(FTM0, CH3 , (unsigned int)(4000u + Motor[1].PWM)  );//输出
        DIR2 = 1;
    }
    
    //PWM3				
    if(Motor[2].PWM >= 0)
    {
        Motor[2].PWM += pwm_Dead[2]; //加死区
        if(Motor[2].PWM >= pwm_Limit[2])   //限幅
            Motor[2].PWM = pwm_Limit[2];
        FTM_PWM_Duty(FTM0, CH4 ,  (unsigned int)(Motor[2].PWM));//输出
        DIR3 = 0;
    }
    else
    {
        Motor[2].PWM -= pwm_Dead[2]; //加死区
        if(Motor[2].PWM <= -pwm_Limit[2])  //限幅
            Motor[2].PWM = -pwm_Limit[2];
        FTM_PWM_Duty(FTM0, CH4 , (unsigned int)(4000u +  Motor[2].PWM)  );//输出
        DIR3 = 1;
    }

    //PWM4				
    if(Motor[3].PWM >= 0)
    {
        Motor[3].PWM += pwm_Dead[3]; //加死区
        if(Motor[3].PWM >= pwm_Limit[3])   //限幅
            Motor[3].PWM = pwm_Limit[3];
        FTM_PWM_Duty(FTM2, CH0 ,  (unsigned int)(Motor[3].PWM));//输出
        DIR4 = 0;
    }
    else
    {
        Motor[3].PWM -= pwm_Dead[3]; //加死区
        if(Motor[3].PWM <= -pwm_Limit[3])  //限幅
            Motor[3].PWM = -pwm_Limit[3];
        FTM_PWM_Duty(FTM2, CH0 , (unsigned int)(4000u +  Motor[3].PWM)  );//输出
        DIR4 = 1;
    }
}


