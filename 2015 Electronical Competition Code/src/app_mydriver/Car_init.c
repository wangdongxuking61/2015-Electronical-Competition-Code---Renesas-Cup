#include"include.h"




/************************************************
*  函数名称：System_Init
*  功能说明：系统所有功能初始化函数
*  参数说明：无
*  函数返回：无
*  修改时间：2014-1-14    no测试
*************************************************/
void System_Init()
{
    //buzzer初始化
    gpio_init(PORTC,14, GPO, LOW);
    Buzzer_Ring_ms(100);

    
    //LED:全灭
    gpio_init(PORTE,24, GPO, HIGH);
    gpio_init(PORTE,25, GPO, HIGH);
    gpio_init(PORTE,26, GPO, HIGH);
    
    
    //做时间测试
    gpio_init(PORTE,28, GPO, LOW);
    //紧急停车指令接口
    //gpio_init(PORTE,8, GPI, LOW);
        
    
    //LCD模块初始化
    LCD_init(TRUE);    //TRUE  和 FALSE是正着还是反着刷屏 
    Disp_single_colour(Yellow);//Black
    //Print_child();    //小破孩刷屏
    
    //拨码开关端口初始化
    Switch_init();
    //重要参数初始化
    Parameters_init_CAR();

    
    //串口初始化
    uart_init(UART0,115200);
    uart_irq_DIS(UART0);//(比赛要DIS，不能是EN)
    
    
//    //CCD初始化
//    adc_init(ADC0, SE8);//PB0
//    gpio_init(PORTB, 1, GPO, LOW);
//    gpio_init(PORTB, 2, GPO, LOW);
    
    
    //1ms的定时器
    pit_init_ms(PIT0, 1);
    set_irq_priority(68,1);
    enable_irq(68);
    
    
    MPU6050_Init(1000,42);
    /*    
    //初始化陀螺仪L3G4200D,初始化加速度计mma8451
    static unsigned char ID1,ID2;
    Init_L3G4200D();//11010011	
    ID1 = Single_Read(L3G4200_Addr,WHO_AM_I);
    MMA845x_init();//00011010
    ID2 = Single_Read(MMA845x_IIC_ADDRESS,WHO_AM_I_REG);
    if(ID1 == 0xD3 && ID2==0x1A)
    {
        LCD_PutString(0,120,"L3G + MMA OK!",Red,Green);
        //printf("IIC OK!!!");
    }
    else
    {
        //printf("IIC ERROR!!!");
    }
    */
    
    //pwm模块,编码器测速初始化
    Pwm_Port_init();
    //Decoder_init();
    //SSI_Init();
    
    //按键
    exti_init(PORTB, 8,falling_up);    //按键中断，下降沿触发中断
    exti_init(PORTB, 6,falling_up);    //按键中断，下降沿触发中断
    exti_init(PORTB, 9,falling_up);    //按键中断，下降沿触发中断
    exti_init(PORTB,10,falling_up);    //按键中断，下降沿触发中断
    exti_init(PORTB, 7,falling_up);    //按键中断，下降沿触发中断
    exti_init(PORTB, 5,falling_up);    //按键中断，下降沿触发中断
    exti_init(PORTB, 3,falling_up);    //按键中断，下降沿触发中断
    set_irq_priority(88,2);
    enable_irq(88); 			//打开PTB的中断

    
    //结束初始化
    delayms(100);
    Buzzer_Ring_ms(100);
}



/************************************************
*  函数名称：Parameters_init_CAR2
*  功能说明：重要变量初始化(CAR2)
*  参数说明：无
*  函数返回：无
*  修改时间：2014-1-14    已经测试
*************************************************/
void Parameters_init_CAR()
{
    for(int loop=1;loop<=7;loop++)
    {
        Mode.para[loop][0] = 100;//P
        Mode.para[loop][1] = 3;  //I
        Mode.para[loop][2] = 52; //D
        Mode.para[loop][3] = 30; //Length适用于第二问，Dir适用于第三问
        Mode.I_max = 2800;
    }
    
    //摆动的周期
    Angle.Period = 1500;
     //DIR为第三问
    Mode.para[3][3] = 0;
    //第五问
    Mode.para[5][3] = 30;
    
    //偏差矫正
    Angle.DirErr[0] = 0;
    Angle.DirErr[1] = 1.6;
    Angle.DirErr[2] = 2.5;
    Angle.DirErr[3] = 2.5;
    Angle.DirErr[4] = 2.0;
    Angle.DirErr[5] = 1.0;
    Angle.DirErr[6] = 0.5;
    Angle.DirErr[7] = 0;
    Angle.DirErr[8] = -1;
    Angle.DirErr[9] = 1.0;
    Angle.DirErr[10] = 2.0;
    Angle.DirErr[11] = 2.0;
    Angle.DirErr[12] = 1.5;
    Angle.DirErr[13] = 0.3;
    Angle.DirErr[14] = 0;
    Angle.DirErr[15] = -1.5;
    Angle.DirErr[16] = -1.5;
    Angle.DirErr[17] = -2.0;
    
    for(int loop=0;loop<=17;loop++)
        Angle.DirErr[loop+18] = Angle.DirErr[loop];
        
    //第二问设置的力度
    /************************ LLLLLLLLL ********************/
    Angle.ForceL[30] = 30.0;//
    Angle.ForceL[32] = 31.7;//
    Angle.ForceL[34] = 33.4;//
    Angle.ForceL[36] = 35.0;//
    Angle.ForceL[38] = 36.6;//
    
    Angle.ForceL[40] = 38.2;//
    Angle.ForceL[42] = 39.9;//
    Angle.ForceL[44] = 41.8;//
    Angle.ForceL[46] = 43.5;//
    Angle.ForceL[48] = 45.0;//
    
    Angle.ForceL[50] = 46.9;//46.5
    Angle.ForceL[52] = 48.2;//
    Angle.ForceL[54] = 49.9;//
    Angle.ForceL[56] = 51.4;//
    Angle.ForceL[58] = 52.9;//
    
    Angle.ForceL[60] = 54.2;//
    Angle.ForceL[62] = 55.7;//
    Angle.ForceL[64] = 57.2;//
    Angle.ForceL[66] = 58.9;//
    Angle.ForceL[68] = 60.4;//
    Angle.ForceL[70] = 61.9;//2cm=1.5
    
    /************************ HHHHHHHHHH ********************/
    Angle.ForceH[30] = Angle.ForceL[30];
    Angle.ForceH[32] = Angle.ForceL[32];
    Angle.ForceH[34] = Angle.ForceL[34];
    Angle.ForceH[36] = Angle.ForceL[36];
    Angle.ForceH[38] = Angle.ForceL[38];
    
    Angle.ForceH[40] = Angle.ForceL[40];
    Angle.ForceH[42] = Angle.ForceL[42];
    Angle.ForceH[44] = Angle.ForceL[44];
    Angle.ForceH[46] = Angle.ForceL[46];
    Angle.ForceH[48] = Angle.ForceL[48];
    
    Angle.ForceH[50] = Angle.ForceL[50];
    Angle.ForceH[52] = Angle.ForceL[52];
    Angle.ForceH[54] = Angle.ForceL[54];
    Angle.ForceH[56] = Angle.ForceL[56]; 
    Angle.ForceH[58] = Angle.ForceL[58];//1cm
    
    Angle.ForceH[60] = Angle.ForceL[60];//1.5cm   2cm=1.4
    Angle.ForceH[62] = Angle.ForceL[62];//3cm
    Angle.ForceH[64] = Angle.ForceL[64];//2cm
    Angle.ForceH[66] = Angle.ForceL[66];//2cm
    Angle.ForceH[68] = Angle.ForceL[68];//2cm
    Angle.ForceH[70] = Angle.ForceL[70];//2cm
    
    for(int loop=31;loop<=69;loop+=2)
    {
        Angle.ForceH[loop] = (Angle.ForceH[loop-1] + Angle.ForceH[loop+1])/2.0;
        Angle.ForceL[loop] = (Angle.ForceL[loop-1] + Angle.ForceL[loop+1])/2.0;
    }
}





/************************************************
*  函数名称：Buzzer_Ring_ms
*  功能说明：蜂鸣器叫
*  参数说明：unsigned int n，n为n毫秒
*  函数返回：无
*  修改时间：2014-1-14    已经测试
*************************************************/
void Buzzer_Ring_ms(unsigned int n)
{
    buzzer = 1;
    delayms(n);
    buzzer = 0;
}



/************************************************
*  函数名称：Switch_init
*  功能说明：拨码开关8个引脚初始化
*  参数说明：无
*  函数返回：无
*  修改时间：2014-1-14    已经测试
*************************************************/
void Switch_init()
{
    gpio_init(PORTC, 6, GPI, LOW);
    gpio_init(PORTC, 7, GPI, LOW);
    gpio_init(PORTC, 8, GPI, LOW);
    gpio_init(PORTC, 9, GPI, LOW);
    gpio_init(PORTC,10, GPI, LOW);
    gpio_init(PORTC,11, GPI, LOW);
    gpio_init(PORTC,12, GPI, LOW);
    gpio_init(PORTC,13, GPI, LOW);
}


/************************************************
*  函数名称：PwmPort_init
*  功能说明：两路pwm输出初始化,FTM0的CH6对应硬件D6，FTM0的CH5对应硬件D5
             DIR_L对应硬件A19，DIR_R对应硬件A24
*  参数说明：无
*  函数返回：无
*  修改时间：2014-1-14    已经测试
*************************************************/
void Pwm_Port_init()
{ 
    FTM_PWM_init(FTM0, CH2, 10000, 0);       //PWM1
    gpio_init(PORTC, 2, GPO,HIGH);           //EN1
    gpio_init(PORTC, 1, GPO, LOW);           //DIR1
    FTM_PWM_Duty(FTM0, CH2 ,  0);            //输出
    
    FTM_PWM_init(FTM0, CH3, 10000, 0);       //PWM2
    gpio_init(PORTA,14, GPO,HIGH);           //EN2
    gpio_init(PORTA,12, GPO, LOW);           //DIR2
    FTM_PWM_Duty(FTM0, CH3 ,  0);            //输出
    
    FTM_PWM_init(FTM0, CH4, 10000, 0);       //PWM3
    gpio_init(PORTA,15, GPO,HIGH);           //EN3
    gpio_init(PORTA,13, GPO, LOW);           //DIR3
    FTM_PWM_Duty(FTM0, CH4 ,  0);            //输出

    
    FTM_PWM_init(FTM2, CH0, 10000, 0);       //PWM4
    //硬件EN = 1
    gpio_init(PORTA,11, GPO, LOW);           //DIR4
    FTM_PWM_Duty(FTM2, CH0 ,  0);            //输出
}


/************************************************
*  函数名称：Decoder_init
*  功能说明：两路测速初始化,FTM1对应硬件A8，A9，FTM1对应硬件A10，A11，
*  参数说明：无
*  函数返回：无
*  修改时间：2014-1-14    已经测试
*************************************************/
void Decoder_init(void)
{
    gpio_init(PORTA,17, GPO, LOW);           //Speed_Select_L
    gpio_init(PORTA,16, GPO, LOW);           //Speed_Select_R
  
#ifndef SpeedTimeShare  //不启用分时复用
    /*************FTM1******************/
    SIM_BASE_PTR->SCGC5 |= SIM_SCGC5_PORTA_MASK;  //使能PortA的时钟
    SIM_BASE_PTR->SCGC6 |= SIM_SCGC6_FTM1_MASK;   //使能FlexTimer0(FTM1)的时钟
    PORTA_PCR8 |= PORT_PCR_MUX(0x6);              //复用PortA8的引脚功能为FTM1_QD_PHA
    PORTA_PCR9 |= PORT_PCR_MUX(0x6);              //复用PortA9的引脚功能为FTM1_QD_PHB

    FTM1_CNT = 0;
    FTM1_MOD = 10000;
    FTM1_CNTIN = 0;
    FTM1_CNT = 0;
    FTM1_MODE |= FTM_MODE_WPDIS_MASK;
    FTM1_MODE |= FTM_MODE_FTMEN_MASK;
    FTM1_QDCTRL |= FTM_QDCTRL_QUADEN_MASK;
    FTM1_CONF |= FTM_CONF_BDMMODE(3);
    FTM1_SC |= FTM_SC_PS(0);                      //计时器时钟预分频因子为 1
    FTM1_SC |= FTM_SC_TOIE_MASK;
    (void)FTM1_SC;
    FTM1_SC &= ~FTM_SC_TOF_MASK;
    FTM1_SC |= FTM_SC_CLKS(1);     //设置参考时钟为总线时钟并使能输出
#endif
    
    /***********FTM2*************/
    SIM_BASE_PTR->SCGC5 |= SIM_SCGC5_PORTA_MASK;//使能PortA的时钟
    SIM_BASE_PTR->SCGC3 |= SIM_SCGC3_FTM2_MASK; //使能FlexTimer2(FTM2)的时钟
    PORTA_PCR10 |= PORT_PCR_MUX(0x6);           //复用PortA10的引脚功能为FTM2_QD_PHA
    PORTA_PCR11 |= PORT_PCR_MUX(0x6);           //复用PortA11的引脚功能为FTM2_QD_PHB

    FTM2_CNT = 0;
    FTM2_MOD = 10000;
    FTM2_CNTIN = 0;
    FTM2_CNT = 0;
    FTM2_MODE |= FTM_MODE_WPDIS_MASK;
    FTM2_MODE |= FTM_MODE_FTMEN_MASK;
    FTM2_QDCTRL |= FTM_QDCTRL_QUADEN_MASK;
    FTM2_CONF |= FTM_CONF_BDMMODE(3);
    FTM2_SC |= FTM_SC_PS(0);
    FTM2_SC |= FTM_SC_TOIE_MASK;//计时器时钟预分频因子为 1
    (void)FTM2_SC;
    FTM2_SC &= ~FTM_SC_TOF_MASK;
    FTM2_SC |= FTM_SC_CLKS(1);//设置参考时钟为总线时钟并使能输出
}


