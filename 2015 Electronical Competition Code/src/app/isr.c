#include "include.h"


/*************************************************************************
*                             野火嵌入式开发工作室
*
*  函数名称：PORTB_IRQHandler
*  功能说明：PORTB端口中断服务函数，请忽视这个函数吧，比较乱，只是个界面，按键改改参数啥的，不涉及算法什么的
*  参数说明：无
*  函数返回：无
*  修改时间：2012-1-25    已测试
*  备    注：引脚号需要自己初始化来清除
*************************************************************************/
void PORTB_IRQHandler()
{    
    if(PORTB_ISFR & (1 << 8))       //SELECT            //PTB8触发中断
    {
        PORTB_ISFR  |= (1 << 8);                        //写1清中断标志位
        
        //变到次级
        Lcd.level++; if(Lcd.level > 1) Lcd.level=1;else Lcd.flushFg=1;;
        Lcd.Add = 0;
    }
    else if(PORTB_ISFR & (1 << 9)) //UP:变量增大
    {
        PORTB_ISFR  |= (1 << 9);
        
        //次级改参数
        if(Lcd.level == 1)
        {
            if(Mode.mode ==0 )  //mode = 0 改变四路PWM
            {
                Motor[Lcd.Add].PWM += 300;     if(Motor[Lcd.Add].PWM >= 3900u) Motor[Lcd.Add].PWM = 3900u;
            }
            else if(Mode.mode == 2 || Mode.mode == 5)  //mode = 2 ,5 改变振幅
            {
                Mode.para[Mode.mode][3]++;         if(Mode.para[Mode.mode][3] > 70) Mode.para[Mode.mode][3] = 70;
                return;
            }
            else if(Mode.mode == 3 )  //mode = 3 改变方向
            {
                Mode.para[3][3]++;         if(Mode.para[3][3] >= 36) Mode.para[3][3] = 0;
                return;
            }
        }
    }
    else if(PORTB_ISFR & (1 << 6))//DOWN：变量较小
    {
        PORTB_ISFR  |= (1 << 6);
        
        //次级改参数
        if(Lcd.level == 1)
        {
            if(Mode.mode == 0)  //mode = 0 改变四路PWM
            {
                Motor[Lcd.Add].PWM -= 300;      if(Motor[Lcd.Add].PWM <= 0)     Motor[Lcd.Add].PWM = 0;
            }
            else if(Mode.mode == 2 || Mode.mode == 5)  //mode = 2 ,5改变振幅
            {
                Mode.para[Mode.mode][3]--;         if(Mode.para[Mode.mode][3] < 30) Mode.para[Mode.mode][3] = 30;
                return;
            }
            else if(Mode.mode == 3 )  //mode = 3 改变方向
             {
                 Mode.para[3][3]-- ;         if(Mode.para[3][3] < 0) Mode.para[3][3] = 35;
                 return;
             }
        }
    }
    
    
    
    else if(PORTB_ISFR & (1 <<10))//LEFT：菜单上拉
    {
        PORTB_ISFR  |= (1 <<10);
        
        //初级和次级都上下拉（初次级有各自最大行），初级还要改变mode
        Lcd.Add --;    if(Lcd.Add <= 0)       Lcd.Add=0;
        //初级还要改变mode
        if(Lcd.level == 0)      
            Mode.mode = Lcd.Add;
    }
    else if(PORTB_ISFR & (1 << 7))//RIGHT ：菜单下拉
    {
        PORTB_ISFR  |= (1 << 7);
        
        //初级和次级都上下拉（初次级有各自最大行）
        Lcd.Add ++;    if(Lcd.Add >= Lcd.AddMax[ Lcd.level ] )       Lcd.Add = Lcd.AddMax[ Lcd.level ];
        //初级还要改变mode
        if(Lcd.level == 0)
            Mode.mode = Lcd.Add;
    }
    
    
    else if(PORTB_ISFR & (1 << 5))//KEY1：返回
    {
        PORTB_ISFR  |= (1 << 5);
        
        //退出start模式
        if(Lcd.level==1 && Mode.mode!=0 && Mode.startFg[Mode.mode]==1) 
        {
            Mode.startFg[Mode.mode] = 0;
            Buzzer_Ring_ms(10);
            return;
        }
        else //变到初级
        {
            Lcd.level--; if(Lcd.level < 0) Lcd.level=0;else Lcd.flushFg=1;
        }
    }
    else if(PORTB_ISFR & (1 << 3))//KEY2 ：模式start
    {  
        PORTB_ISFR  |= (1 << 3);
        
        //模式开始 & 停止
        if(Lcd.level==1 && Mode.mode!=0)
        {
            Mode.startFg[Mode.mode] = 1;
            Mode.TimeStart = Time_1ms;
        }
    }
    
#ifdef DEBUG
    delayms(70);
#else
    delayms(180);
#endif
    Buzzer_Ring_ms(10);
}



/*************************************************************************
*  函数名称：VSYNC_IRQ
*  功能说明：PORTD端口中断服务函数
*  参数说明：无
*  函数返回：无
*  修改时间：2012-1-25    已测试
*  备    注：引脚号需要自己初始化来清除
*************************************************************************/
void VSYNC_IRQ(void)
{    
    static u16 czd=0;
    static u32 flag;
    //Clear Interrupt flag
    flag = PORTD_ISFR;
    PORTD_ISFR = flag;
    czd++;
    if(img_flag == IMG_START)	//需要开始采集图像
    {
        DMA_PORTx2BUFF_Init(CAMERA_DMA_CH, (void *)&PTB_BYTE2_IN, (void *)Image_fire, PTE27, DMA_BYTE1, CAMERA_SIZE , DMA_falling);
        DMA_EN(CAMERA_DMA_CH);            		//使能通道CHn 硬件请求
        DMA_DADDR(CAMERA_DMA_CH) = (u32)Image_fire; //恢复地址
        img_flag = IMG_GATHER;		        //标记图像采集中
        disable_irq(90);  
    }
    else					//图像采集错误
    {
        disable_irq(90); 			//关闭PTA的中断
        img_flag = IMG_FAIL;		//标记图像采集失败
    }
}




/*************************************************************************
*  函数名称：DMA0_IRQHandler
*  功能说明：DMA0
*  参数说明：无
*  函数返回：无
*  修改时间：2012-1-25    已测试
*  备    注：引脚号需要根据自己初始化来修改
*************************************************************************/
void DMA0_IRQHandler()
{
    DMA_DIS(CAMERA_DMA_CH);            	//关闭通道CHn 硬件请求
    DMA_IRQ_CLEAN(CAMERA_DMA_CH);           //清除通道传输中断标志位
    img_flag = IMG_FINISH ;  
}






/*************************************************************************
*  函数名称：PIT0_IRQHandler
*  功能说明：PIT0 定时中断服务函数
*  参数说明：无
*  函数返回：无
*  修改时间：2014-1-14    已测试
*  备    注：
*************************************************************************/
void PIT0_IRQHandler(void)
{	 
    PIT_Flag_Clear(PIT0);
    Time_1ms++;
    Water_LEDs();//LED流水灯，证明在进定时器中断
    
    /**************************************************************/
    /*********************Own code*********************************/
    /**************************************************************/   
    
    
    //姿态解算
    if(Time_1ms%2==0)//偶数ms
    {
        MPU6050_updateSensor(); //600us，测得加速度计和陀螺仪的原始值（已经减去零偏）
		//20us，2次神马滤波，只是加速度计数据用到了，具体参数计算请问强哥
		//用智能车的代码滤波效果也是一样的，我只是比较懒所以没改，因为比赛第一天感觉可能用到四轴算法，才移植过来的
		//亲测智能车的滤波代码效果也是一样，所以不用担心，具体可以参考隋国斌代码
        Filter_2nd_LPF2ndFilter();
    }
    else//奇数ms
    {
        Angle_Calculate();	//用滤波后的数据角度融合
        Mode_PID();			//每一问的PID计算
        Motor_Out();		//4个电机输出
    }
    
    
    
    /*********************************************************************/
    /**************************Own code end*******************************/
    /*********************************************************************/
    PIT_Flag_Clear(PIT0);       //清中断标志位
}







/*************************************************************************
*                             野火嵌入式开发工作室
*
*  函数名称：USART0_IRQHandler
*  功能说明：串口0 中断 接收 服务函数
*  参数说明：无
*  函数返回：无
*  修改时间：2014-1-14    已测试
*  备    注：
*************************************************************************/
void USART0_IRQHandler(void)
{
    unsigned char ch;
    //接收一个字节数据并回发
    ch = uart_getchar (UART0);      //接收到一个数据
    switch(ch)
    {
      //case 'a':break;
      //default: printf("error");
    }
    uart_putchar(UART0,'\n');
}




/*************************************************************************
*                             野火嵌入式开发工作室
*
*  函数名称：SysTick_Handler
*  功能说明：系统滴答定时器中断服务函数
*  参数说明：无
*  函数返回：无
*  修改时间：2012-2-18    已测试
*  备    注：ucos里用得到
*************************************************************************/
void SysTick_Handler(void)
{
    //    OSIntEnter();
    //    OSTimeTick();
    //    OSIntExit();
}





/*************************************************************************
*                             野火嵌入式开发工作室
*
*  函数名称：HardFault_Handler
*  功能说明：硬件上访中断服务函数
*  参数说明：无
*  函数返回：无
*  修改时间：2012-2-4    已测试
*  备    注：可以用LED闪烁来指示发生了硬件上访
*************************************************************************/
void HardFault_Handler(void)
{
    while (1)
    {
        printf("\n****硬件上访错误!!!*****\r\n\n");
    }
}




/*************************************************************************
*                             野火嵌入式开发工作室
*
*  函数名称：PendSV_Handler
*  功能说明：PendSV（可悬起系统调用）中断服务函数
*  参数说明：无
*  函数返回：无
*  修改时间：2012-2-15    已测试
*  备    注：uC/OS用来切换任务
*************************************************************************/
void PendSV_Handler(void)
{
}





/*************************************************************************
*                             野火嵌入式开发工作室
*
*  函数名称：FTM0_IRQHandler
*  功能说明：FTM0输入捕捉中断服务函数
*  参数说明：无
*  函数返回：无
*  修改时间：2012-2-25
*  备    注：引脚号需要根据自己初始化来修改，参考现有的代码添加自己的功能
*************************************************************************/
void FTM0_IRQHandler()
{
}




/*************************************************************************
*                             野火嵌入式开发工作室
*
*  函数名称：FTM1_IRQHandler
*  功能说明：FTM1输入捕捉中断服务函数
*  参数说明：无
*  函数返回：无
*  修改时间：2012-2-25
*  备    注：引脚号需要根据自己初始化来修改，参考现有的代码添加自己的功能
*************************************************************************/
void FTM1_IRQHandler()
{
}



