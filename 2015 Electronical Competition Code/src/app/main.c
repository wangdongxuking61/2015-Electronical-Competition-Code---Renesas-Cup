//总括：
//1.变量和结构体声明请看include.h
//2.由于是根据智能车工程更改过去的，有好多没用的文件，请谨慎阅读，重要的代码只有主函数和定时器中断函数
//	显示屏刷界面代码请忽视，因为比较乱，即使本人看的话基本上也看不懂了=。=，所以把重点放在角度融合，
//	PID控制，和工程代码的组织
//3.main.c:定义了变量，main()初始化了所有模块，然后一直刷屏
//4.include.h:包含了各种h文件，还有各种变量的声明
//5.isr.c:
//	PORTB_IRQHandler():每个按键均触发这个中断，用于改变模式和更改变量
//	VSYNC_IRQ(void)：摄像头场中断，只有发挥部分第三问用了，也就是程序对应的模式7
//	DMA0_IRQHandler():摄像头场DMA存储满了中断
//	PIT0_IRQHandler(void)：最重要的角度融合，PID计算，还有电机输出
//6.ISR_fun.c:包含了isr.c调用的函数
//7.IIC.c:包含了MPU6050的底层，初始化、采集数据、滤波函数等
//8.Car_init.c:各种模块初始化
//9.char.h：被LCD.c包含，在显示屏上刷出0~9的大字
//10：Motor_Out()中记得开偏角过大的保护，除了发挥第三问，剩下时候都开，防止损坏

/*我的坐标系和电机编号
					
					1号电机(Motor[0])
					+列方向角度
					-gyro.y
					-acc.x								
2号电机(Motor[1])						4号电机行(Motor[3])
+行方向角度								-行方向角度
-gyro.y									+gyro.y
+acc.x									-acc.x					
					3号电机(Motor[2])
					-列方向角度
					+gyro.y
					+acc.x
*/

#include "include.h"



/*************************************************************************
*  模块名称：没有名称
*  功能说明：各种全局变量的定义以及初始化，具体声明请看include.h
*  参数说明：无
*  函数返回：无
*  修改时间：2012-2-14
*************************************************************************/
struct CarThreeNum      Acc_ADC_Data={0,0,0},   Acc_ADC={0,0,0},        Acc_Offset={0,0,0};
//Acc_ADC_Data：经过滤波的加速度计的值，请看Filter_2nd_LPF2ndFilter()
//Acc_ADC：采集的加速度计的原始值，在测得好零偏后减去了零偏，请看MPU6050_Read_Acc_Data()代码
//Acc_Offset：加速度计上电要采集的3个轴零偏值，请看MPU6050_CalOffset_Acc()代码

struct CarThreeNum      Gyro_dps={0,0,0},       Gyro_ADC={0,0,0},       Gyro_Offset={0,0,0};
//Acc_ADC_Data：经过滤波的加速度计的值，请看Filter_2nd_LPF2ndFilter()
//Acc_ADC：采集的加速度计的原始值，在测得好零偏后减去了零偏，请看MPU6050_Read_Acc_Data()代码
//Acc_Offset：加速度计上电要采集的3个轴零偏值，请看MPU6050_CalOffset_Acc()代码

struct CarAngle         Angle={0,0,0,0,0,0,0,0,0,0,0};//解释请看include.c中的声明
struct CarFourNum       Q={1,0,0,0};//没用到
struct CarThreeNum      Gravity={0,0,0};//没用到

struct CarMotor Motor[4]= {{0},{0},{0},{0}};//4个电机的pwm，解释请看include.c中的声明
struct CarLcd   Lcd     = {0,0,0,{7,3}};//解释请看include.c中的声明
struct CarMode  Mode    = {0,0,0,0,0,0,{{0}},{0}};//解释请看include.c中的声明
long int        Time_1ms = 0;//ms计数器，中断里面++了




/*************************************************************************
*  函数名称：main
*  功能说明：主函数
*  参数说明：无
*  函数返回：无
*  修改时间：2012-2-14    一直在测试
*************************************************************************/
void main()
{
    char str[40];//用于刷屏
    DisableInterrupts;
    delayms(100);
    System_Init();//所有数据和模块初始化
    delayms(100);
    EnableInterrupts;
    
	
    //while这段时间是等待加速计和陀螺仪采集好零偏值，
	//请看MPU6050_updateSensor(),MPU6050_CalOffset_Gyro(),MPU6050_CalOffset_Acc(),MPU6050_Read_Acc_Data(),MPU6050_Read_Gyro_Data()
	//采集完成后Acc_CALIBRATED,Gyro_CALIBRATED都会置0
    while(Acc_CALIBRATED==1 || Gyro_CALIBRATED==1);
    Buzzer_Ring_ms(100);
	
	
	//主循环，一直在刷屏
    while(1)
    {
		//单独用于模式7，由于摄像头用到了b口的DMA，貌似和按键的外部中断冲突了，所以摄像头一旦初始化
		//就不能按键了，只能掉电，所以摄像头功能最后测，冲突的问题还没解决，应该是库的问题，大家以后注意
        if(Mode.mode==7 && Mode.startFg[Mode.mode]==1)
        {
            //鹰眼摄像头初始化,包括场中断和DMA
            static int first_run=0;
            if(first_run++==0)
            {
                Ov7725_Init();
                buzzer=0;
                Disp_single_colour(Yellow);
                Angle.aveL = 130;
                Angle.aveH = 71;
            }
            //图像采集，这4个数字是苍蝇拍活动的矩形区域
            static int H1=10,H2=140,L1=70,L2=200;
            if(Time_1ms%300)//300ms处理一次
            {
                ov7725_get_img();//采集图像
                Process_Image();//处理图像，得到苍蝇拍位置Angle.aveL和Angle.aveH
				//不要误会Process_Image这个函数是Process.c中的函数，因为根本就没有包含这个.c和.h，还是因为太懒，没把这个文件删了
				//这个Process_Image是LCD.c中的函数
             }
            if(sw8==1)//拨码第八位
            {
                Send_Image_to_LCD(Image_fire);
                Draw_single_line('H',H1,Black);
                Draw_single_line('H',H2,Black);
                Draw_single_line('L',L1,Black);
                Draw_single_line('L',L2,Black);
                
                Draw_single_line('H',Angle.aveH,Black);
                Draw_single_line('L',Angle.aveL,Black);
            }
            continue;
        }
            
        
		//以下代码用于模式1~6，以下基本上不要看，不涉及算法或pid什么的，只是单纯的各种花式刷屏，比较乱。。。
		//思路就是：有个主菜单，有个次级菜单，主菜单有8个模式可以选择，进入某个模式后就显示相应模式的参数，
		//按下“开始”按键，这个模式就启动了，按下“取消”按键就退出执行模式，再下“取消”就回到主菜单
        if(Lcd.flushFg == 1)//LCD刷新一下
        {
            Lcd.flushFg = 0;
            Disp_single_colour(Yellow);
        }
        
        //LCD模式显示
        if(sw1 == 1)
        {
            //初级菜单
            if(Lcd.level==0)
            {
                char strMode[8][40]={"set PWM","L=60 , a Line","set Line Length","set Line Dir","stop!!!","set circle Length","circle vs fan","find fly pai"};
                for(int loop=0;loop <= Lcd.AddMax[ Lcd.level ];loop++)
                {
                    sprintf(str,"select Mode = %d : %s",loop,strMode[loop]);
                    if(loop == Lcd.Add) 
                        LCD_PutString(20,loop*20,str,Red,Green);
                    else 
                        LCD_PutString(20,loop*20,str,Red,Yellow);
                }
            }
            //次级菜单
            else if(Lcd.level==1)
            {
                //mode = 0 次级改变电机PWM
                if(Mode.mode==0)
                {
                    for(int loop=0;loop <= Lcd.AddMax[1];loop++)
                    {
                        sprintf(str,"Mode : 0 , Motor[%d].PWM = %5d",loop,(int)(Motor[loop].PWM));
                        if(loop == Lcd.Add)LCD_PutString(20,loop*20,str,Red,Green);else LCD_PutString(20,loop*20,str,Red,Yellow);
                    }
                }
                //mode ！= 0 次级改变参数
                else            
                {
                    char strPID[4][7]={"P   ","I   ","D   ","Length"};
                    if(Mode.mode == 3) sprintf( strPID[3],"Dir   ");
                    for(int loop=0;loop <= Lcd.AddMax[1];loop++)
                    {
                        //正常显示
                        sprintf(str,"Mode = %d , %s = %5d", Mode.mode , strPID[loop] , Mode.para[Mode.mode][loop]);
                        if(loop == Lcd.Add)
                            LCD_PutString(20,loop*20,str,Red,Green);
                        else 
                            LCD_PutString(20,loop*20,str,Red,Yellow);
                    }
                    //显示模式开启与否
                    if(Mode.startFg[Mode.mode]==1)
                    {
                        sprintf(str,"start = %3ld",(Time_1ms - Mode.TimeStart)/1000);
                        printBigNum((Time_1ms - Mode.TimeStart)/1000%100,Lcd.AddMax[1]*20+40);
                        if(Time_1ms - Mode.TimeStart>0 && Time_1ms - Mode.TimeStart<1000)
                            buzzer = 1;
                        else
                            buzzer = 0;
                    }
                    else 
                        sprintf(str,"end  ");
                    LCD_PutString(20,Lcd.AddMax[1]*20+20,str,Red,Yellow);
                }
            }
        }
        else if(sw2 == 1)//显示参数
            ;//LcdPrintAnglePara();
        else if(sw3 == 1)//send infomation 
            SCI_Send_Datas(UART0);
        else
            Lcd.flushFg = 1;
        
        
        //测试时间,方波测试，平时不开启，只是用于时间的测试
        #if 0
        static int Test_num=0;
        if(Test_num==1)
        {
            Test_IO = 1;         
            Test_num = 0;
            //测量运行时间的函数放在这里
            Attitude_Updata_Quaternion(0.002);
            
        }
        else
        {
            Test_IO = 0;
            Test_num = 1;
            //测量运行时间的函数放在这里
            Attitude_Updata_Quaternion(0.002);
            
        }
        #endif
    }
}
