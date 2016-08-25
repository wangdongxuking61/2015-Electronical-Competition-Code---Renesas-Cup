/******************** (C) COPYRIGHT 2011 野火嵌入式开发工作室 ********************
* 文件名       ：uart.c
* 描述         ：串口函数
* 备注         ：参考苏州大学的例程和飞思卡尔官方的例程
*
* 实验平台     ：野火kinetis开发板
* 库版本       ：
* 嵌入系统     ：
*
* 作者         ：
* 淘宝店       ：http://firestm32.taobao.com
* 技术支持论坛 ：http://www.ourdev.cn/bbs/bbs_list.jsp?bbs_id=1008
**********************************************************************************/

#include "common.h"
#include "uart.h"
#include "assert.h"
#include "include.h"

volatile struct UART_MemMap *UARTx[6] = {UART0_BASE_PTR, UART1_BASE_PTR, UART2_BASE_PTR, UART3_BASE_PTR, UART4_BASE_PTR, UART5_BASE_PTR}; //定义五个指针数组保存 UARTx 的地址

/*************************************************************************
*                             野火嵌入式开发工作室
*
*  函数名称：uart_init
*  功能说明：初始化串口，设置波特率
*  参数说明：UARTn       模块号（UART0~UART5）
*            baud        波特率，如9600、19200、56000、115200等
*  函数返回：无
*  修改时间：2012-1-20
*  备    注：在官方例程上修改
*************************************************************************/
void uart_init (UARTn uratn, u32 baud)
{
    register uint16 sbr, brfa;
    uint8 temp;
    u32 sysclk;     //时钟

    /* 配置 UART功能的 GPIO 接口 开启时钟 */
    switch(uratn)
    {
      case UART0:
        if(UART0_RX == PTA1)
            PORTA_PCR1 = PORT_PCR_MUX(0x2);      //在PTA1上使能UART0_RXD
        else if(UART0_RX == PTA15)
            PORTA_PCR15 = PORT_PCR_MUX(0x3);     //在PTA15上使能UART0_RXD
        else if(UART0_RX == PTB16)
            PORTB_PCR16 = PORT_PCR_MUX(0x3);     //在PTB16上使能UART0_RXD
        else if(UART0_RX == PTD6)
            PORTD_PCR6 = PORT_PCR_MUX(0x3);      //在PTD6上使能UART0_RXD
        else
            assert_failed(__FILE__, __LINE__);   //设置管脚有误？

        if(UART0_TX == PTA2)
            PORTA_PCR2 = PORT_PCR_MUX(0x2);     //在PTA2上使能UART0_RXD
        else if(UART0_TX == PTA14)
            PORTA_PCR14 = PORT_PCR_MUX(0x3);     //在PTA14上使能UART0_RXD
        else if(UART0_TX == PTB17)
            PORTB_PCR17 = PORT_PCR_MUX(0x3);     //在PTB17上使能UART0_RXD
        else if(UART0_TX == PTD7)
            PORTD_PCR7 = PORT_PCR_MUX(0x3);     //在PTD7上使能UART0_RXD
        else
            assert_failed(__FILE__, __LINE__);  //设置管脚有误？


        SIM_SCGC4 |= SIM_SCGC4_UART0_MASK;      //使能 UARTn 时钟
        break;

      case UART1:
        if(UART1_RX == PTC3)
            PORTC_PCR3 = PORT_PCR_MUX(0x3);     //在PTC3上使能UART1_RXD
        else if(UART1_RX == PTE1)
            PORTE_PCR1 = PORT_PCR_MUX(0x3);     //在PTE1上使能UART1_RXD
        else
            assert_failed(__FILE__, __LINE__);  //设置管脚有误？

        if(UART1_TX == PTC4)
            PORTC_PCR4 = PORT_PCR_MUX(0x3);     //在PTC4上使能UART1_RXD
        else if(UART1_TX == PTE0)
            PORTE_PCR0 = PORT_PCR_MUX(0x3);     //在PTE0上使能UART1_RXD
        else
            assert_failed(__FILE__, __LINE__);  //设置管脚有误？

        SIM_SCGC4 |= SIM_SCGC4_UART1_MASK;
        break;

      case UART2:
        PORTD_PCR3 = PORT_PCR_MUX(0x3);         //在PTD3上使能UART2_TXD功能
        PORTD_PCR2 = PORT_PCR_MUX(0x3);         //在PTD2上使能UART2_RXD
        SIM_SCGC4 |= SIM_SCGC4_UART2_MASK;
        break;

      case UART3:
        if(UART3_RX == PTB10)
            PORTB_PCR10 = PORT_PCR_MUX(0x3);     //在PTB10上使能UART3_RXD
        else if(UART3_RX == PTC16)
            PORTC_PCR16 = PORT_PCR_MUX(0x3);     //在PTC16上使能UART3_RXD
        else if(UART3_RX == PTE5)
            PORTE_PCR5 = PORT_PCR_MUX(0x3);      //在PTE5上使能UART3_RXD
        else 
            assert_failed(__FILE__, __LINE__);                   //设置管脚有误？

        if(UART3_TX == PTB11)
            PORTB_PCR11 = PORT_PCR_MUX(0x3);     //在PTB11上使能UART3_RXD
        else if(UART3_TX == PTC17)
            PORTC_PCR17 = PORT_PCR_MUX(0x3);     //在PTC17上使能UART3_RXD
        else if(UART3_TX == PTE4)
            PORTE_PCR4 = PORT_PCR_MUX(0x3);     //在PTE4上使能UART3_RXD
        else
            assert_failed(__FILE__, __LINE__);                   //设置管脚有误？

        SIM_SCGC4 |= SIM_SCGC4_UART3_MASK;
        break;

      case UART4:
        if(UART4_RX == PTC14)
            PORTC_PCR14 = PORT_PCR_MUX(0x3);     //在PTC14上使能UART4_RXD
        else if(UART4_RX == PTE25)
            PORTE_PCR25 = PORT_PCR_MUX(0x3);     //在PTE25上使能UART4_RXD
        else
            assert_failed(__FILE__, __LINE__);                   //设置管脚有误？

        if(UART4_TX == PTC15)
            PORTC_PCR15 = PORT_PCR_MUX(0x3);     //在PTC15上使能UART4_RXD
        else if(UART4_TX == PTE24)
            PORTE_PCR24 = PORT_PCR_MUX(0x3);     //在PTE24上使能UART4_RXD
        else
            assert_failed(__FILE__, __LINE__);                   //设置管脚有误？

        SIM_SCGC1 |= SIM_SCGC1_UART4_MASK;
        break;

      case UART5:
        if(UART5_RX == PTD8)
            PORTD_PCR8 = PORT_PCR_MUX(0x3);     //在PTD8上使能UART5_RXD
        else if(UART5_RX == PTE9)
            PORTE_PCR9 = PORT_PCR_MUX(0x3);     //在PTE9上使能UART5_RXD
        else
            assert_failed(__FILE__, __LINE__);                   //设置管脚有误？

        if(UART5_TX == PTD9)
            PORTD_PCR9 = PORT_PCR_MUX(0x3);     //在PTD9上使能UART5_RXD
        else if(UART5_TX == PTE8)
            PORTE_PCR8 = PORT_PCR_MUX(0x3);     //在PTE8上使能UART5_RXD
        else
            assert_failed(__FILE__, __LINE__);                   //设置管脚有误？

        SIM_SCGC1 |= SIM_SCGC1_UART5_MASK;
        break;
      default:
        break;
    }


    //设置的时候，应该禁止发送接受
    UART_C2_REG(UARTx[uratn]) &= ~(UART_C2_TE_MASK  | UART_C2_RE_MASK );

    //配置成8位无校验模式
    //设置 UART 数据格式、校验方式和停止位位数。通过设置 UART 模块控制寄存器 C1 实现；
    UART_C1_REG(UARTx[uratn]) = 0;	// 全部直接使用默认设置就行，所以直接清0

    //计算波特率，串口0、1使用内核时钟，其它串口使用外设时钟
    if ((uratn == UART0) | (uratn == UART1))
        sysclk = core_clk_khz * 1000;            //内核时钟
    else
        sysclk = bus_clk_khz * 1000;  //外设时钟

    //设置 UART 数据通讯波特率。通过设置 UART 模块的波特率寄存器
    sbr = (u16)(sysclk / (baud << 4));

    /* Save off the current value of the UARTx_BDH except for the SBR field */
    temp = UART_BDH_REG(UARTx[uratn]) & ~(UART_BDH_SBR(0x1F));

    UART_BDH_REG(UARTx[uratn]) = temp |  UART_BDH_SBR(((sbr & 0x1F00) >> 8));
    UART_BDL_REG(UARTx[uratn]) = (u8)(sbr & UART_BDL_SBR_MASK);

    //brfa = (((sysclk*32)/(baud * 16)) - (sbr * 32));
    brfa = (((sysclk << 5) / (baud << 4)) - (sbr << 5));

    /* Save off the current value of the UARTx_C4 register except for the BRFA field */
    temp = UART_C4_REG(UARTx[uratn]) & ~(UART_C4_BRFA(0x1F));

    UART_C4_REG(UARTx[uratn]) = temp |  UART_C4_BRFA(brfa);

    /* 允许发送和接收 */
    UART_C2_REG(UARTx[uratn]) |= (UART_C2_TE_MASK | UART_C2_RE_MASK );

    //设置是否允许接收和发送中断。通过设置 UART 模块的 C2 寄存器的
    //RIE 和 TIE 位实现。如果使能中断，必须首先实现中断服务程序；
}

/*************************************************************************
*                             野火嵌入式开发工作室
*
*  函数名称：uart_getchar
*  功能说明：无限时间等待串口接受一个字节
*  参数说明：UARTn       模块号（UART0~UART5）
*  函数返回：接收到的字节
*  修改时间：2012-1-20
*  备    注：官方例程
*************************************************************************/
char uart_getchar (UARTn uratn)
{
    /* Wait until character has been received */
    while (!(UART_S1_REG(UARTx[uratn]) & UART_S1_RDRF_MASK));

    /* Return the 8-bit data from the receiver */
    return UART_D_REG(UARTx[uratn]);
}

/*************************************************************************
*                             野火嵌入式开发工作室
*
*  函数名称：uart_pendchar
*  功能说明：有限时间等待串口接受一个字节
*  参数说明：UARTn       模块号（UART0~UART5）
*  函数返回：接收到的字节
*  修改时间：2012-1-20
*  备    注：
*************************************************************************/
char uart_pendchar (UARTn uratn, char *ch)
{
    u32 i = 0;

    while(++i < 0xffffff)                                         //时间限制
    {
        if(UART_S1_REG(UARTx[uratn]) & UART_S1_RDRF_MASK)         //查询是否接受到数据
        {
            *ch  =   UART_D_REG(UARTx[uratn]);                    //接受到8位的数据
            return  1;                                            //返回 1 表示接收成功
        }
    }

    *ch = 0;                                                     //接收不到，应该清空了接收区
    return 0;                                                    //返回0表示接收失败
}


/*************************************************************************
*                             野火嵌入式开发工作室
*
*  函数名称：uart_pendstr
*  功能说明：有限时间等待串口接受字符串
*  参数说明：UARTn       模块号（UART0~UART5）
*  函数返回：接收到的字节
*  修改时间：2012-1-20
*  备    注：
*************************************************************************/
char uart_pendstr (UARTn uratn, char *str)
{
    u32 i = 0;
    while(uart_pendchar(uratn, str + i++));

    return (i <= 1 ? 0 : 1);
}



/*************************************************************************
*                             野火嵌入式开发工作室
*
*  函数名称：uart_putchar
*  功能说明：串口发送一个字节
*  参数说明：UARTn       模块号（UART0~UART5）
*  函数返回：无
*  修改时间：2012-1-20
*  备    注：官方例程，printf会调用这函数
*************************************************************************/
void uart_putchar (UARTn uratn, char ch)
{
    //等待发送缓冲区空
    while(!(UART_S1_REG(UARTx[uratn]) & UART_S1_TDRE_MASK));
    //发送数据
    UART_D_REG(UARTx[uratn]) = (u8)ch;
}



/*************************************************************************
*                             野火嵌入式开发工作室
*
*  函数名称：uart_query
*  功能说明：查询是否接受到一个字节
*  参数说明：UARTn       模块号（UART0~UART5）
*  函数返回：1           接收到一个字节了
*            0           没有接收到
*  修改时间：2012-1-20
*  备    注：官方例程
*************************************************************************/
int uart_query (UARTn uratn)
{
    return (UART_S1_REG(UARTx[uratn]) & UART_S1_RDRF_MASK);
}

/*************************************************************************
*                             野火嵌入式开发工作室
*
*  函数名称：uart_sendN
*  功能说明：串行 发送指定len个字节长度字符串 （包括 NULL 也会发送）
*  参数说明：UARTn       模块号（UART0~UART5）
*            buff        发送缓冲区
*            len         发送长度
*  函数返回：无
*  修改时间：2012-1-20
*  备    注：
*************************************************************************/
void uart_sendN (UARTn uratn, uint8 *buff, uint16 len)
{
    int i;
    for(i = 0; i < len; i++)
    {
        uart_putchar(uratn, buff[i]);
    }
}

/*************************************************************************
*                             野火嵌入式开发工作室
*
*  函数名称：uart_sendStr
*  功能说明：串行发送字符串
*  参数说明：UARTn       模块号（UART0~UART5）
*            str         字符串
*  函数返回：无
*  修改时间：2012-1-20
*  备    注：
*************************************************************************/
void uart_sendStr (UARTn uratn, const u8 *str)
{
    while(*str)
    {
        uart_putchar(uratn, *str++);
    }
}

/*************************************************************************
*                             野火嵌入式开发工作室
*
*  函数名称：uart_irq_EN
*  功能说明：开串口接收中断
*  参数说明：UARTn       模块号（UART0~UART5）
*  函数返回：无
*  修改时间：2012-1-20
*  备    注：
*************************************************************************/
void uart_irq_EN(UARTn uratn)
{
    UART_C2_REG(UARTx[uratn]) |= UART_C2_RIE_MASK;    //开放UART接收中断
    enable_irq((uratn << 1) + 45);			        //开接收引脚的IRQ中断
}


/*************************************************************************
*                             野火嵌入式开发工作室
*
*  函数名称：uart_irq_DIS
*  功能说明：关串口接收中断
*  参数说明：UARTn       模块号（UART0~UART5）
*  函数返回：无
*  修改时间：2012-1-20
*  备    注：
*************************************************************************/
void uart_irq_DIS(UARTn uratn)
{
    UART_C2_REG(UARTx[uratn]) &= ~UART_C2_RIE_MASK;   //禁止UART接收中断
    disable_irq((uratn << 1) + 45);			        //关接收引脚的IRQ中断
}


/************************************************
*  函数名称：SCISend_UnsignedInt
*  功能说明：SCI发送UnsignedInt数据
*  参数说明：UARTn为第几个串口，pnum为传送值
*  函数返回：无
*  修改时间：2014-1-14    已经测试
*************************************************/
void SCISend_UnsignedInt(UARTn uratn,unsigned int pnum)
{
    if(pnum<10)
    {  uart_putchar(uratn,pnum+'0');}
    else if((10<=pnum)&&(pnum<100))
    {
        uart_putchar(uratn,pnum/10+'0');
        uart_putchar(uratn,pnum%10+'0');
    }
    else if(pnum<=999)
    {
        uart_putchar(uratn,(unsigned char)(pnum/100+'0'));
        uart_putchar(uratn,(unsigned char)(pnum/10%10+'0'));
        uart_putchar(uratn,(unsigned char)(pnum%10+'0'));
    }
    else if(pnum<=9999)
    {
        uart_putchar(uratn,pnum/1000+'0');uart_putchar(uratn,pnum/100%10+'0');
        uart_putchar(uratn,pnum/10%10+'0');uart_putchar(uratn,pnum%10+'0');
    }
    else if(pnum<=99999)
    {
        uart_putchar(uratn,pnum/10000+48);uart_putchar(uratn,pnum/1000%10+48);
        uart_putchar(uratn,pnum/100%10+48);uart_putchar(uratn,pnum/10%10+48);
        uart_putchar(uratn,pnum%10+48);
    }
}


/************************************************
*  函数名称：SCISend_Int
*  功能说明：SCI发送SCISend_Int数据
*  参数说明：UARTn为第几个串口，pnum为传送值
*  函数返回：无
*  修改时间：2014-1-14    已经测试
*************************************************/
void SCISend_Int(UARTn uratn,int pnum)
{
    if(pnum<0)
    { pnum=-pnum; uart_putchar(uratn,'-');}
    if(pnum<10)
    {  uart_putchar(uratn,pnum+'0');}
    else if((10<=pnum)&&(pnum<100))
    {
        uart_putchar(uratn,pnum/10+'0');uart_putchar(uratn,pnum%10+'0');}
    else if(pnum<=999)
    {
        uart_putchar(uratn,(unsigned char)(pnum/100+'0'));
        uart_putchar(uratn,(unsigned char)(pnum/10%10+'0'));
        uart_putchar(uratn,(unsigned char)(pnum%10+'0'));
    }
    else if(pnum<=9999)
    {
        uart_putchar(uratn,pnum/1000+'0');uart_putchar(uratn,pnum/100%10+'0');
        uart_putchar(uratn,pnum/10%10+'0');uart_putchar(uratn,pnum%10+'0');
    }
    else if(pnum<=99999)
    {
        uart_putchar(uratn,pnum/10000+48);uart_putchar(uratn,pnum/1000%10+48);
        uart_putchar(uratn,pnum/100%10+48);uart_putchar(uratn,pnum/10%10+48);
        uart_putchar(uratn,pnum%10+48);
    }
}

/************************************************
*  函数名称：SCISendFloat
*  功能说明：SCI发送float数据
*  参数说明：UARTn为第几个串口，f为传送值
*  函数返回：无
*  修改时间：2014-1-14    已经测试
*************************************************/
void SCISendFloat(UARTn uratn,float f)
{				
    if(f<0)
    {
        uart_putchar(uratn,'-');
        f=(-1)*f;
    }
    uart_putchar(uratn,((int)(f))/100%10+'0');
    uart_putchar(uratn,((int)(f))/10%10+'0');
    uart_putchar(uratn,((int)(f))%10+'0');
    uart_putchar(uratn,'.');
    uart_putchar(uratn,((int)(f*10))%10+'0');
    uart_putchar(uratn,((int)(f*100))%10+'0');
    uart_putchar(uratn,' ');
}


/************************************************
*  函数名称：SCISend_to_PIDDebug
*  功能说明：SCI发送到串口示波器，学长的上位机
*  参数说明：UARTn为第几个串口
*  函数返回：无
*  修改时间：2014-1-14    已经测试
*************************************************/
void SCISend_to_PIDDebug(UARTn uratn)
{ 		
    uart_putchar(uratn,'I');	 		            	
    SCISend_Int(uratn,(int)(0));		//red,m_angle
    uart_putchar(uratn,'|');	
    SCISend_Int(uratn,(int)(0));		//blue,	
    uart_putchar(uratn,'|');
    SCISend_Int(uratn,(int)(0));			//Gyro_valuelight blue
    uart_putchar(uratn,'\r');
    uart_putchar(uratn,'\n');
}




/************************************************
*  函数名称：SCI_Send_Datas
*  功能说明：SCI发送到串口示波器，自己的上位机,AT+UART=115200,1,2,\r\n
*  参数说明：UARTn为第几个串口
*  函数返回：无
*  修改时间：2014-1-14    已经测试
*************************************************/
void SCI_Send_Datas(UARTn uratn)
{
  static unsigned short int send_data[3][8] = { { 0 }, { 0 }, { 0 } };
  
  send_data[0][0] = (unsigned short int)(Angle.NowH);//Angle.rotation
  send_data[0][1] = (unsigned short int)(Angle.GoalH);//Angle.pitch
  send_data[0][2] = (unsigned short int)(Angle.G_angleH);//Angle.roll
  send_data[0][3] = (unsigned short int)(Angle.m_angleH);//Angle.yaw
  send_data[0][4] = (unsigned short int)(0);//Angle.rotation);
  send_data[0][5] = (unsigned short int)(0);
  send_data[0][6] = (unsigned short int)(0);
  send_data[0][7] = (unsigned short int)(0);
  
  send_data[1][0] = (unsigned short int)(Gyro_ADC.x);
  send_data[1][1] = (unsigned short int)(Gyro_ADC.y);
  send_data[1][2] = (unsigned short int)(Gyro_ADC.z);
  send_data[1][3] = (unsigned short int)(0);
  send_data[1][4] = (unsigned short int)(0);
  send_data[1][5] = (unsigned short int)(0);
  send_data[1][6] = (unsigned short int)(0);
  send_data[1][7] = (unsigned short int)(0);
  
  send_data[2][0] = (unsigned short int)(Acc_ADC_Data.x);
  send_data[2][1] = (unsigned short int)(Acc_ADC_Data.y);
  send_data[2][2] = (unsigned short int)(Acc_ADC_Data.z);//.z
  send_data[2][3] = (unsigned short int)(0);
  send_data[2][4] = (unsigned short int)(0);
  send_data[2][5] = (unsigned short int)(0);
  send_data[2][6] = (unsigned short int)(0);
  send_data[2][7] = (unsigned short int)(0);
  
  uart_putchar(uratn, 'S');
  uart_putchar(uratn, 'T');
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 8; j++)
    {
      uart_putchar(uratn, (unsigned char)(send_data[i][j] & 0x00ff));
      uart_putchar(uratn, (unsigned char)(send_data[i][j] >> 8u));
    }
  
}


/************************************************
*  函数名称：SCI_Send_CCD
*  功能说明：SCI发送到串口示波器，自己上位机的ccd显示
*  参数说明：UARTn为第几个串口
*  函数返回：无
*  修改时间：2014-1-14    已经测试
*************************************************/
void SCI_Send_CCD(UARTn uratn, int ccdNum)
{
//  if (ccdNum != 1 && ccdNum != 2)
//    return;
//  uart_putchar(uratn, 255);
//  uart_putchar(uratn, 255);
//  uart_putchar(uratn, ccdNum);//number
//  for (int i = 0; i < 128; i++)
//    if (ccdData1[i] == 255)
//      uart_putchar(uratn, 254);
//    else
//      uart_putchar(uratn, ccdData1[i]);
//    if (ccdNum == 2)
//      for (int i = 0; i < 128; i++)
//	if (ccdData2[i] == 255)
//          uart_putchar(uratn, 254);
//	else
//          uart_putchar(uratn, ccdData2[i]);
}

/************************************************
*  函数名称：SCI_Send_fireImage
*  功能说明：//send fireImage camera Data
*  参数说明：UARTn为第几个串口
*  函数返回：无
*  修改时间：2014-1-14    已经测试
*************************************************/
void SCI_Send_fireImage(UARTn uratn)
{
	int H = 240, L = 320;
	//send "CAMERA"
	uart_putchar(uratn, 'C');
	uart_putchar(uratn, 'A');
	uart_putchar(uratn, 'M');
	uart_putchar(uratn, 'E');
	uart_putchar(uratn, 'R');
	uart_putchar(uratn, 'A');
	uart_putchar(uratn, (unsigned char)(H >> 8));//send Hang
	uart_putchar(uratn, (unsigned char)(H & 0x00FF));
	uart_putchar(uratn, (unsigned char)(L >> 8));//send Lie
	uart_putchar(uratn, (unsigned char)(L & 0x00FF));
	//send all data
	for (int i = 0; i < H; i++)
	for (int j = 0; j < L / 8; j++)
		uart_putchar(uratn, Image_fire[i][j]);
}


/************************************************
*  函数名称：Send_to_DataScope
*  功能说明：SCI发送到串口示波器：DataScope 
*  参数说明：UARTn为第几个串口
*  函数返回：无
*  修改时间：2014-1-14    已经测试
*************************************************/
float OutData[4] = {0};
void Send_to_DataScope(UARTn uratn)
{
//    delayms(5);
//    OutData[0]=Speed.Car;
//    OutData[1]=Speed.PWM_Integral;
//    OutData[2]=Speed.Goal+500;
//    OutData[3]=Speed.Goal-500;
//    OutPut_Data(uratn);
}
unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT)
{
    unsigned short CRC_Temp;
    unsigned char i,j;
    CRC_Temp = 0xffff;

    for (i=0;i<CRC_CNT; i++){      
        CRC_Temp ^= Buf[i];
        for (j=0;j<8;j++) {
            if (CRC_Temp & 0x01)
                CRC_Temp = (CRC_Temp >>1 ) ^ 0xa001;
            else
                CRC_Temp = CRC_Temp >> 1;
        }
    }
    return(CRC_Temp);
}
void OutPut_Data(UARTn uratn)
{
    int temp[4] = {0};
    unsigned int temp1[4] = {0};
    unsigned char databuf[10] = {0};
    unsigned char i;
    unsigned short CRC16 = 0;
    for(i=0;i<4;i++)
    {
        temp[i]  = (int)OutData[i];
        temp1[i] = (unsigned int)temp[i];
    }
    for(i=0;i<4;i++) 
    {
        databuf[i*2]   = (unsigned char)(temp1[i]%256);
        databuf[i*2+1] = (unsigned char)(temp1[i]/256);
    }
    CRC16 = CRC_CHECK(databuf,8);
    databuf[8] = CRC16%256;
    databuf[9] = CRC16/256;
    for(i=0;i<10;i++)
        uart_putchar(uratn,databuf[i]);
}