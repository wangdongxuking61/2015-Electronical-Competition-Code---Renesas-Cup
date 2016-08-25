#include "include.h"
/******************** (C) COPYRIGHT 2011 野火嵌入式开发工作室 ********************
* 文件名       ：IIC.c
* 描述         ：加速度计和陀螺仪软件模拟IIC驱动程序
* 实验平台     ：凌立印象开发板
* 库版本       ：基于野火库
* 嵌入系统     ：
* 作者         ：xuxu
**********************************************************************************/




/************************************************
*  函数名称：IIC_start
*  功能说明：IIC start
*  参数说明：无
*  函数返回：无
*  修改时间：2014-1-14    已经测试
*************************************************/
void IIC_start()
{
    SCLout;
    SDAout;
    SCL_L;
    asm("nop");
    SDA_H;
    nop5();
    SCL_H;
    nops();
    SDA_L;
    nops();
    SCL_L;
}



/************************************************
*  函数名称：IIC_stop
*  功能说明：IIC end//送停止位 SDA=0->1
*  参数说明：无
*  函数返回：无
*  修改时间：2014-1-14    已经测试
*************************************************/
void IIC_stop()
{
    SCLout;
    SDAout;
    SCL_L;nop5();
    SDA_L;nop5();
    SCL_H;nops();
    SDA_H;nops();
    SCL_L;
}




/************************************************
*  函数名称：IIC_send_byte
*  功能说明：IIC end字节发送程序
*  参数说明：c为字节
*  函数返回：无，不考虑从应答位
*  修改时间：2014-1-14    已经测试
*************************************************/
void send_byte(unsigned char c)
{
    unsigned char i;
    SCLout;
    SDAout;asm("nop");
    for(i=0;i<8;i++)
    {
        SCL_L;
        if((c<<i) & 0x80)
            SDA_H; //判断发送位
        else 
            SDA_L;
        nop5();
        SCL_H;
        nops();
        SCL_L;
    }
    nops();
    SDA_H; //发送完8bit，释放总线准备接收应答位
    nop5();
    SCL_H;
    nops(); //sda上数据即是从应答位
    SCL_L; //不考虑从应答位|但要控制好时序
}



/************************************************
*  函数名称：IIC_read_byte
*  功能说明：字节接收程序,接收器件传来的数据
*  参数说明：无
*  函数返回：return: uchar型1字节
*  修改时间：2014-1-14    已经测试
*************************************************/
unsigned char read_byte(void)
{
    unsigned char i;
    unsigned char c;
    SDAin;
    SCLout;
    c=0;
    SCL_L;
    nop5();
    for(i=0;i<8;i++)
    {
        nop5();
        SCL_L; //置时钟线为低，准备接收数据位
        nops();
        SCL_H; //置时钟线为高，使数据线上数据有效
        nop5();
        c<<=1;
        if(SDA_read)
            c+=1; //读数据位，将接收的数据存c
    }
    SCL_L;
    return c;
}




/************************************************
*  函数名称：IIC_I2C_Single_Write
*  功能说明：//写入寄存器
*  参数说明：SlaveAddress设备ID，寄存器地址address，thedata为写入数据
*  函数返回：return: uchar型1字节
*  修改时间：2014-1-14    已经测试
*************************************************/
void I2C_Single_Write(unsigned char SlaveAddress,unsigned char address, unsigned char thedata)
{
    IIC_start();		//启动
    send_byte(SlaveAddress);	//写入设备ID及写信号
    send_byte(address);	//X地址
    send_byte(thedata);	//写入设备ID及读信
    IIC_stop();
}



/************************************************
*  函数名称：IIC_I2C_Single_Read
*  功能说明：读寄存器 T = 46us(debug模式)
*  参数说明：SlaveAddress设备ID，寄存器地址address
*  函数返回：return1个字节，ret为读出数据
*  修改时间：2014-1-14    已经测试
*************************************************/
unsigned char I2C_Single_Read(unsigned char SlaveAddress,unsigned char address)
{
    unsigned char ret = 100;
    IIC_start();		//启动
    send_byte(SlaveAddress);	//写入设备ID及写信号
    send_byte(address);	//X地址
    IIC_start();		//重新发送开始
    send_byte(SlaveAddress+1);	//写入设备ID及读信
    ret = read_byte();	//读取一字节
    IIC_stop();
    return ret;
}


//加速度计和陀螺仪数据零偏是否采集完成的标志位
unsigned char Acc_CALIBRATED=1;
unsigned char Gyro_CALIBRATED=1;


//MPU6050初始化，传入参数：采样率，低通滤波频率
void MPU6050_Init(unsigned short sample_rate, unsigned short lpf)
{
    unsigned char default_filter;
    gpio_init(PORTC,19,GPO,LOW);
    gpio_init(PORTD, 0,GPO,LOW);
    //InitMPU6050()
    switch (lpf)
    {
    case 5:
        default_filter = MPU6050_LPF_5HZ;
        break;
    case 10:
        default_filter = MPU6050_LPF_10HZ;
        break;
    case 20:
        default_filter = MPU6050_LPF_20HZ;
        break;
    case 42:
        default_filter = MPU6050_LPF_42HZ;
        break;
    case 98:
        default_filter = MPU6050_LPF_98HZ;
        break;
    case 188:
        default_filter = MPU6050_LPF_188HZ;
        break;
    case 256:
        default_filter = MPU6050_LPF_256HZ;
        break;
    default:
        default_filter = MPU6050_LPF_98HZ;
        break;
    }	
    
    //设备复位
    I2C_Single_Write(MPU6050_ADDRESS,MPU_RA_PWR_MGMT_1, 0x80);	
    
    delayms(5);
    
    //陀螺仪采样率，0x00(1000Hz)   采样率 = 陀螺仪的输出率 / (1 + SMPLRT_DIV)
    I2C_Single_Write(MPU6050_ADDRESS,MPU_RA_SMPLRT_DIV, (1000/sample_rate - 1));	
    //设置设备时钟源，陀螺仪Z轴
    I2C_Single_Write(MPU6050_ADDRESS, MPU_RA_PWR_MGMT_1, 0x03);	
    //i2c旁路模式
    I2C_Single_Write(MPU6050_ADDRESS, MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 0 << 4 | 0 << 3 | 0 << 2 | 1 << 1 | 0 << 0); 
    // INT_PIN_CFG   -- INT_LEVEL_HIGH, INT_OPEN_DIS, LATCH_INT_DIS, INT_RD_CLEAR_DIS, FSYNC_INT_LEVEL_HIGH, FSYNC_INT_DIS, I2C_BYPASS_EN, CLOCK_DIS
    //低通滤波频率，0x03(42Hz)
    I2C_Single_Write(MPU6050_ADDRESS,MPU_RA_CONFIG, default_filter);	
    //陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
    I2C_Single_Write(MPU6050_ADDRESS, MPU_RA_GYRO_CONFIG, 0x18); 
    //加速计自检、测量范围(不自检，+-8G)			
    I2C_Single_Write(MPU6050_ADDRESS,MPU_RA_ACCEL_CONFIG, 2 << 3);	
}

//读取加速度
unsigned char mpu6050_buffer[12];short acc_temp[3];
void MPU6050_Read_Acc_Data(void)
{
    mpu6050_buffer[0] = I2C_Single_Read(MPU6050_ADDRESS,MPU_RA_ACCEL_XOUT_L); 
    mpu6050_buffer[1] = I2C_Single_Read(MPU6050_ADDRESS,MPU_RA_ACCEL_XOUT_H);
    acc_temp[0] = ((short)((mpu6050_buffer[1]<<8u) | mpu6050_buffer[0])) - ((short)(Acc_Offset.x));  //加速度X轴
    
    mpu6050_buffer[2] = I2C_Single_Read(MPU6050_ADDRESS,MPU_RA_ACCEL_YOUT_L);
    mpu6050_buffer[3] = I2C_Single_Read(MPU6050_ADDRESS,MPU_RA_ACCEL_YOUT_H);
    acc_temp[1] = ((short)((mpu6050_buffer[3]<<8u) | mpu6050_buffer[2]))- ((short)(Acc_Offset.y));  //加速度Y轴
    
    mpu6050_buffer[4] = I2C_Single_Read(MPU6050_ADDRESS,MPU_RA_ACCEL_ZOUT_L);
    mpu6050_buffer[5] = I2C_Single_Read(MPU6050_ADDRESS,MPU_RA_ACCEL_ZOUT_H);
    acc_temp[2] = ((short)((mpu6050_buffer[5]<<8u) | mpu6050_buffer[4]))- ((short)(Acc_Offset.z));  //加速度Z轴
    
    Acc_ADC.x = (float)acc_temp[0];
    Acc_ADC.y = (float)acc_temp[1];
    Acc_ADC.z = (float)acc_temp[2];
    
    MPU6050_CalOffset_Acc();//加速度零偏矫正,过了最开始的采集零偏的时间后Acc_CALIBRATED被置0，进入函数就返回了
}



//读取角速度
void MPU6050_Read_Gyro_Data(void)
{
    short gyro_temp[3];
    
    mpu6050_buffer[6] = I2C_Single_Read(MPU6050_ADDRESS,MPU_RA_GYRO_XOUT_L); 
    mpu6050_buffer[7] = I2C_Single_Read(MPU6050_ADDRESS,MPU_RA_GYRO_XOUT_H);
    gyro_temp[0] = ((short)((mpu6050_buffer[7]<<8u) | mpu6050_buffer[6]))  - ((short)(Gyro_Offset.x));	//陀螺仪X轴
    
    mpu6050_buffer[8] = I2C_Single_Read(MPU6050_ADDRESS,MPU_RA_GYRO_YOUT_L);
    mpu6050_buffer[9] = I2C_Single_Read(MPU6050_ADDRESS,MPU_RA_GYRO_YOUT_H);
    gyro_temp[1] = ((short)((mpu6050_buffer[9]<<8u) | mpu6050_buffer[8]))  -  ((short)(Gyro_Offset.y));	//陀螺仪Y轴
    
    mpu6050_buffer[10] = I2C_Single_Read(MPU6050_ADDRESS,MPU_RA_GYRO_ZOUT_L);
    mpu6050_buffer[11] = I2C_Single_Read(MPU6050_ADDRESS,MPU_RA_GYRO_ZOUT_H);
    gyro_temp[2] = ((short)((mpu6050_buffer[11]<<8u) | mpu6050_buffer[10])) - ((short)(Gyro_Offset.z));	  //陀螺仪Z轴		
    
    Gyro_ADC.x = (float)gyro_temp[0];
    Gyro_ADC.y = (float)gyro_temp[1];
    Gyro_ADC.z = (float)gyro_temp[2];
    
    MPU6050_CalOffset_Gyro();
    
    //	if(_fabsf(Gyro_ADC.x) < GyroAD_Limit) Gyro_ADC.x = 0;
    //	if(_fabsf(Gyro_ADC.y) < GyroAD_Limit) Gyro_ADC.y = 0;
    //	if(_fabsf(Gyro_ADC.z) < GyroAD_Limit) Gyro_ADC.z = 0;
    
    Gyro_dps.x = radians(Gyro_ADC.x * MPU6050G_s2000dps);   // dps
    Gyro_dps.y = radians(Gyro_ADC.y * MPU6050G_s2000dps);   // dps
    Gyro_dps.z = radians(Gyro_ADC.z * MPU6050G_s2000dps);   // dps	
}

//加速度零偏矫正
void MPU6050_CalOffset_Acc(void)
{
    if(Acc_CALIBRATED)
    {
        static struct CarThreeNum	tempAcc={0,0,0};
        static unsigned short cnt_a=0;
        
        if(cnt_a==0)
        {
            Acc_Offset.x = 0;
            Acc_Offset.y = 0;
            Acc_Offset.z = 0;
            tempAcc.x = 0;
            tempAcc.y = 0;
            tempAcc.z = 0;
            cnt_a = 1;
            return;
        }			
        tempAcc.x = (tempAcc.x + Acc_ADC.x) / 2.0f;
        tempAcc.y = (tempAcc.y + Acc_ADC.y) / 2.0f;
        tempAcc.z = (tempAcc.z + Acc_ADC.z) / 2.0f;
        if(cnt_a == CALIBRATING_ACC_CYCLES)
        {
            Acc_Offset.x = tempAcc.x;
            Acc_Offset.y = tempAcc.y;
            Acc_Offset.z = tempAcc.z - ADC_1G;
            cnt_a = 0;
            Acc_CALIBRATED = 0;
            //param.SAVE_ACC_OFFSET();//保存数据
            return;
        }
        cnt_a++;		
    }	
}

//陀螺仪零偏矫正
void MPU6050_CalOffset_Gyro(void)
{
    if(Gyro_CALIBRATED)
    {
        static struct CarThreeNum	tempGyro={0,0,0};
        static unsigned short cnt_g=0;
        if(cnt_g==0)
        {
            Gyro_Offset.x = 0;
            Gyro_Offset.y = 0;
            Gyro_Offset.z = 0;
            tempGyro.x = 0;
            tempGyro.y = 0;
            tempGyro.z = 0;
            cnt_g = 1;
            return;
        }
        tempGyro.x = (tempGyro.x + Gyro_ADC.x) / 2.0f;
        tempGyro.y = (tempGyro.y + Gyro_ADC.y) / 2.0f;
        tempGyro.z = (tempGyro.z + Gyro_ADC.z) / 2.0f;
        
        if(cnt_g == CALIBRATING_GYRO_CYCLES)
        {
            Gyro_Offset.x = tempGyro.x;
            Gyro_Offset.y = tempGyro.y;
            Gyro_Offset.z = tempGyro.z;
            cnt_g = 0;
            Gyro_CALIBRATED = 0;
            //param.SAVE_GYRO_OFFSET();//保存数据
            return;
        }
        cnt_g++;
    }
}

//更新传感器数据
void MPU6050_updateSensor(void)
{
    //读取加速度
    MPU6050_Read_Acc_Data();
    //读取角速度
    MPU6050_Read_Gyro_Data();	
}








/*
二阶滤波器的基本差分方程
*/
//截止频率:30Hz 采样频率:500Hz
#define b0 0.1883633f
#define b1 0
#define a1 1.023694f
#define a2 0.2120577f

struct CarThreeNum LastIn={0,0,0};
struct CarThreeNum PreOut={0,0,0};
struct CarThreeNum LastOut={0,0,0};

void Filter_2nd_LPF2ndFilter()//这个函数用于加速度计数据滤波，使用直立车滤波也效果一样
{
    Acc_ADC_Data.x = b0 * Acc_ADC.x + b1 * LastIn.x -  a1 * LastOut.x - a2 * PreOut.x ;
    Acc_ADC_Data.y = b0 * Acc_ADC.y + b1 * LastIn.y -  a1 * LastOut.y - a2 * PreOut.y;
    Acc_ADC_Data.z = b0 * Acc_ADC.z + b1 * LastIn.z -  a1 * LastOut.z - a2 * PreOut.z;
    
    LastIn.x = Acc_ADC.x;
    LastIn.y = Acc_ADC.y;
    LastIn.z = Acc_ADC.z;
    
    PreOut.x = LastOut.x;
    PreOut.y = LastOut.y;
    PreOut.z = LastOut.z;
    
    LastOut.x = Acc_ADC_Data.x;
    LastOut.y = Acc_ADC_Data.y;
    LastOut.z = Acc_ADC_Data.z;
}



/*
更新四元数
*/
#define Kp 1.0f
#define Ki 0.0f 
float norm;
float halfT;
float vx,vy,vz;
float ex,ey,ez;
float gx,gy,gz;
float ax,ay,az;	
static float exInt = 0, eyInt = 0, ezInt = 0;//定义姿态解算误差的积分

    
    
/***************************************************
对加速度计进行二阶低通滤波
参数gx，gy，gz分别对应三个轴的角速度，单位是弧度/秒
　参数ax，ay，az分别对应三个轴的加速度原始数据
***************************************************/
void Attitude_Updata_Quaternion(float deltaT)//没用到
{
    halfT = deltaT / 2.0f;
    ax = Acc_ADC_Data.x;
    ay = Acc_ADC_Data.y;
    az = Acc_ADC_Data.z;
    
    gx = Gyro_dps.x;
    gy = Gyro_dps.y;
    gz = Gyro_dps.z;
    
    //将加速度的原始数据，归一化，得到单位加速度
    norm = (double)sqrt((double)(ax * ax + ay * ay + az * az));
    if(norm == 0) 
        return;
    ax = ax / norm;
    ay = ay / norm;
    az = az / norm;
    
    /**************************************************
    把四元数换算成“方向余弦矩阵”中的第三列的三个元素。
    根据余弦矩阵和欧拉角的定义，地理坐标系的重力向量，
    转到机体坐标系，正好是这三个元素。所以这里的vx、vy、vz，
    其实就是当前的机体坐标参照系上，换算出来的重力单位向量。
    (用表示机体姿态的四元数进行换算)
    ***************************************************/
    vx = 2.0f * (Q.q2 * Q.q4 - Q.q1 * Q.q3);
    vy = 2.0f * (Q.q1 * Q.q2 + Q.q3 * Q.q4);
    vz = 1.0f - 2.0f * ( Q.q2 * Q.q2 + Q.q3 * Q.q3);//Q.w * Q.w + Q.z * Q.z;
    
    /***************************************************
    向量间的误差，可以用向量积(也叫外积、叉乘)来表示，	ex、
    ey、ez就是两个重力向量的叉积。这个叉积向量仍旧是位于机体
    坐标系上的，而陀螺积分误差也是在机体坐标系，而且叉积的大
    小与陀螺积分误差成正比，正好拿来纠正陀螺。由于陀螺是对机
    体直接积分，所以对陀螺的纠正量会直接体现在对机体坐标系的
    纠正。
    ***************************************************/
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);
    
    /***************************************************
    用叉乘误差来做PI修正陀螺零偏，通过调节Kp，Ki两个参数，可
    以控制加速度计修正陀螺仪积分姿态的速度
    ***************************************************/
    if(Ki > 0)
    {
        exInt = exInt + ex * Ki;
        eyInt = eyInt + ey * Ki;
        ezInt = ezInt + ez * Ki;
        gx = gx + Kp * ex + exInt;
        gy = gy + Kp * ey + eyInt;
        gz = gz + Kp * ez + ezInt;
    }
    else
    {
        gx = gx + Kp * ex;
        gy = gy + Kp * ey;
        gz = gz + Kp * ez;
    }
    
    //四元数微分方程 
    Q.q1 += (-Q.q2 * gx - Q.q3 * gy - Q.q4 * gz) * halfT;
    Q.q2 += ( Q.q1 * gx + Q.q3 * gz - Q.q4 * gy) * halfT;
    Q.q3 += ( Q.q1 * gy - Q.q2 * gz + Q.q4 * gx) * halfT;
    Q.q4 += ( Q.q1 * gz + Q.q2 * gy - Q.q3 * gx) * halfT;
    
    //四元数单位化
    norm = sqrt(Q.q1 * Q.q1 + Q.q2 * Q.q2 + Q.q3 * Q.q3 + Q.q4 * Q.q4);
    
    if(norm == 0) return;
    
    Q.q1 = Q.q1 / norm;
    Q.q2 = Q.q2 / norm;
    Q.q3 = Q.q3 / norm;
    Q.q4 = Q.q4 / norm;
    
    Quaternion_to_euler();
    
    //旋转角度
    if(Q.q3!=0)
        Angle.rotation = atan2f(Q.q2,Q.q3) * 57.29577951f;
    
    // 返回该四元数的等效旋转矩阵中的重力分量   
    Gravity.x = 2*(Q.q2*Q.q4 - Q.q1*Q.q3);								
    Gravity.y = 2*(Q.q1*Q.q2 + Q.q3*Q.q4);						  
    Gravity.z = 1-2*(Q.q2*Q.q2 + Q.q3*Q.q3);
    
    //和竖直方向的角度
    float normal = sqrt(Gravity.x * Gravity.x + Gravity.y * Gravity.y);
    Angle.vertical = 90.0f - atan2f(Gravity.z , normal) * 57.29577951f; 
}


//四元素转换陈欧拉角
void Quaternion_to_euler()
{
    Angle.roll = degrees(atan2f(2.0f*(Q.q1*Q.q2 + Q.q3*Q.q4),1 - 2.0f*(Q.q2*Q.q2 + Q.q3*Q.q3)));
    //使用safe_asin()来处理pitch接近90/-90时的奇点
    Angle.pitch = degrees(safe_asin(2.0f*(Q.q1*Q.q3 - Q.q2*Q.q4)));
    Angle.yaw = degrees(atan2f(2.0f*(Q.q2*Q.q3 - Q.q1*Q.q4), 2.0f*(Q.q1*Q.q1 + Q.q2*Q.q2) - 1));
}






//保证输入值是有效的
float safe_asin(float v)
{
    if (isnan(v)) {
        return 0.0f;
    }
    if (v >= 1.0f) {
        return M_PI/2;
    }
    if (v <= -1.0f) {
        return -M_PI/2;
    }
    return asinf(v);
}
//角度转弧度
float radians(float deg) 
{
    return deg * DEG_TO_RAD;
}

//弧度转角度
float degrees(float rad) 
{
    return rad * RAD_TO_DEG;
}

//32位整形数限幅
int constrain_int32(int amt, int low, int high) 
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}