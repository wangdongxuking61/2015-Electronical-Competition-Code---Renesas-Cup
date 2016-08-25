#include "include.h"

//ccd数组
unsigned char Pixel[128]; 	  
int BoomTime = 12;
int black_centre;



void StartBoom() 		 
{
	unsigned char i;
	SI_1  = 1;          
	SamplingDelay();
	CLK_1 = 1;          
	SamplingDelay();
	SI_1  = 0;         
	SamplingDelay();
	CLK_1 = 0;         
	for(i=0; i<127; i++)
	{
		SamplingDelay();  SamplingDelay();
		CLK_1 = 1;    
		SamplingDelay(); SamplingDelay();
		CLK_1 = 0 ;       
	}
	SamplingDelay();SamplingDelay();
	CLK_1 = 1;         
	SamplingDelay(); SamplingDelay();
	CLK_1 = 0;           
}




void CatchPixel()
{
	unsigned char i;
	
	SI_1  = 1;           
	SamplingDelay();
	CLK_1 = 1;          
	SamplingDelay();
	SI_1  = 0;           
	SamplingDelay();
	//Delay_10us for sample the first pixel
	for(i = 0; i < 10; i++) 
	{ 
		Cpu_Delay1us();
	}
	Pixel[0] = ad_once(ADC0,SE8,ADC_8bit);
	CLK_1 = 0;           			
	for(i=1; i<128; i++) 		//Sampling Pixel 2~128
	{
		SamplingDelay();SamplingDelay();
		CLK_1 = 1;       
		SamplingDelay();SamplingDelay();
		Pixel[i] = ad_once(ADC0,SE8,ADC_8bit);
		CLK_1 = 0;      
	}
	SamplingDelay();SamplingDelay();
	CLK_1 = 1;           
	SamplingDelay();SamplingDelay();
	CLK_1 = 0;          
}






void SendPixel()
{
	static unsigned char ImageLine[256];
	
	//画当前的一行线
	int tmp;
	for(int i=0;i<128;i++)
	{
		tmp=2*i;
		//还原
		LCD_SetPos(tmp,tmp,239 - ImageLine[tmp],239 - ImageLine[tmp]);
		write_word(Yellow);
		if(Pixel[tmp] > 239)
			ImageLine[tmp] = 239;
		else
			ImageLine[tmp] = Pixel[i];
		//上色
		LCD_SetPos(tmp,tmp,239 - ImageLine[tmp],239 -ImageLine[tmp]);
		write_word(Black);
		
		tmp++;
		//还原
		LCD_SetPos(tmp,tmp,239 - ImageLine[tmp],239 - ImageLine[tmp]);
		write_word(Yellow);
		if(Pixel[tmp] > 239)
			ImageLine[tmp] = 239;
		else
			ImageLine[tmp] = Pixel[i];
		//上色
		LCD_SetPos(tmp,tmp,239 - ImageLine[tmp],239 -ImageLine[tmp]);
		write_word(Black);
	}
}








void Cpu_Delay1us() 
{ 	  
	asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
	asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
	asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
}

void  Cpu_Delay200ns()
{
	asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
	asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
}

void SamplingDelay()
{
	asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
	asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
}


/*
int delaytimeL,delaytimeR;
int Middle=64,halfwidthh=50,black_RR,black_LL;
int get_black() 
{ 
	int i,ccd_start=1,ccd_end=126,decrease=0;
	int Image_MAX,Image_MIN,MAX_MIN_DELTA;
	unsigned char getleft_flag=0,getright_flag=0; 
	int Left_Count, Right_Count;
	
	Image_MAX=Pixel[ccd_start],Image_MIN=Pixel[ccd_start];
	for(i=ccd_start+1;i<ccd_end;i++) 
	{
		if(Pixel[i]>Image_MAX) 
			Image_MAX=Pixel[i];
		if(Pixel[i]<Image_MIN) 
			Image_MIN=Pixel[i];
	}
	MAX_MIN_DELTA = (int)((Image_MAX - Image_MIN) / 4.0);
	
	
	Right_Count = Middle ;
	while((Pixel[Right_Count-3]-Pixel[Right_Count]) < MAX_MIN_DELTA
		  && Right_Count < ccd_end)
	{Right_Count++;}
	if(Right_Count<ccd_end-2)
	{
		black_RR = Right_Count-decrease;
		getright_flag=1;
	}
	else
		getright_flag=0;
	
	Left_Count = Middle;
	while(Pixel[Left_Count+3]-Pixel[Left_Count] < MAX_MIN_DELTA
		  && Left_Count > ccd_start)	  
	{Left_Count--;}
	if(Left_Count > ccd_start+2)
	{
		black_LL = Left_Count + decrease;
		getleft_flag = 1;
	} 
	else
		getleft_flag = 0;
	
	//返回算中线
	if(getleft_flag==0 && getright_flag==0)
	{
		Middle = 64;
	}
	else if(getleft_flag!=1 && getright_flag==1)
	{
		//if(black_R > 127 - Edge_limit)
		{
			//Middle = 64;
		}
		//else
		{
			Middle = black_RR - halfwidthh;
			black_LL = black_RR - halfwidthh*2;
		}
		if(Right_angle_fg==1 && Time_1ms>Time_1ms_re[1]+delaytimeL)
		{
			Time_1ms_re[2] = Time_1ms;
			Right_angle_fg=2;
		}
	}
	else if(getleft_flag==1 && getright_flag!=1)
	{
		//if(black_L < Edge_limit)
		{
			//Middle=64;
		}
		//else
		{
			Middle=black_LL + halfwidthh;
			black_RR= black_LL + halfwidthh*2; 
		}
		if(Right_angle_fg==1 && Time_1ms>Time_1ms_re[1]+delaytimeR)
		{
			Time_1ms_re[3] = Time_1ms;
			Right_angle_fg = 3;
		}
	}
	else if(getleft_flag==1 && getright_flag==1) 
	{
		halfwidthh = (int)((black_RR - black_LL)/2.0) ;
		if(halfwidthh < 30)
			halfwidthh = 30;
		else if(halfwidthh >60)
			halfwidthh = 60; 
		Middle = (int)((black_RR + black_LL)/2.0); 
		if((Right_angle_fg==2 && Time_1ms>Time_1ms_re[2]+100)||
		   (Right_angle_fg==3&& Time_1ms>Time_1ms_re[3]+100))
		{
			Time_1ms_re[4] = Time_1ms;
			Right_angle_fg=4;
		}
	}
	
	if(Middle < 10)
		Middle = 10;
	else if(Middle > 120)
		Middle = 120;
	return(Middle); 
}*/




int find_black()
{
	for(int i=30;i<96;i++)
		if(Pixel[i]<Pixel[i-5]-25 && Pixel[i]<Pixel[i+5]-25)
			return i;
	return 0;
}