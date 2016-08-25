#ifndef _CCD_H_
#define _CCD_H_


#define SI_1  PTB1_OUT
#define CLK_1 PTB2_OUT

extern unsigned char Pixel[128]; 


void StartBoom();		
void CatchPixel();
void SendPixel();

void Cpu_Delay1us();						
void Cpu_Delay200ns();
void SamplingDelay();

#endif 
