#ifndef _SSI_H
#define	_SSI_H

#define SSInops() { asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");   \
                    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");   \
                    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");   \
                    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");   \
                    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");   \
                    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");   \
                    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");   \
                    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");   \
}
void SSI_Init();
unsigned short int SSIRead();

#endif /* __SSI_H */
