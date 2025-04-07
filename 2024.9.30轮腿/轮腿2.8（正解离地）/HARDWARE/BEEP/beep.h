#ifndef __CAN_H
#define __CAN_H	 
#include "sys.h"	


#define BEEP PCout(9)

void BEEP_Init(void);
void Sound(u16 frq);
#endif
