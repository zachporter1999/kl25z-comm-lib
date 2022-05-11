#include <MKL25Z4.h>
#include "delay.h"

void Delay (uint32_t dly) {
  volatile uint32_t t;

	for (t=dly*10000; t>0; t--)
		;
}
// *******************************ARM University Program Copyright © ARM Ltd 2013*************************************   
