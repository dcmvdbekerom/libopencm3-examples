
#include "utils.h"

void wait (int n)
{
	int i;
	for (i = 0; i < n; i++) {	/* Wait a bit. */
		__asm__("nop");
	}		
}