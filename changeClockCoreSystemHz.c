#include <stdint.h>

void changeClockHz() {
	//FAST(4MHz) -> FCRDIV(/2)-> C2[IRCS] -> MCGCK -> CG
	// MCG_SC [FCRDIV 1->3] = 001
	uint8_t *ptr = (uint8_t*) 0x40064008;
	*ptr &= ~(7U << 1);
	*ptr = (2U << 1);
	// C2[IRCS] - Fast IRC
	ptr = (uint8_t*) 0x40064001;
	*ptr |= (1 << 0);

	// MCGCLOCK -Select IRC-01
	ptr = (uint8_t*) 0x40064000;
	*ptr &= ~(3U << 6);
	*ptr |= (1U << 6);
	// OUTDIV
	uint32_t *ptr2 = (uint32_t*) 0x40048044;
	*ptr2 &= ~(0xF << 28);

	// CORE Clock = 1MHz
	SystemCoreClockUpdate();
//	(void) SystemCoreClock;

}
