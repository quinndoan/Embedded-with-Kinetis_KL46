
#include <stdint.h>
// đây là bản mẫu của a Nghĩa :>>
// sử dụng pointer để cập nhật vào register và cũng không define, library
void SystemCoreClockUpdate(void);

void delay(void)

{

    uint32_t i = 0;

    for (i = 0; i < 10000; ++i)

    {

        __asm("NOP"); /* delay */

    }

}

int check_bit_12(uint32_t *reg) {

    // Tạo mặt nạ bit tại vị trí thứ 12

    uint32_t mask = 1U << 12;



    // Lấy giá trị bit thứ 12 của thanh ghi

    int bit12 = (*reg & mask) != 0;

    // Trả về kết quả (1 nếu bit thứ 12 là 1, 0 nếu bit thứ 12 là 0)

    return bit12;

}

void main(){

	uint8_t *ptr = (uint8_t*)0x40064008;

	*ptr &= ~(7U << 1);

	*ptr |= (2U << 1);

	ptr = (uint8_t*)0x40064001;

	*ptr |= (1 << 0);



	ptr = (uint8_t*)0x40064000;

	*ptr &= ~(3U << 6);

	*ptr |= (1U << 6);



	uint32_t *ptr2 = (uint32_t*)0x40048044;

	*ptr2 &= ~(0xf << 28);



	SystemCoreClockUpdate();

	//SystemCoreClock;



	// enable clock for PORT D

	uint32_t *SIM_SCGC5 = (uint32_t *)0x40048038;

	*SIM_SCGC5 |= (1U << 12);



	// port - pin PD5

	uint32_t *PORTD_PCR5 = (uint32_t *)0x4004C014;

	*PORTD_PCR5 &= ~(7U << 8);

	*PORTD_PCR5 |= (1U << 8);


	// GPIO - configure pd5 = output

	uint32_t *GPIOD_PDDR = (uint32_t *)0x400FF0D4;

	*GPIOD_PDDR |= (1U << 5);



	//led on - port data output (0)

	uint32_t *GPIOD_PDOR = (uint32_t *)0x400FF0C0;



	// enable clock for PORT C

	*SIM_SCGC5 |= (1U << 11);



	// port - pin PC12 - GPIO

	uint32_t *PORTC_PCR12 = (uint32_t *)0x4004B030;

	*PORTC_PCR12 &= ~(7U << 8);

	*PORTC_PCR12 |= (1U << 8);



	*PORTC_PCR12 |= 3U;

	// GPIO - config pc12 = input

	uint32_t *GPIOC_PDDR = (uint32_t *)0x400FF094;

	*GPIOC_PDDR &= ~(1U << 12);

	//read in put pc12

	uint32_t *GPIOC_PDIR = (uint32_t *)0x400FF090;



	// port - pin PC3

	uint32_t *PORTC_PCR3 = (uint32_t *)0x4004B00C;

	*PORTC_PCR3 &= ~(7U << 8);

	*PORTC_PCR3 |= (1U << 8);

	// GPIO - config pc3 = input

	//uint32_t * GPIOC_PDDR = 0x400FF094;

	*GPIOC_PDDR &= ~(1U << 3);

	//read in put pc12

	//uint32_t * GPIOC_PDIR = 0x400FF090;



	while(1){

		if(((*GPIOC_PDIR >> 12)&1) == 1){

			*GPIOD_PDOR &= ~(1U << 5);

		}

		else {

			*GPIOD_PDOR |= (1U << 5);

		}

		// led off - port data output (1)



//		delay();

	}

}