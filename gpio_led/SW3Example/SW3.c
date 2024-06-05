#include <stdint.h>
// đây là code mẫu của ChatGPT
#define SIM_SCGC5   (*(volatile uint32_t *)0x40048038)
#define PORTD_PCR3  (*(volatile uint32_t *)0x4004C00C)
#define PORTD_PCR5  (*(volatile uint32_t *)0x4004C014)
#define GPIOD_PDDR  (*(volatile uint32_t *)0x400FF0D4)		// Data Direction
#define GPIOD_PDIR  (*(volatile uint32_t *)0x400FF0D0)
#define GPIOD_PSOR  (*(volatile uint32_t *)0x400FF0C4)
#define GPIOD_PCOR  (*(volatile uint32_t *)0x400FF0C8)
#define GPIOD_PTOR  (*(volatile uint32_t *)0x400FF0CC)

#define GPIO_PDOR (*(volatile uint32_t *) 0x400FF0C0)	// GIPO output

#define NVIC_ISER   (*(volatile uint32_t *)0xE000E100)

#define SIM_SCGC5_PORTD_MASK  (1U << 12)
#define PORT_PCR_MUX_GPIO     (1U << 8)
#define PORT_PCR_IRQC_MASK    (0xF << 16)
#define PORT_PCR_IRQC_FALLING_EDGE (0xA << 16)

void init_led_and_button(void) {
    // Enable clock for PORTD
    SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;

    // Configure PTD5 as GPIO for LED
    PORTD_PCR5 = PORT_PCR_MUX_GPIO;
    GPIOD_PDDR |= (1U << 5); // Set direction as output

    // Configure PTD3 as GPIO for button with interrupt
    PORTD_PCR3 = PORT_PCR_MUX_GPIO | PORT_PCR_IRQC_FALLING_EDGE;
    GPIOD_PDDR &= ~(1U << 3);  // Set PTD3 as input

    // Enable interrupt for PORTD
    NVIC_ISER |= (1U << 31);   // NVIC interrupt for PORTD is at bit position 31
}

void led_on(void) {
    //GPIOD_PCOR = (1U << 5);// Bật PTD5
	GPIO_PDOR &= ~(1U<<5);
}

void led_off(void) {
   // GPIOD_PSOR = (1U << 5);    // Tắt PTD5
	GPIO_PDOR |= (1U<<5);
}

void PORTD_IRQHandler(void) {
    // Check if the interrupt is from PTD3 (button SW3 press)
    if (PORTD_PCR3 & (1U << 24)) {
        if (GPIOD_PDIR & (1U << 5)) {
            led_off();
        } else {
            led_on();
        }
        PORTD_PCR3 |= (1U << 24); // Clear interrupt flag by writing 1 to ISF (Interrupt Status Flag)
    }
}

int test(void) {
    init_led_and_button();

    // Main loop
    while (1) {
        // Do nothing, wait for interrupts
    }
}