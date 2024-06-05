// sử dụng Port Data Direction để điều chỉnh led on hay off
// sử dụng thư viện cho KL47 thay vì define thủ công

#include "stdint.h"
#include "MKL46Z4.h"


void SystemCoreClockUpdate (void);

void initLed(void){
	// Enable Clock for Port
	//SIM->SCGC5 |= (1U<<12);
	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;

	// Set PTD5 as GPIO
	PORTD->PCR[5] &= ~PORT_PCR_MUX_MASK;// since MUX (10-8) alternative 1 as GPIO
	PORTD->PCR[5] |= PORT_PCR_MUX(1U);
	//Set Port as Output
	GPIOD->PDDR |= (1U<<5);

}

void turn_on_led(){
	//GPIO_PSOR = (1U<<5);		//cách hai, nhanh hơn trong xử lý
	GPIOD->PDOR &= ~(1U<<5);

}
void turn_off_led(){
	//GPIO_PCOR = (1U<<5);
	GPIOD->PDOR |= (1U<<5);
}

void toggle_led(){
	GPIOD->PDOR ^= (1U<<5);
}
int main(){
	initLed();
	turn_on_led();
	turn_off_led();

}