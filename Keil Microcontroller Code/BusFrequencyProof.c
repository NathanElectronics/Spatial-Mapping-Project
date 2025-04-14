#include <stdint.h>
#include "tm4c1294ncpdt.h"
#include "PLL.h"
#include "SysTick.h"


void PortL_Init(void){
	//Use PortN onboard LED	
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R10;				// activate clock for Port N
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R10) == 0){};	// allow time for clock to stabilize
	GPIO_PORTL_DIR_R |= 0x01;        								// make PN0 out (PN0 built-in LED1)
  GPIO_PORTL_AFSEL_R &= ~0x01;     								// disable alt funct on PN0
  GPIO_PORTL_DEN_R |= 0x01;        								// enable digital I/O on PN0
																									// configure PN1 as GPIO
  GPIO_PORTN_AMSEL_R &= ~0x01;     								// disable analog functionality on PN0		
	
	return;
}


void DutyCycle_Percent(uint8_t duty){
		float percent;	
		percent = ((float)duty*1000)/(255);
		int percent_int;
		percent_int = (int)percent;
		GPIO_PORTL_DATA_R ^= 0b00000100;
		SysTick_Wait10us(percent_int);  //Note the use of the new function 10ns instead of 10ms
		GPIO_PORTL_DATA_R ^= 0b00000100;
		SysTick_Wait10us(1000-percent_int);
}

int main(void){
	
	PLL_Init();																			// Default Set System Clock to 120MHz
	SysTick_Init();																	// Initialize SysTick configuration
	PortL_Init();																		// Initialize Port N 
	
	uint8_t duty = 64;
	while(1){
		DutyCycle_Percent(duty);
	}
}





