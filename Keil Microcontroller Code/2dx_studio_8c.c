/*  Time of Flight for 2DX3 - Project Deliverable 2
                Code written to support data collection from VL53L1X using the Ultra Light Driver.
                I2C methods written based upon MSP432E4 Reference Manual Chapter 19.
                Specific implementation was based upon format specified in VL53L1X.pdf pg19-21
                Code organized according to en.STSW-IMG009\Example\Src\main.c
                
                The VL53L1X is run with default firmware settings.


            Written by Nathan Chan
            Last Update: March 30, 2024

*/

#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"

//System Clock: 18MHz, PSYSDIV = 26 --> 480MHz/(26+1) = 18MHz

uint16_t numSteps = 0;
int stepIndex = 0;
uint8_t delay = 2;
volatile uint8_t Start_State = 0;
uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
int status=0;

uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
uint16_t wordData;
uint16_t Distance;
uint16_t SignalRate;
uint16_t AmbientRate;
uint16_t SpadNum; 
uint8_t RangeStatus;
uint8_t dataReady = 0;
uint8_t RdByte;
uint8_t RdWord;

// Enable interrupts
void EnableInt(void) { __asm("    cpsie   i\n"); }

// Disable interrupts
void DisableInt(void) { __asm("    cpsid   i\n"); }

// Low power wait
void WaitForInt(void) { __asm("    wfi\n"); }

// Global variable visible in Watch window of debugger
volatile unsigned long FallingEdges = 0;

#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // number of receive attempts before giving up

void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          																// 7) disable analog functionality on PB2,3

                                                                            // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                        						// 8) configure for 100 kbps clock
        
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
		GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
		GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
																																																			// configure PG0 as GPIO
		//GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
		GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
}

// Initialize Port N
// PN0, PN1
void PortN_Init(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;               // Activate clock for Port N
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R12) == 0){};    // Allow time for clock to stabilize
    
    GPIO_PORTN_DIR_R |= 0x07;         // Make PN0, PN1
    GPIO_PORTN_AFSEL_R &= ~0x07;      // Disable alt funct on PN0, PN1
    GPIO_PORTN_DEN_R |= 0x07;         // Enable digital I/O on PN0, PN1
    GPIO_PORTN_AMSEL_R &= ~0x07;      // Disable analog functionality on PN0, PN1
}

//Initialize Port F
// Assigned PF0 onboard LED
void PortF_Init(void){
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;                 	// Activate the clock for Port F
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R5) == 0){};					// Allow time for clock to stabilize
		
	GPIO_PORTF_DIR_R=0b00000001;															// Enable PF0 and PF4 as outputs
	GPIO_PORTF_DEN_R=0b00000001;															// Enable PF0 and PF4 as digital pins	
}

//Initialize Port H
// Stepper motor full wave drive
void PortH_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;				// activate clock for Port H
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){};	// allow time for clock to stabilize
	GPIO_PORTH_DIR_R = 0x0F;        								// configure pins as output
  GPIO_PORTH_DEN_R = 0x0F;        								// enable digital I/O 
																									// configure as GPIO
	return;
}


void PortJ_Init(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;               // Activate clock for Port J
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R8) == 0){};    // Allow time for clock to stabilize
				
		GPIO_PORTJ_DIR_R &= ~0x03;														 // Configure PJ0 and PJ1 as inputs
		GPIO_PORTJ_DEN_R |= 0x03;															 // Enables digital function on PJ0 and PJ1
			
		GPIO_PORTJ_PCTL_R &= ~0x000000FF;    									 // Disable alternate functions
    GPIO_PORTJ_AMSEL_R &= ~0x03;            							 // Disable analog function
    GPIO_PORTJ_PUR_R |= 0x03;           									 // Enable pull up resistors 
}


//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.

void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
}

// Interrupt initialization for GPIO Port J IRQ# 51
void PortJ_Interrupt_Init(void){
    FallingEdges = 0;                     // Initialize counter

    GPIO_PORTJ_IS_R = 0;                  // PJ0 and PJ1 is edge-sensitive 
    GPIO_PORTJ_IBE_R = 0;                 // PJ0 and PJ1 is not triggered by both edges 
    GPIO_PORTJ_IEV_R = 0;                 // PJ0 and PJ1 is falling edge event 
    GPIO_PORTJ_ICR_R = 0x03;              // Clear interrupt flag
    GPIO_PORTJ_IM_R = 0x03;               // Arm interrupt on PJ0 and PJ1
    
    NVIC_EN1_R = 0x00080000;              // Enable interrupt 51 in NVIC (EN1 register)
	//PRI12 therefore, 4*n = 48, 4(n+1) = 49, 4(n+2) = 50, 4(n+3) = 51 --> 31:29 Bits 
	//Priority 5 = 101 --> 1010 = A
    NVIC_PRI12_R = 0xA0000000;            // Set interrupt priority to 5
		
    EnableInt();                           // Enable global interrupts
}

void GPIOJ_IRQHandler(void) {
	//MIS Register is Masked Interrupt Status for Port J that tells you which pins caused the interrupts
    if(GPIO_PORTJ_MIS_R & 0x02) {
        Start_State ^= 1;
        GPIO_PORTJ_ICR_R = 0x02;
			// Need to clear interrupt flag to prep it for next interrupt call
    }
}

void DutyCycle_Percent(uint8_t duty){
		float percent;	
		percent = ((float)duty*1000)/(255); // Converts the duty cycle 255 --> to a 1000 scale for 1kHz waveform transmission
		int percent_int;
		percent_int = (int)percent; // --> converts float to integer form
		GPIO_PORTN_DATA_R ^= 0b00000100;
		SysTick_Wait10us(percent_int);  //Note the use of the new function 10ns instead of 10ms
		GPIO_PORTN_DATA_R ^= 0b00000100;
		SysTick_Wait10us(1000-percent_int);
	
		// Toggling LED on and off depending on percentage int given a 1000us/1kHz waveform
}

void Measuring(void){
	
    if(Start_State) {
        GPIO_PORTF_DATA_R |= 0x01; // Turn ON PF0 to indicate it being in measurement mode for Project Specs
       
        // Step the motor once full wave 
        GPIO_PORTH_DATA_R = 0b00000011;  
        SysTick_Wait10ms(delay);
        GPIO_PORTH_DATA_R = 0b00000110;  
        SysTick_Wait10ms(delay);
        GPIO_PORTH_DATA_R = 0b00001100; 
        SysTick_Wait10ms(delay);        
        GPIO_PORTH_DATA_R = 0b00001001; 
        SysTick_Wait10ms(delay);
        
        numSteps += 4;
        
        // If we've reached 64 steps, take a measurement, modulus of 64 because 2048 steps/32 measurements = 64
        if((numSteps % 128) == 0) {
            // Wait for data to be ready
            dataReady = 0;
            while (dataReady == 0) {
                status = VL53L1X_CheckForDataReady(dev, &dataReady);
                VL53L1_WaitMs(dev, 5);
                
                // Check if user pressed button to stop during measurement wait stop scan mid rotation while maintaining its numsteps
                if (!Start_State) {
                    break;
                }
            }
            
            // If we didn't break out due to a stop command, proceed with measurement
            if (Start_State) {
                status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
                status = VL53L1X_GetDistance(dev, &Distance);
                status = VL53L1X_GetSignalRate(dev, &SignalRate);
                status = VL53L1X_GetAmbientRate(dev, &AmbientRate);
                status = VL53L1X_GetSpadNb(dev, &SpadNum);
                
                sprintf(printf_buffer,"%u, %u, %u, %u, %u\r\n", RangeStatus, Distance, SignalRate, AmbientRate, SpadNum);
                FlashLED2(1);
                UART_printf(printf_buffer);
                
                // Clear the interrupt to prepare for next measurement
                status = VL53L1X_ClearInterrupt(dev);
            }
        }
        
        if (numSteps >= 2048) { //Once full rotation is complete after 2048 steps
            FlashLED1(5); // Signal end of full rotation using additional status LED
            numSteps = 0;
            Start_State = 0; // Stop after a full rotation
            GPIO_PORTF_DATA_R &= ~0x01; // Turn OFF PF0 after full rotation
            GPIO_PORTH_DATA_R = 0b00000000; // Turn off all stepper motor coils
        }
    } else {
        // Motor is stopped
        GPIO_PORTF_DATA_R &= ~0x01; // Turn OFF PF0 to indicate motor is stopped
        GPIO_PORTH_DATA_R = 0b00000000; // Turn off all stepper motor coils
    }
}


//*********************************************************************************************************
//*********************************************************************************************************
//******************************					MAIN Function				*********************************************
//*********************************************************************************************************
//*********************************************************************************************************

int main(void) {
    PLL_Init();	
    SysTick_Init();
    onboardLEDs_Init();
    I2C_Init();
    UART_Init();
    PortH_Init();
		PortN_Init();
    PortJ_Init();
    PortJ_Interrupt_Init();
    
    // hello world!
    UART_printf("Program Begins\r\n");
    
    int mynumber = 1;
    
    sprintf(printf_buffer,"2DX ToF Program Studio Code %d\r\n",mynumber);
    UART_printf(printf_buffer);
    
    // Those basic I2C read functions can be used to check your own I2C functions //
    status = VL53L1X_GetSensorId(dev, &wordData);
    
    sprintf(printf_buffer,"(Model_ID, Module_Type)=0x%x\r\n",wordData);
    UART_printf(printf_buffer);
    
    // 1 Wait for device booted
    while(sensorState==0) {
        status = VL53L1X_BootState(dev, &sensorState);
        SysTick_Wait10ms(10);
    }
    
    UART_printf("ToF Chip Booted!\r\n Please Wait...\r\n");
		FlashLED2(2); //Indicates successful ToF Chip Boot UART Transmission Starts
    
    status = VL53L1X_ClearInterrupt(dev);  //clear interrupt has to be called to enable next interrupt
    
    // 2 Initialize the sensor with the default setting  
    status = VL53L1X_SensorInit(dev);
    Status_Check("SensorInit", status);
    
    // 3 Optional functions to be used to change the main ranging parameters according the application requirements to get the best ranging performances 
    status = VL53L1X_SetDistanceMode(dev, 2); 
    
    // 4 Start ranging
    status = VL53L1X_StartRanging(dev);   // This function has to be called to enable the ranging
    status = VL53L1_RdByte(dev, 0x010F, &byteData);
    status = VL53L1_RdByte(dev, 0x0110, &byteData);
    status = VL53L1_RdWord(dev, 0x010F, &wordData);
    
    numSteps = 0;
    uint8_t duty = 128;

    while(1) {
        Measuring();
				// For proof of clock speed
				if(Start_State == 0){
						DutyCycle_Percent(duty);
				}
    }
    
    // This line should never be reached in this implementation
    VL53L1X_StopRanging(dev);
    return 0;
}
