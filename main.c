//******************** (C) Yifeng ZHU ********************
// @file    main.c
// @author  Yifeng Zhu
// @version V1.0.0
// @date    November-11-2012
// @note    
// @brief   C code for STM32L1xx Discovery Kit
// @note
//          This code is for the book "Embedded Systems with ARM Cortex-M3 
//          Microcontrollers in Assembly Language and C, Yifeng Zhu, 
//          ISBN-10: 0982692625.
// @attension
//          This code is provided for education purpose. The author shall not be 
//          held liable for any direct, indirect or consequential damages, for any 
//          reason whatever. More information can be found from book website: 
//          http://www.eece.maine.edu/~zhu/book
//********************************************************

#include <stdint.h>

/* Standard STM32L1xxx driver headers */
#include "stm32l1xx.h"

/* STM32L1xx Discovery Kit:
    - USER Pushbutton: connected to PA0 (GPIO Port A, PIN 0), CLK RCC_AHBENR_GPIOAEN
    - RESET Pushbutton: connected RESET
    - GREEN LED: connected to PB7 (GPIO Port B, PIN 7), CLK RCC_AHBENR_GPIOBEN 
    - BLUE LED: connected to PB6 (GPIO Port B, PIN 6), CLK RCC_AHBENR_GPIOBEN
    - Linear touch sensor/touchkeys: PA6, PA7 (group 2),  PC4, PC5 (group 9),  PB0, PB1 (group 3)
*/



//******************************************************************************************
//* The main program starts here
//******************************************************************************************
void GPIO_Clock_Enable(int shiftbit){
	// Select enable the clock to GPIO Port 	
	RCC->AHBENR		|= 0x01<< shiftbit; 
}

void GPIO_Pin_Led_Init(int numled){
	// Set pin 6,7 I/0 mode as general-purpose output
	// 00 = digital input(default),   01 = digital output
	// 10 = alternative function,     11 = analog
	GPIOB->MODER &= ~(0x03<<(2*numled));
	GPIOB->MODER |= 0x01<<(2*numled);
	
	// Set output type of pin 6,7 as push-pull
	GPIOB->OTYPER &= ~(1<<numled);
	
	// Set output speed
	GPIOB->OSPEEDR &= ~(0x03<<(2*numled));
	GPIOB->OSPEEDR |= 0x01<<(2*numled);
	
	// Set I/O as no pull-up pull-down
	GPIOB->PUPDR &= ~(0x03<<(2*numled));
	
}

void SysTick_MSI_Clock_Config(int shiftbit){
	// Setting MSIRANGE bits of RCC_ICSCR
	RCC->ICSCR &= ~(0x07<<shiftbit);
	RCC->ICSCR |= 0x06<<shiftbit;
	// Internal Multi Speed clock enable 
	RCC->CR |= RCC_CR_MSION;
	// Wait for internal Multi Speed clock ready flag
	while(!(RCC->CR & RCC_CR_MSIRDY));	
}

void SysTick_Initialize(uint32_t counts){
	// Disable SysTick IRQ and SysTick counter
	SysTick->CTRL = 0;
	
	// Set reload register
	SysTick->LOAD = counts - 1;
	
	// Set priority
	NVIC_SetPriority (SysTick_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
	
	// Reset the SysTick counter value
	SysTick->VAL = 0;
	
	// Select processcer clock
	// 1 = processor clock;  0 = external clock
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE;
	
	// Enable SysTick IRQ and SysTick timer
	SysTick->CTRL |= SysTick_CTRL_ENABLE;
	
	// Enables SysTick exception request
	// 1 = counting down to zero asserts the SysTick exception request
	// 1 = counting down to zero does not asserts the SysTick exception request
	SysTick->CTRL |= SysTick_CTRL_TICKINT;	
}

uint32_t TimingDelay = 0;

//void SysTick_Handler(void){
//  if(TimingDelay !=0)
//	TimingDelay --;
//}

void Delay(uint32_t nTime){
	TimingDelay = nTime;
	while(TimingDelay != 0);
}

void Toggle_Led_1(){
	GPIOB->ODR ^= 1<< 6 | 1<<7;
}

void SysTick_Handler(void){
	Toggle_Led_1();
}

void Push_Button(){
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	SYSCFG->EXTICR[0] &= ~(0X000F);
	EXTI->RTSR |= EXTI_RTSR_TR0;
	EXTI->IMR |= EXTI_IMR_MR0;
	NVIC_SetPriority(ADC1_IRQn,1);
	NVIC_EnableIRQ(EXTI0_IRQn);
}

void EXTI0_IRQHandler(void){
	if(EXTI->PR & (1<<0)){
		GPIOB->ODR ^=1<<6;
		EXTI->PR |= 1<<0;
	}
}

int main(void){
	// Enable to GPIO Port B
	GPIO_Clock_Enable(1);
	// Use red led PB6
	GPIO_Pin_Led_Init(6);
	// Use blue led PB7
	GPIO_Pin_Led_Init(7);
	// The AHB clock frequency as 4.194 MHz
	SysTick_MSI_Clock_Config(13);
	//SysTick_Initialize(4194);
	SysTick_Initialize(4194*1000);
	while(1){
	// Delay 1s
	//Delay(1000);
			//Toggle_Led_1();	
			;
		}
}



