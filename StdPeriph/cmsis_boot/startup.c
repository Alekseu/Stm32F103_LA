///*
// * startup.c
// *
// *  Created on: 10 но€б. 2016 г.
// *      Author: hudienko_a
// */
//
//#include "stm32f10x.h"
//#include "../stm_lib/inc/stm32f10x_rcc.h"
////#include "stm32f10x_usart.h"
////#include "stm32f10x_gpio.h"
//
///*
// * Startup minimal
// */
//
//extern  void main();
//#define VECT_TAB_OFFSET  0x0
//
//extern unsigned int _estack;
//typedef void (* const pfn)(void);
//
//void __attribute__((weak))
//Reset_Handler(void);
//
//__attribute__ ((section(".isr_vector")))
//pfn g_pfnVectors[] =
//{
//		(pfn) &_estack, // The initial stack pointer
//		Reset_Handler
//
//};
//
//extern unsigned int __bss_start__;
//extern unsigned int __bss_end__;
//
//// Begin address for the initialisation values of the .data section.
//// defined in linker script
//extern unsigned int _sidata;
//// Begin address for the .data section; defined in linker script
//extern unsigned int _sdata;
//// End address for the .data section; defined in linker script
//extern unsigned int _edata;
//
//
//inline void
//__attribute__((always_inline))
//bss_init(unsigned int* section_begin, unsigned int* section_end)
//{
//  // Iterate and clear word by word.
//  // It is assumed that the pointers are word aligned.
//  unsigned int *p = section_begin;
//  while (p < section_end)
//    *p++ = 0;
//}
//
//inline void
//__attribute__((always_inline))
//data_init(unsigned int* from, unsigned int* section_begin,
//    unsigned int* section_end)
//{
//  // Iterate and copy word by word.
//  // It is assumed that the pointers are word aligned.
//  unsigned int *p = section_begin;
//  while (p < section_end)
//    *p++ = *from++;
//}
//
//
//void SetClockExt72()
//{
//		  RCC->CR |= (uint32_t)0x00000001;
//
//		  /* Reset SW, HPRE, PPRE1, PPRE2, ADCPRE and MCO bits */
//
//		  RCC->CFGR &= (uint32_t)0xF8FF0000;
//
//
//		  /* Reset HSEON, CSSON and PLLON bits */
//		  RCC->CR &= (uint32_t)0xFEF6FFFF;
//
//		  /* Reset HSEBYP bit */
//		  RCC->CR &= (uint32_t)0xFFFBFFFF;
//
//		  /* Reset PLLSRC, PLLXTPRE, PLLMUL and USBPRE/OTGFSPRE bits */
//		  RCC->CFGR &= (uint32_t)0xFF80FFFF;
//
//
//		  /* Disable all interrupts and clear pending bits  */
//		  RCC->CIR = 0x009F0000;
//
//
//		  __IO uint32_t StartUpCounter = 0, HSEStatus = 0;
//
//		   /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration ---------------------------*/
//		   /* Enable HSE */
//		   RCC->CR |= ((uint32_t)RCC_CR_HSEON);
//
//		   /* Wait till HSE is ready and if Time out is reached exit */
//		   do
//		   {
//		     HSEStatus = RCC->CR & RCC_CR_HSERDY;
//		     StartUpCounter++;
//		   } while((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));
//
//		   if ((RCC->CR & RCC_CR_HSERDY) != RESET)
//		   {
//		     HSEStatus = (uint32_t)0x01;
//		   }
//		   else
//		   {
//		     HSEStatus = (uint32_t)0x00;
//		   }
//
//		   if (HSEStatus == (uint32_t)0x01)
//		   {
//		     /* Enable Prefetch Buffer */
//		     FLASH->ACR |= FLASH_ACR_PRFTBE;
//
//		     /* Flash 2 wait state */
//		     FLASH->ACR &= (uint32_t)((uint32_t)~FLASH_ACR_LATENCY);
//		     FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_2;
//
//
//		     /* HCLK = SYSCLK */
//		     RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;
//
//		     /* PCLK2 = HCLK */
//		     RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE2_DIV1;
//
//		     /* PCLK1 = HCLK */
//		     RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE1_DIV2;
//
//		     /*  PLL configuration: PLLCLK = HSE * 9 = 72 MHz */
//		     RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE |
//		                                         RCC_CFGR_PLLMULL));
//		     RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_HSE | RCC_CFGR_PLLMULL9);
//
//		     /* Enable PLL */
//		     RCC->CR |= RCC_CR_PLLON;
//
//		     /* Wait till PLL is ready */
//		     while((RCC->CR & RCC_CR_PLLRDY) == 0)
//		     {
//		     }
//
//		     /* Select PLL as system clock source */
//		     RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
//		     RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;
//
//		     /* Wait till PLL is used as system clock source */
//		     while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)0x08)
//		     {
//		     }
//		   }
//		   else
//		   { /* If HSE fails to start-up, the application will have wrong clock
//		          configuration. User can add here some code to deal with this error */
//		   }
//
//}
//
//void __attribute__ ((section(".after_vectors")))
//Reset_Handler(void)
//  {
//	bss_init(&__bss_start__, &__bss_end__);
//
//	data_init(&_sidata, &_sdata, &_edata);
//
//	SetClockExt72();
//
//	main();
//  }
//
//
