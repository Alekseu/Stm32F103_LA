/*
 * la_sampling.cpp
 *
 *  Created on: 11.11.2012
 *      Author: user
 */

#include "la_sampling.h"

#include <stdlib.h>
#include "sump.h"


Sampler sampler;

static uint32_t transferCount;
static uint32_t delayCount;

static void SamplingFrameCompelte();
static void SamplingExternalEventInterrupt();
static void SamplingRLEExternalEventInterrupt();
static void SamplingManualStart();

template <class samplesType, uint32_t FLAG, uint32_t MAX_COUNT>
static void SamplingRLEFrameInterrupt() __attribute__ ( ( isr ) ) ;

template <class samplesType, uint32_t FLAG, uint32_t MAX_COUNT>
static void SamplingRLETailFrameInterrupt() __attribute__ ( ( isr ) ) ;

static InterruptHandler samplingManualToExternalTransit = NULL;
static InterruptHandler samplingRLETailFrameInterrupt = NULL;
static InterruptHandler comletionHandler = NULL;
uint32_t  samplingRam[MAX_SAMPLING_RAM/4];
uint32_t  rleTempSamplingRamA[MAX_RLE_SAMPLE_COUNT];
uint32_t  rleTempSamplingRamB[MAX_RLE_SAMPLE_COUNT];

static uint32_t rlePtr;
static uint32_t rleValue;
static uint32_t rleRepeatCount;
static int rleDelayCount;
static bool rleTailSampling;

#define RLE_16BIT_FLAG 0x8000
#define RLE_8BIT_FLAG 0x80

#define RLE_16BIT_MAX_COUNT 0x7fff
#define RLE_8BIT_MAX_COUNT 0x7f


extern "C"
{

	uint8_t _triggered=0;

#include "../StdPeriph/cmsis_boot/stm32f10x.h"
#include "../StdPeriph/stm_lib/inc/stm32f10x_dma.h"
#include "../StdPeriph/stm_lib/inc/stm32f10x_rcc.h"
#include "../StdPeriph/stm_lib/inc/stm32f10x_gpio.h"

#include "../StdPeriph/stm_lib/inc/stm32f10x_tim.h"
#include "../StdPeriph/stm_lib/inc/stm32f10x_exti.h"
#include "../StdPeriph/stm_lib/inc/misc.h"


	void TIM8_UP_IRQHandler()
	{
		if (TIM_GetITStatus(TIM8,TIM_IT_Update) != RESET)
		{
			TIM_ClearITPendingBit(TIM8, TIM_IT_Update);
			SamplingFrameCompelte();
		}
	}

	void DMA2_Channel4_5_IRQHandler(void) //tx
	{
		if (DMA2->ISR & DMA2_IT_TC5)
		{
			DMA2->IFCR =DMA2_IT_TC5;

			if(samplingRLETailFrameInterrupt!=0)
			{
				samplingRLETailFrameInterrupt();
			}

			return;
		}
		if (DMA2->ISR & DMA2_IT_HT5)
		{
			DMA2->IFCR =DMA2_IT_HT5;

			return;
		}
	}


	void Exti_Interrupt(uint8_t chan)
	{
		samplingManualToExternalTransit();
	}

	void EXTI0_IRQHandler()
	{
		if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
		Exti_Interrupt(0);
		 EXTI_ClearITPendingBit(EXTI_Line0);
		}
	}

	void EXTI1_IRQHandler()
	{
		if (EXTI_GetITStatus(EXTI_Line1) != RESET) {
				Exti_Interrupt(1);
				 EXTI_ClearITPendingBit(EXTI_Line1);
				}
	}

	void EXTI2_IRQHandler()
	{
		if (EXTI_GetITStatus(EXTI_Line2) != RESET) {
				Exti_Interrupt(2);
				 EXTI_ClearITPendingBit(EXTI_Line2);
				}
	}

	void EXTI3_IRQHandler()
	{
		if (EXTI_GetITStatus(EXTI_Line3) != RESET) {
				Exti_Interrupt(3);
				 EXTI_ClearITPendingBit(EXTI_Line3);
				}
	}

	void EXTI4_IRQHandler()
	{
		if (EXTI_GetITStatus(EXTI_Line4) != RESET) {
				Exti_Interrupt(4);
				 EXTI_ClearITPendingBit(EXTI_Line4);
				}
	}

	void EXTI9_5_IRQHandler()
	{
		if (EXTI_GetITStatus(EXTI_Line5) != RESET)
		{
			Exti_Interrupt(5);
			EXTI_ClearITPendingBit(EXTI_Line5);
		}
		if (EXTI_GetITStatus(EXTI_Line6) != RESET) {
			Exti_Interrupt(6);
			EXTI_ClearITPendingBit(EXTI_Line6);
		}
		if (EXTI_GetITStatus(EXTI_Line7) != RESET) {
			Exti_Interrupt(7);
			EXTI_ClearITPendingBit(EXTI_Line7);
		}
		if (EXTI_GetITStatus(EXTI_Line8) != RESET) {
			Exti_Interrupt(8);
			EXTI_ClearITPendingBit(EXTI_Line8);
		}
		if (EXTI_GetITStatus(EXTI_Line9) != RESET) {
			Exti_Interrupt(9);
			EXTI_ClearITPendingBit(EXTI_Line9);
		}

	}

	void EXTI15_10_IRQHandler()
	{
		if (EXTI_GetITStatus(EXTI_Line10) != RESET)
		{
			Exti_Interrupt(10);
			EXTI_ClearITPendingBit(EXTI_Line10);
		}
		if (EXTI_GetITStatus(EXTI_Line11) != RESET) {
			Exti_Interrupt(11);
			EXTI_ClearITPendingBit(EXTI_Line11);
		}
		if (EXTI_GetITStatus(EXTI_Line12) != RESET) {
			Exti_Interrupt(12);
			EXTI_ClearITPendingBit(EXTI_Line12);
		}
		if (EXTI_GetITStatus(EXTI_Line13) != RESET) {
			Exti_Interrupt(13);
			EXTI_ClearITPendingBit(EXTI_Line13);
		}
		if (EXTI_GetITStatus(EXTI_Line14) != RESET) {
			Exti_Interrupt(14);
			EXTI_ClearITPendingBit(EXTI_Line14);
		}
		if (EXTI_GetITStatus(EXTI_Line15) != RESET) {
			Exti_Interrupt(15);
			EXTI_ClearITPendingBit(EXTI_Line15);
		}
	}



}


void Sampler::SetBufferSize(uint32_t value)
{
	transferCount = value;
}
void Sampler::SetDelayCount(uint32_t value)
{
	delayCount = value & 0xfffffffe;
	rleDelayCount = value & 0xfffffffe;
}

void Sampler::SetupSamplingTimer()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);

	//Main sampling timer
	TIM1->DIER = 0;
	TIM1->SR &= ~TIM_SR_UIF;
	TIM1->CNT = 0;
	TIM1->PSC = 0;
	TIM1->CR1 = TIM_CR1_URS;
	TIM1->ARR = period;//actual period is +1 of this value
	TIM1->CR2 = 0;
	TIM1->DIER = TIM_DIER_UDE;
	TIM1->EGR = TIM_EGR_UG;
}

uint32_t Sampler::CalcDMATransferSize()
{
	uint32_t dmaSize = 0;
	//handle 8/16/32 bit samplings
	switch(flags & SUMP_FLAG1_GROUPS)
	{
	case SUMP_FLAG1_GR_16BIT:
		transferSize = 2;
		dmaSize =DMA_PeripheralDataSize_HalfWord;
		break;
	case SUMP_FLAG1_GR_32BIT:
		transferSize = 4;
		dmaSize = DMA_PeripheralDataSize_Word;
		break;
	case SUMP_FLAG1_GR_8BIT:
	default:
		dmaSize = DMA_PeripheralDataSize_Byte;
		transferSize = 1;
		break;
	}
	return dmaSize;
}

void Sampler::SetupSamplingDMA(void *dataBuffer, uint32_t dataTransferCount)
{
	 RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
	uint32_t dmaSize = CalcDMATransferSize();

	DMA_InitTypeDef   DMA_InitStructure;
	DMA_DeInit(DMA2_Channel5);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(SAMPLING_PORT->IDR);//0x40013804;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)dataBuffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = dataTransferCount;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_PeripheralDataSize = dmaSize;
	DMA_InitStructure.DMA_MemoryDataSize = dmaSize;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Enable;
	DMA_Init(DMA2_Channel5, &DMA_InitStructure);


	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel =DMA2_Channel4_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init( &NVIC_InitStructure );

	DMA_ClearFlag(DMA2_IT_TC5|DMA2_IT_HT5);
	TIM_DMACmd(TIM1, TIM_DMA_Update, ENABLE);

#ifdef SAMPLING_RLE_FORCE_ZERO_ON_MSB
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN ;
	GPIO_Init(SAMPLING_PORT, &GPIO_InitStructure);
	SAMPLING_PORT->ODR = 0;
#endif
}

void Sampler::SetupRLESamplingDMA(void *dataBufferA, void *dataBufferB, uint32_t dataTransferCount)
{
	GPIO_InitTypeDef GPIO_InitStructure;
//	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_All;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
//	GPIO_Init(SAMPLING_PORT, &GPIO_InitStructure);

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
	uint32_t dmaSize = CalcDMATransferSize();

	DMA_InitTypeDef   DMA_InitStructure;
		DMA_DeInit(DMA2_Channel5);
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(SAMPLING_PORT->IDR);//0x40013804;
		DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)dataBufferA;
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
		DMA_InitStructure.DMA_BufferSize = dataTransferCount;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
		DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
		DMA_InitStructure.DMA_M2M = DMA_M2M_Enable;
		DMA_Init(DMA2_Channel5, &DMA_InitStructure);

		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_InitStructure.NVIC_IRQChannel =DMA2_Channel4_5_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init( &NVIC_InitStructure );

		DMA_ClearFlag(DMA2_IT_TC5|DMA2_IT_HT5);

	switch(transferSize)
	{
	case 1:

		samplingRLETailFrameInterrupt = SamplingRLETailFrameInterrupt<uint8_t, RLE_8BIT_FLAG, RLE_8BIT_MAX_COUNT>;

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
		break;
	default:
	case 2:
		samplingRLETailFrameInterrupt = SamplingRLETailFrameInterrupt<uint16_t, RLE_16BIT_FLAG, RLE_16BIT_MAX_COUNT>;

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
		break;
	}
#ifdef SAMPLING_RLE_FORCE_ZERO_ON_MSB
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN ;
	GPIO_Init(SAMPLING_PORT, &GPIO_InitStructure);
	SAMPLING_PORT->ODR = 0;
#endif
}

void Sampler::SetupRegularEXTITrigger(InterruptHandler interruptHandler)
{

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD|RCC_APB2Periph_AFIO,ENABLE);

//	GPIO_InitTypeDef GPIO_InitStructure;
//	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_All;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
//	GPIO_Init(SAMPLING_PORT, &GPIO_InitStructure);

//	//Trigger setup
	uint32_t rising = triggerMask & triggerValue;
	uint32_t falling = triggerMask & ~triggerValue;

	uint32_t extiCR = 0;
	switch((uint32_t)SAMPLING_PORT)
	{
	case GPIOA_BASE:extiCR = 0x0000;break;
	case GPIOB_BASE:extiCR = 0x1111;break;
	case GPIOC_BASE:extiCR = 0x2222;break;
	case GPIOD_BASE:extiCR = 0x3333;break;
	case GPIOE_BASE:extiCR = 0x4444;break;
	case GPIOF_BASE:extiCR = 0x5555;break;
	case GPIOG_BASE:extiCR = 0x6666;break;

	}


	/* Tell system that you will use PB0 for EXTI_Line0 */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource0);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource1);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource2);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource3);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource4);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource5);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource6);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource7);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource8);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource9);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource10);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource11);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource12);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource13);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource14);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource15);

	EXTI->IMR  = 0;//mask;
	EXTI->PR = 0xffffffff;
	EXTI->RTSR = rising;
	EXTI->FTSR = falling;
//
	__DSB();

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel =EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init( &NVIC_InitStructure );

	NVIC_InitStructure.NVIC_IRQChannel =EXTI1_IRQn;
	NVIC_Init( &NVIC_InitStructure );

	NVIC_InitStructure.NVIC_IRQChannel =EXTI2_IRQn;
	NVIC_Init( &NVIC_InitStructure );

	NVIC_InitStructure.NVIC_IRQChannel =EXTI3_IRQn;
	NVIC_Init( &NVIC_InitStructure );

	NVIC_InitStructure.NVIC_IRQChannel =EXTI4_IRQn;
	NVIC_Init( &NVIC_InitStructure );

	NVIC_InitStructure.NVIC_IRQChannel =EXTI9_5_IRQn;
	NVIC_Init( &NVIC_InitStructure );

	NVIC_InitStructure.NVIC_IRQChannel =EXTI15_10_IRQn;
	NVIC_Init( &NVIC_InitStructure );

#ifdef SAMPLING_MANUAL //push-button-trigger
//	TIM8->SMCR = TIM_SMCR_TS_0 | TIM_SMCR_TS_1 | TIM_SMCR_TS_2;//External trigger input
//	TIM8->SMCR |= TIM_SMCR_SMS_1 | TIM_SMCR_SMS_2;
//	TIM8->DIER |= TIM_DIER_TIE;

	samplingManualToExternalTransit = interruptHandler;
#endif
}

void Sampler::SetupDelayTimer()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);
	//After-trigger delay timer
	TIM8->CR1 = TIM_CR1_URS;//stop timer too
	TIM8->CNT = 0;
	TIM8->ARR = delayCount;//  / transferSize;
	TIM8->PSC = TIM1->ARR;
	TIM8->CR2 = 0;
	TIM8->EGR = TIM_EGR_UG;
	TIM8->SR &= ~TIM_SR_UIF;
	TIM8->DIER = TIM_DIER_UIE;

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM8_UP_IRQn;///SamplingFrameCompelte
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init( &NVIC_InitStructure );

}

void Sampler::SetupRegular()
{
	//Sampling scheme:
	//TIM1 overflows with sampling period. Overflow triggers DMA read from sampling port.
	//DMA works in circular mode (so no bursts possible) continuously writing data from GPIO port to
	//sampling ram buffer
	//Once trigger interrupt shoots TIM8 comes in play counting "sampling period" * "delay count" ticks.
	//Once TIM8 overflows and sampled data are sent to host.
	SetupSamplingTimer();
	SetupSamplingDMA(samplingRam, transferCount);
	SetupDelayTimer();
	SetupRegularEXTITrigger(SamplingExternalEventInterrupt);
}

void Sampler::SetupRLE()
{
	//Sampling scheme:
	//Same as regular mode TIM1 provides reference clock for taking samples, but data are being stored in
	//the temporary buffer. Once buffer is full (DMA still works in circular mode) compression functions
	//scans it, writing to final sampling buffer pairs:
	//	1. Value, with zeroed MSB
	//	2. Repeat count for this value, MSB set high

	for(int i = 0; i < MAX_RLE_SAMPLE_COUNT; i++)
	{
		rleTempSamplingRamA[i] = rleTempSamplingRamB[i] = 0;
	}
	rlePtr = 0;
	rleValue = 0;
	rleRepeatCount = 0;
	rleDelayCount = delayCount;
	samplingRam[0] = rleValue;
	rleTailSampling = false;

	SetupSamplingTimer();
	SetupRLESamplingDMA(rleTempSamplingRamA, rleTempSamplingRamB, MAX_RLE_SAMPLE_COUNT);
	SetupRegularEXTITrigger(SamplingRLEExternalEventInterrupt);

	//GPIOD->PUPDR = GPIO_Mode_IPD;
	//GPIOD->PUPDR = 2 << 4; //????
}

void Sampler::Start()
{
	if(flags & SUMP_FLAG1_ENABLE_RLE)
	{
		SetupRLE();
	}
	else
	{
		SetupRegular();
	}

	DMA_ITConfig(DMA2_Channel5, DMA_IT_TC|DMA_IT_TE, ENABLE);
	DMA_Cmd(DMA2_Channel5,ENABLE);

	TIM1->CR1 |= TIM_CR1_CEN;//enable timer
}

void Sampler::Stop()
{
	DMA_Cmd(DMA2_Channel5,DISABLE);
	TIM1->CR1 &= ~TIM_CR1_CEN;//stop sampling timer
}

void Sampler::Arm(InterruptHandler handler)
{
	EXTI->PR = 0xffffffff;//clear pending
	__DSB();
	EXTI->IMR = triggerMask;

	comletionHandler = handler;

	//мое
	//transferSize = 2;
}

uint32_t Sampler::ActualTransferCount()
{
	return transferCount - (DMA2_Channel5->CNDTR & ~3);
}

uint8_t* Sampler::GetBufferTail()
{
	return (uint8_t*)(samplingRam) + ActualTransferCount() * transferSize;
}

uint32_t Sampler::GetBufferTailSize()
{
	return ActualTransferCount() * transferSize;
}

uint32_t Sampler::GetBufferSize()
{
	return transferCount * transferSize;
}

uint8_t* Sampler::GetBuffer()
{
	return (uint8_t*)samplingRam;
}

void SamplingClearBuffer()
{
	for(int i = 0; i < MAX_SAMPLING_RAM/4; i++)
		samplingRam[i] = 0;
}

static void SamplingFrameCompelte()
{
	TIM1->CR1 &= ~TIM_CR1_CEN;
	TIM8->CR1 &= ~TIM_CR1_CEN;
	TIM8->SR  &= ~TIM_SR_UIF;

	DMA_Cmd(DMA2_Channel5,DISABLE);

	if(comletionHandler != NULL)
		comletionHandler();
}


static void SamplingExternalEventInterrupt()
{
	EXTI->PR = 0xffffffff;
	__DSB();
	EXTI->IMR = 0;
	TIM8->CNT = 0;
	TIM8->SR  &= ~TIM_SR_UIF;
	TIM8->CR1 |= TIM_CR1_CEN;

}

static void SamplingRLEExternalEventInterrupt()
{
	EXTI->PR = 0xffffffff;
	__DSB();
	EXTI->IMR = 0;
	rleDelayCount = delayCount;
	rleTailSampling = true;

	//GPIOD->PUPDR = 1 << 4;

}

static void SamplingManualStart()
{
	TIM8->SR &= ~TIM_SR_TIF;
	TIM8->DIER &= ~TIM_DIER_TIE;
	//call regular handler
	//SamplingExternalEventInterrupt();

	samplingManualToExternalTransit();
}

template <class samplesType, uint32_t FLAG, uint32_t MAX_COUNT>
static void SamplingRLEFrameInterrupt()
{
//	DMA2->HIFCR = DMA_HIFCR_CTCIF5;
//
	static samplesType * store = (samplesType*)samplingRam;
	samplesType * samples = (samplesType*)rleTempSamplingRamA;//((DMA2_Channel5->CCR & DMA_CCR_CT5) ? rleTempSamplingRamA : rleTempSamplingRamB);
	int n = MAX_RLE_SAMPLE_COUNT;
	uint32_t newValue;

	do
	{
		newValue =  *samples++
#ifndef SAMPLING_RLE_FORCE_ZERO_ON_MSB
				& MAX_COUNT
#endif
				;

		if(rleValue == newValue)
		{
			rleRepeatCount++;
			if(MAX_COUNT == rleRepeatCount)//repeat count overflow
			{
				store[rlePtr++] = rleValue;
				store[rlePtr++] = rleRepeatCount | FLAG;
				rleRepeatCount = 0;
				if(rlePtr >= transferCount)
				{
					rlePtr = 0;
				}
			}
		}
		else//change detected
		{
			store[rlePtr++] = rleValue;
			store[rlePtr++] = rleRepeatCount | FLAG;
			if(rlePtr >= transferCount)
			{
				rlePtr = 0;
			}

			rleRepeatCount = 0;
			rleValue = newValue;
		}
	}
	while(--n);
}

template <class samplesType, uint32_t FLAG, uint32_t MAX_COUNT>
static void SamplingRLETailFrameInterrupt()
{
//	DMA2->HIFCR = DMA_HIFCR_CTCIF5;
//
	static samplesType * store = (samplesType*)samplingRam;
	samplesType * samples = (samplesType*)rleTempSamplingRamA;//((DMA2_Stream5->CR & DMA_SxCR_CT) ? rleTempSamplingRamA : rleTempSamplingRamB);
	int n = MAX_RLE_SAMPLE_COUNT;
	uint32_t newValue;

	do
	{
		newValue =  *samples++
#ifndef SAMPLING_RLE_FORCE_ZERO_ON_MSB
				& MAX_COUNT
#endif
				;

		if(rleValue == newValue)
		{
			rleRepeatCount++;
			if(MAX_COUNT == rleRepeatCount)//repeat count overflow
			{
				store[rlePtr++] = rleValue;
				store[rlePtr++] = rleRepeatCount | FLAG;
				rleRepeatCount = 0;

				rleDelayCount -= 2;
				if(rleDelayCount <= 0)
				{
					SamplingFrameCompelte();
					return;
				}

				if(rlePtr >= transferCount)
				{
					rlePtr = 0;
				}
			}
		}
		else//change detected
		{
			{
				store[rlePtr++] = rleValue;
				store[rlePtr++] = rleRepeatCount | FLAG;

				rleDelayCount -= 2;
				if(rleDelayCount <= 0)
				{
					SamplingFrameCompelte();
					return;
				}
			}

			if(rlePtr >= transferCount)
			{
				rlePtr = 0;
			}

			rleRepeatCount = 0;
			rleValue = newValue;
		}
	}
	while(--n);
}
