#include <lcd.h>
#include <Touch.h>
#include <stm32f10x_dma.h>
#include <stm32f10x_adc.h>
#include <stm32f10x.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_exti.h>
#include <stm32f10x_usart.h>
#include <stm32f10x_exti.h>
#include <stm32f10x_tim.h>
#include <core_cm3.h>
#include <misc.h>

#define TRUE  1
#define FALSE 0

#define INTERRUPTS 13

// 진동 센서, 압력 센서 개수
#define NUM_OF_INPUT 5
// 레코드 크기(개수)
#define RECORD_SIZE 600

// analog index
#define PC0 0
#define PC1 1
#define PC2 2
#define PC3 3
#define PC4 4

// Hertz
#define INF 0
#define DO  954
#define RE  850
#define MI  757
#define FA  715
#define SOL 637
#define RA  567
#define SI  505

// 압력센서로부터 읽어들이는 아날로그값
__IO uint32_t ADC_DualConvertedValueTab[NUM_OF_INPUT];

// 인터럽트 2번 방지
uint8_t DuplicatedInterrupt[INTERRUPTS];

// 버튼 클릭 여부, 인터럽트 2번 방지
uint8_t ButtonEvent[NUM_OF_INPUT];

// 각 진동센서에 할당된 주파수(=음)
uint32_t PitchOfSound[NUM_OF_INPUT];

// 주파수(=음)를 저장하는 배열
uint32_t RecordOfSound[RECORD_SIZE];

// 현재 배열 위치
uint32_t CurrentRecordIndex;

// 현재 녹음 상태
uint8_t RecordEnable;

// 레코드 시작 위치
uint32_t RecordStartIndex;

// 레코드 끝 위치
uint32_t RecordEndIndex;

// 재생 여부
uint8_t OnPlay;

// 재생 위치
uint8_t PlayIndex;

void RCC_Configuration(void);
void GPIO_Configuration(void);
void EXTI_ConfigurationUtil(uint8_t, uint8_t, uint32_t);
void EXTI_Configuration(void);
void NVIC_ConfigurationUtil(uint8_t, uint8_t, uint8_t);
void NVIC_Configuration(void);
void DMA_Configuration(void);
void ADC_Configuration(void);
void PWM_Configuration(void);

// 압력 센서값을 입력받아 대응하는 주파수를 반환한다.
uint32_t GeneratePitch(uint32_t analogValue);

void InitLCD(void);
void Set_Sample(void);

void ClearRecord(void);

// 진동센서로부터 읽어들인 아날로그값으로부터 각 진동센서에 주파수(=음)을 할당한다.
void Set_PitchOfSound(void);

static void Delay(volatile unsigned int nTime);

void LED_ON(uint16_t);
void LED_OFF(uint16_t);

// 주파수(=음)를 입력받아 해당하는 음을 출력한다.
void Sound(unsigned int);

// 디버깅을 위한 LCD 출력
void PrintLabelForDebuging(void);
void printAnalogValue(int, int);

void EXTI0_IRQHandler(void);
void EXTI1_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
void TIM2_IRQHandler(void);

int main() {
	SystemInit();
	RCC_Configuration();
	LCD_Init();
	LCD_Clear(WHITE);
	GPIO_Configuration();
	PWM_Configuration();
	EXTI_Configuration();
	NVIC_Configuration();
	DMA_Configuration();
	ADC_Configuration();

	PrintLabelForDebuging();
	Set_Sample();
	while (TRUE) {
	}
}

void Set_Sample(void) {
	int i;
	int sampleSize;
	uint32_t sample[] = {
	SOL, SOL, RA, RA, SOL, SOL, MI, INF, SOL,
	SOL, MI, MI, RE, INF, SOL, SOL, RA, RA,
	SOL, SOL, MI, INF, SOL, MI, RE, MI, DO };

	sampleSize = 27;
	RecordStartIndex = 0;
	RecordEndIndex = 0;

	for (i = 0; i < sampleSize; ++i) {
		RecordOfSound[RecordEndIndex++] = sample[i];
	}
}

void InitLCD(void) {
	LCD_Clear(WHITE);
	PrintLabelForDebuging();
}

void ClearRecord(void) {
	int i;

	for (i = 0; i < RECORD_SIZE; ++i) {
		RecordOfSound[i] = 0;
	}
	RecordStartIndex = RecordEndIndex = 0;
}

/*
 * 압력 센서 값
 * 4096 도
 * 3978 레
 * 3878 미
 * 3809 파
 * 3760 솔
 * 3705 라
 * 3648 시
 * 3613 도
 */
uint32_t GeneratePitch(uint32_t analogValue) {
	uint32_t pitch;
	if (analogValue >= 3978) {
		pitch = DO;
	} else if (analogValue >= 3878) {
		pitch = RE;
	} else if (analogValue >= 3809) {
		pitch = MI;
	} else if (analogValue >= 3760) {
		pitch = FA;
	} else if (analogValue >= 3705) {
		pitch = SOL;
	} else if (analogValue >= 3648) {
		pitch = RA;
	} else {
		pitch = SI;
	}
	return pitch;
}

void Set_PitchOfSound(void) {
	int temp;

	temp = ADC_DualConvertedValueTab[0];
	if (temp > 4021) {
		PitchOfSound[0] = DO;
	} else if (temp > 3985) {
		PitchOfSound[0] = RE;
	} else if (temp > 3880) {
		PitchOfSound[0] = MI;
	} else if (temp > 3818) {
		PitchOfSound[0] = FA;
	} else if (temp > 3724) {
		PitchOfSound[0] = SOL;
	} else if (temp > 3664) {
		PitchOfSound[0] = RA;
	} else {
		PitchOfSound[0] = SI;
	}

	temp = ADC_DualConvertedValueTab[1];
	if (temp > 4045) {
		PitchOfSound[1] = DO;
	} else if (temp > 3972) {
		PitchOfSound[1] = RE;
	} else if (temp > 3877) {
		PitchOfSound[1] = MI;
	} else if (temp > 3842) {
		PitchOfSound[1] = FA;
	} else if (temp > 3780) {
		PitchOfSound[1] = SOL;
	} else if (temp > 3703) {
		PitchOfSound[1] = RA;
	} else {
		PitchOfSound[1] = SI;
	}

	temp = ADC_DualConvertedValueTab[2];
	if (temp > 4017) {
		PitchOfSound[2] = DO;
	} else if (temp > 3960) {
		PitchOfSound[2] = RE;
	} else if (temp > 3932) {
		PitchOfSound[2] = MI;
	} else if (temp > 3912) {
		PitchOfSound[2] = FA;
	} else if (temp > 3880) {
		PitchOfSound[2] = SOL;
	} else if (temp > 3781) {
		PitchOfSound[2] = RA;
	} else {
		PitchOfSound[2] = SI;
	}

	temp = ADC_DualConvertedValueTab[3];
	if (temp > 4010) {
		PitchOfSound[3] = DO;
	} else if (temp > 3938) {
		PitchOfSound[3] = RE;
	} else if (temp > 3848) {
		PitchOfSound[3] = MI;
	} else if (temp > 3797) {
		PitchOfSound[3] = FA;
	} else if (temp > 3745) {
		PitchOfSound[3] = SOL;
	} else if (temp > 3682) {
		PitchOfSound[3] = RA;
	} else {
		PitchOfSound[3] = SI;
	}
}

static void Delay(volatile unsigned int nTime) {
	for (; nTime > 0; nTime--)
		;
}

void RCC_Configuration() {
	// for TIM2, TIM4
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM4, ENABLE);

	// for DMA
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	// for ADC1, GPIOA, GPIOB, GPIOC, GPIOD, TIM4
	RCC_APB2PeriphClockCmd(
	RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA |
	RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE); //enable ADC clock

	// for Interrupt
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
}

void GPIO_Configuration() {
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; // digital input
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; // digital input & selection
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; // digital input
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_3 | GPIO_Pin_2
			| GPIO_Pin_1 | GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; // analog input
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// Configure the GPIOD ports for output (led) debug
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4
			| GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	// Configure the GPIOD ports for input
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; // push down
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	// Configure the GPIOB ports for Sound(Analog output)
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; // TIM4 Channel 1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void EXTI_ConfigurationUtil(uint8_t GPIO_PortSource, uint8_t GPIO_PinSource,
		uint32_t EXTI_LineNo) {
	EXTI_InitTypeDef EXTI_InitStruture;

	GPIO_EXTILineConfig(GPIO_PortSource, GPIO_PinSource);
	EXTI_InitStruture.EXTI_Line = EXTI_LineNo;
	EXTI_InitStruture.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruture.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStruture.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStruture);
}

void EXTI_Configuration() {

	// Configure EXTI to generate an interrupt on falling edge.
	EXTI_ConfigurationUtil(GPIO_PortSourceGPIOA, GPIO_PinSource6,
	EXTI_Line6);
	EXTI_ConfigurationUtil(GPIO_PortSourceGPIOB, GPIO_PinSource0,
	EXTI_Line0);
	EXTI_ConfigurationUtil(GPIO_PortSourceGPIOB, GPIO_PinSource1,
	EXTI_Line1);
	EXTI_ConfigurationUtil(GPIO_PortSourceGPIOB, GPIO_PinSource8,
	EXTI_Line8);
	EXTI_ConfigurationUtil(GPIO_PortSourceGPIOC, GPIO_PinSource5,
	EXTI_Line5);
	EXTI_ConfigurationUtil(GPIO_PortSourceGPIOD, GPIO_PinSource11,
	EXTI_Line11);
	EXTI_ConfigurationUtil(GPIO_PortSourceGPIOD, GPIO_PinSource12,
	EXTI_Line12);

	// Clear EXTI Line Pending Bit
	EXTI_ClearITPendingBit(EXTI_Line0);
	EXTI_ClearITPendingBit(EXTI_Line1);
	EXTI_ClearITPendingBit(EXTI_Line5);
	EXTI_ClearITPendingBit(EXTI_Line6);
	EXTI_ClearITPendingBit(EXTI_Line8);
	EXTI_ClearITPendingBit(EXTI_Line11);
	EXTI_ClearITPendingBit(EXTI_Line12);
}

void NVIC_ConfigurationUtil(uint8_t NVIC_IRQChannel, uint8_t preemptionPriority,
		uint8_t subPriority) {
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = NVIC_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = preemptionPriority;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = subPriority;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void NVIC_Configuration() {
	NVIC_ConfigurationUtil(EXTI0_IRQn, 3, 3);
	NVIC_ConfigurationUtil(EXTI1_IRQn, 3, 3);
	NVIC_ConfigurationUtil(EXTI9_5_IRQn, 3, 3);
	NVIC_ConfigurationUtil(TIM2_IRQn, 1, 1); // Enable TIM2 Global Interrupt
	NVIC_ConfigurationUtil(EXTI15_10_IRQn, 3, 3);

	// Enable the Key EXTI line Interrupt
	NVIC_ClearPendingIRQ(EXTI0_IRQn);
	NVIC_ClearPendingIRQ(EXTI1_IRQn);
	NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
	NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
}

void DMA_Configuration() {
	DMA_InitTypeDef DMA_InitStructure;

	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & ADC1->DR;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) ADC_DualConvertedValueTab;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 5;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word; // 32bit
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word; // 32bit
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; // memory to memory
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	// Enable DMA1 Channel1
	DMA_Cmd(DMA1_Channel1, ENABLE);
}

void ADC_Configuration(void) {
	ADC_InitTypeDef myADC;

	//configure ADC1 parameters
	myADC.ADC_Mode = ADC_Mode_Independent;
	myADC.ADC_ScanConvMode = ENABLE;
	myADC.ADC_ContinuousConvMode = ENABLE;
	myADC.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	myADC.ADC_DataAlign = ADC_DataAlign_Right;
	myADC.ADC_NbrOfChannel = 5;
	ADC_Init(ADC1, &myADC);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1,
	ADC_SampleTime_55Cycles5); // PC0 as Input
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 2,
	ADC_SampleTime_55Cycles5); // PC1 as Input
	ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 3,
	ADC_SampleTime_55Cycles5); // PC2 as Input
	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 4,
	ADC_SampleTime_55Cycles5); // PC3 as Input
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 5,
	ADC_SampleTime_55Cycles5); // PC4 as Input
	ADC_DMACmd(ADC1, ENABLE);

	//enable
	ADC_Cmd(ADC1, ENABLE);

	//Calibrate ADC *optional
	ADC_ResetCalibration(ADC1);
	while (ADC_GetResetCalibrationStatus(ADC1))
		;
	ADC_StartCalibration(ADC1);
	while (ADC_GetCalibrationStatus(ADC1))
		;

	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void PWM_Configuration(void) {
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	/* TIM2 Initialize */
	TIM_TimeBaseStructure.TIM_Period = 10000 - 1; // 1kHz
	TIM_TimeBaseStructure.TIM_Prescaler = 3600 - 1; // 1MHz
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	/* TIM2 Enale */
	TIM_Cmd(TIM2, ENABLE);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); // interrupt enable

	// TIM4 TimeBase Initialize
	TIM_TimeBaseStructure.TIM_Period = 850; // ESC Input clock 1KHz -> 1ms
	TIM_TimeBaseStructure.TIM_Prescaler = 35; // STM32F107 TIM3 MAX Clock 24MHz
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	// TIM4 OutputChannel Initialize
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	// CH1 Enable
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM4, ENABLE);
	TIM_Cmd(TIM4, ENABLE);

	Delay(2400);
}

void LED_ON(uint16_t pinNo) {
	GPIO_SetBits(GPIOD, pinNo);
}

void LED_OFF(uint16_t pinNo) {
	GPIO_ResetBits(GPIOD, pinNo);
}

void Sound(unsigned int hertz) {
	TIM_TimeBaseInitTypeDef TIM4_TimeBaseInit;
	TIM_OCInitTypeDef TIM4_OCInit;

	// TIM4 TimeBase Initialize
	TIM4_TimeBaseInit.TIM_Period = hertz; // ESC Input clock 1KHz -> 1ms
	TIM4_TimeBaseInit.TIM_Prescaler = 35; // STM32F107 TIM3 MAX Clock 24MHz
	TIM4_TimeBaseInit.TIM_ClockDivision = 0;
	TIM4_TimeBaseInit.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM4_TimeBaseInit);

	// TIM4 OutputChannel Initialize
	TIM4_OCInit.TIM_OCMode = TIM_OCMode_PWM1;
	TIM4_OCInit.TIM_OCPolarity = TIM_OCPolarity_High;

	// CH1 Enable
	TIM4_OCInit.TIM_OutputState = TIM_OutputState_Enable;
	TIM4_OCInit.TIM_Pulse = 299;
	TIM_OC1Init(TIM4, &TIM4_OCInit);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM4, ENABLE);
	TIM_Cmd(TIM4, ENABLE);

	if (hertz == DO) {
		LCD_ShowString(100, 180, "DO  ", BLUE, WHITE);
	} else if (hertz == RE) {
		LCD_ShowString(100, 180, "RE  ", RED, WHITE);
	} else if (hertz == MI) {
		LCD_ShowString(100, 180, "MI  ", MAGENTA, WHITE);
	} else if (hertz == FA) {
		LCD_ShowString(100, 180, "FA  ", GREEN, WHITE);
	} else if (hertz == SOL) {
		LCD_ShowString(100, 180, "SOL ", BLACK, WHITE);
	} else if (hertz == RA) {
		LCD_ShowString(100, 180, "RA  ", RED, WHITE);
	} else if (hertz == SI) {
		LCD_ShowString(100, 180, "SI  ", BROWN, WHITE);
	} else if (hertz == INF) {
		LCD_ShowString(100, 180, "  ", WHITE, WHITE);
	}
	Delay(2000000);

	TIM4_OCInit.TIM_Pulse = 0;
	TIM_OC1Init(TIM4, &TIM4_OCInit);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM4, ENABLE);
	TIM_Cmd(TIM4, ENABLE);
}

void PrintLabelForDebuging(void) { // with LCD
	unsigned char* message[] = { "Button 1: Configure Pitch", "Button 2: Play",
			"Joystick Button: Record" };

	int coorX = 10, coorY = 70;
	int i;

	LCD_ShowString(60, 30, "Monday 06 ", BLUE, WHITE);

	LCD_ShowString(coorX, coorY, "Press Button!", BLACK, WHITE);
	coorY += 20;
	for (i = 0; i < 3; ++i) {
		LCD_ShowString(coorX, coorY, message[i], BLACK, WHITE);
		coorY += 20;
	}
}

void printAnalogValue(int value, int index) {
	int coorX = 50, coorY = 60;
	LCD_ShowNum(coorX + 60, coorY + index * 20, value, 4, BLACK, WHITE);
}

void EXTI0_IRQHandler(void) {
	if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
		EXTI_ClearITPendingBit(EXTI_Line0);

		if (OnPlay)
			return;

		if (DuplicatedInterrupt[0] == 0) {
			++DuplicatedInterrupt[0];

			if (RecordEnable) {
				RecordOfSound[CurrentRecordIndex] = PitchOfSound[0];
			}

			Sound(PitchOfSound[0]);
		}
	}
}

void EXTI1_IRQHandler(void) {
	if (EXTI_GetITStatus(EXTI_Line1) != RESET) {
		EXTI_ClearITPendingBit(EXTI_Line1);

		if (OnPlay)
			return;

		if (DuplicatedInterrupt[1] == 0) {
			++DuplicatedInterrupt[1];

			if (RecordEnable) {
				RecordOfSound[CurrentRecordIndex] = PitchOfSound[1];
			}

			Sound(PitchOfSound[1]);
		}
	}
}

void EXTI9_5_IRQHandler(void) {
	if (EXTI_GetITStatus(EXTI_Line5) != RESET) {
		EXTI_ClearITPendingBit(EXTI_Line5);

		if (OnPlay)
			return;

		if (DuplicatedInterrupt[5] == 0) {
			++DuplicatedInterrupt[5];

			if (RecordEnable) {
				RecordOfSound[CurrentRecordIndex] = PitchOfSound[2];
			}

			Sound(PitchOfSound[2]);
		}
	}
	if (EXTI_GetITStatus(EXTI_Line6) != RESET) {
		EXTI_ClearITPendingBit(EXTI_Line6);

		if (OnPlay)
			return;

		if (DuplicatedInterrupt[6] == 0) {
			++DuplicatedInterrupt[6];

			if (RecordEnable) {
				RecordOfSound[CurrentRecordIndex] = PitchOfSound[3];
			}

			Sound(PitchOfSound[3]);
		}
	}
	if (EXTI_GetITStatus(EXTI_Line8) != RESET) { // line 8: selection
		EXTI_ClearITPendingBit(EXTI_Line8);

		if (DuplicatedInterrupt[8] == 0) {
			++DuplicatedInterrupt[8];

			if (RecordEnable) { // 녹음중이면 끝냄
				InitLCD();
				RecordEndIndex = CurrentRecordIndex;
				RecordEnable = FALSE;
				LED_OFF(GPIO_Pin_7);
			} else { 			// 녹음시작
				ClearRecord();
				LCD_Clear(WHITE);
				LCD_ShowString(60, 30, "Recoding..", RED, WHITE);
				RecordStartIndex = 0;
				RecordEnable = TRUE;
				LED_ON(GPIO_Pin_7);
			}
		}

	}
}

void TIM2_IRQHandler(void) {
	int i;
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update); // Clear the interrupt flag

		for (i = 0; i < INTERRUPTS; ++i) {
			DuplicatedInterrupt[i] = 0;
		}

		if (OnPlay) { // 재생중이면 음 출력

			if (RecordOfSound[PlayIndex] != 0) {
				LED_ON(GPIO_Pin_2);
				Sound(RecordOfSound[PlayIndex]);
				LED_OFF(GPIO_Pin_2);
			} else {
				Sound(RecordOfSound[PlayIndex]);
			}
			LCD_ShowNum(100, 220, PlayIndex / 2, 4, BLACK, WHITE);
			LCD_ShowString(140, 220, "s", BLACK, WHITE);
			++PlayIndex;

			if (PlayIndex >= RecordEndIndex) {
				InitLCD();
				OnPlay = FALSE;
			}
		}

		if (RecordEnable) {
			++CurrentRecordIndex;
			LCD_ShowNum(100, 220, CurrentRecordIndex / 2, 4, BLACK, WHITE);
			LCD_ShowString(140, 220, "s", BLACK, WHITE);
			if (CurrentRecordIndex >= RECORD_SIZE - 1) {
				InitLCD();
				RecordEnable = FALSE;
				RecordEndIndex = CurrentRecordIndex;
				LED_OFF(GPIO_Pin_7);
			}
		}
	}
}

void EXTI15_10_IRQHandler(void) {
	int i;

	if (EXTI_GetITStatus(EXTI_Line11) != RESET) { // line 11: button 1
		EXTI_ClearITPendingBit(EXTI_Line11);

		if (DuplicatedInterrupt[11] == 0) {
			++DuplicatedInterrupt[11];

			Set_PitchOfSound();

			LCD_ShowString(10, 160, "Pitch Setting", BLACK, WHITE);
			for (i = 0; i < 4; ++i) {
				if (PitchOfSound[i] == DO) {
					LCD_ShowString(10, 180 + i * 20, "DO  ", BLUE, WHITE);
				} else if (PitchOfSound[i] == RE) {
					LCD_ShowString(10, 180 + i * 20, "RE  ", RED, WHITE);
				} else if (PitchOfSound[i] == MI) {
					LCD_ShowString(10, 180 + i * 20, "MI  ", MAGENTA, WHITE);
				} else if (PitchOfSound[i] == FA) {
					LCD_ShowString(10, 180 + i * 20, "FA  ", GREEN, WHITE);
				} else if (PitchOfSound[i] == SOL) {
					LCD_ShowString(10, 180 + i * 20, "SOL ", CYAN, WHITE);
				} else if (PitchOfSound[i] == RA) {
					LCD_ShowString(10, 180 + i * 20, "RA  ", LGRAY, WHITE);
				} else if (PitchOfSound[i] == SI) {
					LCD_ShowString(10, 180 + i * 20, "SI  ", BROWN, WHITE);
				}
			}
		}

	} else if (EXTI_GetITStatus(EXTI_Line12) != RESET) { // line 12: button 2
		EXTI_ClearITPendingBit(EXTI_Line12);
		if (DuplicatedInterrupt[12] == 0) {
			++DuplicatedInterrupt[12];

			// 녹음중이면 종료
			if (RecordEnable)
				return;

			if (OnPlay) { // 재생중이면 종료
				InitLCD();
				OnPlay = FALSE;
			} else { // 재생중이지 않으면 재생 시작
				PlayIndex = 0;
				LCD_ShowString(60, 30, "Playing...", BLUE, WHITE);
				OnPlay = TRUE;
			}
		}

	}
}
