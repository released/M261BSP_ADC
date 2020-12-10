/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    A project template for M261 MCU.
 *
 * Copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


typedef enum{
	flag_DEFAULT = 0 ,

	
	flag_END	
}Flag_Index;

volatile uint32_t BitFlag = 0;
#define BitFlag_ON(flag)							(BitFlag|=flag)
#define BitFlag_OFF(flag)							(BitFlag&=~flag)
#define BitFlag_READ(flag)							((BitFlag&flag)?1:0)
#define ReadBit(bit)								(uint32_t)(1<<bit)

#define is_flag_set(idx)							(BitFlag_READ(ReadBit(idx)))
#define set_flag(idx,en)							( (en == 1) ? (BitFlag_ON(ReadBit(idx))) : (BitFlag_OFF(ReadBit(idx))))

uint32_t conter_tick = 0;

enum
{
	PB7_NU2_A0 = 0 ,
	PB6_NU2_A1 , 
	PB5_NU2_A2 , 
	PB4_NU2_A3 , 
	PB0_NU2_A4 , 
	PB1_NU2_A5 , 
};

enum
{
	ADC_CH0 = 0 ,
	ADC_CH1 , 
	ADC_CH2 , 
	ADC_CH3 , 
	ADC_CH4 , 
	
	ADC_CH5 , 
	ADC_CH6 , 
	ADC_CH7 , 
	ADC_CH8 , 
	ADC_CH9 , 

	ADC_CH10 , 
	ADC_CH11 , 
	ADC_CH12 , 
	ADC_CH13 , 
	ADC_CH14 , 
	
};

int32_t adc_array[6] = {0};

void tick_counter(void)
{
	conter_tick++;
}

uint32_t get_tick(void)
{
	return (conter_tick);
}

void set_tick(uint32_t t)
{
	conter_tick = t;
}

int32_t ReadEADC(EADC_T *eadc, uint32_t u32ChannelNum)
{
	uint32_t u32ModuleMask;
	int32_t  i32ConversionData;
	 
    u32ModuleMask = (BIT0 << u32ChannelNum);

	EADC_START_CONV(eadc, u32ModuleMask);
	while (EADC_GET_PENDING_CONV(eadc) != 0);

	i32ConversionData = EADC_GET_CONV_DATA(eadc, u32ChannelNum);

	return i32ConversionData;
}

void InitEADC(EADC_T *eadc, uint32_t u32ChannelNum)
{
	EADC_ConfigSampleModule(eadc, u32ChannelNum, EADC_SOFTWARE_TRIGGER, u32ChannelNum);

	EADC_SetExtendSampleTime(eadc, u32ChannelNum, 10);
}

void UARTx_Process(void)
{
	uint8_t res = 0;
	
	res = UART_READ(UART0);

	printf("%s  : 0x%2X\r\n" , __FUNCTION__ ,res);

	if (res > 0x7F)
	{
		printf("invalid command\r\n");
	}
	else
	{
		switch(res)
		{
	
			case '1':

				break;	

			case 'X':
			case 'x':
			case 'Z':
			case 'z':
			
				NVIC_SystemReset();
			
				break;		
			
		}
	}
}

void UART0_IRQHandler(void)
{
    if(UART_GET_INT_FLAG(UART0, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk))     /* UART receive data available flag */
    {
        while(UART_GET_RX_EMPTY(UART0) == 0)
        {
			UARTx_Process();
        }
    }

    if(UART0->FIFOSTS & (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk))
    {
        UART_ClearIntFlag(UART0, (UART_INTSTS_RLSINT_Msk| UART_INTSTS_BUFERRINT_Msk));
    }
}



void UART0_Init(void)
{
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);

	/* Set UART receive time-out */
	UART_SetTimeoutCnt(UART0, 20);

	UART0->FIFO &= ~UART_FIFO_RFITL_4BYTES;
	UART0->FIFO |= UART_FIFO_RFITL_8BYTES;

	/* Enable UART Interrupt - */
	UART_ENABLE_INT(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_TOCNTEN_Msk | UART_INTEN_RXTOIEN_Msk);
	
	NVIC_EnableIRQ(UART0_IRQn);

	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetPLLClockFreq : %8d\r\n",CLK_GetPLLClockFreq());
	printf("CLK_GetHCLKFreq : %8d\r\n",CLK_GetHCLKFreq());	
	printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());	
}


void TMR3_IRQHandler(void)
{
	static uint16_t CNT = 0;	
	static uint32_t log = 0;	
	
    if(TIMER_GetIntFlag(TIMER3) == 1)
    {
        TIMER_ClearIntFlag(TIMER3);
		tick_counter();

		if ((get_tick() % 10) == 0)
		{
//			set_flag(flag_SPI_Transmit_timing , ENABLE);
		}
	
		if (CNT++ > 1000)
		{		
			CNT = 0;
//			printf("%s : %2d\r\n" , __FUNCTION__ , log++);
		}
    }
}

void TIMER3_Init(void)
{
    TIMER_Open(TIMER3, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER3);
    NVIC_EnableIRQ(TMR3_IRQn);	
    TIMER_Start(TIMER3);
}



void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk|CLK_PWRCTL_HIRCEN_Msk|CLK_PWRCTL_LXTEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk|CLK_STATUS_HIRCSTB_Msk|CLK_STATUS_LXTSTB_Msk);
	
    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

	CLK->PCLKDIV = CLK_PCLKDIV_APB0DIV_DIV1 | CLK_PCLKDIV_APB1DIV_DIV1;

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable EADC module clock */
    CLK_EnableModuleClock(EADC_MODULE);

    /* EADC clock source is PCLK1, set divider to 8, ADC clock is PCLK1/8 MHz */
    CLK_SetModuleClock(EADC_MODULE, 0, CLK_CLKDIV0_EADC(3));

    CLK_EnableModuleClock(TMR3_MODULE);
    CLK_SetModuleClock(TMR3_MODULE, CLK_CLKSEL1_TMR3SEL_PCLK1, 0);

	CLK_EnableModuleClock(TMR0_MODULE);
	CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, 0);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~SYS_GPB_MFPH_PB12MFP_Msk) | SYS_GPB_MFPH_PB12MFP_UART0_RXD;
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~SYS_GPB_MFPH_PB13MFP_Msk) | SYS_GPB_MFPH_PB13MFP_UART0_TXD;

    /* Set PB.0 and PB.1 to input mode */
    PB->MODE &= ~(GPIO_MODE_MODE0_Msk | GPIO_MODE_MODE1_Msk| GPIO_MODE_MODE4_Msk| GPIO_MODE_MODE5_Msk| GPIO_MODE_MODE6_Msk| GPIO_MODE_MODE7_Msk);

    /* Configure the EADC analog input pins.  */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB0MFP_Msk) | SYS_GPB_MFPL_PB0MFP_EADC0_CH0;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB1MFP_Msk) | SYS_GPB_MFPL_PB1MFP_EADC0_CH1;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB4MFP_Msk) | SYS_GPB_MFPL_PB4MFP_EADC0_CH4;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB5MFP_Msk) | SYS_GPB_MFPL_PB5MFP_EADC0_CH5;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB6MFP_Msk) | SYS_GPB_MFPL_PB6MFP_EADC0_CH6;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB7MFP_Msk) | SYS_GPB_MFPL_PB7MFP_EADC0_CH7;

    /* Disable the digital input path to avoid the leakage current for EADC analog input pins. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT0 | BIT1| BIT4| BIT5| BIT6| BIT7);  /* Disable PB0 and PB1 */



    /* Lock protected registers */
    SYS_LockReg();
}


/*
 * This is a template project for M251 series MCU. Users could based on this project to create their
 * own application without worry about the IAR/Keil project settings.
 *
 * This template application uses external crystal as HCLK source and configures UART0 to print out
 * "Hello World", users may need to do extra system configuration based on their system design.
 */
int main()
{
	uint8_t i = 0;

    SYS_Init();

	UART0_Init();

	TIMER3_Init();

    EADC_Open(EADC, EADC_CTL_DIFFEN_SINGLE_END);

	InitEADC(EADC, ADC_CH0);	
	InitEADC(EADC, ADC_CH1);
	InitEADC(EADC, ADC_CH4);
	InitEADC(EADC, ADC_CH5);
	InitEADC(EADC, ADC_CH6);
	InitEADC(EADC, ADC_CH7);

    /* Got no where to go, just loop forever */
    while(1)
    {
		TIMER_Delay(TIMER0, 3000);
	
		adc_array[PB1_NU2_A5] = ReadEADC(EADC, ADC_CH1);
		adc_array[PB0_NU2_A4] = ReadEADC(EADC, ADC_CH0);
		adc_array[PB4_NU2_A3] = ReadEADC(EADC, ADC_CH4);
		adc_array[PB5_NU2_A2] = ReadEADC(EADC, ADC_CH5);
		adc_array[PB6_NU2_A1] = ReadEADC(EADC, ADC_CH6);
		adc_array[PB7_NU2_A0] = ReadEADC(EADC, ADC_CH7);

		for (i = PB7_NU2_A0 ; i < (PB1_NU2_A5+1) ; i++ )
		{
			printf("0x%3X , " ,adc_array[i] );
		}
		printf("\r\n");
		
    }
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
