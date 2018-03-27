/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Date: 15/06/10 12:01p $
 * @brief    NUC970 Driver Sample Code
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
 #include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "nuc970.h"
#include "sys.h"
#include "uart.h"



char outDmaUartBuffer[] = {"BLA-BLA-BLA\r\n"};
char inDmaUartBuffer[32];

void uart1Initialization(void)
{
	UART_T param;
	int  retval;
	/* configure UART */
	param.uFreq = 12000000;
	param.uBaudRate = 115200; 
	param.ucUartNo = UART1;
	param.ucDataBits = DATA_BITS_8;
	param.ucStopBits = STOP_BITS_1;
	param.ucParity = PARITY_NONE;
	param.ucRxTriggerLevel = UART_FCR_RFITL_1BYTE;	
	retval = uartOpen(&param); 

	
	/* set TX interrupt mode */
	//retval = uartIoctl(param.ucUartNo, UART_IOC_SETTXMODE, UARTINTMODE, 0);	
	/* set RX interrupt mode */
	retval = uartIoctl(param.ucUartNo, UART_IOC_SETRXMODE, UARTINTMODE, 0);
	
	//retval = uartIoctl(param.ucUartNo, UART_IOC_ENABLEHWFLOWCONTROL, 0, 0);
	outpw(REG_SYS_GPI_MFPL, (inpw(REG_SYS_GPI_MFPL) & 0xf00fffff) | 0x09900000);// GPE2, 3, 4, 5 //TX, RX, RTS, CTS 
	
	//uartWrite(UART1,"UART1 is init!!!",17);
	return;	
}

void gdmaInit(void)
{
	  outpw(REG_CLK_HCLKEN,(inpw(REG_CLK_HCLKEN)&0xFFFFEFFF) | 0x1000 );
		outpw(REG_GDMA_CTL0, 0x0);
		outpw(REG_GDMA_SRCB0, (int)&outDmaUartBuffer[0]);
		outpw(REG_GDMA_DSTB0, REG_UART1_THR);	
		outpw(REG_GDMA_TCNT0,strlen(outDmaUartBuffer));
		outpw(REG_GDMA_CTL0, 0x10041);
		while(!(inpw(REG_GDMA_INTCS) & 0x100));
		while(!(inpw(REG_UART1_ISR) & 0xff & UART_ISR_THRE_IF_Msk));
		outpw(REG_GDMA_INTCS,0x100);
	
}

int main (void)
{

	sysDisableCache();
	sysInvalidCache();
	sysSetMMUMappingMethod(MMU_DIRECT_MAPPING);
	sysEnableCache(CACHE_WRITE_BACK);
	uart1Initialization();
	gdmaInit();

	
	while(1)
	{
		gdmaInit();
	}
}

