/**************************************************************************//**
* @file     uart.c
* @version  V1.00
* $Revision: 6 $
* $Date: 15/06/10 11:59a $
* @brief    NUC970 UART driver source file
*
* @note
* Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "nuc970.h"
#include "sys.h"
#include "uart.h"



static void _uartEnableInterrupt(INT nNum, UINT32 uVal)
{
    UINT32 uReg = 0;

    uReg = inpw(REG_UART0_IER+(nNum * UARTOFFSET));
    uReg |= uVal;
    outpw(REG_UART0_IER+(nNum * UARTOFFSET), uReg);
}

static void _uartDisableInterrupt(INT nNum, UINT32 uVal)
{
    UINT32 uReg = 0;

    if(uVal == DISABLEALLIER)
        outpw(REG_UART0_IER+(nNum * UARTOFFSET), 0);
    else {
        uReg = inpw(REG_UART0_IER+(nNum * UARTOFFSET));
        uReg &= ~uVal;
        outpw(REG_UART0_IER+(nNum * UARTOFFSET), uReg);
    }
}

static INT _uartSetBaudRate(INT nNum, UART_T *val)
{
    UINT32 u32Reg;
    UINT32 uOffset = nNum * UARTOFFSET;
    UINT32 u32Baud_Div;
    UINT32 u32Clk = val->uFreq;
    UINT32 u32baudrate = val->uBaudRate;

    //if (val->uFreq > 200000000)  /* Max frequency 200MHz */
    //  return -1;

    u32Baud_Div = UART_BAUD_MODE2_DIVIDER(u32Clk, u32baudrate);

    if(u32Baud_Div > 0xFFFF)
        u32Reg = (UART_BAUD_MODE0 | UART_BAUD_MODE0_DIVIDER(u32Clk, u32baudrate));
    else
        u32Reg = (UART_BAUD_MODE2 | u32Baud_Div);

    outpw(REG_UART0_BAUD + uOffset, u32Reg);

    return 0;
}

static INT _uartConfigureUART(PVOID pvParam)
{
    INT retval;
    BOOL bIsMemoryAllocOk;
    UINT32 u32Reg;
    UINT32 uOffset;
    UINT32 uNum=0;

    UART_T *param = (UART_T *) pvParam;

    uOffset = param->ucUartNo * UARTOFFSET;
    uNum = param->ucUartNo;

    /* Check UART channel */
    if ( uNum > UARTA )
        return UART_ERR_CHANNEL_INVALID;

    /* Check the supplied parity */
    if ( (param->ucParity != PARITY_NONE) &&
         (param->ucParity != PARITY_EVEN) &&
         (param->ucParity != PARITY_ODD)  &&
         (param->ucParity != (PARITY_ODD | PARITY_STICK)) &&
         (param->ucParity != (PARITY_EVEN | PARITY_STICK)) )
        return UART_ERR_PARITY_INVALID;

    /* Check the supplied number of data bits */
    if ( (param->ucDataBits != DATA_BITS_5) &&
         (param->ucDataBits != DATA_BITS_6) &&
         (param->ucDataBits != DATA_BITS_7) &&
         (param->ucDataBits != DATA_BITS_8) )
        return UART_ERR_DATA_BITS_INVALID;

    /* Check the supplied number of stop bits */
    if ( (param->ucStopBits != STOP_BITS_1) &&
         (param->ucStopBits != STOP_BITS_2) )
        return UART_ERR_STOP_BITS_INVALID;

    /* Check the supplied number of trigger level bytes */
    if ( (param -> ucUartNo == UART1) || (param -> ucUartNo == UART2) || (param -> ucUartNo == UART4) ||
         (param -> ucUartNo == UART6) || (param -> ucUartNo == UART8) || (param -> ucUartNo == UARTA)) {
        /* UART1,2,4,6,8,A */
        if ( (param->ucRxTriggerLevel != UART_FCR_RFITL_1BYTE)   &&
             (param->ucRxTriggerLevel != UART_FCR_RFITL_4BYTES)  &&
             (param->ucRxTriggerLevel != UART_FCR_RFITL_8BYTES)  &&
             (param->ucRxTriggerLevel != UART_FCR_RFITL_14BYTES) &&
             (param->ucRxTriggerLevel != UART_FCR_RFITL_30BYTES) &&
             (param->ucRxTriggerLevel != UART_FCR_RFITL_46BYTES) &&
             (param->ucRxTriggerLevel != UART_FCR_RFITL_62BYTES) )
            return UART_ERR_TRIGGERLEVEL_INVALID;
    } else {
        /* UART0,3,5,7,9 */
        if ( (param->ucRxTriggerLevel != UART_FCR_RFITL_1BYTE)  &&
             (param->ucRxTriggerLevel != UART_FCR_RFITL_4BYTES) &&
             (param->ucRxTriggerLevel != UART_FCR_RFITL_8BYTES) &&
             (param->ucRxTriggerLevel != UART_FCR_RFITL_30BYTES) )
            return UART_ERR_TRIGGERLEVEL_INVALID;
    }

    /* Enable UART clock */
    if(param->ucUartNo < ALLCHANNEL) {
        outpw(REG_CLK_PCLKEN0, inpw(REG_CLK_PCLKEN0) | (1 << (16+param->ucUartNo)) );
    }

    /* Reset TX/RX FIFOs */
    u32Reg = inpw(REG_UART0_FCR+uOffset);
    outpw(REG_UART0_FCR+uOffset, (u32Reg | (0x03 << 1)) );

    /* Setup baud rate */
    retval = _uartSetBaudRate(param->ucUartNo, param);
    if (retval < 0)
        return UART_ERR_SET_BAUDRATE_FAIL;

    /* Setup parity, data bits, and stop bits */
    outpw(REG_UART0_LCR+uOffset, (param->ucParity | param->ucDataBits | param->ucStopBits));

    /* Setup Rx time out value */
    outpw(REG_UART0_TOR+uOffset, 0x80+0x20);

    /* Setup FIFO trigger level */
    outpw(REG_UART0_FCR+uOffset, param->ucRxTriggerLevel);

    /* only exec once unless call uartClose() */
    return 0;
}


void uartSetRTSSignal(int uart, uint8_t lvl)
{
	 UINT32 uOffset = uart * UARTOFFSET;
	 if(uArg0 == UART_RTS_HIGH)      /* set RTS signal high */
		outpw(REG_UART0_MCR+uOffset, inpw(REG_UART0_MCR+uOffset) & ~0x02);
	else if(uArg0 == UART_RTS_LOW)  /* set RTS signal low  */
		outpw(REG_UART0_MCR+uOffset, inpw(REG_UART0_MCR+uOffset) | 0x02);

}

void uartEnableFlowControl(int uart)
{
	UINT32 uOffset = nNum * UARTOFFSET;
	/* Implement H/W flow control on TX & RX interrupt mode. */
	//dev->bIsUseUARTTxInt = TRUE;
	//dev->bIsUseUARTRxInt = TRUE;
	_uartEnableInterrupt(nNum, UART_IER_RDA_IEN_Msk);

	/*
		Set up RTS mechanism.
		In uartReceiveChars(), if uRecCnt >= _uart_nMaxRxBuf then set RTS high to stop RX.
		In uartReadRxBuf(), if uRecCnt <= _uart_nMinRxBuf then set RTS low to re-start RX.
	*/
	//_uart_nMaxRxBuf = (UARTRXBUFSIZE[nNum] * 3) / 4;
	//_uart_nMinRxBuf = UARTRXBUFSIZE[nNum] / 2;
	//FDEBUG("max[%d] min[%d]\n", _uart_nMaxRxBuf, _uart_nMinRxBuf);

	/* Set RTS high level trigger */
	outpw(REG_UART0_MCR+uOffset, (inpw(REG_UART0_MCR+uOffset) | UART_RTS_IS_HIGH_LEV_TRG) );
	/* Set RTS high level trigger */
	outpw(REG_UART0_MSR+uOffset, (inpw(REG_UART0_MSR+uOffset) | UART_CTS_IS_HIGH_LEV_TRG) );

	/* Set Auto CTS/RTS */
	outpw(REG_UART0_IER+uOffset, inpw(REG_UART0_IER+uOffset) | (0x3 << 12));

	/* Enable MODEM status interrupt */
	//_uartEnableInterrupt(nNum, UART_IER_MODEM_IEN_Msk);

	/*
		Maintain H/W flow control flag by read Modem Status Register.
		If CTS high, stop TX.
		If CTS low, start TX.
	*/
	//if( inpw(REG_UART0_MSR+uOffset) & 0x10 )  /* CTS external signal is low  */
	//  _uart_cHWTXStopped = 0;       /* TX started                  */
	//else                              /* CTS external signal is high */
	//  _uart_cHWTXStopped = 1;       /* TX stopped                  */

	/* Set RTS as logic 0, RX re-start */
	//outpb(REG_UART0_MCR+uOffset, inpb(REG_UART0_MCR+uOffset) | 0x02);  /* set RTS signal low  */
	//_uart_cHWRXStopped = 0;  // RX started
	
}

INT uartIoctl(INT nNum, UINT32 uCmd, UINT32 uArg0, UINT32 uArg1)
{
    INT32 retval;
    UINT32 uReg;
    UINT32 uOffset = nNum * UARTOFFSET;

    UART_BUFFER_T *dev;

    if((nNum < UART0) || (nNum > UARTA))
        return UART_ENODEV;

    dev = (UART_BUFFER_T *) &UART_DEV[nNum];

    /* Check UART initial status */
    if(dev->bIsUARTInitial == FALSE) {
        if((uCmd != UART_IOC_GETERRNO) &&
           (uCmd != UART_IOC_GETUARTREGISTERVALUE))
            return UART_EIO;
    }

    switch(uCmd) {
        case UART_IOC_DISABLEHWFLOWCONTROL:

            /* Disable MODEM status interrupt */
            _uartDisableInterrupt(nNum, UART_IER_MODEM_IEN_Msk);
            _uart_cFlowControlMode = 0;
            _uart_cHWTXStopped = 0;
            _uart_cHWRXStopped = 0;
            break;

     

        case UART_IOC_SET_RS485_MODE:
            outpw((REG_UART0_FUN_SEL+uOffset), 0x3);
            outpw((REG_UART0_MCR+uOffset), 0x0);
            outpw((REG_UART0_LCR+uOffset), (UART_LCR_SPE_Msk | UART_LCR_EPE_Msk | UART_LCR_PBE_Msk | (0x3 << UART_LCR_WLS_Pos)));
            outpw((REG_UART0_ALT_CSR+uOffset), uArg0|(uArg1 << UART_ALT_CSR_ADDR_MATCH_Pos));
            break;

        case UART_IOC_SEND_RS485_ADDRESS:

            while(!((inpw(REG_UART0_FSR + uOffset)) & UART_FSR_TE_FLAG_Msk));
            uReg = inpw(REG_UART0_LCR + uOffset);
            outpw((REG_UART0_LCR+uOffset), (UART_LCR_SPE_Msk | UART_LCR_PBE_Msk | (0x3 << UART_LCR_WLS_Pos)));
            outpw((REG_UART0_THR+uOffset), uArg0);
            while(!((inpw(REG_UART0_FSR + uOffset)) & UART_FSR_TE_FLAG_Msk));

            outpw((REG_UART0_LCR+uOffset), uReg);

            break;

        case UART_IOC_SET_RS485_RXOFF:
            uReg = inpw(REG_UART0_FCR + uOffset);
            if(uArg0 == 1)
                uReg |= UART_FCR_RX_DIS_Msk;
            else
                uReg &= ~UART_FCR_RX_DIS_Msk;

            outpw((REG_UART0_FCR + uOffset), uReg);

            break;

        case UART_IOC_SET_ALTCTL_REG:

            outpw((REG_UART0_ALT_CSR + uOffset), uArg0);

            break;

        case UART_IOC_GET_ALTCTL_REG:

            *(PUINT32)uArg0 = inpw(REG_UART0_ALT_CSR + uOffset);

            break;

        case UART_IOC_SET_LIN_MODE:

            outpw((REG_UART0_FUN_SEL+uOffset), 0x1); // Select LIN function

            /* Select LIN function setting : Tx enable, Rx enable and break field length */
            uReg = inpw(REG_UART0_ALT_CSR + uOffset);
            uReg &= ~(UART_ALT_CSR_LIN_TX_EN_Msk | UART_ALT_CSR_LIN_RX_EN_Msk | UART_ALT_CSR_UA_LIN_BKFL_Msk);
            uReg |= (uArg0 | (uArg1 << UART_ALT_CSR_UA_LIN_BKFL_Pos));
            outpw((REG_UART0_ALT_CSR + uOffset), uReg);

            break;

        default:
            return UART_ENOTTY;
    }

    return Successful;
}

