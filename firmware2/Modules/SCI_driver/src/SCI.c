/*
 * SCI.c
 *
 *  Created on: Feb 29, 2024
 *      Author: roland
 */
#include <SCI.h>

void SCI_Init(void)
{
    /* GPIO Setup */

    EALLOW;

    // Clock config
    CpuSysRegs.PCLKCR7.bit.SCI_A = 1;

    /* SCIA Setup */
    SciaRegs.SCICCR.bit.STOPBITS = 0;               /* 1 stop bit. */
    SciaRegs.SCICCR.bit.PARITYENA = 0;              /* No parity. */
    SciaRegs.SCICCR.bit.LOOPBKENA = 0;              /* No loop-back. */
    SciaRegs.SCICCR.bit.ADDRIDLE_MODE = 0;          /* Idle mode communication - RS232. */
    SciaRegs.SCICCR.bit.SCICHAR = 7;                /* 8 data bits. */

    SciaRegs.SCICTL1.bit.TXWAKE = 0;                /* No address/wake preamble. */
    SciaRegs.SCICTL1.bit.RXENA = 1;                 /* Enable RX. */
    SciaRegs.SCICTL1.bit.TXENA = 1;                 /* Enable TX. */

    /* Baud config for 57600 baud.
     * LSPCLK = 50MHz (LOSPCP configured on CPU1).
     * BRR = LSPCLK / (BAUD * 8) - 1 = 50000000 / (57600 * 8) - 1 = 107.5 -> 108.
     * NOTE: CPU2 cannot read LOSPCP correctly (reads 0), so BRR is hardcoded. */
    SciaRegs.SCIHBAUD.bit.BAUD = (U16)0;
    SciaRegs.SCILBAUD.bit.BAUD = (U16)108;

    SciaRegs.SCIFFTX.bit.SCIFFENA    = (U16)1;         /* Enable FIFO for communication interface. */
    SciaRegs.SCIFFTX.bit.TXFIFORESET = (U16)1;
    SciaRegs.SCIFFRX.bit.RXFIFORESET = (U16)1;

    SciaRegs.SCICTL1.bit.SWRESET = 1;               /* Reset SCI communication. */
    EDIS;
}

#warning "Function deprecated!"
void SCI_SendData(const U16 *data_pU16, U16 n_data_U16)
{
//    U16 data_counter_U16;
//    for(data_counter_U16 = 0; data_counter_U16 < n_data_U16; data_counter_U16++)
    while(n_data_U16--)
    {
        while(!SciaRegs.SCICTL2.bit.TXRDY);
        SciaRegs.SCITXBUF.bit.TXDT = *(data_pU16++);
    }
}

/**
 * @brief
 * @param enabled_b
 */
#pragma FUNC_ALWAYS_INLINE ( SCI_SetRxEnableState )
inline void SCI_SetRxEnableState(boolean enabled_b)
{
    if(enabled_b != False_b)
    {
        SciaRegs.SCICTL1.bit.RXENA = (U16)1;
    }
    else
    {
        SciaRegs.SCICTL1.bit.RXENA = (U16)0;
    }
}

/**
 * @brief
 * @param enabled_b
 */
#pragma FUNC_ALWAYS_INLINE ( SCI_SetTxEnableState )
inline void SCI_SetTxEnableState(boolean enabled_b)
{
    if(enabled_b != False_b)
    {
        SciaRegs.SCICTL1.bit.TXENA = (U16)1;
    }
    else
    {
        SciaRegs.SCICTL1.bit.TXENA = (U16)0;
    }
}
