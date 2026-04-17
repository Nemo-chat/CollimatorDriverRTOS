/**
 * @file spi.c
 * @brief Serial Peripheral Interface, communication standard for LCD display
 * @details Initialization function, function for sending data
 *
 * =================================================================
 * @author Bc. Vadym Holysh
 *
 * =================================================================
 * KEM, FEI, TUKE
 * @date 06.03.2024
 * @defgroup SPI Serial Peripheral Interface
 * @{
 */

#include "spi.h"

void spi_PinsInit(void)
{
    EALLOW;
    // GPIO16 → SPISIMOA
    GpioCtrlRegs.GPAGMUX2.bit.GPIO16 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO16  = 1;    
    // GPIO17 → SPISOMIA
    GpioCtrlRegs.GPAGMUX2.bit.GPIO17 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO17  = 1;
    // GPIO18 → SPICLKA
    GpioCtrlRegs.GPAGMUX2.bit.GPIO18 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO18  = 1;
    // GPIO19 → SPISTEA
    GpioCtrlRegs.GPAGMUX2.bit.GPIO19 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO19  = 1;
    
    GpioCtrlRegs.GPAGMUX2.bit.GPIO24 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO24  = 0;
    
    GpioCtrlRegs.GPADIR.bit.GPIO24 = 1;
    // MOSI + CLK + CS = outputs
    GpioCtrlRegs.GPADIR.bit.GPIO16 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO18 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO19 = 1;
    // MISO = input
    GpioCtrlRegs.GPADIR.bit.GPIO17 = 0;

    GpioCtrlRegs.GPACSEL3.bit.GPIO16 = 2; // select master core CPU2
    GpioCtrlRegs.GPACSEL3.bit.GPIO17 = 2; // select master core CPU2
    GpioCtrlRegs.GPACSEL3.bit.GPIO18 = 2; // select master core CPU2
    GpioCtrlRegs.GPACSEL3.bit.GPIO19 = 2; // select master core CPU2
    GpioCtrlRegs.GPACSEL4.bit.GPIO24 = 2; // select master core CPU2

    DevCfgRegs.CPUSEL6.bit.SPI_A = 1;   // 0 = CPU1 owner, 1 = CPU2 owner
    
    EDIS;
}

/**
 * @brief Initialization SPI module with FIFO.
 * @details 600 kHz < Baudrate < 920 kHz
 * @param Reference Baudrate.
 */

void spi_vInit(float u16BaudRate){

    EALLOW;

    ClkCfgRegs.LOSPCP.bit.LSPCLKDIV = 1; /* set LSPCLK divider on =/2 - 50*/
    CpuSysRegs.PCLKCR8.bit.SPI_A = 1;
    EDIS;

    SpiaRegs.SPICCR.bit.SPISWRESET = 0;

    #if CLOCK_POLARITY == 1
        SpiaRegs.SPICCR.bit.CLKPOLARITY = 1;  /*sampling falling edge*/
    #else
        SpiaRegs.SPICCR.bit.CLKPOLARITY = 0;  /*sampling raising edge*/
    #endif

    #if CLOCK_PHASE == 1
        SpiaRegs.SPICTL.bit.CLK_PHASE = 1;   /*with delay*/
    #else
        SpiaRegs.SPICTL.bit.CLK_PHASE = 0;   /*without delay*/
    #endif


    #if MASTER_SLAVE_MODE == 1
        SpiaRegs.SPICTL.bit.MASTER_SLAVE = 1;      /*master-mode*/
    #else
        SpiaRegs.SPICTL.bit.MASTER_SLAVE = 0;      /*slave-mode*/
    #endif

    SpiaRegs.SPICCR.bit.SPICHAR = 7;           /*8 bit*/
    SpiaRegs.SPICCR.bit.SPILBK = 1 ;           /*loop-back mode*/
    SpiaRegs.SPICTL.bit.TALK = 1;              /*enable transmit data*/
    SpiaRegs.SPIBRR.bit.SPI_BIT_RATE =
    (Uint16)(((float)MCU_FREQ/2.0) / (u16BaudRate)) - 1;     /*bit rate*/
    SpiaRegs.SPIPRI.bit.FREE = 1;              /*free mode debug*/
    SpiaRegs.SPISTS.bit.INT_FLAG = 1;
    SpiaRegs.SPISTS.bit.OVERRUN_FLAG = 1;

    /* Initialization FIFO*/

    SpiaRegs.SPIFFTX.bit.SPIFFENA = 1;          // SPI FIFO enhancements are enabled
    SpiaRegs.SPIFFTX.bit.TXFFINTCLR = 1;        // clear SPIFFTX
    SpiaRegs.SPIFFRX.bit.RXFFOVFCLR = 1;        // Receive FIFO overflow clear
    SpiaRegs.SPIFFRX.bit.RXFFINTCLR = 1;        // Receive FIFO Interrupt Clear
    SpiaRegs.SPIFFTX.bit.TXFIFO = 1;            // release transmit FIFO from reset
    SpiaRegs.SPIFFRX.bit.RXFIFORESET = 1;       // Re-enable receive FIFO operation
    SpiaRegs.SPIPRI.bit.FREE = 1;               // Emulation Free Run regardless of suspend
    SpiaRegs.SPIFFTX.bit.SPIRST = 1;            // SPI FIFO can resume transmit or receive


    SpiaRegs.SPICCR.bit.SPISWRESET = 1;
    }

/**
 * @brief Send data - one char using FIFO.
 * @param Data.
 */
void spi_vSendChar(char cData){

//    while (SpibRegs.SPISTS.bit.BUFFULL_FLAG == 1); //Overenie ci je buffer rdy
//           SpibRegs.SPITXBUF = cData;

    //while(SpibRegs.SPISTS.bit.INT_FLAG != 0);
    //do{}while(SpibRegs.SPISTS.bit.BUFFULL_FLAG == 1);
    while(SpiaRegs.SPIFFTX.bit.TXFFST != 0);
    SpiaRegs.SPITXBUF = (cData & 0xFF) << 8;

}

/**
 * @brief Send data - one string using FIFO.
 * @param Address to char array.
 * @param Length array
 */
void spi_u16SendData(char *pcData, Uint16 u16Length){

//    int counter;
//    for (counter = 0; counter < u16Length; counter++)
//    {
//        spi_vSendChar(pcData[counter]);
//    }

    while(u16Length--){
        //while(SpiaRegs.SPISTS.bit.BUFFULL_FLAG == 1);
        while(SpiaRegs.SPIFFTX.bit.TXFFST != 0);
        SpiaRegs.SPITXBUF = (*(pcData++))<<(8);
    }
}

/**
 * @brief Send data - one string using FIFO.
 * @param Address to char array.
 * @return Length char array.
 */
Uint16 spi_u16SendString(char *pcData){

    Uint16 i;

    for(i = 0; pcData[i] != '\0'; i++){
        spi_vSendChar(pcData[i]);
    }

    return (i);
}

/**
 * @}
 */



