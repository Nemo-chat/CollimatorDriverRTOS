/**
 * Code edited by Bc. Vadym Holysh, date: April 23, 2026.
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



