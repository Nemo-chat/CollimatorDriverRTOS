/*
 * SCI.c
 *
 *  Created on: Feb 29, 2024
 *      Author: roland
 */
#include <SCI.h>

void SCI_PinsInit(void)
{
    EALLOW;
    /* SCIRXDA */
    GpioCtrlRegs.GPAGMUX2.bit.GPIO28 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1;
    GpioCtrlRegs.GPACSEL4.bit.GPIO28 = 2; // select master core CPU2

    /* SCITXDA */
    GpioCtrlRegs.GPAGMUX2.bit.GPIO29 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO28  = 1;   // TX = output
    GpioCtrlRegs.GPACSEL4.bit.GPIO29 = 2; // select master core CPU2

    DevCfgRegs.CPUSEL5.bit.SCI_A = 1;     // 0 = CPU1 owner, 1 = CPU2 owner

    EDIS;
}