/*
 * InterruptServiceRoutines.c
 *
 *  Created on: Mar 28, 2024
 *      Author: roland
 */
#include <main.h>
#include <ATB_interface.h>
#include <MTCL_interface.h>
#include <TEST.h>
#include "InterruptServiceRoutines.h"

void IPC_ISRvInit_CPU1(__interrupt void (*ipc2_isr_cpu1)(void))
{
    DINT;									// Disable global interrupts
    IpcRegs.IPCCLR.all = 0xFFFFFFFF;		// Clear IPC Flags
    EALLOW;
    PieVectTable.IPC2_INT = ipc2_isr_cpu1;		// Register the IPC2 ISR in the PIE vector table
	EDIS;
    PieCtrlRegs.PIEIER1.bit.INTx15 = 1;	    // IPC2 ISR is connected to PIE group 1, interrupt 15   
    IER |= M_INT1;							// Enable CPU interrupt 1 for PIE group 1
    EINT;									// Enable global interrupts
}

void TrackGPIOsInit(void)
{
    EALLOW;
    /* GPIO pin configuration for tracking signals. */
    /* CPU1 */
    GpioCtrlRegs.GPCDIR.bit.GPIO79   = (U16)1;
    GpioDataRegs.GPCCLEAR.bit.GPIO79 = (U16)0;

    GpioCtrlRegs.GPCDIR.bit.GPIO90   = (U16)1;
    GpioDataRegs.GPCCLEAR.bit.GPIO90 = (U16)1;

    GpioCtrlRegs.GPCDIR.bit.GPIO92   = (U16)1;
    GpioDataRegs.GPCCLEAR.bit.GPIO92 = (U16)1;

    GpioCtrlRegs.GPCDIR.bit.GPIO94   = (U16)1;
    GpioDataRegs.GPCCLEAR.bit.GPIO94 = (U16)1;

    /* CPU2 */
    GpioCtrlRegs.GPCDIR.bit.GPIO70   = (U16)1; /* Output */
    GpioDataRegs.GPCCLEAR.bit.GPIO70 = (U16)1;
    GpioCtrlRegs.GPCCSEL1.bit.GPIO70 = (U16)2;  

    GpioCtrlRegs.GPCDIR.bit.GPIO72   = (U16)1; /* Output */
    GpioDataRegs.GPCCLEAR.bit.GPIO72 = (U16)1;     
    GpioCtrlRegs.GPCCSEL2.bit.GPIO72 = (U16)2;
    
    GpioCtrlRegs.GPCDIR.bit.GPIO74   = (U16)1; /* Output */
    GpioDataRegs.GPCCLEAR.bit.GPIO74 = (U16)1;   
    GpioCtrlRegs.GPCCSEL2.bit.GPIO74 = (U16)2;

    GpioCtrlRegs.GPCDIR.bit.GPIO77   = (U16)1; /* Output */
    GpioDataRegs.GPCCLEAR.bit.GPIO77 = (U16)1;
    GpioCtrlRegs.GPCCSEL2.bit.GPIO77 = (U16)2;   

    EDIS;
}
