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

void IPC_ISRvInit_CPU2(interrupt void (*ipc3_isr_cpu1)(void))
{
    DINT;									// Disable global interrupts
    IpcRegs.IPCCLR.all = 0xFFFFFFFF;		// Clear IPC Flags
    EALLOW;
    PieVectTable.IPC3_INT = ipc3_isr_cpu1;	// Register the IPC3 ISR in the PIE vector table

	EDIS;
    PieCtrlRegs.PIEIER1.bit.INTx16 = 1;	    // IPC3 ISR is connected to PIE group 1, interrupt 16
    IER |= M_INT1;							// Enable CPU interrupt 1 for PIE group 1
    EINT;									// Enable global interrupts
}


void TrackGPIOsInit(void)
{
    EALLOW;
    /* GPIO pin configuration for tracking signals. */

    GpioCtrlRegs.GPCDIR.bit.GPIO71   = (U16)1; /* Output */
    GpioDataRegs.GPCCLEAR.bit.GPIO71 = (U16)0;

    GpioCtrlRegs.GPCDIR.bit.GPIO72   = (U16)1; /* Output */
    GpioDataRegs.GPCCLEAR.bit.GPIO72 = (U16)0;  

    GpioCtrlRegs.GPCDIR.bit.GPIO73   = (U16)1; /* Output */
    GpioDataRegs.GPCCLEAR.bit.GPIO73 = (U16)0;  

    GpioCtrlRegs.GPCDIR.bit.GPIO74   = (U16)1; /* Output */
    GpioDataRegs.GPCCLEAR.bit.GPIO74 = (U16)0;  

    GpioCtrlRegs.GPCDIR.bit.GPIO75   = (U16)1; /* Output */
    GpioDataRegs.GPCCLEAR.bit.GPIO75 = (U16)0;  

    GpioCtrlRegs.GPCDIR.bit.GPIO76   = (U16)1; /* Output */
    GpioDataRegs.GPCCLEAR.bit.GPIO76 = (U16)0;     
    GpioCtrlRegs.GPCCSEL2.bit.GPIO76 = 3; // select master core CPU2

    GpioCtrlRegs.GPCDIR.bit.GPIO77   = (U16)1; /* Output */
    GpioDataRegs.GPCCLEAR.bit.GPIO77 = (U16)0;   
    GpioCtrlRegs.GPCCSEL2.bit.GPIO77 = 3; // select master core CPU2

    GpioCtrlRegs.GPCDIR.bit.GPIO78   = (U16)1; /* Output */
    GpioDataRegs.GPCCLEAR.bit.GPIO78 = (U16)0;   
    GpioCtrlRegs.GPCCSEL2.bit.GPIO78 = 3; // select master core CPU2

    GpioCtrlRegs.GPCDIR.bit.GPIO79   = (U16)1; /* Output */
    GpioDataRegs.GPCCLEAR.bit.GPIO79 = (U16)0;   
    GpioCtrlRegs.GPCCSEL2.bit.GPIO79 = 3; // select master core CPU2

    EDIS;
}
