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

void IPC_ISRvInit_CPU1(interrupt void (*ipc1_isr_cpu1)(void), interrupt void (*ipc2_isr_cpu1)(void))
{
    DINT;									// Disable global interrupts
    IpcRegs.IPCCLR.all = 0xFFFFFFFF;		// Clear IPC Flags
    EALLOW;
    PieVectTable.IPC1_INT = ipc1_isr_cpu1;		// Register the IPC1 ISR in the PIE vector table
    PieVectTable.IPC2_INT = ipc2_isr_cpu1;		// Register the IPC2 ISR in the PIE vector table
	EDIS;
    PieCtrlRegs.PIEIER1.bit.INTx14 = 1;	    // IPC1 ISR is connected to PIE group 1, interrupt 14
    PieCtrlRegs.PIEIER1.bit.INTx15 = 1;	    // IPC2 ISR is connected to PIE group 1, interrupt 15   
    IER |= M_INT1;							// Enable CPU interrupt 1 for PIE group 1
    EINT;									// Enable global interrupts
}

void TrackGPIOsInit(void)
{
    EALLOW;
    /* GPIO pin configuration for tracking signals. */

    GpioCtrlRegs.GPADIR.bit.GPIO25   = (U16)1; /* Output */
    GpioDataRegs.GPACLEAR.bit.GPIO25 = (U16)0;

    GpioCtrlRegs.GPADIR.bit.GPIO26   = (U16)1; /* Output */
    GpioDataRegs.GPACLEAR.bit.GPIO26 = (U16)0;  

    GpioCtrlRegs.GPADIR.bit.GPIO27   = (U16)1; /* Output */
    GpioDataRegs.GPACLEAR.bit.GPIO27 = (U16)0;  

    GpioCtrlRegs.GPADIR.bit.GPIO6   = (U16)1; /* Output */
    GpioDataRegs.GPACLEAR.bit.GPIO6 = (U16)0;  

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

void TrackGPIO_Set(U16 pin)
{
    GpioDataRegs.GPASET.all = (1U << pin);
}

void TrackGPIO_Clear(U16 pin)
{
    GpioDataRegs.GPACLEAR.all = (1U << pin);
    
}