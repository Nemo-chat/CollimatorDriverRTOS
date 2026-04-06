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

void IPC_ISRvInit_CPU2(interrupt void (*ipc_isr_cpu2)(void), interrupt void (*ipc3_isr_cpu1)(void))
{
    DINT;									// Disable global interrupts
    IpcRegs.IPCCLR.all = 0xFFFFFFFF;		// Clear IPC Flags
    EALLOW;
    PieVectTable.IPC1_INT = ipc_isr_cpu2;	// Register the IPC1 ISR in the PIE vector table
    PieVectTable.IPC3_INT = ipc3_isr_cpu1;	// Register the IPC3 ISR in the PIE vector table

	EDIS;
    PieCtrlRegs.PIEIER1.bit.INTx14 = 1;	    // IPC1 ISR is connected to PIE group 1, interrupt 14
    PieCtrlRegs.PIEIER1.bit.INTx16 = 1;	    // IPC3 ISR is connected to PIE group 1, interrupt 16
    IER |= M_INT1;							// Enable CPU interrupt 1 for PIE group 1
    EINT;									// Enable global interrupts
}