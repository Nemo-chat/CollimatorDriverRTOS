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

void IPC_ISRvInit_CPU1(interrupt void (*ipc0_isr_cpu1)(void), interrupt void (*ipc1_isr_cpu1)(void), interrupt void (*ipc2_isr_cpu1)(void))
{
    DINT;									// Disable global interrupts
    IpcRegs.IPCCLR.all = 0xFFFFFFFF;		// Clear IPC Flags
    EALLOW;
	PieVectTable.IPC0_INT = ipc0_isr_cpu1;		// Register the IPC0 ISR in the PIE vector table
    PieVectTable.IPC1_INT = ipc1_isr_cpu1;		// Register the IPC1 ISR in the PIE vector table
    PieVectTable.IPC2_INT = ipc2_isr_cpu1;		// Register the IPC2 ISR in the PIE vector table

	EDIS;
    PieCtrlRegs.PIEIER1.bit.INTx13 = 1;	    // IPC0 ISR is connected to PIE group 1, interrupt 13
    PieCtrlRegs.PIEIER1.bit.INTx14 = 1;	    // IPC1 ISR is connected to PIE group 1, interrupt 14
    PieCtrlRegs.PIEIER1.bit.INTx15 = 1;	    // IPC2 ISR is connected to PIE group 1, interrupt 15
    
    IER |= M_INT1;							// Enable CPU interrupt 1 for PIE group 1
    EINT;									// Enable global interrupts
}

void take_ipc_mutex_cpu1(void)
{
    IpcRegs.IPCSET.bit.IPC0 = 1;         // Genrate interrupt to take the mutex on the cpu2 side by setting IPC0
    IpcRegs.IPCSET.bit.IPC7 = 1;         // Set flag to indicate that that interrupt occured on cpu2 side

    while(IpcRegs.IPCSTS.bit.IPC0 == 1); 
    while(IpcRegs.IPCSTS.bit.IPC7 == 1); // Wait until CPU2 processes the interrupt and clears the IPC7 flag
            
}

void give_ipc_mutex_cpu1(void)
{
    IpcRegs.IPCSET.bit.IPC1 = 1;         // Genrate interrupt to give the mutex on the cpu2 side by setting IPC1
    IpcRegs.IPCSET.bit.IPC8 = 1;         // Set flag to indicate that that interrupt occured on cpu2 side

    while(IpcRegs.IPCSTS.bit.IPC1 == 1); 
    while(IpcRegs.IPCSTS.bit.IPC8 == 1); // Wait until CPU2 processes the interrupt and clears the IPC8 flag
}