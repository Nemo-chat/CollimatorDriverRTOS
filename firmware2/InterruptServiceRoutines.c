/**
 * @file InterruptServiceRoutines.c
 * @brief IPC interrupt initialization and tracking GPIO setup for CPU2.
 *
 * @date April 23, 2026
 * @author Bc. Vadym Holysh
 * @note Code edited by Bc. Vadym Holysh, date: April 23, 2026.
 */
#include <main.h>
#include "InterruptServiceRoutines.h"

/**
 * @brief Initializes the IPC interrupt for CPU2 and registers the provided ISR.
 * @param ipc3_isr_cpu1 Pointer to the ISR function to be registered for IPC3 interrupt on CPU2.
 */
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

