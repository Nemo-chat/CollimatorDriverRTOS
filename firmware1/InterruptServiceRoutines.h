/*
 * InterruptServiceRoutines.h
 *
 *  Created on: Apr 23, 2026
 *      Author: Bc. Vadym Holysh
 *
 * ==================================================================
 * Code edited by Bc. Vadym Holysh, date: April 23, 2026.
 * 
 */

#ifndef INTERRUPTSERVICEROUTINES_H_
#define INTERRUPTSERVICEROUTINES_H_
#include <main.h>

void IPC_ISRvInit_CPU1(__interrupt void (*ipc2_isr_cpu1)(void));
void TrackGPIOsInit(void);

#endif /* INTERRUPTSERVICEROUTINES_H_ */
