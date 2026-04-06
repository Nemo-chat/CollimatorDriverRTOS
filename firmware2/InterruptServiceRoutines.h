/*
 * InterruptServiceRoutines.h
 *
 *  Created on: Mar 28, 2024
 *      Author: roland
 */

#ifndef INTERRUPTSERVICEROUTINES_H_
#define INTERRUPTSERVICEROUTINES_H_
#include <main.h>

void ISR_MotorControlHandler(void);

void IPC_ISRvInit_CPU2(interrupt void (*ipc_isr_cpu2)(void), interrupt void (*ipc3_isr_cpu1)(void));

//void ComutationU16();

#endif /* INTERRUPTSERVICEROUTINES_H_ */
