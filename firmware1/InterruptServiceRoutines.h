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

void IPC_ISRvInit_CPU1(interrupt void (*ipc0_isr_cpu1)(void), interrupt void (*ipc1_isr_cpu1)(void), interrupt void (*ipc2_isr_cpu1)(void));
void take_ipc_mutex_cpu1(void);
void give_ipc_mutex_cpu1(void);
//void ComutationU16();

#endif /* INTERRUPTSERVICEROUTINES_H_ */
