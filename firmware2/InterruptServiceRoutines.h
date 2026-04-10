/*
 * InterruptServiceRoutines.h
 *
 *  Created on: Mar 28, 2024
 *      Author: roland
 */

#ifndef INTERRUPTSERVICEROUTINES_H_
#define INTERRUPTSERVICEROUTINES_H_
#include <main.h>

typedef enum
{
    ProcessInputsTaskTrack = 71,
    ProcessOutputTaskTrack = 72,
    ApplicaitionTaskTrack = 73,
    WriteToSharedMemoryTaskTrack = 74,
    ISR_IPC_CPU2 = 75,

    PrintTaskTrack = 76,
    CommunicationTaskTrack = 77,
    WriteCommandToSharedMemoryTaskTrack = 78,
    ISR_IPC_CPU1 = 79
    
} TrackGPIOPin_enum;

void ISR_MotorControlHandler(void);

void IPC_ISRvInit_CPU2(interrupt void (*ipc_isr_cpu2)(void), interrupt void (*ipc3_isr_cpu1)(void));

void TrackGPIOsInit(void);
void TrackGPIO_Set(U16 pin);
void TrackGPIO_Clear(U16 pin);
//void ComutationU16();

#endif /* INTERRUPTSERVICEROUTINES_H_ */
