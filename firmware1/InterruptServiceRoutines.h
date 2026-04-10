/*
 * InterruptServiceRoutines.h
 *
 *  Created on: Mar 28, 2024
 *      Author: roland
 */

#ifndef INTERRUPTSERVICEROUTINES_H_
#define INTERRUPTSERVICEROUTINES_H_
#include <main.h>

// typedef struct
// {
//     uint16_t ProcessInputsTaskTrack = 25,
//     uint16_t ProcessOutputTaskTrack = 26,
//     uint16_t ApplicaitionTaskTrack = 27,
//     uint16_t WriteToSharedMemoryTaskTrack = 6,
//     uint16_t ISR_IPC_CPU2 = 75,

//     uint16_t PrintTaskTrack = 76,
//     uint16_t CommunicationTaskTrack = 77,
//     uint16_t WriteCommandToSharedMemoryTaskTrack = 78,
//     uint16_t ISR_IPC_CPU1 = 79

// } TrackGPIOPins_t;

void ISR_MotorControlHandler(void);

void IPC_ISRvInit_CPU1(interrupt void (*ipc1_isr_cpu1)(void), interrupt void (*ipc2_isr_cpu1)(void));
void TrackGPIOsInit(void);
void TrackGPIO_Set(U16 pin);
void TrackGPIO_Clear(U16 pin);

#endif /* INTERRUPTSERVICEROUTINES_H_ */
