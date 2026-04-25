/**
 * @file main.c
 * @brief Main application file for CPU2
 * @details This file contains the main function which initializes the system, 
 *          creates tasks for processing inputs, controlling the application, writing to shared memory, 
 *          and processing outputs. It also includes the necessary setup for 
 *          inter-processor communication (IPC) between CPU1 and CPU2.
 * 
 * =================================================================
 * @author Bc. Samuel Fertal
 * @author Bc. Vadym Holysh
 * @author Bc. Roland Molnar
 * 
 * =================================================================
 * KEM, FEI, TUKE
 * @date 27.02.2024
 * 
 * File reworked by 
 * @author Bc. Vadym Holysh, 
 * @date April 23, 2026.
 * 
 */
#include <main.h>
#include <ATB_interface.h>
#include <AC_interface.h>
#include <ECOM_interface.h>
#include <MDA_interface.h>
#include <SCI.h>
#include "InterruptServiceRoutines.h"
#include "FOC.h"
#include <PI_Controller.h>
#include "spi.h"
#include "dispCtrl.h"

/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "FreeRTOSConfig.h"

/* Task stack sizes */
#define COM_TASK_STACK_SIZE                            512
#define PRINT_TASK_STACK_SIZE                          256
#define WRITE_COMMAND_TO_SHARED_MEMORY_TASK_STACK_SIZE 256

/* Task priorities */
#define PRINT_TASK_PRIORITY                            1
#define COMMUNICATION_TASK_PRIORITY                    2
#define WRITE_COMMAND_TO_SHARED_MEMORY_TASK_PRIORITY   3

/* Stack size for idle task */
#define STACK_SIZE                                     256                       

/* Shared data structure for communication from CPU1 to CPU2 */
typedef struct
{
    MDA_Data_struct mda_data_s;
    MTCL_Control_struct mtcl_control_s;
    MTCL_MovementParams_struct mtcl_movement_params_s;
    F32 mtcl_maximum_position_rad_F32;
    boolean foc_enable_state_b;
    boolean active_service_mode_b;

}SharedDataCPU1TOCPU2_struct;

/* Shared data structure for communication from CPU2 to CPU1 */
typedef struct 
{
    MTCL_MovementParams_struct mtcl_movement_params_s;
    F32 reference_position_rad_F32;
    boolean foc_enable_state_b;
    boolean reset_error_flags_b;
    AC_CommandIndex_enum received_command_e;

}SharedDataCPU2TOCPU1_struct;

/* Task handles */
TaskHandle_t CommunicationTask, PrintTask, WriteCommandToSharedMemoryTask;

/* Flag that indicates whether shared data should be updated */
boolean should_update_shared_data_b = False_b;

/* Assigned shared data structures to specific memory sections */
volatile SharedDataCPU1TOCPU2_struct SharedDataCPU1TOCPU2;
#pragma DATA_SECTION(SharedDataCPU1TOCPU2, "MSGRAM_CPU1_TO_CPU2")

volatile SharedDataCPU2TOCPU1_struct SharedDataCPU2TOCPU1;
#pragma DATA_SECTION(SharedDataCPU2TOCPU1, "MSGRAM_CPU2_TO_CPU1")

/* Idle task memory */
static StaticTask_t idleTaskBuffer;
static StackType_t  idleTaskStack[STACK_SIZE];
#pragma DATA_SECTION(idleTaskStack,   ".freertosStaticStack")
#pragma DATA_ALIGN ( idleTaskStack , portBYTE_ALIGNMENT )

/* Heap memory for FreeRTOS */
#if(configAPPLICATION_ALLOCATED_HEAP == 1)
uint8_t ucHeap[ configTOTAL_HEAP_SIZE ];
#pragma DATA_SECTION(ucHeap,   ".freertosHeap")
#pragma DATA_ALIGN ( ucHeap , portBYTE_ALIGNMENT )
#endif

/* Function prototypes for tasks */
static void CommunicationTask_Func(void *pvParameters);
static void PrintTask_Func(void *pvParameters);
static void WriteCommandToSharedMemoryTask_Func(void *pvParameters);


void setDataToCPU1(AC_CommandIndex_enum);

/* Function to provide memory for the idle task. */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleTaskStackBuffer,
                                    uint32_t *pulIdleTaskStackSize )
{
    *ppxIdleTaskTCBBuffer = &idleTaskBuffer;
    *ppxIdleTaskStackBuffer = idleTaskStack;
    *pulIdleTaskStackSize = STACK_SIZE;
}

/* Function to handle stack overflow */
void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
{
    while(1);
}

/* Function prototypes for ISRs */
__interrupt void ipc3_isr_cpu1(void);

void main(void)
{
    /* Initialization */
    Interrupt_initModule();              /* Initialize the interrupt module */
    Interrupt_initVectorTable();         /* Initialize the interrupt vector table */
    IPC_ISRvInit_CPU2(&ipc3_isr_cpu1);   /* Initialize IPC ISRs for CPU2 */
    spi_vInit(800000);                   /* Initialize SPI module with baud rate of 800 kbps */
    dispCtrl_vInitDisplay();             /* Initialize the display controller */
    ATB_Init();                          /* Initialize the ATB module */
    SCI_Init();                          /* Initialize the SCI module */

    /* Create tasks and check if they were created successfully */
    if(xTaskCreate(CommunicationTask_Func, 
                   (const char *)"CommunicationTask", 
                   COM_TASK_STACK_SIZE, 
                   NULL,
                   (tskIDLE_PRIORITY + COMMUNICATION_TASK_PRIORITY), 
                   &CommunicationTask) != pdTRUE)
    {
        ESTOP0;
    }

    if(xTaskCreate(PrintTask_Func, 
                   (const char *)"PrintTask", 
                   PRINT_TASK_STACK_SIZE, 
                   NULL,
                   (tskIDLE_PRIORITY + PRINT_TASK_PRIORITY), 
                   &PrintTask) != pdTRUE)
    {
        ESTOP0;
    }

    if(xTaskCreate(WriteCommandToSharedMemoryTask_Func, 
                   (const char *)"WriteCommandToSharedMemoryTask", 
                   WRITE_COMMAND_TO_SHARED_MEMORY_TASK_STACK_SIZE, 
                   NULL,
                   (tskIDLE_PRIORITY + WRITE_COMMAND_TO_SHARED_MEMORY_TASK_PRIORITY), 
                   &WriteCommandToSharedMemoryTask) != pdTRUE)
    {
        ESTOP0;
    }

    /* IPC synchronization with CPU1 */
    IpcRegs.IPCSET.bit.IPC17 = 1;
    while(IpcRegs.IPCFLG.bit.IPC17 != 0);

    /* Start the scheduler.  This should not return. */
    vTaskStartScheduler();

    /* Main loop */
    for(;;)
    {

    }
}

// ===================================== TASKS FUNCTIONS ================================================

/**
 * @brief Function for Communication task.
 * @details This task handles the communication with PC via SCI module, 
 *          processes received commands, send updated data from CPU1,, 
 *          and manages the timing of these operations. 
 *          If SET command is received, it notifies the WriteCommandToSharedMemoryTask 
 *          to write the command and relevant data to shared memory for CPU2 to read and execute.
 *          It runs periodically every 1 ms to ensure timely processing of commands and data exchange with CPU1.         
 */
static void CommunicationTask_Func(void *pvParameters)
{
    /* Timing variables for periodic execution */
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    
    for(;;)
    {
        GpioDataRegs.GPCSET.bit.GPIO70 = 1;

        ECOM_MainHandler();         /* Handle serial communication with PC by packets*/

        /* Check if a new command was received and if it's a SET command while not in service mode, 
        then notify the task to write it to shared memory for CPU2 to read and execute */
        if ((AC_GetLastReceivedCommand() < AC_CMD_GET_MOVEMENT_PARAMS_e) && !AC_GetServiceModeActive()) 
        {
            xTaskNotifyGive(WriteCommandToSharedMemoryTask);   
        }
        
        /* Reset command marker for next cycle */
        AC_ClearLastReceivedCommand();

        GpioDataRegs.GPCCLEAR.bit.GPIO70 = 1;
        vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 1 ) );   /* Run this task every 1 ms */
    }
}

/**
 * @brief Function for the Application task.
 * @details This task handles the display refresh and runs periodically every 100 ms.
 */
static void PrintTask_Func(void *pvParameters)
{
    /* Timing variables for periodic execution */
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    for(;;)
    {
        GpioDataRegs.GPCSET.bit.GPIO72 = 1;
        DisplayRefresh();                                       /* Refresh the display with the latest data */

        GpioDataRegs.GPCCLEAR.bit.GPIO72 = 1;
        vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 100 ) ); /* Run this task every 100 ms */
    }
}

/**
 * @brief Function for the Write Command to Shared Memory task.
 * @details This task waits for a notification from the Communication Task about a new command to write to shared memory, 
 *          then writes the command and relevant data to shared memory for CPU1 to read and execute. 
 *          It also manages the IPC handshake with CPU1 to ensure proper synchronization of data exchange.
 */
static void WriteCommandToSharedMemoryTask_Func(void *pvParameters)
{
    for(;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);    /* Wait for a notification from the Communication Task about a new command to write to shared memory */
        GpioDataRegs.GPCSET.bit.GPIO74 = 1;
        setDataToCPU1(AC_GetLastReceivedCommand()); /* Write the command and relevant data to shared memory for CPU1 to read and execute */
        GpioDataRegs.GPCCLEAR.bit.GPIO74 = 1;
    }
}

/* ===================================== OTHER FUNCTIONS ================================================ */

/**
 * @brief Function to set data received from CPU2 to the relevant modules based on the command received.
 * @details This function is called in the IPC ISR when CPU2 sends a command.
 */
void setDataToCPU1(AC_CommandIndex_enum received_cmd)
{
    switch(received_cmd)
    {
    case AC_CMD_SET_MOVEMENT_PARAMS_e:
        MTCL_GetMovementParams(&SharedDataCPU2TOCPU1.mtcl_movement_params_s.MaxSpeed__rad_s__F32,
                                &SharedDataCPU2TOCPU1.mtcl_movement_params_s.MaxAccel__rad_s2__F32,
                                &SharedDataCPU2TOCPU1.mtcl_movement_params_s.MaxTorque__Nm__F32);
        SharedDataCPU2TOCPU1.received_command_e = received_cmd;
        should_update_shared_data_b = True_b;
        break;
    case AC_CMD_SET_REFERENCE_POSITION_e:
        SharedDataCPU2TOCPU1.reference_position_rad_F32 = MTCL_GetReferencePosition_F32();
        SharedDataCPU2TOCPU1.received_command_e = received_cmd;
        should_update_shared_data_b = True_b;
        break;
    case AC_CMD_SET_MOVEMENT_ENABLE_STATE_e:
        SharedDataCPU2TOCPU1.foc_enable_state_b = FOC_GetEnableState();
        SharedDataCPU2TOCPU1.received_command_e = received_cmd;
        should_update_shared_data_b = True_b;
        break;
    case AC_CMD_RESET_ERROR_FLAGS_e:
        SharedDataCPU2TOCPU1.reset_error_flags_b = AC_Reset_ErrorFlags_b;
        SharedDataCPU2TOCPU1.received_command_e = received_cmd;
        should_update_shared_data_b = True_b;
        break;

    default:
        SharedDataCPU2TOCPU1.received_command_e = AC_CMD_NONE_e;
        should_update_shared_data_b = False_b;
        break;
    }
    /* Perform IPC handshake with CPU1 to ensure proper synchronization of data exchange */
    if (should_update_shared_data_b)
    {
        IpcRegs.IPCSET.bit.IPC12 = 1;         /* Flag must be acknowledged by ISR on CPU1 */
        IpcRegs.IPCSET.bit.IPC2  = 1;         /* Generate interrupt on CPU1 */
        while(IpcRegs.IPCFLG.bit.IPC12 != 0); /* Wait until CPU1 enters ISR and ACKs marker */
        while(IpcRegs.IPCFLG.bit.IPC2  != 0); /* Wait for CPU2 to acknowledge the update */
        should_update_shared_data_b = False_b;
    }
}

/* ===================================== IPC ISR ================================================ */

/**
 * @brief IPC ISR that handles updated data from CPU1 via shared memory.
 * @details This ISR is triggered when CPU1 sends an interrupt to CPU2 indicating that new data is available in shared memory.
 *          It reads the updated data from shared memory and updates the relevant modules accordingly.
 */
__interrupt void ipc3_isr_cpu1(void)
{

    GpioDataRegs.GPCSET.bit.GPIO77 = 1;
    IpcRegs.IPCACK.bit.IPC13 = 1;                  /* Acknowledge IPC13 that interrupt has been received */

    /* Read the updated data from shared memory and update the relevant modules accordingly */
    MDA_SetData(&SharedDataCPU1TOCPU2.mda_data_s);         
    MTCL_SetMovementParams(SharedDataCPU1TOCPU2.mtcl_movement_params_s.MaxSpeed__rad_s__F32,
                            SharedDataCPU1TOCPU2.mtcl_movement_params_s.MaxAccel__rad_s2__F32,
                            SharedDataCPU1TOCPU2.mtcl_movement_params_s.MaxTorque__Nm__F32);
    MTCL_SetMaximumPosition_F32(&SharedDataCPU1TOCPU2.mtcl_maximum_position_rad_F32); 
    FOC_SetEnableState(SharedDataCPU1TOCPU2.foc_enable_state_b); 
    s_MTCL_Control_s.over_torque_error_f1 = SharedDataCPU1TOCPU2.mtcl_control_s.over_torque_error_f1; 
    AC_SetServiceModeActive(SharedDataCPU1TOCPU2.active_service_mode_b); 

    /* Update system timer on CPU2, necessary for synchronization. ISR occures every 10 application iteration. Not elegant, but it is what it is. */
    uint8_t i;
    for (i = 0; i < 10; i++) 
    {
        ATB_IncrementTime();
    }
    
    GpioDataRegs.GPCCLEAR.bit.GPIO77 = 1;

    IpcRegs.IPCACK.bit.IPC3 = 1;                /* Clear the IPC3 interrupt flag */
    PieCtrlRegs.PIEACK.bit.ACK1 = 1;			/* Must acknowledge the PIE group */

    portYIELD_FROM_ISR(pdTRUE);                 /* Yield to allow higher priority tasks to run immediately if needed */
}

/**
 * @brief Error handler
 */
#pragma RETAIN ( ErrorHandler )
void ErrorHandler(void)
{
    DINT;                                                       /* Disable interrupts. */
    for(;;)                                                     /* Endless loop. */
    {
        // Do nothing.
    }
}
