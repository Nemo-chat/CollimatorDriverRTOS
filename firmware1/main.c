/**
 * @file main.c
 * @brief Main application file for CPU1
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
#include <MDA_interface.h>
#include <PWM_interface.h>
#include <SCI.h>
#include "InterruptServiceRoutines.h"
#include "FOC.h"
#include "TEST.h"
#include <PI_Controller.h>
#include "spi.h"

/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "FreeRTOSConfig.h"

/* Task stack sizes */
#define PROCESS_INPUT_TASK_STACK_SIZE          256
#define APPLICATION_TASK_STACK_SIZE            256
#define WRITE_TO_SHARED_MEMORY_TASK_STACK_SIZE 256

/* Task priorities */
#define PROCESS_INPUT_TASK_PRIORITY            1
#define APPLICATION_TASK_PRIORITY              2
#define PROCESS_OUTPUT_TASK_PRIORITY           3
#define WRITE_TO_SHARED_MEMORY_TASK_PRIORITY   4

/* Number of ADC interrupts define in which period Write Data to Shared Memory Task is executed */
#define NUMBER_OF_ADC_INTERRUPTS               10

/* Stack size for idle task */
#define STACK_SIZE                             256

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
TaskHandle_t ProcessInputTask, ApplicationTask, WriteToSharedMemoryTask, ProcessOutputTask;
/* The Semaphore for synchronization ISR by ADC and Process Inputs Task. */
SemaphoreHandle_t SyncSemaphore;

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
static void ProcessInputsTask_Func(void *pvParameters);
static void ApplicationTask_Func(void *pvParameters);
static void WriteToSharedMemoryTask_Func(void *pvParameters);
static void ProcessOutputTask_Func(void *pvParameters);

void getDataFromCPU2(void);

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
__interrupt void MDA_AdcConversionCompleteIsr(void);
__interrupt void ipc2_isr_cpu2(void);

void main(void)
{
    /* Create a semaphore. Must be done before ISR starts */
    SyncSemaphore = xSemaphoreCreateBinary();
 
    /* Initialization */
    mcu_vInitClocks();                              /* Initialize uC clock system. */
    EALLOW;
    ClkCfgRegs.LOSPCP.bit.LSPCLKDIV = 2;            /* Set LSPCLK for both CPUs on 50 MHz. */
    EDIS;
    Interrupt_initModule();                         /* Initialize the Interrupt module. */
    Interrupt_initVectorTable();                    /* Initialize the Interrupt vector table. */
    IPC_ISRvInit_CPU1(&ipc2_isr_cpu2);              /* Initialize IPC ISRs for CPU2 */
    spi_PinsInit();                                 /* Initialize SPI pins, select master core CPU2 */
    SCI_PinsInit();                                 /* Initialize SCI pins, select master core CPU2 */
    ATB_Init();                                     /* Initialize Application Time Base, system timer */
    PWM_Init();                                     /* Initialize PWM module */
    FOC_CommutationAlignment();                     /* Perform FOC commutation alignment */
    MDA_Init(&MDA_AdcConversionCompleteIsr);        /* Initialize Motor Data Acquisition module with ADC ISR */
    MTCL_Init();                                    /* Initialize Motor Control module */
    MDA_CalibratePhaseCurrentsOffsets();            /* Calibrate phase currents offsets */
    AC_ManualControlInit();                         /* Initialize manual control - buttons */
    TrackGPIOsInit();                               /* Initialize GPIO tracking for both CPUs */

    /* Configure the shared memory GS11 and GS15 to be accessible by CPU2 for writing and CPU1 only for reading */
    EALLOW;
    MemCfgRegs.GSxMSEL.bit.MSEL_GS11 = 1;
    MemCfgRegs.GSxMSEL.bit.MSEL_GS12 = 1;
    MemCfgRegs.GSxMSEL.bit.MSEL_GS13 = 1;
    MemCfgRegs.GSxMSEL.bit.MSEL_GS14 = 1;
	MemCfgRegs.GSxMSEL.bit.MSEL_GS15 = 1;
	EDIS;

    /*Redundant reset of PI controller structures*/
    PI_ctrl_Init(&PI_id_current_controller);
    PI_ctrl_Init(&PI_iq_current_controller);
    PI_ctrl_Init(&PI_speed_controller);
    PI_ctrl_Init(&PI_position_controller);

    FOC_SetEnableState(True_b);                     /* Enable FOC */
    
    /* Create tasks and check if they were created successfully */
    if(xTaskCreate(ProcessInputsTask_Func, 
                   (const char *)"ProcessInputTask", 
                   PROCESS_INPUT_TASK_STACK_SIZE, 
                   NULL,
                   (tskIDLE_PRIORITY + PROCESS_INPUT_TASK_PRIORITY), 
                   &ProcessInputTask) != pdTRUE)
    {
        ESTOP0;
    }

    if(xTaskCreate(ApplicationTask_Func, 
                   (const char *)"ApplicationTask", 
                   APPLICATION_TASK_STACK_SIZE, 
                   NULL,
                   (tskIDLE_PRIORITY + APPLICATION_TASK_PRIORITY), 
                   &ApplicationTask) != pdTRUE)
    {
        ESTOP0;
    }

    if(xTaskCreate(WriteToSharedMemoryTask_Func, 
                   (const char *)"WriteDataToSharedMemoryTask", 
                   WRITE_TO_SHARED_MEMORY_TASK_STACK_SIZE, 
                   NULL,
                   (tskIDLE_PRIORITY + WRITE_TO_SHARED_MEMORY_TASK_PRIORITY), 
                   &WriteToSharedMemoryTask) != pdTRUE)
    {
        ESTOP0;
    }

    if(xTaskCreate(ProcessOutputTask_Func,
                   (const char *)"ProcessOutputTask",
                   WRITE_TO_SHARED_MEMORY_TASK_STACK_SIZE,
                   NULL,
                   (tskIDLE_PRIORITY + PROCESS_OUTPUT_TASK_PRIORITY),
                   &ProcessOutputTask) != pdTRUE)
    {
        ESTOP0;
    }

    /* IPC synchronization with CPU2 */
    while(IpcRegs.IPCSTS.bit.IPC17 == 0);
    IpcRegs.IPCACK.bit.IPC17 = 1;

    /*Start the scheduler.*/ 
    vTaskStartScheduler();

    /* Main loop */
    for(;;)
    {
    }
}

// ===================================== TASKS FUNCTIONS ================================================

/**
 * @brief Function for the Process Inputs task.
 * @details This task waits for a semaphore given by the ADC ISR, 
 *          then processes the inputs from the ADC, QEP, and buttons, updates the relevant data structures, 
 *          and notifies the Application Task to run.
 */
static void ProcessInputsTask_Func(void *pvParameters)
{
    for(;;)
    {
        xSemaphoreTake(SyncSemaphore, portMAX_DELAY);   /* Wait for the semaphore to be given by the ADC ISR */
        GpioDataRegs.GPCSET.bit.GPIO79 = 1;
        
        ATB_IncrementTime();                            /* Increment the application time counter */      
        MDA_UpdateData();                               /* Update measurement data */
        AC_ManualControlHandler();                      /* Handle manual control inputs */

        xTaskNotifyGive(ApplicationTask);               /* Notify the Application Task to run after processing inputs */
        GpioDataRegs.GPCCLEAR.bit.GPIO79 = 1;
        taskYIELD();                                    /* Allow other tasks to run immediately */
    }
}

/**
 * @brief Function for the Application task.
 * @details This task waits for a notification from the Process Inputs Task, 
 *          then performs the necessary control actions based on the processed inputs, 
 *          updates the control state, and notifies the Process Output Task to apply the new control outputs.
 */
static void ApplicationTask_Func(void *pvParameters)
{
    uint32_t ulEventToProcess;  /* Variable to store the notification value indicating what event to process */
    
    for(;;)
    {
        ulEventToProcess = ulTaskNotifyTake( pdTRUE, portMAX_DELAY );  /* Wait for a notification from the Process Inputs Task */
        if (ulEventToProcess != 0)
        {   
            GpioDataRegs.GPCSET.bit.GPIO90 = 1;

            /* Set reference position from button control if any related button is pressed */
            if (AC_GetBTNData_ps()->any_button_pressed_b)
            {
                MTCL_SetReferencePosition(AC_GetBTNData_ps()->BTN_ReferencePosition__rad__F32);
            }
            
            /* Don't allow ISR from CPU2 during main control handler execution */
            IpcRegs.IPCSET.bit.IPC10 = 1;
            MTCL_MainHandler();                    /* Perform main control handler tasks */
            IpcRegs.IPCCLR.bit.IPC10 = 1;
            
            xTaskNotifyGive(ProcessOutputTask);    /* Notify the Process Output Task to run */
            
            GpioDataRegs.GPCCLEAR.bit.GPIO90 = 1;
            taskYIELD();                           /* Allow other tasks to run immediately*/
        }
    }
}

/**
 * @brief Function for the Process Output task.
 * @details This task waits for a notification from the Application Task, 
 *          then applies the new control outputs to the hardware (updates PWM compare values), 
 *          and periodically notifies the Write to Shared Memory Task to update the shared data for CPU2.
 */
static void ProcessOutputTask_Func(void *pvParameters)
{
    uint32_t ulEventToProcess; /* Variable to store the notification value indicating what event to process */
    for(;;)
    {
        ulEventToProcess = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  /* Wait for a notification from the Application Task */
        if (ulEventToProcess != 0)
        {   
            GpioDataRegs.GPCSET.bit.GPIO92 = 1;
            /* Apply compare values from global buffer */
            PWM_SetCompareValues(g_PWM_CompareValues.cmp_u,
                                 g_PWM_CompareValues.cmp_v,
                                 g_PWM_CompareValues.cmp_w);
            
            /* Increment the application counter and notify the Write to Shared Memory Task if it's time */
            static uint8_t IterationCounter = 0;
            IterationCounter++;
            if (IterationCounter >= NUMBER_OF_ADC_INTERRUPTS)
            {
                IterationCounter = 0;
                xTaskNotifyGive(WriteToSharedMemoryTask);       /* Notify the Write to Shared Memory Task to run */
            }

            GpioDataRegs.GPCCLEAR.bit.GPIO92 = 1;
            taskYIELD();                                        /* Allow other tasks to run immediately */
        }
    }
}

/**
 * @brief Function for the Write to Shared Memory task.
 * @details This task waits for a notification from the Process Output Task, 
 *          then updates the shared data structure with the latest control and measurement data, 
 *          and signals CPU2 via IPC that new data is available then wait until these data will be processed by CPU2.
 */
static void WriteToSharedMemoryTask_Func(void *pvParameters)
{
    for(;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);             /* Wait for a notification from the Process Output Task */
        GpioDataRegs.GPCSET.bit.GPIO94 = 1;

        SharedDataCPU1TOCPU2.mda_data_s = *MDA_GetData_ps();                 
        SharedDataCPU1TOCPU2.foc_enable_state_b = FOC_GetEnableState();      
        SharedDataCPU1TOCPU2.mtcl_control_s = *MTCL_GetControlState_ps();    
        SharedDataCPU1TOCPU2.mtcl_maximum_position_rad_F32 = MTCL_GetMaximumPosition_F32();
        MTCL_GetMovementParams(&SharedDataCPU1TOCPU2.mtcl_movement_params_s.MaxSpeed__rad_s__F32,
                                    &SharedDataCPU1TOCPU2.mtcl_movement_params_s.MaxAccel__rad_s2__F32,
                                    &SharedDataCPU1TOCPU2.mtcl_movement_params_s.MaxTorque__Nm__F32); 
        SharedDataCPU1TOCPU2.active_service_mode_b = AC_GetServiceModeActive(); 

        /* IPC handshake with CPU2 */
        IpcRegs.IPCSET.bit.IPC13 = 1;          /* Flag must be acknowledged by ISR on CPU2 */
        IpcRegs.IPCSET.bit.IPC3  = 1;          /* Generate interrupt on CPU2 */
        while(IpcRegs.IPCFLG.bit.IPC13 != 0);  /* Wait until CPU2 enters ISR and ACKs marker */
        while(IpcRegs.IPCFLG.bit.IPC3  != 0);  /* Wait for CPU1 to acknowledge the update */
        
        GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;
    }
}

/* ===================================== OTHER FUNCTIONS ================================================ */

/**
 * @brief Function to set data received from CPU2 to the relevant modules based on the command received.
 * @details This function is called in the IPC ISR when CPU2 sends a command. 
 *          It checks the command type and updates the control parameters accordingly.
 */
void getDataFromCPU2(void)
{
    switch(SharedDataCPU2TOCPU1.received_command_e)
    {
        case AC_CMD_SET_MOVEMENT_PARAMS_e:
            MTCL_SetMovementParams(SharedDataCPU2TOCPU1.mtcl_movement_params_s.MaxSpeed__rad_s__F32,
                                    SharedDataCPU2TOCPU1.mtcl_movement_params_s.MaxAccel__rad_s2__F32,
                                    SharedDataCPU2TOCPU1.mtcl_movement_params_s.MaxTorque__Nm__F32);
            break;
        case AC_CMD_SET_REFERENCE_POSITION_e:
            if (!AC_GetBTNData_ps()->any_button_pressed_b) // Only set reference position if no button is pressed to avoid conflict with manual control
            {
                MTCL_SetReferencePosition(SharedDataCPU2TOCPU1.reference_position_rad_F32);
            }
            break;
        case AC_CMD_SET_MOVEMENT_ENABLE_STATE_e:
            FOC_SetEnableState(SharedDataCPU2TOCPU1.foc_enable_state_b);
            break;
        case AC_CMD_RESET_ERROR_FLAGS_e:
            if (SharedDataCPU2TOCPU1.reset_error_flags_b)
            {
                MTCL_ResetErrorFlags();
            }
            break;
        default:
            break;
    }
}

/* ===================================== ISRs ================================================ */

/**
 * @brief IPC2 interrupt service routine for CPU2.
 * @details This ISR is triggered when CPU2 sends a command to CPU1. It acknowledges the interrupt,
 *          reads the command and data sent by CPU2, and updates the control parameters accordingly.
 */
__interrupt void ipc2_isr_cpu2(void)
{
    GpioDataRegs.GPCSET.bit.GPIO94 = 1;
    IpcRegs.IPCACK.bit.IPC12 = 1;        /* Acknowledge IPC12 that interrupt has been received */

    getDataFromCPU2();                   /* Read the command and data sent by CPU2 and update control parameters accordingly */
    
    IpcRegs.IPCACK.bit.IPC2 = 1;         /* Clear the IPC2 interrupt flag */
    PieCtrlRegs.PIEACK.bit.ACK1 = 1;	 /* Must acknowledge the PIE group */
    GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;

    portYIELD_FROM_ISR(pdTRUE);          /* Yield to allow higher priority tasks to run immediately if needed */
}

/**
 * @brief ADC conversion completion interrupt. Occurs every 500 microseconds.
 * @details This ISR is triggered when an ADC conversion is complete. It gives a semaphore
 *          to notify the corresponding task and acknowledges the interrupt.
 */
__interrupt void MDA_AdcConversionCompleteIsr(void)
{
    // GpioDataRegs.GPCSET.bit.GPIO94 = 1;

    /* Give the semaphore to unblock the Process Inputs Task */
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(SyncSemaphore, &xHigherPriorityTaskWoken);

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1   = (U16)1;            /* Clear the ADC interrupt flag */                       
    PieCtrlRegs.PIEACK.bit.ACK1         = (U16)1;            /* Acknowledge the PIE group to allow further interrupts */
    
    // GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;

    if (xHigherPriorityTaskWoken == pdTRUE)
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);         /* Yield to allow the unblocked task to run immediately if it has higher priority */
    }
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
