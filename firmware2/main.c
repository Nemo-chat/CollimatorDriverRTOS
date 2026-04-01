/**
 * @file main.c
 * @brief Main application file
 * @details Details
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
 * Start work at RTOS integration 7.9.2025
 */
#include <main.h>
#include <ATB_interface.h>
#include <AC_interface.h>
#include <ECOM_interface.h>
#include <MDA_interface.h>
#include <PWM_interface.h>
#include <SCI.h>
#include "InterruptServiceRoutines.h"
#include "FOC.h"
#include "TEST.h"
#include <PI_Controller.h>
#include "spi.h"
#include "dispCtrl.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "FreeRTOSConfig.h"

// Macro to define stack size of individual tasks
#define COM_TASK_STACK_SIZE 1024
#define PRINT_TASK_STACK_SIZE 256

// Macro to define priorities of individual tasks
#define COM_TASK_PRIORITY   2
#define PRINT_TASK_PRIORITY   1

#define STACK_SIZE  256U

typedef struct
{
    MDA_Data_struct mda_data_s;
    AC_BTNManualControl_struct ac_btn_manual_control_s;
    MTCL_Control_struct mtcl_control_s;
    MTCL_MovementParams_struct mtcl_movement_params_s;
    F32 mtcl_maximum_position_rad_F32;
    boolean foc_enable_state_b;

}SharedDataCPU1TOCPU2_struct;

typedef struct 
{
    MTCL_MovementParams_struct mtcl_movement_params_s;
    F32 reference_position_rad_F32;
    boolean foc_enable_state_b;
    boolean reset_error_flags_b;
    AC_CommandIndex_enum received_command_e;

}SharedDataCPU2TOCPU1_struct;

// Task handles
TaskHandle_t CommunicationTask, PrintTask;

// Execution counters
static volatile uint32_t ctrCommunicationTask = 0;
static volatile uint32_t ctrPrintTask = 0;
static volatile uint32_t ctrInterrputEventIPC0 = 0;
static volatile uint32_t ctrInterrputEventIPC1 = 0;

uint32_t elapsedTime = 0;
uint32_t startTime = 0;
uint32_t endTime = 0;


// Mutex for shared data access
SemaphoreHandle_t IPCSharedMemoryMutex_cpu2; 

// Shared data structure for communication between CPU1 and CPU2
volatile SharedDataCPU1TOCPU2_struct SharedDataCPU1TOCPU2;
#pragma DATA_SECTION(SharedDataCPU1TOCPU2, "MSGRAM_CPU1_TO_CPU2")

// Shared data structure for communication between CPU1 and CPU2
volatile SharedDataCPU2TOCPU1_struct SharedDataCPU2TOCPU1;
#pragma DATA_SECTION(SharedDataCPU2TOCPU1, "MSGRAM_CPU2_TO_CPU1")

static StaticTask_t idleTaskBuffer;
static StackType_t  idleTaskStack[STACK_SIZE];
#pragma DATA_SECTION(idleTaskStack,   ".freertosStaticStack")
#pragma DATA_ALIGN ( idleTaskStack , portBYTE_ALIGNMENT )

#if(configAPPLICATION_ALLOCATED_HEAP == 1)
uint8_t ucHeap[ configTOTAL_HEAP_SIZE ];
#pragma DATA_SECTION(ucHeap,   ".freertosHeap")
#pragma DATA_ALIGN ( ucHeap , portBYTE_ALIGNMENT )
#endif

// Function prototypes
static void CommunicationTask_Func(void *pvParameters);
static void PrintTask_Func(void *pvParameters);

// vApplicationGetIdleTaskMemory
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleTaskStackBuffer,
                                    uint32_t *pulIdleTaskStackSize )
{
    *ppxIdleTaskTCBBuffer = &idleTaskBuffer;
    *ppxIdleTaskStackBuffer = idleTaskStack;
    *pulIdleTaskStackSize = STACK_SIZE;
}

// vApplicationStackOverflowHook
void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
{
    while(1);
}

__interrupt void isr_IPC0_cpu2(void);
__interrupt void isr_IPC1_cpu2(void);

void main(void)
{
    // Create a semaphore.
    IPCSharedMemoryMutex_cpu2 = xSemaphoreCreateMutex();

    /* Initialization */
    Interrupt_initModule();       // Need reveiw if it neccessary
    Interrupt_initVectorTable();  // Need reveiw if it neccessary
    IPC_ISRvInit_CPU2(&isr_IPC0_cpu2, &isr_IPC1_cpu2);          // Initialize IPC ISRs for CPU2
    spi_vInit(800000);
    dispCtrl_vInitDisplay();
    ATB_Init();
    SCI_Init();

    // Create tasks and check if they were created successfully
    if(xTaskCreate(CommunicationTask_Func, 
                   (const char *)"CommunicationTask", 
                   COM_TASK_STACK_SIZE, 
                   NULL,
                   (tskIDLE_PRIORITY + COM_TASK_PRIORITY), 
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

    // IPC synchronization with CPU1 
    IpcRegs.IPCSET.bit.IPC17 = 1;
    while(IpcRegs.IPCFLG.bit.IPC17 != 0);

    // Start the scheduler.  This should not return.
    vTaskStartScheduler();

    /* Main loop */
    for(;;)
    {

    }
}

/**
 * @brief Function for the Process Inputs task.
 */
static void CommunicationTask_Func(void *pvParameters)
{
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    SharedDataCPU1TOCPU2_struct localCopyOfSharedData;
    for(;;)
    {
        xSemaphoreTake(IPCSharedMemoryMutex_cpu2, portMAX_DELAY);                               // Take the mutex before accessing shared data
        take_ipc_mutex_cpu2();                                                // Take the IPC mutex to synchronize with CPU1
        
        MDA_SetData(&SharedDataCPU1TOCPU2.mda_data_s);         // Update MDA data in shared structure
        MTCL_SetMovementParams(SharedDataCPU1TOCPU2.mtcl_movement_params_s.MaxSpeed__rad_s__F32,
                                SharedDataCPU1TOCPU2.mtcl_movement_params_s.MaxAccel__rad_s2__F32,
                                SharedDataCPU1TOCPU2.mtcl_movement_params_s.MaxTorque__Nm__F32); // Update MTCL movement parameters in shared structure
        MTCL_SetMaximumPosition_F32(&SharedDataCPU1TOCPU2.mtcl_maximum_position_rad_F32); // Update MTCL maximum position in shared structure
       
        xSemaphoreGive(IPCSharedMemoryMutex_cpu2);                                              // Give the mutex after accessing shared data
        give_ipc_mutex_cpu2();                                                // Give the IPC mutex to allow CPU1 to access shared data
        
        ECOM_MainHandler();        

        AC_CommandIndex_enum received_cmd = AC_GetLastReceivedCommand();
        boolean should_update_shared_data_b = False_b;

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

        /* Reset command marker for next cycle */
        AC_ClearLastReceivedCommand();
        // Generate interrupt to notify CPU1 about updated data if a valid command was received and processed
        if (should_update_shared_data_b)
        {
            IpcRegs.IPCSET.bit.IPC2 = 1; 
            while(IpcRegs.IPCSTS.bit.IPC2 != 0); // Wait for CPU1 to acknowledge the update
        }

        // Increment the execution counter
        ctrCommunicationTask++;
        vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 120 ) );
    }
}

/**
 * @brief Function for the Application task.
 */
static void PrintTask_Func(void *pvParameters)
{
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    F32 AngularPosition_rad_F32;
    U16 OverTorqueError_f1;
    boolean FOCEnableState_b;

    for(;;)
    {
        xSemaphoreTake(IPCSharedMemoryMutex_cpu2, portMAX_DELAY);                               // Take the mutex before accessing shared data
        take_ipc_mutex_cpu2();
        
        AngularPosition_rad_F32 = SharedDataCPU1TOCPU2.mda_data_s.angular_position__rad__F32;
        OverTorqueError_f1 = SharedDataCPU1TOCPU2.mtcl_control_s.over_torque_error_f1;
        FOCEnableState_b = SharedDataCPU1TOCPU2.foc_enable_state_b;
        
        xSemaphoreGive(IPCSharedMemoryMutex_cpu2);                                              // Give the mutex after accessing shared data
        give_ipc_mutex_cpu2();                                                // Give the IPC mutex to allow CPU1 to access shared data
        
        DisplayRefresh(AngularPosition_rad_F32, OverTorqueError_f1, FOCEnableState_b);
        
        // Increment the execution counter
        ctrPrintTask++;
        
        vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 100 ) );
    }
}

__interrupt void isr_IPC0_cpu2(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    xSemaphoreTakeFromISR(IPCSharedMemoryMutex_cpu2, &xHigherPriorityTaskWoken);

    // Clear the IPC0 interrupt flag
    PieCtrlRegs.PIEACK.bit.ACK1 = 1;			// Must acknowledge the PIE group
	IpcRegs.IPCACK.bit.IPC0 = 1;                // Clear IPC0 flag
    IpcRegs.IPCACK.bit.IPC7 = 1;                // Clear IPC7 flag
    // Increment the interrupt event counter
    ctrInterrputEventIPC0++;
    
    if (xHigherPriorityTaskWoken == pdTRUE)
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

__interrupt void isr_IPC1_cpu2(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    xSemaphoreGiveFromISR(IPCSharedMemoryMutex_cpu2, &xHigherPriorityTaskWoken);

    // Clear the IPC1 interrupt flag
    PieCtrlRegs.PIEACK.bit.ACK1 = 1;			// Must acknowledge the PIE group
    IpcRegs.IPCACK.bit.IPC1 = 1;                // Clear IPC1 flag
    IpcRegs.IPCACK.bit.IPC8 = 1;                // Clear IPC8 flag
    // Increment the interrupt event counter
    ctrInterrputEventIPC1++;
    
    if (xHigherPriorityTaskWoken == pdTRUE)
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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
