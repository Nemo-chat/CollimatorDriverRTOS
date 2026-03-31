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

extern TRAN_struct tran_s;
extern boolean alarm_state;

// Macro to define stack size of individual tasks
#define PROCESS_INPUT_TASK_STACK_SIZE 1024
#define APPLICATION_TASK_STACK_SIZE 256

// Macro to define priorities of individual tasks
#define PROCESS_INPUT_TASK_PRIORITY   2
#define APPLICATION_TASK_PRIORITY   1

#define STACK_SIZE  256U

typedef struct
{
    MDA_Data_struct mda_data_s;
    AC_BTNManualControl_struct ac_btn_manual_control_s;
    MTCL_Control_struct mtcl_control_s;
    boolean foc_enable_state_b;

}SharedDataCPU1TOCPU2_struct;

// Task handles
TaskHandle_t ProcessInputTask, ApplicationTask;

// Execution counters
static volatile uint32_t ctrProcessInputTaskHit = 0;
static volatile uint32_t ctrProcessInputTask = 0;
static volatile uint32_t ctrApplicationTask = 0;
static volatile uint32_t ctrInterrputEvent = 0;

uint32_t elapsedTime = 0;
uint32_t startTime = 0;
uint32_t endTime = 0;

// The Semaphore for synchronization ISR by ADC and Process Inputs Task.
SemaphoreHandle_t SyncSemaphore;
// Mutex for shared data access
SemaphoreHandle_t Mutex; 

// Shared data structure for communication between CPU1 and CPU2
volatile SharedDataCPU1TOCPU2_struct SharedDataCPU1TOCPU2;
#pragma DATA_SECTION(SharedDataCPU1TOCPU2, "MSGRAM_CPU1_TO_CPU2")

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
static void ProcessInputsTask_Func(void *pvParameters);
static void ApplicationTask_Func(void *pvParameters);

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

/**
 * @brief ADC conversion completion interrupt. Occurs every 500 microseconds.
 */
interrupt void MDA_AdcConverstionCompleteIsr(void)
{
    // ISR_MotorControlHandler();
    ctrInterrputEvent++;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(SyncSemaphore, &xHigherPriorityTaskWoken);

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1   = (U16)1;                                   /* Clear interrupt flag. */
    PieCtrlRegs.PIEACK.bit.ACK1         = (U16)1;
    
    if (xHigherPriorityTaskWoken == pdTRUE)
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

}

void main(void)
{
    // Create a semaphore.
    SyncSemaphore = xSemaphoreCreateBinary();
    Mutex = xSemaphoreCreateMutex();

    /* Initialization */
    mcu_vInitClocks();                              /* Initialize uC clock system. */
    spi_PinsInit();                                 /* Initialize SPI pins, select master core CPU2 */
    dispCtrl_vInitDisplay();
    ATB_Init();
    PWM_Init();
    SCI_Init();
    FOC_CommutationAlignment();
    MDA_Init(&MDA_AdcConverstionCompleteIsr);
    MTCL_Init();
    TEST_PinInit();
    MDA_CalibratePhaseCurrentsOffsets();
    AC_ManualControlInit();

    // Configure the shared memory GS11 and GS15 to be accessible by CPU2
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

    FOC_SetEnableState(True_b);

    // Create tasks and check if they were created successfully
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

    // IPC synchronization with CPU2 must be here
    // ....

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
static void ProcessInputsTask_Func(void *pvParameters)
{
    for(;;)
    {
        ctrProcessInputTaskHit++;
        // Wait for the semaphore to be given by the ADC ISR
        xSemaphoreTake(SyncSemaphore, portMAX_DELAY);
        
        // Process the inputs and update the data structure with newly 
        // acquired measurement data from the ADC and QEP.
        MDA_UpdateData();          // Execution time is around 47 microseconds
        AC_ManualControlHandler(); 

        xSemaphoreTake(Mutex, portMAX_DELAY);                               // Take the mutex before accessing shared data
        SharedDataCPU1TOCPU2.mda_data_s = *MDA_GetData_ps();                // Read updated data from shared memory
        SharedDataCPU1TOCPU2.ac_btn_manual_control_s = *AC_GetBTNData_ps(); // Read updated button control data from shared memory
        xSemaphoreGive(Mutex);                                              // Give the mutex after accessing shared data

        xTaskNotifyGive(ApplicationTask); // Notify the Application Task that new data is available

        // Increment the execution counter
        ctrProcessInputTask++;

    }
}

/**
 * @brief Function for the Application task.
 */
static void ApplicationTask_Func(void *pvParameters)
{
    // Wait for notification from the Process Inputs Task. 2x interrupt frequency
    const TickType_t xInterruptFrequency = pdMS_TO_TICKS( 1UL ); 
    // ulEventToProcess will hold the value returned by ulTaskNotifyTake
    uint32_t ulEventToProcess;
    // Local copy of shared data structure
    SharedDataCPU1TOCPU2_struct localCopyOfSharedData;

    for(;;)
    {
        ulEventToProcess = ulTaskNotifyTake( pdTRUE, xInterruptFrequency );
        if (ulEventToProcess != 0)
        {   
            // Increment the application time base
            ATB_IncrementTime();
            
            xSemaphoreTake(Mutex, portMAX_DELAY);                                 // Take the mutex before accessing shared data
            localCopyOfSharedData.mda_data_s = SharedDataCPU1TOCPU2.mda_data_s;   // Read shared data of MDA into local copy
            localCopyOfSharedData.ac_btn_manual_control_s = SharedDataCPU1TOCPU2.ac_btn_manual_control_s; 
            xSemaphoreGive(Mutex);                                                // Give the mutex after accessing shared data

            // Set reference position from button control if any button is pressed
            if (localCopyOfSharedData.ac_btn_manual_control_s.any_button_pressed_b)
            {
                MTCL_SetReferencePosition(localCopyOfSharedData.ac_btn_manual_control_s.BTN_ReferencePosition__rad__F32);
            }
            // Perform application control tasks here
            MTCL_MainHandler(&localCopyOfSharedData.mda_data_s);        // Execution time is around 44 microseconds

            xSemaphoreTake(Mutex, portMAX_DELAY);                                 // Take the mutex before accessing shared data
            SharedDataCPU1TOCPU2.foc_enable_state_b = FOC_GetEnableState();       // Update FOC enable state in shared data
            SharedDataCPU1TOCPU2.mtcl_control_s = *MTCL_GetControlState_ps();     // Update MTCL control struct in shared data
            xSemaphoreGive(Mutex);                                                // Give the mutex after accessing shared data
            
            // Increment the execution counter
            ctrApplicationTask++;
        }
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
