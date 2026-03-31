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
    boolean foc_enable_state_b;

}SharedDataCPU1TOCPU2_struct;

// Task handles
TaskHandle_t CommunicationTask, PrintTask;

// Execution counters
static volatile uint32_t ctrCommunicationTask = 0;
static volatile uint32_t ctrPrintTask = 0;

uint32_t elapsedTime = 0;
uint32_t startTime = 0;
uint32_t endTime = 0;


// Mutex for shared data access
SemaphoreHandle_t IPCSharedMemoryMutex; 

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

void main(void)
{
    // Create a semaphore.
    IPCSharedMemoryMutex = xSemaphoreCreateMutex();

    /* Initialization */
    mcu_vInitClocks();                                          /* Initialize uC clock system. */
    spi_vInit(800000);
    dispCtrl_vInitDisplay();
    ATB_Init();
    SCI_Init();

    // // Create tasks and check if they were created successfully
    // if(xTaskCreate(CommunicationTask_Func, 
    //                (const char *)"CommunicationTask", 
    //                COM_TASK_STACK_SIZE, 
    //                NULL,
    //                (tskIDLE_PRIORITY + COM_TASK_PRIORITY), 
    //                &CommunicationTask) != pdTRUE)
    // {
    //     ESTOP0;
    // }

    if(xTaskCreate(PrintTask_Func, 
                   (const char *)"PrintTask", 
                   PRINT_TASK_STACK_SIZE, 
                   NULL,
                   (tskIDLE_PRIORITY + PRINT_TASK_PRIORITY), 
                   &PrintTask) != pdTRUE)
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
static void CommunicationTask_Func(void *pvParameters)
{
    for(;;)
    {
        ECOM_MainHandler();        // Execution time is around 20 microseconds
        // Increment the execution counter
        ctrCommunicationTask++;
    }
}

/**
 * @brief Function for the Application task.
 */
static void PrintTask_Func(void *pvParameters)
{
    for(;;)
    {
        DisplayRefresh();
        // Increment the execution counter
        ctrPrintTask++;
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
