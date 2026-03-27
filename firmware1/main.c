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

extern TRAN_struct tran_s;
extern boolean alarm_state;


void main(void)
{
    /* Initialization */

    mcu_vInitClocks();                                          /* Initialize uC clock system. */
    spi_vInit(800000);
    dispCtrl_vInitDisplay();
    ATB_Init();
    PWM_Init();
    SCI_Init();
    FOC_CommutationAlignment();
    MDA_Init();
    MTCL_Init();
    TEST_PinInit();
    MDA_CalibratePhaseCurrentsOffsets();
    AC_ManualControlInit();

    /*Redundant reset of PI controller structures*/
    PI_ctrl_Init(&PI_id_current_controller);
    PI_ctrl_Init(&PI_iq_current_controller);
    PI_ctrl_Init(&PI_speed_controller);
    PI_ctrl_Init(&PI_position_controller);


    FOC_SetEnableState(True_b);

    /* Main loop */
    while(1)
    {
        ECOM_MainHandler();
        AC_ManualControlHandler();
        DisplayRefresh();
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
