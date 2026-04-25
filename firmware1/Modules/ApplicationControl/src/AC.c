/**
 * @file AC.c
 * @brief Application control module
 * @details Module for controlling application using user interface or external serial communication.
 *
 * =================================================================
 * @author Bc. Roland Molnar
 *
 * =================================================================
 * KEM, FEI, TUKE
 * @date 29.02.2024
 * @defgroup AC Application control
 * @{
 */
#include <AC_core.h>
#include <AC_interface.h>
#include <ByteConversions.h>
#include <MDA_interface.h>
#include <MTCL_interface.h>
#include <ATB_interface.h>
#include <FOC.h>

AC_BTNManualControl_struct s_AC_BTNManualControl_s = {False_b, 0.0f};    /**< Button manual control struct. */
boolean AC_Reset_ErrorFlags_b = False_b;                                 /**< Flag for resetting error flags in motor control module. */
boolean s_AC_ServiceModeActive_b = False_b;                               /**< Service mode active flag. */
U16 AC_BTN1_PRESSED_b = 0;                                               /**< Button 1 pressed flag. */
U16 AC_BTN2_PRESSED_b = 0;                                               /**< Button 2 pressed flag. */
U16 AC_BTN3_PRESSED_b = 0;                                               /**< Button 3 pressed flag. */
U16 AC_BTN4_PRESSED_b = 0;                                               /**< Button 4 pressed flag. */

AC_BtnDebounce_struct AC_Btn1Debounce_s = {0};
AC_BtnDebounce_struct AC_Btn2Debounce_s = {0};
AC_BtnDebounce_struct AC_Btn3Debounce_s = {0};
AC_BtnDebounce_struct AC_Btn4Debounce_s = {0};

boolean AC_GetServiceModeActive(void)
{
    return s_AC_ServiceModeActive_b;
}

/**
 * @brief Control button initialization.
 */
void AC_ManualControlInit(void)
{
    EALLOW;
    /* SB2 - Button1 */
    GpioCtrlRegs.GPBDIR.bit.GPIO40 = 0;     /* Pin as input. */
    GpioCtrlRegs.GPBPUD.bit.GPIO40 = 0;     /* Enable internal pull up. */

    /* SB3 - Button2 */
    GpioCtrlRegs.GPBDIR.bit.GPIO41 = 0;     /* Pin as input. */
    GpioCtrlRegs.GPBPUD.bit.GPIO41 = 0;     /* Enable internal pull up. */

    GpioCtrlRegs.GPBDIR.bit.GPIO42 = 0;     /* Pin as input. */
    GpioCtrlRegs.GPBPUD.bit.GPIO42 = 0;     /* Enable internal pull up. */

    /* SB5 - Button4 */
    GpioCtrlRegs.GPBDIR.bit.GPIO43 = 0;     /* Pin as input. */
    GpioCtrlRegs.GPBPUD.bit.GPIO43 = 0;     /* Enable internal pull up. */
    EDIS;
}


U16 AC_BtnDebounce_U16(AC_BtnDebounce_struct* debounce_ps, boolean current_state_b)
{
    U16 return_state_U16 = DEBOUNCE_NO_CHANGE_e;

    if(current_state_b != debounce_ps->last_state_b)
    {
        if(current_state_b)
        {
            debounce_ps->rising_edge_ticks_U32 = ATB_GetTicks_U32();
        }
        else
        {
            debounce_ps->falling_edge_ticks_U32 = ATB_GetTicks_U32();
        }
    }

    if(current_state_b && ATB_CheckTicksPassed_U16(debounce_ps->rising_edge_ticks_U32, ATB_MS_TO_TICKS_dM_U32(50)))
    {
        if(debounce_ps->debounced_state_b == False_b)
        {
            debounce_ps->debounced_state_b = True_b;
            debounce_ps->falling_edge_ticks_U32 = 0; // Reset falling edge ticks to avoid false detection when button is held down
        }
        return_state_U16 = DEBOUNCE_RISING_EDGE_e;
    }
    else if(!current_state_b && ATB_CheckTicksPassed_U16(debounce_ps->falling_edge_ticks_U32, ATB_MS_TO_TICKS_dM_U32(50)))
    {
        if(debounce_ps->debounced_state_b == True_b)
        {
            debounce_ps->debounced_state_b = False_b;
            debounce_ps->rising_edge_ticks_U32 = 0; // Reset rising edge ticks to avoid false detection when button is held down
        }
        return_state_U16 = DEBOUNCE_FALLING_EDGE_e;
    }
    debounce_ps->last_state_b = current_state_b;

    return return_state_U16;
}

void AC_ManualControlHandler(void)
{
    /* Button 3: toggle service mode (independent of BTN1/BTN2) */
    AC_BTN3_PRESSED_b = AC_BtnDebounce_U16(&AC_Btn3Debounce_s, AC_BTN3_PRESSED_db);
    if (AC_BTN3_PRESSED_b == DEBOUNCE_RISING_EDGE_e)
    {
        AC_Btn3Debounce_s.was_pressed_b = True_b;
    }
    if (AC_Btn3Debounce_s.was_pressed_b && AC_BTN3_PRESSED_b == DEBOUNCE_FALLING_EDGE_e)
    {
        AC_Btn3Debounce_s.was_pressed_b = False_b;
        s_AC_ServiceModeActive_b = !s_AC_ServiceModeActive_b;
    }

    /* Button 4: toggle FOC enable/disable (independent of other buttons) */
    AC_BTN4_PRESSED_b = AC_BtnDebounce_U16(&AC_Btn4Debounce_s, AC_BTN4_PRESSED_db);
    if (AC_BTN4_PRESSED_b == DEBOUNCE_RISING_EDGE_e)
    {
        AC_Btn4Debounce_s.was_pressed_b = True_b;
    }
    if (AC_Btn4Debounce_s.was_pressed_b && AC_BTN4_PRESSED_b == DEBOUNCE_FALLING_EDGE_e)
    {
        AC_Btn4Debounce_s.was_pressed_b = False_b;
        FOC_SetEnableState(!FOC_GetEnableState());
    }

    /* Buttons 1 & 2: manual position control */
    AC_BTN1_PRESSED_b = (AC_BtnDebounce_U16(&AC_Btn1Debounce_s, AC_BTN1_PRESSED_db));
    AC_BTN2_PRESSED_b = (AC_BtnDebounce_U16(&AC_Btn2Debounce_s, AC_BTN2_PRESSED_db));

    /* No action if both pressed or both not pressed */
    if (AC_BTN1_PRESSED_b == AC_BTN2_PRESSED_b)
    {
        s_AC_BTNManualControl_s.any_button_pressed_b = False_b;
    }
    else
    {
        if (AC_BTN1_PRESSED_b == DEBOUNCE_RISING_EDGE_e)
        {
            s_AC_BTNManualControl_s.BTN_ReferencePosition__rad__F32 = MDA_GetData_ps()->angular_position__rad__F32 + 0.058448f;
            s_AC_BTNManualControl_s.any_button_pressed_b = True_b;
        }
        else if (AC_BTN2_PRESSED_b == DEBOUNCE_RISING_EDGE_e)
        {
            s_AC_BTNManualControl_s.BTN_ReferencePosition__rad__F32 = MDA_GetData_ps()->angular_position__rad__F32 - 0.058448f;
            s_AC_BTNManualControl_s.any_button_pressed_b = True_b;
        }
    }
}

inline const AC_BTNManualControl_struct* AC_GetBTNData_ps(void)
{
    return &s_AC_BTNManualControl_s;
}