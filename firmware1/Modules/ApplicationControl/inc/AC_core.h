/**
 * @file AC_core.h
 * @brief Application control module
 * @details Module for controlling application using user interface or external serial communication.
 *
 * =================================================================
 * @author Bc. Roland Molnar
 *
 * =================================================================
 * KEM, FEI, TUKE
 * @date 29.02.2024
 * @addtogroup AC Application control core
 * Code edited by Bc. Vadym Holysh, date: April 23, 2026.
 * @{
*/

#include <app_types.h>
#ifndef AC_CORE_H_
#define AC_CORE_H_

#define AC_CORE_CHECK_INDEX_BOUND_dM_b(idx)   ( idx < (sizeof(AC_Functions) / sizeof(AC_ControlFunction_pF)) )

#define AC_BTN1_PRESSED_db          ( (boolean)!GpioDataRegs.GPBDAT.bit.GPIO40 )    /**< Check if button 1 is pressed. */
#define AC_BTN2_PRESSED_db          ( (boolean)!GpioDataRegs.GPBDAT.bit.GPIO41 )    /**< Check if button 2 is pressed. */
#define AC_BTN3_PRESSED_db          ( (boolean)!GpioDataRegs.GPBDAT.bit.GPIO42 )    /**< Check if button 3 is pressed. */
#define AC_BTN4_PRESSED_db          ( (boolean)!GpioDataRegs.GPBDAT.bit.GPIO43 )    /**< Check if button 4 is pressed. */

/* Button debouncing structure. */
typedef struct
{
    U32 rising_edge_ticks_U32;
    U32 falling_edge_ticks_U32;
    boolean last_state_b;
    boolean debounced_state_b;
    boolean was_pressed_b; 
} AC_BtnDebounce_struct;

/* Debounced button state. */
typedef enum
{
    DEBOUNCE_NO_CHANGE_e    = 0,
    DEBOUNCE_RISING_EDGE_e  = 1,
    DEBOUNCE_FALLING_EDGE_e = 2
} AC_BtnDebouncedState_enum;

U16 AC_BtnDebounce_U16(AC_BtnDebounce_struct* debounce_ps, boolean current_state_b);

#endif /* AC_CORE_H_ */

/**
 * @}
 */
