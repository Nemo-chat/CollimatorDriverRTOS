/**
 * @file AC_interface.h
 * @brief Application control module
 * @details Module for controlling application using user interface or external serial communication.
 *
 * =================================================================
 * @author Bc. Roland Molnar
 *
 * =================================================================
 * KEM, FEI, TUKE
 * @date 29.02.2024
 * @addtogroup AC Application control interface
 * @{
 */
#include <app_types.h>

#ifndef INC_AC_INTERFACE_H_
#define INC_AC_INTERFACE_H_

typedef struct 
{
    boolean any_button_pressed_b;
    F32 BTN_ReferencePosition__rad__F32;

}AC_BTNManualControl_struct;

void AC_ExecuteCommand( const U16 * const command_payload_pU16,
                        const U16 payload_size_U16,
                        U16 * response_data_pU16,
                        U16 * response_data_size_pU16 );

void AC_ManualControlInit(void);
void AC_ManualControlHandler(void);
const AC_BTNManualControl_struct* AC_GetBTNData_ps(void);

#endif /* INC_AC_INTERFACE_H_ */

/**
 * @}
 */
