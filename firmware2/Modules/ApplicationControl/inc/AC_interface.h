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

/* Last command received via external communication */
typedef enum
{
    AC_CMD_NONE_e = 0xFF,
    AC_CMD_SET_MOVEMENT_PARAMS_e = 0,
    AC_CMD_SET_REFERENCE_POSITION_e = 1,
    AC_CMD_SET_MOVEMENT_ENABLE_STATE_e = 2,
    AC_CMD_RESET_ERROR_FLAGS_e = 3,
    AC_CMD_GET_MOVEMENT_PARAMS_e = 4,
    AC_CMD_GET_MECHANICAL_DATA_e = 5,
    AC_CMD_GET_ELECTRICAL_DATA_e = 6,
    AC_CMD_GET_MAXIMUM_POSITION_e = 7,
    AC_CMD_GET_FOC_STATE_e = 8,
    AC_CMD_GET_SERVICE_MODE_e = 9
} AC_CommandIndex_enum;

extern AC_CommandIndex_enum AC_LastReceivedCommand_e;

extern boolean AC_Reset_ErrorFlags_b;

void AC_ExecuteCommand( const U16 * const command_payload_pU16,
                        const U16 payload_size_U16,
                        U16 * response_data_pU16,
                        U16 * response_data_size_pU16 );

AC_CommandIndex_enum AC_GetLastReceivedCommand(void);
void AC_ClearLastReceivedCommand(void);

boolean AC_GetServiceModeActive(void);
void AC_SetServiceModeActive(boolean active_b);

void AC_ManualControlInit(void);
void AC_ManualControlHandler(void);
const AC_BTNManualControl_struct* AC_GetBTNData_ps(void);

#endif /* INC_AC_INTERFACE_H_ */

/**
 * @}
 */
