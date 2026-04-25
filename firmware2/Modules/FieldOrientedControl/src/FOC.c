/**
 * @file FOC.c
 * @brief Field oriented control
 *
 * =================================================================
 * @author Bc. Samuel Fertal
 * @author Bc. Vadym Holysh
 * @author Bc. Roland Molnar
 *
 * =================================================================
 * KEM, FEI, TUKE
 * @date 02.05.2024
 * @defgroup FOC FOC
 * @{
 */
#include <FOC.h>

static boolean s_FOC_EnableState_b = False_b;

/**
 * @brief Set FOC on/off
 * @param new_state_b is new enable state of the FOC.
 */
void FOC_SetEnableState(boolean new_state_b)
{


    if(new_state_b != s_FOC_EnableState_b){
        PI_ctrl_Init(&PI_id_current_controller);
        PI_ctrl_Init(&PI_iq_current_controller);
        PI_ctrl_Init(&PI_speed_controller);
        PI_ctrl_Init(&PI_position_controller);
        s_MTCL_ReferencePosition__rad__F32 = MDA_GetData_ps()->angular_position__rad__F32;
    }
    s_FOC_EnableState_b = new_state_b;
}

/**
 * @brief Set FOC enable status
 * @returns True_b when FOC is enabled
 * @returns False when FOC is disabled.
 */
boolean FOC_GetEnableState(void)
{
    return s_FOC_EnableState_b;
}

/**
 * @brief Get current motor torque.
 * @returns current motor torque.
 */
F32 FOC_GetTorque__Nm__F32(void)
{
    return MDA_GetData_ps()->currents_s.iq__A__F32 * MOTOR_TORQUE_CONSTANT__Nm_A__df32;
}

/**
 * @}
 */
