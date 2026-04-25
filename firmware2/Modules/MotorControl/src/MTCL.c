/**
 * @file MTCL.c
 *
 * @section MTCL MTCL Motor control module
 * @brief Motor control module
 * @details Module for controlling actual motor data and caluclating motor desired data
 * =================================================================
 * @author Bc. Roland Molnar
 * @author Bc. Vadym Holysh
 * @author Bc. Samuel Fertal
 * =================================================================
 * KEM, FEI, TUKE
 * @date 29.02.2024
 * @defgroup MTCL Motor control module
 * @{
 */

#include <MTCL_core.h>
#include <FOC.h>
#include <ATB_interface.h>

MTCL_Control_struct s_MTCL_Control_s           = {0,0,0,1,0,0};                                                        /**< Motor control struct. */
static MTCL_TorqueCheck_struct s_Torque_check_s = {0};                                                                  /**< Motor torque error check struct. */
static PC_Data_struct s_PC_data_s               = {0};                                                                  /**< Motor trajectory data struct. */

F32 s_MTCL_ReferencePosition__rad__F32      = 0.0f;                                                                 /**< User requested position. */
static F32 s_MTCL_MaxSpeed__rad_s__F32       = DEFAULT_RUN_SPEED__rad_s__dF32;                                       /**< Current value of maximum speed. */
static F32 s_MTCL_MaxAccel__rad_s2__F32      = DEFAULT_RUN_ACCEL__rad_s2__dF32;                                      /**< Current value of maximum acceleration. */
static F32 s_MTCL_MaxTorque__Nm__F32         = DEFAULT_RUN_TORQUE__Nm__dF32;                                         /**< Current value of maximum Torque. - not used. */

static F32 s_MTCL_MaxPosition__rad__F32      = 5.8448f;  // 10 cm in rad                                             /**< Maximum position for valid request. */
static F32 prev_request_pos__F32__           = 0.0f;                                                                 /**< Previously requested position. */                                                                                                          /**< Debug variable. */

/**
 * @brief Write new movement parameters
 * @param max_speed__rad_s__F32 New maximum speed.
 * @param max_accel__rad_s2__F32 New maximum acceleration.
 * @param max_torque__Nm__F32 New maximum torque.
 * @returns Status
 * @retval 0 Writing failed.
 * @retval 1 Writing was successful.
 */
boolean MTCL_SetMovementParams(const F32 max_speed__rad_s__F32, const F32 max_accel__rad_s2__F32, const F32 max_torque__Nm__F32)
{
    boolean return_state_b = True_b;
    if( (max_speed__rad_s__F32 > MAX_SPEED__rad_s__dF32) || (max_speed__rad_s__F32 < MIN_SPEED__rad_s__dF32)
        || (max_accel__rad_s2__F32 > MAX_ACCEL__rad_s2__dF32) || (max_accel__rad_s2__F32 < MIN_ACCEL__rad_s2__dF32)
        || (max_torque__Nm__F32 > MAX_TORQUE__Nm__dF32) || (max_torque__Nm__F32 < MIN_TORQUE__Nm__dF32))
    {
        return_state_b = False_b;
    }
    else
    {
        s_MTCL_MaxSpeed__rad_s__F32 = max_speed__rad_s__F32;
        s_MTCL_MaxAccel__rad_s2__F32 = max_accel__rad_s2__F32;
        s_MTCL_MaxTorque__Nm__F32 = max_torque__Nm__F32;
    }

    return return_state_b;
}

/**
 * @brief get current movement configuration.
 * @param max_speed__rad_s__F32 is a pointer to max speed value to be written.
 * @param max_accel__rad_s2__F32 is a pointer to max acceleration value to be written.
 * @param max_torque__Nm__F32 is a pointer to max torque value to be written.
 */
void MTCL_GetMovementParams(F32 * const max_speed__rad_s__F32, F32 * const max_accel__rad_s2__F32, F32 * const max_torque__Nm__F32)
{
    *max_speed__rad_s__F32 = s_MTCL_MaxSpeed__rad_s__F32;
    *max_accel__rad_s2__F32 = s_MTCL_MaxAccel__rad_s2__F32;
    *max_torque__Nm__F32 = s_MTCL_MaxTorque__Nm__F32;
}

/**
 * @brief Set new reference position
 * @param new_position__rad__F32 is new position in radians.
 * @returns Status
 * @retval 0 Position was not set. Out of defined limits.
 * @retval 1 Position was set.
 */
boolean MTCL_SetReferencePosition(const F32 new_position__rad__F32)
{
    boolean return_state_b = True_b;
    if( (new_position__rad__F32 < 0.0f) || (new_position__rad__F32 > (s_MTCL_MaxPosition__rad__F32 + 0.058448f)) )
    {
        if((new_position__rad__F32 < 0.0f))
        {
        s_MTCL_ReferencePosition__rad__F32 = 0.0f;
        }
        else
        {
        s_MTCL_ReferencePosition__rad__F32 = s_MTCL_MaxPosition__rad__F32;
        }

        return_state_b = False_b;
    }
    else
    {
        s_MTCL_ReferencePosition__rad__F32 = new_position__rad__F32;
    }
    return return_state_b;
}

F32 MTCL_GetReferencePosition_F32(void)
{
    return s_MTCL_ReferencePosition__rad__F32;
}

/**
 * @brief Reset trajectory data.
 * @param Full_Reset makes the position reset
 */
static inline void PC_Reset_Data(boolean Full_Reset)
{
    s_PC_data_s.Ticks__s__F32 = 0.0f;
    s_PC_data_s.ticks_enabled = False_b;
    s_PC_data_s.tj.Acceleration__rad_s_2__F32 = 0.0f;
    s_PC_data_s.tj.Speed__rad_s__F32 = 0.0f;
    if(Full_Reset) s_PC_data_s.tj.Position__rad__F32 = 0.0f;
}

/**
 * @brief Get motor control struct
 * @returns pointer to structure
 */
inline const MTCL_Control_struct* MTCL_GetControlState_ps(void)
{
    return &s_MTCL_Control_s;
}

/**
 * @brief Get maximum position in radians.
 * @returns Maximum position in radians.
 */
inline F32 MTCL_GetMaximumPosition_F32(void)
{
    return s_MTCL_MaxPosition__rad__F32;
}

inline void MTCL_SetMaximumPosition_F32(F32* new_max_position__rad__F32)
{
    s_MTCL_MaxPosition__rad__F32 = *new_max_position__rad__F32;
}

/**
 * @}
 */
