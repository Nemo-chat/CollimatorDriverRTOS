/**
 * @file MTCL_core.h
 * @brief Motor control submodule.
 * @details Manages motor position requests and motor control states.
 *
 * =================================================================
 * @author Bc. Roland Molnar
 *
 * =================================================================
 * KEM, FEI, TUKE
 * @date 29.02.2024
 * @addtogroup MTCL Motor control core
 * @{
 */

#ifndef MODULES_MOTORCONTROL_INC_MTCL_CORE_H_
#define MODULES_MOTORCONTROL_INC_MTCL_CORE_H_
#include <MTCL_interface.h>
#include <mcu.h>

#define MAX_SPEED__rad_s__dF32                  ( 400.0f )              /**< Maximum speed limitation, in radians per second */
#define MIN_SPEED__rad_s__dF32                  ( 0.1f )                /**< Minimum speed limitation, in radians per second */
#define MAX_ACCEL__rad_s2__dF32                 ( 100.0f )              /**< Maximum acceleration limitation, in radians per second^2 */
#define MIN_ACCEL__rad_s2__dF32                 ( 0.1f )                /**< Minmum acceleration limitation, in radians per second^2 */
#define MAX_TORQUE__Nm__dF32                    ( 1.0f )                /**< Maximum torque limitation, in Newtonmeters*/
#define MIN_TORQUE__Nm__dF32                    ( 0.1f )                /**< Minimum torque limitation, in Newtonmeters*/

#define DEFAULT_RUN_SPEED__rad_s__dF32          ( 2.0f )                /**< Default speed value, in radians per second*/
#define DEFAULT_RUN_ACCEL__rad_s2__dF32         ( 50.0f )               /**< Default acceleration value, in radians per second^2*/
#define DEFAULT_RUN_TORQUE__Nm__dF32            ( 0.2f )                /**< Default speed value, in Newtenmeters*/


typedef struct
{
    U16     torque_exceed_counter_U16;                                                                  /**< Number of PWM cycles of exceeded torque. */
} MTCL_TorqueCheck_struct;                                                                              /**< Torque exceed struct. */

boolean MTCL_TorqueExceedCheck(void);
static void MTCL_CalculateTrajectory(F32 Requested_Position__rad__F32,
                                     F32 MaxMechSpeed_rad_s1_F32,
                                     F32 MaxAcc_rad_s2_F32);
static void PC_Reset_Data(boolean Full_Reset);

#endif /* MODULES_MOTORCONTROL_INC_MTCL_CORE_H_ */

/**
 * @}
 */
