/**
 * Code edited by Bc. Vadym Holysh, date: April 23, 2026.
 * @file FOC.h
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
 */

#ifndef MODULES_FIELDORIENTEDCONTROL_INC_FOC_H_
#define MODULES_FIELDORIENTEDCONTROL_INC_FOC_H_

#include <MDA_interface.h>
#include <PI_Controller.h>
#include <TRAN.h>
#include <FAST_MATH_FUNC.h>
#include <MTCL_interface.h>

/*MOTOR CONSTANT*/
#define MOTOR_TORQUE_CONSTANT__Nm_A__df32                  ( (F32)0.036f )              /**< Motor torque constant value */

void    FOC_SetEnableState(boolean new_state_b);
boolean FOC_GetEnableState(void);
F32     FOC_GetTorque__Nm__F32(void);

#endif /* MODULES_FIELDORIENTEDCONTROL_INC_FOC_H_ */
