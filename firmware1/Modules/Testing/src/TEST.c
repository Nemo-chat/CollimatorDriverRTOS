/*
 * Code edited by Bc. Vadym Holysh, date: April 23, 2026.
 * TEST.c
 *
 *  Created on: Mar 20, 2024
 *      Author: roland
 */
#include <TEST.h>
#include <ATB_interface.h>
#include <PWM_interface.h>
#include <TRAN.h>
#include <FAST_MATH_FUNC.h>
#include <MDA_interface.h>
#include <PI_Controller.h>
#include <MTCL_interface.h>
#include <MDA_core.h>
F32 scalar_freq__Hz__F32                = (F32)0.0;
F32 current_forced_el_angle__rad__F32   = (F32)0.0;

void TEST_ScalarMotorMovementHandler(void)
{
    current_forced_el_angle__rad__F32 += TWO_PI_dF32 * scalar_freq__Hz__F32 * (F32)50.0e-6;
    current_forced_el_angle__rad__F32 = FM_RemainderAfterFloatDivision_F32(current_forced_el_angle__rad__F32, TWO_PI_dF32);
    PWM_ForceAngle(current_forced_el_angle__rad__F32, (F32)2, (F32)24.0);
    kukam_prud();
}

// F32 current_a[512];
// F32 current_b[512];
// F32 current_c[512];
//F32 current_d[1024];
//F32 current_q[1024];

U16 sample=0;
F32 speed[256];
F32 reference_speed[256];

// F32 prud_iq[3000];
// F32 position[256];
// F32 reference_position[256];
void kukam_prud(){
    if(sample < 256){
    //    MDA_GetRawPhaseCurrents( &current_a[sample],
    //                             &current_b[sample],
    //                             &current_c[sample] );

        reference_speed[sample] = PC_GetData_ps()->tj.Speed__rad_s__F32;
        speed[sample] = MDA_GetData_ps()->rotor_mech_speed__rad_s1__F32;
        // prud_iq[sample] = MDA_GetData_ps()->currents_s.iq__A__F32 ;//PC_GetData_ps()->tj.Acceleration__rad_s_2__F32;
        // position[sample] = MDA_GetData_ps()->angular_position__rad__F32;
        // reference_position[sample] = PC_GetData_ps()->tj.Position__rad__F32;
        sample++;
    }
    else sample = 0;
}

void TEST_SteppingHandler(void)
{
    static U32 last_exec_time_ticks_U32 = (U32)0;
    static F32 forced_angle_F32 = (F32)0;
    static boolean first_exec_b = True_b;

    if(first_exec_b)
    {
        last_exec_time_ticks_U32 = ATB_GetTicks_U32();
        first_exec_b = False_b;
        PWM_ForceAngle((F32)0.0, (F32)1.25, (F32)24.0);
    }

    if(ATB_CheckTicksPassed_U16(last_exec_time_ticks_U32, ATB_MS_TO_TICKS_dM_U32(250)))
    {
        last_exec_time_ticks_U32 = ATB_GetTicks_U32();
        PWM_ForceAngle(forced_angle_F32, (F32)1.25, (F32)24.0);
        forced_angle_F32 += TWO_PI_dF32 / (F32)4.0;
        forced_angle_F32 = FM_RemainderAfterFloatDivision_F32(forced_angle_F32, TWO_PI_dF32);
    }
}

void TEST_PinInit(void)
{
    EALLOW;
    GpioCtrlRegs.GPCDIR.bit.GPIO72 = (U16)1;
    EDIS;
}



