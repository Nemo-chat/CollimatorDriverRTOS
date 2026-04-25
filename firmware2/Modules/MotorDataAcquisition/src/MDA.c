/**
 * Code edited by Bc. Vadym Holysh, date: April 23, 2026.
 * @file MDA.c
 * @brief Motor data acquisition module
 * @details Module for readeading and evaluating measurement data for further regulation and control.
 *
 * =================================================================
 * @author Bc. Roland Molnar
 *
 * =================================================================
 * KEM, FEI, TUKE
 * @date 30.03.2024
 * @defgroup MDA Motor data acquisition
 * @{
 */
#include <MDA_interface.h>

static MDA_Data_struct s_MDA_data_s;                                              /**< Module data structure. */

/**
 * @brief Access motor data for motor control.
 * @returns pointer to acquired motor currents, speed and position data.
 */
inline const MDA_Data_struct* MDA_GetData_ps(void)
{
    return (const MDA_Data_struct*)&s_MDA_data_s;
}

inline void MDA_SetData(const MDA_Data_struct* new_data_ps)
{
    s_MDA_data_s = *new_data_ps;
}

/**
 * @}
 */
