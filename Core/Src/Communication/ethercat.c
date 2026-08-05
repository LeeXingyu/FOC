/*
 * ethercat.c
 *
 *  Created on: Apr 9, 2026
 *      Author: Administrator
 */

#include "Communication/ethercat.h"
#include "applInterface.h"
#include "cia402appl.h"
#include "el9800hw.h"
#include "MotorControl/Core/mc_interface.h"
#include "main.h"

static EtherCAT_Cia402Shadow_t s_ecat_shadow = {0};

void LAN9253_Init(void)
{
    (void)HW_Init();
    (void)MainInit();
    (void)CiA402_Init();
    s_ecat_shadow.controlword = 0U;
    s_ecat_shadow.statusword = MC_Get_Cia402_Statusword();
    s_ecat_shadow.mode_of_operation = (uint8_t)g_axis.enCtrlMode;
    s_ecat_shadow.mode_display = (uint8_t)g_axis.enCtrlMode;
    s_ecat_shadow.axis_state = (uint8_t)g_axis.state;
    s_ecat_shadow.axis_error = (uint8_t)g_axis.error;
}

void LAN9253_Process(void)
{
    MainLoop();
    s_ecat_shadow.statusword = MC_Get_Cia402_Statusword();
    s_ecat_shadow.mode_of_operation = (uint8_t)g_axis.enCtrlMode;
    s_ecat_shadow.mode_display = (uint8_t)g_axis.enCtrlMode;
    s_ecat_shadow.axis_state = (uint8_t)g_axis.state;
    s_ecat_shadow.axis_error = (uint8_t)g_axis.error;
}

void LAN9253_SetTaskHandle(void *taskHandle)
{
    g_ethercat_task_handle = (TaskHandle_t)taskHandle;
}

const EtherCAT_Cia402Shadow_t *LAN9253_GetCia402Shadow(void)
{
    return &s_ecat_shadow;
}
