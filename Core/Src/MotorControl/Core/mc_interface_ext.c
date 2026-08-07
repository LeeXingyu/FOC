#include "mc_interface_ext.h"

#include "main.h"
#include "motor_parameters.h"
#include "mc_interface.h"
#include "Utils/Pid/pid_position_type.h"

#include <string.h>

static uint32_t s_cia402_position_window = 0U;
static uint16_t s_cia402_position_window_time = 0U;
static int16_t s_cia402_velocity_sensor_selection = 0;
static uint32_t s_cia402_motor_rated_current = (uint32_t)(CURRENT_SCALE * 1000.0f);
static uint32_t s_cia402_motor_rated_torque = 0U;
static uint32_t s_cia402_dc_link_voltage_mv = 0U;
static int32_t s_cia402_position_range_limit[2] = {0, 0};
static int32_t s_cia402_home_offset = 0;
static int32_t s_cia402_software_position_limit[2] = {0, 0};
static uint8_t s_cia402_polarity = 0U;
static uint32_t s_cia402_max_profile_velocity = 0U;
static uint32_t s_cia402_max_motor_speed = 0U;
static uint32_t s_cia402_end_velocity = 0U;
static uint32_t s_cia402_quick_stop_deceleration = 0U;
static int16_t s_cia402_motion_profile_type = 0;
static uint32_t s_cia402_torque_slope = 0U;
static uint32_t s_cia402_max_acceleration = 0U;
static uint32_t s_cia402_max_deceleration = 0U;
static int8_t s_cia402_homing_method = 0;
static uint32_t s_cia402_homing_speed[2] = {0U, 0U};
static uint32_t s_cia402_homing_acceleration = 0U;
static int16_t s_cia402_interpolation_submode = 0;
static int32_t s_cia402_interpolation_data[4] = {0};
static uint8_t s_cia402_interpolation_time_value = 0U;
static int8_t s_cia402_interpolation_time_index = 0;
static uint32_t s_cia402_interpolation_config[4] = {0U, 0U, 0U, 0U};
static uint32_t s_cia402_position_encoder_resolution[2] = {ENC_COUNTS_PER_REV, 1U};
static uint32_t s_cia402_velocity_encoder_resolution[2] = {ENC_COUNTS_PER_REV, 1U};
static uint32_t s_cia402_gear_ratio[2] = {1U, 1U};
static uint32_t s_cia402_feed_constant[2] = {1U, 1U};
static uint32_t s_cia402_position_factor[2] = {1U, 1U};
static uint32_t s_cia402_velocity_factor[2] = {1U, 1U};
static uint32_t s_cia402_acceleration_factor[2] = {1U, 1U};
static uint16_t s_cia402_touch_probe_function = 0U;
static uint16_t s_cia402_touch_probe_status = 0U;
static int32_t s_cia402_touch_probe_value[4] = {0};
static uint16_t s_cia402_error_code = 0U;
static uint32_t s_cia402_supported_drive_modes =
    (1UL << 0) | (1UL << 2) | (1UL << 3) | (1UL << 7) | (1UL << 8) | (1UL << 9);
static uint32_t s_cia402_digital_inputs = 0U;
static uint32_t s_cia402_physical_outputs = 0U;
static uint32_t s_cia402_output_mask = 0U;
static uint16_t s_cia402_positive_torque_limit = 0U;
static uint16_t s_cia402_negative_torque_limit = 0U;
static uint32_t s_cia402_sdo_rx_cobid = 0U;
static uint32_t s_cia402_sdo_tx_cobid = 0U;
static uint8_t s_cia402_sdo_node_id = 1U;
static uint32_t s_cia402_rpdo34_comm[2][5] = {
    {0x00000401UL, 255U, 0U, 0U, 0U},
    {0x00000501UL, 255U, 0U, 0U, 0U}
};
static uint32_t s_cia402_tpdo34_comm[2][5] = {
    {0x80000381UL, 255U, 0U, 0U, 20U},
    {0x80000481UL, 255U, 0U, 0U, 20U}
};
static uint32_t s_cia402_rpdo34_map[2][8] = {{0}};
static uint32_t s_cia402_tpdo34_map[2][8] = {{0}};
static uint8_t s_cia402_rpdo34_map_count[2] = {0U, 0U};
static uint8_t s_cia402_tpdo34_map_count[2] = {0U, 0U};

static void MC_Cia402Ext_CopyU16(uint8_t *value, uint8_t *size, uint16_t data)
{
    value[0] = (uint8_t)data;
    value[1] = (uint8_t)(data >> 8);
    *size = 2U;
}

static void MC_Cia402Ext_CopyU32(uint8_t *value, uint8_t *size, uint32_t data)
{
    (void)memcpy(value, &data, 4U);
    *size = 4U;
}

static void MC_Cia402Ext_CopyS32(uint8_t *value, uint8_t *size, int32_t data)
{
    (void)memcpy(value, &data, 4U);
    *size = 4U;
}

static void MC_Cia402Ext_CopyS16(uint8_t *value, uint8_t *size, int16_t data)
{
    (void)memcpy(value, &data, 2U);
    *size = 2U;
}

static uint16_t MC_Cia402Ext_MapAxisError(void)
{
    switch (g_axis.error)
    {
        case AXIS_ERROR_NONE:               return 0x0000U;
        case AXIS_ERROR_OVERCURRENT:        return 0x2211U;
        case AXIS_ERROR_OVERVOLTAGE:        return 0x2310U;
        case AXIS_ERROR_GATE_DRIVER:        return 0x4310U;
        case AXIS_ERROR_ENCODER:            return 0x7380U;
        case AXIS_ERROR_OVERTEMP:           return 0x4210U;
        case AXIS_ERROR_UNDERVOLTAGE:       return 0x2320U;
        case AXIS_ERROR_CALIBRATION_FAILED: return 0x6320U;
        default:                            return 0x6100U;
    }
}

static int32_t MC_Cia402Ext_GetActualVelocityRpm(void)
{
    uint8_t polePairs = MC_Get_Pole_Pairs();
    return (int32_t)(FIXP30_toF(g_axis.speedCtrl.speedMeas_pu) *
                     FREQUENCY_SCALE * 60.0f /
                     (float)((polePairs == 0U) ? 1U : polePairs));
}

static int32_t MC_Cia402Ext_GetActualPositionCounts(void)
{
    return (int32_t)(g_axis.posCtrl.iAbsRawPos - g_axis.posCtrl.iZeroAngle);
}

static int16_t MC_Cia402Ext_GetActualCurrentmA(void)
{
    return (int16_t)(FIXP30_toF(g_axis.currCtrl.calcIdq.Q) * CURRENT_SCALE * 1000.0f);
}

static int16_t MC_Cia402Ext_GetDemandCurrentmA(void)
{
    return (int16_t)(FIXP30_toF(g_axis.currCtrl.refIdq.Q) * CURRENT_SCALE * 1000.0f);
}

static uint32_t MC_Cia402Ext_GetBusVoltagemV(void)
{
    return (uint32_t)(FIXP30_toF(g_axis.busVoltage) * VOLTAGE_SCALE * 1000.0f);
}

static bool MC_Cia402Ext_ReadSubIndexedPair(uint8_t subIndex,
                                            const uint32_t data[2],
                                            uint8_t *value, uint8_t *size)
{
    if (subIndex == 0U)
    {
        value[0] = 2U;
        *size = 1U;
        return true;
    }
    if ((subIndex == 1U) || (subIndex == 2U))
    {
        MC_Cia402Ext_CopyU32(value, size, data[subIndex - 1U]);
        return true;
    }
    return false;
}

static bool MC_Cia402Ext_ReadPdoComm(uint16_t index, uint8_t subIndex,
                                     uint8_t *value, uint8_t *size)
{
    uint8_t slot = (index == 0x1402U || index == 0x1802U) ? 0U : 1U;
    uint32_t *table = (index == 0x1402U || index == 0x1403U) ?
                      s_cia402_rpdo34_comm[slot] : s_cia402_tpdo34_comm[slot];

    if (subIndex == 0U)
    {
        value[0] = (index == 0x1402U || index == 0x1403U) ? 2U : 5U;
        *size = 1U;
        return true;
    }
    if ((subIndex >= 1U) && (subIndex <= 5U))
    {
        if ((index == 0x1402U || index == 0x1403U) && (subIndex > 2U))
        {
            return false;
        }
        if (subIndex == 2U)
        {
            value[0] = (uint8_t)table[1];
            *size = 1U;
        }
        else if ((subIndex == 3U) || (subIndex == 5U))
        {
            MC_Cia402Ext_CopyU16(value, size, (uint16_t)table[subIndex - 1U]);
        }
        else
        {
            MC_Cia402Ext_CopyU32(value, size, table[subIndex - 1U]);
        }
        return true;
    }
    return false;
}

static bool MC_Cia402Ext_ReadPdoMap(uint16_t index, uint8_t subIndex,
                                    uint8_t *value, uint8_t *size)
{
    uint8_t slot = (index == 0x1602U || index == 0x1A02U) ? 0U : 1U;
    uint32_t *table = (index == 0x1602U || index == 0x1603U) ?
                      s_cia402_rpdo34_map[slot] : s_cia402_tpdo34_map[slot];
    uint8_t count = (index == 0x1602U || index == 0x1603U) ?
                    s_cia402_rpdo34_map_count[slot] : s_cia402_tpdo34_map_count[slot];

    if (subIndex == 0U)
    {
        value[0] = count;
        *size = 1U;
        return true;
    }
    if ((subIndex >= 1U) && (subIndex <= 8U))
    {
        MC_Cia402Ext_CopyU32(value, size, table[subIndex - 1U]);
        return true;
    }
    return false;
}

bool MC_Cia402Ext_ReadObject(uint16_t index, uint8_t subIndex,
                             uint8_t *value, uint8_t *size)
{
    int32_t temp32;

    if ((value == NULL) || (size == NULL))
    {
        return false;
    }

    s_cia402_error_code = MC_Cia402Ext_MapAxisError();
    s_cia402_dc_link_voltage_mv = MC_Cia402Ext_GetBusVoltagemV();
    s_cia402_sdo_rx_cobid = 0x600U + (uint32_t)s_cia402_sdo_node_id;
    s_cia402_sdo_tx_cobid = 0x580U + (uint32_t)s_cia402_sdo_node_id;

    switch (index)
    {
        case 0x603FU:
            if (subIndex != 0U) return false;
            MC_Cia402Ext_CopyU16(value, size, s_cia402_error_code);
            return true;
        case 0x6062U:
            if (subIndex != 0U) return false;
            MC_Cia402Ext_CopyS32(value, size, (int32_t)g_axis.posCtrl.fPosRef);
            return true;
        case 0x6063U:
            if (subIndex != 0U) return false;
            MC_Cia402Ext_CopyS32(value, size, (int32_t)g_axis.posCtrl.iAbsRawPos);
            return true;
        case 0x6067U:
            if (subIndex != 0U) return false;
            MC_Cia402Ext_CopyU32(value, size, s_cia402_position_window);
            return true;
        case 0x6068U:
            if (subIndex != 0U) return false;
            MC_Cia402Ext_CopyU16(value, size, s_cia402_position_window_time);
            return true;
        case 0x6069U:
            if (subIndex != 0U) return false;
            MC_Cia402Ext_CopyS32(value, size, MC_Cia402Ext_GetActualVelocityRpm());
            return true;
        case 0x606AU:
            if (subIndex != 0U) return false;
            MC_Cia402Ext_CopyS16(value, size, s_cia402_velocity_sensor_selection);
            return true;
        case 0x606BU:
            if (subIndex != 0U) return false;
            MC_Cia402Ext_CopyS32(value, size, MC_Cia402Ext_GetActualVelocityRpm());
            return true;
        case 0x6074U:
            if (subIndex != 0U) return false;
            MC_Cia402Ext_CopyS16(value, size, MC_Cia402Ext_GetDemandCurrentmA());
            return true;
        case 0x6075U:
            if (subIndex != 0U) return false;
            MC_Cia402Ext_CopyU32(value, size, s_cia402_motor_rated_current);
            return true;
        case 0x6076U:
            if (subIndex != 0U) return false;
            MC_Cia402Ext_CopyU32(value, size, s_cia402_motor_rated_torque);
            return true;
        case 0x6077U:
            if (subIndex != 0U) return false;
            MC_Cia402Ext_CopyS16(value, size, MC_Cia402Ext_GetActualCurrentmA());
            return true;
        case 0x6078U:
            if (subIndex != 0U) return false;
            MC_Cia402Ext_CopyS16(value, size, MC_Cia402Ext_GetActualCurrentmA());
            return true;
        case 0x6079U:
            if (subIndex != 0U) return false;
            MC_Cia402Ext_CopyU32(value, size, s_cia402_dc_link_voltage_mv);
            return true;
        case 0x607BU:
            return MC_Cia402Ext_ReadSubIndexedPair(subIndex,
                                                   (const uint32_t *)s_cia402_position_range_limit,
                                                   value, size);
        case 0x607CU:
            if (subIndex != 0U) return false;
            MC_Cia402Ext_CopyS32(value, size, s_cia402_home_offset);
            return true;
        case 0x607DU:
            return MC_Cia402Ext_ReadSubIndexedPair(subIndex,
                                                   (const uint32_t *)s_cia402_software_position_limit,
                                                   value, size);
        case 0x607EU:
            if (subIndex != 0U) return false;
            value[0] = s_cia402_polarity;
            *size = 1U;
            return true;
        case 0x607FU:
            if (subIndex != 0U) return false;
            MC_Cia402Ext_CopyU32(value, size, s_cia402_max_profile_velocity);
            return true;
        case 0x6080U:
            if (subIndex != 0U) return false;
            MC_Cia402Ext_CopyU32(value, size, s_cia402_max_motor_speed);
            return true;
        case 0x6082U:
            if (subIndex != 0U) return false;
            MC_Cia402Ext_CopyU32(value, size, s_cia402_end_velocity);
            return true;
        case 0x6085U:
            if (subIndex != 0U) return false;
            MC_Cia402Ext_CopyU32(value, size, s_cia402_quick_stop_deceleration);
            return true;
        case 0x6086U:
            if (subIndex != 0U) return false;
            MC_Cia402Ext_CopyS16(value, size, s_cia402_motion_profile_type);
            return true;
        case 0x6087U:
            if (subIndex != 0U) return false;
            MC_Cia402Ext_CopyU32(value, size, s_cia402_torque_slope);
            return true;
        case 0x608FU:
            return MC_Cia402Ext_ReadSubIndexedPair(subIndex,
                                                   s_cia402_position_encoder_resolution,
                                                   value, size);
        case 0x6090U:
            return MC_Cia402Ext_ReadSubIndexedPair(subIndex,
                                                   s_cia402_velocity_encoder_resolution,
                                                   value, size);
        case 0x6091U:
            return MC_Cia402Ext_ReadSubIndexedPair(subIndex, s_cia402_gear_ratio, value, size);
        case 0x6092U:
            return MC_Cia402Ext_ReadSubIndexedPair(subIndex, s_cia402_feed_constant, value, size);
        case 0x6093U:
            return MC_Cia402Ext_ReadSubIndexedPair(subIndex, s_cia402_position_factor, value, size);
        case 0x6094U:
            return MC_Cia402Ext_ReadSubIndexedPair(subIndex, s_cia402_velocity_factor, value, size);
        case 0x6095U:
            return MC_Cia402Ext_ReadSubIndexedPair(subIndex, s_cia402_acceleration_factor, value, size);
        case 0x6098U:
            if (subIndex != 0U) return false;
            value[0] = (uint8_t)s_cia402_homing_method;
            *size = 1U;
            return true;
        case 0x6099U:
            return MC_Cia402Ext_ReadSubIndexedPair(subIndex, s_cia402_homing_speed, value, size);
        case 0x609AU:
            if (subIndex != 0U) return false;
            MC_Cia402Ext_CopyU32(value, size, s_cia402_homing_acceleration);
            return true;
        case 0x60B8U:
            if (subIndex != 0U) return false;
            MC_Cia402Ext_CopyU16(value, size, s_cia402_touch_probe_function);
            return true;
        case 0x60B9U:
            if (subIndex != 0U) return false;
            MC_Cia402Ext_CopyU16(value, size, s_cia402_touch_probe_status);
            return true;
        case 0x60BAU:
        case 0x60BBU:
        case 0x60BCU:
        case 0x60BDU:
            if (subIndex != 0U) return false;
            MC_Cia402Ext_CopyS32(value, size, s_cia402_touch_probe_value[index - 0x60BAU]);
            return true;
        case 0x60C0U:
            if (subIndex != 0U) return false;
            MC_Cia402Ext_CopyS16(value, size, s_cia402_interpolation_submode);
            return true;
        case 0x60C1U:
            if (subIndex == 0U)
            {
                value[0] = 4U;
                *size = 1U;
                return true;
            }
            if ((subIndex >= 1U) && (subIndex <= 4U))
            {
                MC_Cia402Ext_CopyS32(value, size, s_cia402_interpolation_data[subIndex - 1U]);
                return true;
            }
            return false;
        case 0x60C2U:
            if (subIndex == 1U)
            {
                value[0] = s_cia402_interpolation_time_value;
                *size = 1U;
                return true;
            }
            if (subIndex == 2U)
            {
                value[0] = (uint8_t)s_cia402_interpolation_time_index;
                *size = 1U;
                return true;
            }
            return false;
        case 0x60C4U:
            if (subIndex == 0U)
            {
                value[0] = 4U;
                *size = 1U;
                return true;
            }
            if ((subIndex >= 1U) && (subIndex <= 4U))
            {
                MC_Cia402Ext_CopyU32(value, size, s_cia402_interpolation_config[subIndex - 1U]);
                return true;
            }
            return false;
        case 0x60C5U:
            if (subIndex != 0U) return false;
            MC_Cia402Ext_CopyU32(value, size, s_cia402_max_acceleration);
            return true;
        case 0x60C6U:
            if (subIndex != 0U) return false;
            MC_Cia402Ext_CopyU32(value, size, s_cia402_max_deceleration);
            return true;
        case 0x60E0U:
            if (subIndex != 0U) return false;
            MC_Cia402Ext_CopyU16(value, size, s_cia402_positive_torque_limit);
            return true;
        case 0x60E1U:
            if (subIndex != 0U) return false;
            MC_Cia402Ext_CopyU16(value, size, s_cia402_negative_torque_limit);
            return true;
        case 0x60FDU:
            if (subIndex != 0U) return false;
            MC_Cia402Ext_CopyU32(value, size, s_cia402_digital_inputs);
            return true;
        case 0x60FEU:
            if (subIndex == 1U)
            {
                MC_Cia402Ext_CopyU32(value, size, s_cia402_physical_outputs);
                return true;
            }
            if (subIndex == 2U)
            {
                MC_Cia402Ext_CopyU32(value, size, s_cia402_output_mask);
                return true;
            }
            return false;
        case 0x6502U:
            if (subIndex != 0U) return false;
            MC_Cia402Ext_CopyU32(value, size, s_cia402_supported_drive_modes);
            return true;
        case 0x1200U:
            if (subIndex == 0U)
            {
                value[0] = 3U;
                *size = 1U;
                return true;
            }
            if (subIndex == 1U)
            {
                MC_Cia402Ext_CopyU32(value, size, s_cia402_sdo_rx_cobid);
                return true;
            }
            if (subIndex == 2U)
            {
                MC_Cia402Ext_CopyU32(value, size, s_cia402_sdo_tx_cobid);
                return true;
            }
            if (subIndex == 3U)
            {
                value[0] = s_cia402_sdo_node_id;
                *size = 1U;
                return true;
            }
            return false;
        case 0x1402U:
        case 0x1403U:
        case 0x1802U:
        case 0x1803U:
            return MC_Cia402Ext_ReadPdoComm(index, subIndex, value, size);
        case 0x1602U:
        case 0x1603U:
        case 0x1A02U:
        case 0x1A03U:
            return MC_Cia402Ext_ReadPdoMap(index, subIndex, value, size);
        default:
            temp32 = 0;
            (void)temp32;
            return false;
    }
}

bool MC_Cia402Ext_WriteObject(uint16_t index, uint8_t subIndex,
                              const uint8_t *value, uint8_t size)
{
    if (value == NULL)
    {
        return false;
    }

    switch (index)
    {
        case 0x6067U:
            if ((subIndex != 0U) || (size != 4U)) return false;
            (void)memcpy(&s_cia402_position_window, value, 4U);
            return true;
        case 0x6068U:
            if ((subIndex != 0U) || (size != 2U)) return false;
            (void)memcpy(&s_cia402_position_window_time, value, 2U);
            return true;
        case 0x606AU:
            if ((subIndex != 0U) || (size != 2U)) return false;
            (void)memcpy(&s_cia402_velocity_sensor_selection, value, 2U);
            return true;
        case 0x6075U:
            if ((subIndex != 0U) || (size != 4U)) return false;
            (void)memcpy(&s_cia402_motor_rated_current, value, 4U);
            return true;
        case 0x6076U:
            if ((subIndex != 0U) || (size != 4U)) return false;
            (void)memcpy(&s_cia402_motor_rated_torque, value, 4U);
            return true;
        case 0x607BU:
            if ((subIndex < 1U) || (subIndex > 2U) || (size != 4U)) return false;
            (void)memcpy(&s_cia402_position_range_limit[subIndex - 1U], value, 4U);
            return true;
        case 0x607CU:
            if ((subIndex != 0U) || (size != 4U)) return false;
            (void)memcpy(&s_cia402_home_offset, value, 4U);
            return true;
        case 0x607DU:
            if ((subIndex < 1U) || (subIndex > 2U) || (size != 4U)) return false;
            (void)memcpy(&s_cia402_software_position_limit[subIndex - 1U], value, 4U);
            return true;
        case 0x607EU:
            if ((subIndex != 0U) || (size != 1U)) return false;
            s_cia402_polarity = value[0];
            return true;
        case 0x607FU:
            if ((subIndex != 0U) || (size != 4U)) return false;
            (void)memcpy(&s_cia402_max_profile_velocity, value, 4U);
            return true;
        case 0x6080U:
            if ((subIndex != 0U) || (size != 4U)) return false;
            (void)memcpy(&s_cia402_max_motor_speed, value, 4U);
            return true;
        case 0x6082U:
            if ((subIndex != 0U) || (size != 4U)) return false;
            (void)memcpy(&s_cia402_end_velocity, value, 4U);
            return true;
        case 0x6085U:
            if ((subIndex != 0U) || (size != 4U)) return false;
            (void)memcpy(&s_cia402_quick_stop_deceleration, value, 4U);
            return true;
        case 0x6086U:
            if ((subIndex != 0U) || (size != 2U)) return false;
            (void)memcpy(&s_cia402_motion_profile_type, value, 2U);
            return true;
        case 0x6087U:
            if ((subIndex != 0U) || (size != 4U)) return false;
            (void)memcpy(&s_cia402_torque_slope, value, 4U);
            return true;
        case 0x608FU:
        case 0x6090U:
        case 0x6091U:
        case 0x6092U:
        case 0x6093U:
        case 0x6094U:
        case 0x6095U:
        {
            uint32_t *table = NULL;
            if ((subIndex < 1U) || (subIndex > 2U) || (size != 4U)) return false;
            switch (index)
            {
                case 0x608FU: table = s_cia402_position_encoder_resolution; break;
                case 0x6090U: table = s_cia402_velocity_encoder_resolution; break;
                case 0x6091U: table = s_cia402_gear_ratio; break;
                case 0x6092U: table = s_cia402_feed_constant; break;
                case 0x6093U: table = s_cia402_position_factor; break;
                case 0x6094U: table = s_cia402_velocity_factor; break;
                default:      table = s_cia402_acceleration_factor; break;
            }
            (void)memcpy(&table[subIndex - 1U], value, 4U);
            return true;
        }
        case 0x6098U:
            if ((subIndex != 0U) || (size != 1U)) return false;
            s_cia402_homing_method = (int8_t)value[0];
            return true;
        case 0x6099U:
            if ((subIndex < 1U) || (subIndex > 2U) || (size != 4U)) return false;
            (void)memcpy(&s_cia402_homing_speed[subIndex - 1U], value, 4U);
            return true;
        case 0x609AU:
            if ((subIndex != 0U) || (size != 4U)) return false;
            (void)memcpy(&s_cia402_homing_acceleration, value, 4U);
            return true;
        case 0x60B8U:
            if ((subIndex != 0U) || (size != 2U)) return false;
            (void)memcpy(&s_cia402_touch_probe_function, value, 2U);
            return true;
        case 0x60C0U:
            if ((subIndex != 0U) || (size != 2U)) return false;
            (void)memcpy(&s_cia402_interpolation_submode, value, 2U);
            return true;
        case 0x60C1U:
            if ((subIndex < 1U) || (subIndex > 4U) || (size != 4U)) return false;
            (void)memcpy(&s_cia402_interpolation_data[subIndex - 1U], value, 4U);
            return true;
        case 0x60C2U:
            if ((subIndex == 1U) && (size == 1U))
            {
                s_cia402_interpolation_time_value = value[0];
                return true;
            }
            if ((subIndex == 2U) && (size == 1U))
            {
                s_cia402_interpolation_time_index = (int8_t)value[0];
                return true;
            }
            return false;
        case 0x60C4U:
            if ((subIndex < 1U) || (subIndex > 4U) || (size != 4U)) return false;
            (void)memcpy(&s_cia402_interpolation_config[subIndex - 1U], value, 4U);
            return true;
        case 0x60C5U:
            if ((subIndex != 0U) || (size != 4U)) return false;
            (void)memcpy(&s_cia402_max_acceleration, value, 4U);
            return true;
        case 0x60C6U:
            if ((subIndex != 0U) || (size != 4U)) return false;
            (void)memcpy(&s_cia402_max_deceleration, value, 4U);
            return true;
        case 0x60E0U:
            if ((subIndex != 0U) || (size != 2U)) return false;
            (void)memcpy(&s_cia402_positive_torque_limit, value, 2U);
            return true;
        case 0x60E1U:
            if ((subIndex != 0U) || (size != 2U)) return false;
            (void)memcpy(&s_cia402_negative_torque_limit, value, 2U);
            return true;
        case 0x60FEU:
            if ((subIndex == 1U) && (size == 4U))
            {
                (void)memcpy(&s_cia402_physical_outputs, value, 4U);
                return true;
            }
            if ((subIndex == 2U) && (size == 4U))
            {
                (void)memcpy(&s_cia402_output_mask, value, 4U);
                return true;
            }
            return false;
        case 0x1200U:
            if ((subIndex == 1U) && (size == 4U))
            {
                (void)memcpy(&s_cia402_sdo_rx_cobid, value, 4U);
                return true;
            }
            if ((subIndex == 2U) && (size == 4U))
            {
                (void)memcpy(&s_cia402_sdo_tx_cobid, value, 4U);
                return true;
            }
            if ((subIndex == 3U) && (size == 1U))
            {
                s_cia402_sdo_node_id = value[0];
                return true;
            }
            return false;
        case 0x1402U:
        case 0x1403U:
        case 0x1802U:
        case 0x1803U:
        {
            uint8_t slot = (index == 0x1402U || index == 0x1802U) ? 0U : 1U;
            uint32_t *table = (index == 0x1402U || index == 0x1403U) ?
                              s_cia402_rpdo34_comm[slot] : s_cia402_tpdo34_comm[slot];
            if (subIndex == 1U && size == 4U)
            {
                (void)memcpy(&table[0], value, 4U);
                return true;
            }
            if (subIndex == 2U && size == 1U)
            {
                table[1] = value[0];
                return true;
            }
            if ((index == 0x1802U || index == 0x1803U) &&
                ((subIndex == 3U) || (subIndex == 5U)) && size == 2U)
            {
                uint16_t temp;
                (void)memcpy(&temp, value, 2U);
                table[subIndex - 1U] = temp;
                return true;
            }
            return false;
        }
        case 0x1602U:
        case 0x1603U:
        case 0x1A02U:
        case 0x1A03U:
        {
            uint8_t slot = (index == 0x1602U || index == 0x1A02U) ? 0U : 1U;
            uint32_t *table = (index == 0x1602U || index == 0x1603U) ?
                              s_cia402_rpdo34_map[slot] : s_cia402_tpdo34_map[slot];
            uint8_t *count = (index == 0x1602U || index == 0x1603U) ?
                             &s_cia402_rpdo34_map_count[slot] : &s_cia402_tpdo34_map_count[slot];
            if ((subIndex == 0U) && (size == 1U))
            {
                *count = value[0];
                return true;
            }
            if ((subIndex >= 1U) && (subIndex <= 8U) && (size == 4U))
            {
                (void)memcpy(&table[subIndex - 1U], value, 4U);
                return true;
            }
            return false;
        }
        default:
            return false;
    }
}
