# Encoder Switch Guide

## Scope

This document describes the encoder options currently adapted in this project and how they affect:

- SPI configuration
- DMA read mode
- native angle resolution
- FOC compatibility path
- speed and position calculation path

Current supported encoders:

- `KTH7824`
- `AS5047P`
- `MT6835`

## Selection Entry

Encoder model selection is configured in:

- `Core/Inc/main.h`

Use the following macros:

```c
#define BOARD_ENCODER_TYPE_KTH7824   0U
#define BOARD_ENCODER_TYPE_AS5047P   1U
#define BOARD_ENCODER_TYPE_MT6835    2U

#define MOTOR_ENCODER_TYPE           BOARD_ENCODER_TYPE_MT6835
#define LOAD_ENCODER_TYPE            MOTOR_ENCODER_TYPE
```

Examples:

```c
#define MOTOR_ENCODER_TYPE           BOARD_ENCODER_TYPE_KTH7824
#define LOAD_ENCODER_TYPE            BOARD_ENCODER_TYPE_KTH7824
```

```c
#define MOTOR_ENCODER_TYPE           BOARD_ENCODER_TYPE_AS5047P
#define LOAD_ENCODER_TYPE            BOARD_ENCODER_TYPE_AS5047P
```

```c
#define MOTOR_ENCODER_TYPE           BOARD_ENCODER_TYPE_MT6835
#define LOAD_ENCODER_TYPE            BOARD_ENCODER_TYPE_MT6835
```

## Architecture Summary

The project now uses a two-layer compatibility design.

### Layer 1: FOC compatibility angle

FOC continues to use a unified 16-bit angle domain:

- `ENCODER_FOC_COMPAT_COUNT = 65536`
- `g_axis.fbdk.uAngleRaw`

This keeps current FOC control logic stable when switching encoder types.

### Layer 2: Native encoder precision

Speed and position related logic use native encoder resolution:

- `g_axis.fbdk.uAngleRawNative`
- `Get_Angle_RawNative()`
- `Get_Angle_CountNative()`
- `g_axis.posCtrl.uOffsetAngleRawNative`

This allows `MT6835` to keep its high-resolution advantage without forcing a full FOC rewrite.

## Encoder Comparison

| Encoder | SPI Mode | Transfer Width | Native Counts | Native Precision | FOC Input | Speed/Position Path |
|---|---|---:|---:|---|---|---|
| `KTH7824` | Mode 3 (`CPOL=1`, `CPHA=1`) | 16-bit, 1 word DMA | `65536` | 16-bit | direct compat | native = compat |
| `AS5047P` | Mode 1 (`CPOL=0`, `CPHA=1`) | 16-bit, 1 word DMA | `16384` | 14-bit | mapped to 16-bit compat | native 14-bit |
| `MT6835` | Mode 3 (`CPOL=1`, `CPHA=1`) | 8-bit, 6 byte DMA | `2097152` | 21-bit | mapped to 16-bit compat | native 21-bit |

## Per-Encoder Notes

### KTH7824

- SPI mode: `Mode 3`
- Native count: `65536`
- Since native count already matches the project compat count, this encoder is the simplest case.
- FOC angle, speed estimation, and position calculations are naturally aligned.

Practical impact:

- No precision loss from compat mapping
- Good baseline for regression comparison

### AS5047P

- SPI mode: `Mode 1`
- Native count: `16384`
- Read by 16-bit DMA transaction
- Angle is expanded into the common 16-bit compat domain for FOC

Practical impact:

- FOC still receives a stable 16-bit normalized input
- Native angle precision is lower than `KTH7824` and `MT6835`
- Speed and position calculations use the true native count `16384`

### MT6835

- SPI mode: `Mode 3`
- Native count: `2097152`
- Read by 6-byte DMA burst
- FOC uses 16-bit normalized compat angle
- Speed and position logic use native 21-bit angle

Practical impact:

- FOC migration risk stays low
- Low-speed speed estimation and position resolution are improved
- Full 21-bit precision is preserved for native-side calculations

## Current Project Behavior

### FOC path

The following path still uses compat angle:

- `g_axis.fbdk.uAngleRaw`
- `Get_Encoder_Raw()`
- `Get_Encoder_RawCompat()`
- `ENCODER_COUNT = 65536`

This means switching to another adapted encoder should not require large changes in the FOC core.

### Native position path

The following path uses native angle:

- `Get_Angle()`
- `Get_Angle_RawNative()`
- `Get_Angle_CountNative()`
- `g_axis.posCtrl.uOffsetAngleRawNative`

This means position calibration and electrical angle reconstruction use real encoder resolution.

### Native speed path

The following path uses native angle:

- `Sensor_Update_Kalman()`

Speed is computed from native angle delta and native wrap count, which is especially useful for:

- `MT6835`
- low-speed estimation
- reduced quantization effect

### Parameter identification lock check

The following path uses native angle:

- lock angle start point
- lock drift delta comparison

The existing threshold `lockMaxAngleDeltaRaw` is still configured in the old compat scale, but runtime automatically converts it to native counts.

This keeps configuration meaning stable while improving accuracy on high-resolution encoders.

## Files Involved

Main selection and board configuration:

- `Core/Inc/main.h`

Encoder dispatch layer:

- `Core/Inc/MotorControl/Fbdk/encoder.h`
- `Core/Src/MotorControl/Fbdk/encoder.c`

Per-encoder drivers:

- `Core/Inc/Board/KTH7824/kth7824.h`
- `Core/Src/Board/KTH7824/kth7824.c`
- `Core/Inc/Board/AS5047P/as5047p.h`
- `Core/Src/Board/AS5047P/as5047p.c`
- `Core/Inc/Board/MT6835/mt6835.h`
- `Core/Src/Board/MT6835/mt6835.c`

Speed and position feedback:

- `Core/Inc/MotorControl/Fbdk/speed_pos_type.h`
- `Core/Inc/MotorControl/Fbdk/speed_pos_fbdk.h`
- `Core/Src/MotorControl/Fbdk/speed_pos_fbdk.c`

Offset calibration and motor control init:

- `Core/Src/MotorControl/Tasks/mc_tasks.c`
- `Core/Src/MotorControl/Control/motor_control.c`

Parameter identification:

- `Core/Src/MotorControl/Control/param_identify.c`

## Switching Procedure

### Switch to another already adapted encoder

Only do the following:

1. Change `MOTOR_ENCODER_TYPE` and `LOAD_ENCODER_TYPE` in `Core/Inc/main.h`
2. Rebuild and flash
3. Verify SPI communication and angle update
4. Re-run offset calibration if required

No large control-layer refactor should be needed.

### Add a new encoder model in the future

Recommended steps:

1. Add a dedicated driver under `Core/Inc/Board/...` and `Core/Src/Board/...`
2. Implement:
   - init
   - DMA start
   - DMA complete decode
   - native count definition
3. Add dispatch support in `encoder.c`
4. Keep FOC compat mapping to 16-bit unless a full FOC precision migration is planned

## Validation Checklist

After switching encoder type, verify:

1. SPI communication is normal
2. `g_axis.fbdk.uAngleRaw` updates normally
3. `g_axis.fbdk.uAngleRawNative` updates normally
4. `g_axis.fbdk.fAngle` is continuous
5. `g_axis.fbdk.fSpeedKalman` is stable at low speed
6. Offset calibration completes successfully
7. Closed-loop FOC can enter run state

## Recommendation

For current project evolution:

1. Keep FOC electrical angle path on 16-bit compat
2. Keep speed and position path on native angle
3. Use `MT6835` when low-speed smoothness and position precision are important
4. Use `KTH7824` as a simple regression reference
5. Use `AS5047P` when existing hardware already depends on its SPI mode and protocol
