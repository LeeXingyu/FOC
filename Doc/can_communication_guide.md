# CAN Communication Guide

## 1. Overview

This document describes the current CAN communication content and runtime flow in this project. It is based on the actual implementation in:

- `Core/Src/Communication/mcp2518fd/canopen.c`
- `Core/Src/Communication/mcp2518fd/can_telemetry.c`
- `Core/Src/app_freertos.c`
- `Core/Src/MotorControl/Control/param_identify.c`
- `Core/Src/MotorControl/Tasks/mc_tasks.c`
- `Core/Src/main.c`

Current communication chip is `MCP2518FD`. The project currently uses it in a CAN protocol handling framework, with RX command parsing in `canopen.c` and all TX uplink frames funneled through `can_telemetry.c`.

## 2. Overall Architecture

Current communication flow is:

1. `MCP2518FD` receives a CAN frame.
2. `COMM_INT` pin generates an external interrupt.
3. `HAL_GPIO_EXTI_Callback()` only sets `g_comm_int_irq_pending = 1`.
4. `Communication_Task()` polls that pending flag every 1 ms.
5. When the flag is set, the thread calls `MCP2518FD_ProcessRxIrq()`.
6. `MCP2518FD_ProcessRxIrq()` reads RX FIFO by SPI and dispatches the command.
7. Command execution result is packed as a status response and queued for transmission.
8. `CAN_Telemetry_Service1ms()` sends queued response frames first, then sends periodic telemetry frames when the queue is empty.

Core characteristic:

- Interrupt is only used for notification.
- Actual MCP2518FD register/FIFO access is done in the communication thread.
- Outgoing frames are centralized to one service path, which avoids scattered direct TX behavior.

## 3. Thread and Interrupt Flow

### 3.1 Communication thread

`Communication_Task()` runs with `osDelay(1)`, so the service cadence is approximately 1 ms.

When `g_system_comm_mode == COMM_PROTO_CAN`, the thread executes:

1. `CAN_Telemetry_Service1ms()`
2. `MCP2518FD_Service1ms()`
3. Check `g_comm_int_irq_pending`
4. If pending, clear the flag and call `MCP2518FD_ProcessRxIrq()`

This means the communication thread is responsible for:

- periodic telemetry scheduling
- queued response transmission
- parameter state change reporting
- actual RX IRQ servicing

### 3.2 External interrupt path

`COMM_INT_Pin` is configured as falling-edge EXTI input in `gpio.c`.

After MCP2518FD RX FIFO has data:

1. MCP2518FD asserts INT
2. MCU EXTI enters `HAL_GPIO_EXTI_Callback()`
3. callback sets `g_comm_int_irq_pending = 1U`
4. `Communication_Task()` later handles the actual SPI read

This design reduces work inside ISR and keeps SPI transaction out of interrupt context.

## 4. MCP2518FD Initialization Behavior

`CANFD_INIT()` performs the following main configuration:

- chip reset
- ECC enable
- RAM init
- module configure
- TX FIFO configure
- RX FIFO configure
- filter/mask configure
- bit timing configure
- interrupt GPIO mode configure
- RX event enable
- normal mode select

Current key settings:

- TX FIFO: `CAN_FIFO_CH2`
- RX FIFO: `CAN_FIFO_CH1`
- RX event: `CAN_RX_FIFO_NOT_EMPTY_EVENT`
- module RX interrupt enabled
- bit timing configured by:
  - `DRV_CANFDSPI_BitTimeConfigure(..., CAN_1000K_4M, CAN_SSP_MODE_AUTO, CAN_SYSCLK_40M)`

Current transmit frame object in `MCP2518FD_TransmitMessageQueue()` is configured as:

- `IDE = 0`
- `RTR = 0`
- `BRS = 0`
- `FDF = 0`

So the current project transmit path is still sending standard CAN-format frames on the software side. Even though the chip is `MCP2518FD`, whether the bus behaves as classical CAN or CAN FD depends on these frame control bits and the matching upper-computer bitrate settings.

## 5. Receive Command Protocol

## 5.1 Command IDs

Current RX commands are:

| SID | Function | Payload length |
| --- | --- | --- |
| `0x101` | start motor | `0` |
| `0x102` | stop motor | `0` |
| `0x103` | set speed Kp | `4` |
| `0x104` | set speed Ki | `4` |
| `0x105` | set speed reference | `4` |
| `0x106` | switch to speed mode | `0` |
| `0x107` | switch to position mode | `0` |
| `0x108` | switch to open-loop / VF mode | `0` |
| `0x109` | set pole pairs | `1` |
| `0x10A` | start calibration | `1` |
| `0x10B` | stop calibration | `0` |
| `0x10C` | clear flash calibration data | `0` |
| `0x10D` | read motor params from flash | `0` |

## 5.2 Payload format

For `0x103`, `0x104`, `0x105`:

- payload is 4-byte `float`
- byte order is little-endian

For `0x109`:

- payload is 1 byte pole-pair count
- value `0` is invalid

For `0x10A`:

- payload is 1 byte calibration mode

Current supported values:

- `0`: start full calibration chain from `IDLE`
- `5`: start full parameter calibration in run context

Other values are rejected as invalid argument.

## 5.3 Command validation

Command dispatch checks:

- SID match
- expected payload length
- state legality
- argument range
- calibration busy state

Current status values returned by command response:

| Value | Meaning |
| --- | --- |
| `0` | `OK` |
| `1` | `BAD_LEN` |
| `2` | `BAD_ARG` |
| `3` | `BAD_STATE` |
| `4` | `BUSY` |
| `5` | `UNKNOWN` |

## 6. Response and Uplink Frames

All command responses and parameter snapshots are queued first, then sent by `CAN_Telemetry_Service1ms()`.

## 6.1 Response frame IDs

| SID | Function |
| --- | --- |
| `0x181` | command execution status |
| `0x182` | parameter state and validity |
| `0x183` | parameter result block 1: `Rs`, `Ld` |
| `0x184` | parameter result block 2: `Lq`, `Ke` |

### 0x181 command status frame

Payload layout:

| Byte | Meaning |
| --- | --- |
| 0..1 | original RX command SID, little-endian |
| 2 | command status |
| 3 | current `g_axis.state` |
| 4 | extra field, currently usually `0` |
| 5 | current parameter-calibration state |
| 6 | received payload length |
| 7 | current pole-pair count |

### 0x182 parameter state frame

Payload layout:

| Byte | Meaning |
| --- | --- |
| 0 | current `g_axis.state` |
| 1 | parameter state |
| 2 | pole pairs |
| 3 | validity bits |
| 4..5 | pole pairs packed as `uint16_t` |
| 6..7 | reserved |

Validity bits:

- bit0: `Rs` valid
- bit1: `Ld` valid
- bit2: `Lq` valid
- bit3: `Ke` valid

### 0x183 / 0x184 parameter result frames

Frame data is packed as little-endian IEEE754 `float`:

- `0x183`: `Rs`, `Ld`
- `0x184`: `Lq`, `Ke`

## 6.2 Periodic telemetry frame IDs

Current periodic telemetry frame IDs are:

| SID | Content |
| --- | --- |
| `0x201` | axis state, error, ctrl mode, comm id, raw TSENA |
| `0x202` | current reference `Id`, `Iq` |
| `0x203` | measured current `Id`, `Iq` |
| `0x204` | speed reference, measured speed |
| `0x205` | bus voltage, TSENB temperature |
| `0x206` | TSENC temperature, TSENA temperature |

### Periodic frame content details

`0x201`

- byte0: `g_axis.state`
- byte1: `g_axis.error`
- byte2: `g_axis.enCtrlMode`
- byte3: `raw_COMM_ID`
- byte4..5: `raw_TSENA`

`0x202`

- float `refId`
- float `refIq`

`0x203`

- float `calcId`
- float `calcIq`

`0x204`

- float `speedRef`
- float `speedMeas`

`0x205`

- float `busVoltage`
- float `temp_TSENB_c`

`0x206`

- float `temp_TSENC_c`
- float `temp_TSENA_c`

## 7. Telemetry Transmission Timing

Current telemetry scheduler is in `CAN_Telemetry_Service1ms()`.

Behavior:

1. If TX queue is not empty, send one queued frame immediately and return.
2. If TX queue is empty, execute periodic scheduler.
3. Periodic scheduler sends one telemetry frame every `100 ms`.

Current constant:

- `CAN_TELEMETRY_PERIOD_MS = 100`

Important implication:

- one telemetry frame is sent every 100 ms
- there are 6 periodic frames total
- a full telemetry round therefore takes about `600 ms`

So the per-frame rate and full-cycle rate are different:

- single frame update interval: `100 ms`
- full 6-frame refresh cycle: `600 ms`

Because queued frames have higher priority, command responses and parameter snapshots can temporarily delay periodic telemetry.

## 8. Calibration and Flash Flow

## 8.1 State chain

Current motor state path is:

`IDLE -> OFFSET_CALIB -> ENCODER_CALIB -> PARAM_CALIB -> RUN`

`High_Frequency_Task()` handles state execution according to `g_axis.state`.

## 8.2 Calibration trigger

Calibration is triggered by CAN command `0x10A`.

Supported behaviors:

- `data[0] = 0`: start full chain from idle
- `data[0] = 5`: start full parameter calibration

`0x10B` is used to stop parameter calibration.

## 8.3 Flash save and restore

Current flash-related functions are:

- `ParamId_SaveToFlash()`
- `ParamId_LoadFromFlash()`
- `ParamId_ClearFlash()`
- `ParamId_RestoreFromFlashToAxis()`

Current design is:

1. calibration finishes
2. result is marked pending for flash save
3. `ParamId_ModuleBackgroundService()` performs actual flash write later
4. background service is called from `Medium_Frequency_Task()`

So flash write is deferred, not done directly inside the high-frequency path.

Boot restore logic:

1. project initializes default compile-time parameters first
2. then tries `ParamId_RestoreFromFlashToAxis()`
3. if flash data is valid, restore identified values into `Axis_t`
4. if flash data is invalid or absent, keep compile-time defaults

This avoids repeated flash reads in later control calls because runtime values are synchronized into `g_axis` at boot.

## 9. Notes About Current CAN Sending Strategy

Current project strategy is:

- RX parse in `canopen.c`
- TX unify in `can_telemetry.c`

This has several practical advantages:

- all outbound traffic has one scheduling出口
- command response and periodic telemetry no longer compete via scattered direct TX calls
- easier to control upload rate
- easier to insert new response frames later

Current TX queue depth:

- `CAN_TX_QUEUE_DEPTH = 16`

If many responses are generated in a short time, periodic telemetry may be postponed until the queue drains.

## 10. Debug Suggestions

When checking CAN behavior, debug in this order:

1. confirm `g_system_comm_mode == COMM_PROTO_CAN`
2. confirm `Communication_Task()` is running every 1 ms
3. confirm `COMM_INT` pin really toggles on RX
4. confirm `g_comm_int_irq_pending` is being set in `HAL_GPIO_EXTI_Callback()`
5. confirm `MCP2518FD_ProcessRxIrq()` is entered
6. confirm RX SID and DLC are accepted by `Can_DispatchBySid()`
7. confirm response frames are enqueued
8. confirm `CAN_Telemetry_Service1ms()` is draining the queue

If upper computer sees periodic data but command does not work, priority checks are:

- wrong SID
- wrong DLC
- invalid float payload endian/order
- axis state not matching command requirement
- calibration already busy

If command is accepted but no parameter data returns, priority checks are:

- flash has not been written yet
- flash content is invalid by magic/version/CRC
- queue is full and response frame was not enqueued

## 11. Practical Summary

Current project CAN behavior can be summarized as:

- reception is interrupt-notified, thread-processed
- transmission is telemetry-service centralized
- command frames use `0x101` to `0x10D`
- command response uses `0x181`
- parameter snapshot uses `0x182` to `0x184`
- periodic runtime telemetry uses `0x201` to `0x206`
- periodic telemetry cadence is one frame per `100 ms`
- full telemetry cycle is about `600 ms`
- calibration result is stored to flash by deferred background service
- boot restores valid flash parameters back into `Axis_t`

This document should be updated together with:

- CAN SID changes
- telemetry payload changes
- calibration state-machine changes
- flash struct version changes
- MCP2518FD bit timing or frame mode changes
