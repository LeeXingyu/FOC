# CAN Communication Guide

## 1. Overview

This document describes the current CAN / CAN FD communication implementation in this project.

Primary reference files:

- `Core/Inc/Communication/mcp2518fd/canopen.h`
- `Core/Src/Communication/mcp2518fd/canopen.c`
- `Core/Src/Communication/mcp2518fd/can_telemetry.c`
- `Core/Src/app_freertos.c`
- `Core/Src/MotorControl/Control/param_identify.c`

The project uses `MCP2518FD` as the external CAN controller.

Current protocol characteristics:

- standard 11-bit arbitration ID
- ID contains both function code and node ID
- command function codes stay fixed
- telemetry mapping is selected by compile-time macro
- `APP_USE_CAN_FD = 0`: classical CAN telemetry layout
- `APP_USE_CAN_FD = 1`: CAN FD telemetry layout

## 2. CAN ID Format

### 2.1 Arbitration ID layout

Current macro definitions:

```c
#define CAN_NODE_ID_BITS             4U
#define CAN_NODE_ID_MASK             0x0FU
#define CAN_FUNCTION_CODE_SHIFT      CAN_NODE_ID_BITS
#define CAN_MAKE_ID(func, node)      ((((uint16_t)(func)) << 4) | ((uint16_t)(node) & 0x0F))
```

Meaning:

- `SID[10:4]`: function code
- `SID[3:0]`: node ID

Equivalent formula:

```c
SID = (func << 4) | node
```

### 2.2 Node ID range

Current node ID range:

- `0 ~ 15`

Default node ID:

- `1`

The runtime node ID is maintained by:

- `ParamId_GetCanNodeId()`
- `ParamId_SaveCanNodeIdToFlash()`

The node ID is stored to flash together with motor-related parameters.

## 3. CAN / CAN FD Compile-Time Switch

The protocol behavior is selected by:

```c
#ifndef APP_USE_CAN_FD
#define APP_USE_CAN_FD 1
#endif
```

Current macro behavior in `canopen.h`:

### 3.1 When `APP_USE_CAN_FD = 1`

- `APP_CAN_FRAME_FDF = 1`
- `APP_CAN_FRAME_BRS = 1`
- `APP_CAN_MAX_DATA_BYTES = 16`
- `APP_CAN_RX_FETCH_BYTES = 16`
- `APP_CAN_BITTIME_SETUP = CAN_500K_2M`

Meaning:

- arbitration bit rate: `500 kbps`
- data bit rate: `2 Mbps`

### 3.2 When `APP_USE_CAN_FD = 0`

- `APP_CAN_FRAME_FDF = 0`
- `APP_CAN_FRAME_BRS = 0`
- `APP_CAN_MAX_DATA_BYTES = 8`
- `APP_CAN_RX_FETCH_BYTES = 8`
- `APP_CAN_BITTIME_SETUP = CAN_500K_2M`

Meaning:

- only classical CAN frame format is used
- nominal bit timing remains `500 kbps`

## 4. Communication Thread Flow

The communication thread is `Communication_Task()` in `app_freertos.c`.

When `g_system_comm_mode == COMM_PROTO_CAN`, the loop executes:

1. `CAN_Telemetry_Service1ms()`
2. `MCP2518FD_Service1ms()`
3. if `g_comm_int_irq_pending != 0`, clear the flag and call `MCP2518FD_ProcessRxIrq()`
4. call `CAN_Telemetry_Service1ms()` once more after RX handling

Important behavior:

- interrupt only notifies
- actual SPI access is done in thread context
- TX queue has higher priority than periodic telemetry
- RX-handled command status can be sent in the same thread iteration after being queued

## 5. MCP2518FD Initialization

`CANFD_INIT()` currently performs:

- device reset
- ECC enable
- RAM initialization
- module configuration
- TX FIFO configuration
- RX FIFO configuration
- node filter configuration
- broadcast `GET_ID` filter configuration
- bit timing configuration
- interrupt GPIO configuration
- RX event enable
- switch to normal mode

Key settings:

- TX FIFO: `CAN_FIFO_CH2`
- RX FIFO: `CAN_FIFO_CH1`
- TX payload size: `APP_CAN_TX_FIFO_PAYLOAD_SIZE`
- RX payload size: `APP_CAN_RX_FIFO_PAYLOAD_SIZE`
- bit timing: `APP_CAN_BITTIME_SETUP`

## 6. Receive Filter Strategy

Two hardware receive filters are used.

### 6.1 Filter0: local node command filter

Purpose:

- accept frames targeted to the current local node

Matching rule:

- compare `SID[3:0]`
- lower 4 bits must equal current node ID

### 6.2 Filter1: broadcast `GET_ID`

Purpose:

- support single-device commissioning when the host does not know the node ID

Matching frame:

- `CAN_MAKE_ID(CAN_FC_GET_ID, 0)`
- example: `0x0E0`

This is intended for single-device use only.

## 7. Command Protocol

Receive-side dispatch uses:

```c
func = CAN_GET_FUNC(sid)
node = CAN_GET_NODE(sid)
```

Command matching is based on function code.

### 7.1 Command function codes

| Function code | Example SID for node `1` | Command | Payload length |
| --- | --- | --- | --- |
| `0x01` | `0x011` | stop motor | `0` |
| `0x02` | `0x021` | start motor | `0` |
| `0x03` | `0x031` | switch to speed mode | `0` |
| `0x04` | `0x041` | switch to position mode | `0` |
| `0x05` | `0x051` | switch to open loop / VF mode | `0` |
| `0x06` | `0x061` | set speed reference | `4` |
| `0x07` | `0x071` | set speed Kp | `4` |
| `0x08` | `0x081` | set speed Ki | `4` |
| `0x09` | `0x091` | set pole pairs | `1` |
| `0x0A` | `0x0A1` | start calibration | `1` |
| `0x0B` | `0x0B1` | stop calibration | `0` |
| `0x0C` | `0x0C1` | read flash params | `0` |
| `0x0D` | `0x0D1` | clear flash | `0` |
| `0x0E` | `0x0E1` | get node ID | `0` |
| `0x0F` | `0x0F1` | set node ID | `1` |

### 7.2 Command payload rules

`SET_REF_SPEED`, `SET_SPEED_KP`, `SET_SPEED_KI`

- 4-byte little-endian IEEE754 `float`

`SET_POLE_PAIRS`

- 1 byte
- `0` is invalid

`CALIB_START`

- 1 byte mode

Supported values:

- `0`: full chain
- `5`: parameter identification full flow

`SET_ID`

- 1 byte new node ID
- valid range: `0 ~ 15`

`GET_ID`

- no payload
- broadcast `E0` is accepted by dedicated hardware filter

### 7.3 Command status return values

| Value | Meaning |
| --- | --- |
| `0` | `OK` |
| `1` | `BAD_LEN` |
| `2` | `BAD_ARG` |
| `3` | `BAD_STATE` |
| `4` | `BUSY` |
| `5` | `UNKNOWN` |

## 8. Response Frames

Response function codes are always preserved, independent of `CAN` or `CANFD`.

| Function code | Example SID for node `1` | Meaning |
| --- | --- | --- |
| `0x20` | `0x201` | command status response |
| `0x21` | `0x211` | parameter state |
| `0x22` | `0x221` | parameter result block 1 |
| `0x23` | `0x231` | parameter result block 2, classical CAN only |

### 8.1 Command status response `0x20x`

Current payload definition:

| Byte | Meaning |
| --- | --- |
| `0..1` | original command SID, little-endian |
| `2` | command status |
| `3` | current `g_axis.state` |
| `4` | extra field |
| `5` | parameter identification state |
| `6` | received payload length |
| `7` | current pole-pair count |

When `APP_USE_CAN_FD = 1`, extra bytes are appended:

| Byte | Meaning |
| --- | --- |
| `8` | current node ID |
| `9` | `raw_COMM_ID` |
| `10..11` | `raw_TSENA` |

So:

- classical CAN: `8 B`
- CAN FD: `12 B`

For `GET_ID` / `SET_ID`:

- `extra` returns the node ID value

Broadcast `E0` rule:

- request SID: `0x0E0`
- response SID uses real local node ID, for example `0x205`

### 8.2 Parameter state response `0x21x`

Payload:

| Byte | Meaning |
| --- | --- |
| `0` | current axis state |
| `1` | parameter state |
| `2` | pole pairs |
| `3` | validity bits |
| `4..5` | pole pairs as `uint16_t` |

When `APP_USE_CAN_FD = 1`, extra bytes are appended:

| Byte | Meaning |
| --- | --- |
| `6` | node ID |
| `7` | node mask (`0x0F`) |
| `8` | FDF flag |
| `9` | BRS flag |

So:

- classical CAN: `8 B`
- CAN FD: `12 B`

Validity bits:

- bit0: `Rs` valid
- bit1: `Ld` valid
- bit2: `Lq` valid
- bit3: `Ke` valid

### 8.3 Parameter result response

Classical CAN:

- `0x22x`: `Rs`, `Ld`
- `0x23x`: `Lq`, `Ke`

CAN FD:

- `0x22x`: `Rs`, `Ld`, `Lq`, `Ke` in one `16 B` frame
- `0x23x` is not used for periodic parameter result output in CAN FD mode

## 9. Periodic Telemetry Mapping

The telemetry mapping is different for `CANFD` and classical `CAN`.

### 9.1 CAN FD telemetry mapping

When `APP_USE_CAN_FD = 1`:

| Function code | Example SID for node `4` | Payload length | Meaning |
| --- | --- | --- | --- |
| `0x30` | `0x304` | `16 B` | FOC full data |
| `0x31` | `0x314` | `8 B` | status |
| `0x32` | `0x324` | `16 B` | speed + bus voltage + temperature |
| `0x33` | `0x334` | `12 B` | temperature group |

#### `0x30x` FOC frame

Payload:

| Byte | Meaning |
| --- | --- |
| `0..3` | `refId` |
| `4..7` | `refIq` |
| `8..11` | `calcId` |
| `12..15` | `calcIq` |

#### `0x31x` status frame

Payload:

| Byte | Meaning |
| --- | --- |
| `0` | `g_axis.state` |
| `1` | `g_axis.error` |
| `2` | `g_axis.enCtrlMode` |
| `3` | `raw_COMM_ID` |
| `4..5` | `raw_TSENA` |
| `6` | node ID |
| `7` | pole pairs |

#### `0x32x` speed / power frame

Payload:

| Byte | Meaning |
| --- | --- |
| `0..3` | `speedRef` |
| `4..7` | `speedMeas` |
| `8..11` | `busVoltage` |
| `12..15` | `temp_TSENB_c` |

#### `0x33x` temperature frame

Payload:

| Byte | Meaning |
| --- | --- |
| `0..3` | `temp_TSENC_c` |
| `4..7` | `temp_TSENA_c` |
| `8..11` | `temp_TSENB_c` |

### 9.2 Classical CAN telemetry mapping

When `APP_USE_CAN_FD = 0`:

| Function code | Example SID for node `4` | Payload length | Meaning |
| --- | --- | --- | --- |
| `0x30` | `0x304` | `6 B` | status |
| `0x31` | `0x314` | `8 B` | current reference |
| `0x32` | `0x324` | `8 B` | current calculation |
| `0x33` | `0x334` | `8 B` | speed |
| `0x34` | `0x344` | `8 B` | bus voltage + TSENB |
| `0x35` | `0x354` | `8 B` | TSENC + TSENA |

#### `0x30x` status frame

Payload:

| Byte | Meaning |
| --- | --- |
| `0` | `g_axis.state` |
| `1` | `g_axis.error` |
| `2` | `g_axis.enCtrlMode` |
| `3` | `raw_COMM_ID` |
| `4..5` | `raw_TSENA` |

#### `0x31x` current reference frame

- `refId`
- `refIq`

#### `0x32x` current calculation frame

- `calcId`
- `calcIq`

#### `0x33x` speed frame

- `speedRef`
- `speedMeas`

#### `0x34x` bus / temperature B frame

- `busVoltage`
- `temp_TSENB_c`

#### `0x35x` temperature frame

- `temp_TSENC_c`
- `temp_TSENA_c`

## 10. Periodic Scheduler

`CAN_Telemetry_Service1ms()` behavior:

1. if TX queue is not empty, send one queued response frame and return
2. if TX queue is empty, update periodic elapsed counters
3. scan telemetry slots in configured order
4. if a slot is due, build one frame, send it, clear its elapsed counter, and return

Important point:

- in one `1 ms` service call, at most one frame is sent

### 10.1 Period configuration

Current period constants:

- current reference: `20 ms`
- current calculation: `20 ms`
- speed: `20 ms`
- status: `100 ms`
- bus / temperature group: `200 ms`
- temperature slow group: `1000 ms`

### 10.2 Actual slot usage

CAN FD mode uses:

- `0x30x` every `20 ms`
- `0x32x` every `20 ms`
- `0x31x` every `100 ms`
- `0x33x` every `200 ms`

Classical CAN mode uses:

- `0x31x` every `20 ms`
- `0x32x` every `20 ms`
- `0x33x` every `20 ms`
- `0x30x` every `100 ms`
- `0x34x` every `200 ms`
- `0x35x` every `1000 ms`

## 11. ID Read / Write Behavior

### 11.1 `GET_ID`

Function code:

- `0x0E`

Targeted query:

- send to `0x0Ex`

Broadcast query:

- send to `0x0E0`

Reply:

- response uses `0x20x`
- response `extra` returns current node ID

### 11.2 `SET_ID`

Function code:

- `0x0F`

Payload:

- `1 B` new node ID

Behavior:

1. validate node ID range
2. save to flash
3. reconfigure hardware RX filter to new node ID
4. queue command status response

## 12. Flash and Calibration

Current flash-related functions:

- `ParamId_SaveToFlash()`
- `ParamId_LoadFromFlash()`
- `ParamId_ClearFlash()`
- `ParamId_RestoreFromFlashToAxis()`

Flash content includes:

- identified motor parameters
- pole pairs
- current control related parameters
- CAN node ID

Calibration-related commands:

- `0x0A`: start calibration
- `0x0B`: stop calibration
- `0x0C`: read flash parameters
- `0x0D`: clear flash

## 13. Practical Summary

Current design summary:

- node ID uses lower 4 bits of standard 11-bit ID
- command function codes are fixed and shared by CAN / CAN FD
- response function codes `0x20 ~ 0x23` are fixed
- telemetry function codes differ by compile-time mode
- CAN FD tries to pack same-type runtime data into fewer larger frames
- classical CAN keeps split runtime telemetry frames
- TX queue is always sent before periodic telemetry
- broadcast `E0` is reserved for single-device ID discovery
- current CAN FD timing is `500 kbps` arbitration + `2 Mbps` data

## 14. Current Host Parsing Recommendation

### 14.1 When `APP_USE_CAN_FD = 1`

Monitor:

- `0x20x`: command response
- `0x21x`: parameter state
- `0x22x`: parameter result
- `0x30x`: FOC
- `0x31x`: status
- `0x32x`: speed / power
- `0x33x`: temperature

### 14.2 When `APP_USE_CAN_FD = 0`

Monitor:

- `0x20x`
- `0x21x`
- `0x22x`
- `0x23x`
- `0x30x`
- `0x31x`
- `0x32x`
- `0x33x`
- `0x34x`
- `0x35x`

This document should be updated whenever one of the following changes:

- function code allocation
- telemetry payload layout
- CAN / CAN FD mode mapping
- command payload definitions
- flash structure version
- bit timing configuration
