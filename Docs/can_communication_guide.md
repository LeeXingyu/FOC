# CAN Communication Guide

## 1. Scope

This document describes the current MCP2518FD CAN/CAN FD implementation.
The primary control protocol is now standard CANopen with a CiA 402 device
profile. The older function-code protocol remains in the firmware as a
diagnostic and compatibility path, but it is not the primary CiA 402
interface.

The default build uses standard CANopen only:

```c
#define APP_USE_LEGACY_CAN_PROTOCOL 0U
```

Set this switch to `1U` only for a compatibility build that must also accept
the older function-code commands and periodic telemetry.

Primary implementation files:

- `Core/Inc/Communication/mcp2518fd/canopen.h`
- `Core/Src/Communication/mcp2518fd/canopen.c`
- `Core/Src/Communication/mcp2518fd/can_telemetry.c`
- `Core/Src/MotorControl/Core/mc_interface.c`
- `Core/Src/app_freertos.c`

The MCP2518FD is an external CAN controller connected over SPI. The firmware
uses standard 11-bit CAN identifiers for CANopen frames.

## 2. CAN and CAN FD Configuration

The compile-time switch is defined in
`Core/Inc/Communication/mcp2518fd/canopen.h`:

```c
#ifndef APP_USE_CAN_FD
#define APP_USE_CAN_FD 0
#endif
```

When `APP_USE_CAN_FD = 0`:

- Classical CAN frame format is used.
- Maximum payload is 8 bytes.
- Nominal bit rate is configured by `APP_CAN_BITTIME_SETUP`.

When `APP_USE_CAN_FD = 1`:

- CAN FD frame format is used.
- Bit rate switching is enabled.
- Maximum payload is 16 bytes.
- The configured timing is nominal 500 kbps and data 2 Mbps.

The CANopen CiA 402 objects and COB-IDs use the same standard identifier
values in both modes. CAN FD only changes the available payload size and the
legacy telemetry layout.

`APP_USE_LEGACY_CAN_PROTOCOL = 0U` suppresses the old `0x301..0x351`
telemetry and command-status frames. In this default mode, `0x300 + NodeID`
is reserved for standard RPDO2 and must not be interpreted as telemetry.
The firmware transmit path also rejects any outgoing identifier that is not
the configured EMCY, TPDO1, TPDO2, SDO response, or heartbeat COB-ID.

## 3. Node ID

The CANopen node ID is stored by the parameter module:

- `ParamId_GetCanNodeId()`
- `ParamId_SaveCanNodeIdToFlash()`

The valid CANopen node ID range is:

```text
1..127
```

Node ID `0` is reserved for broadcast and is not a valid local node ID.
The default node ID is `1`.

Changing the node ID updates the local CAN acceptance configuration. The
standard CANopen COB-IDs are calculated from the current node ID.

## 4. Standard CANopen COB-IDs

The following standard identifiers are implemented:

| COB-ID | Direction | Function |
| --- | --- | --- |
| `0x000` | master -> all | NMT |
| `0x080` | master -> all | SYNC |
| `0x080 + NodeID` | device -> master | EMCY |
| `0x180 + NodeID` | device -> master | TPDO1 |
| `0x200 + NodeID` | master -> device | RPDO1 |
| `0x280 + NodeID` | device -> master | TPDO2 |
| `0x300 + NodeID` | master -> device | RPDO2 |
| `0x580 + NodeID` | device -> master | SDO response |
| `0x600 + NodeID` | master -> device | SDO request |
| `0x700 + NodeID` | device -> master | Heartbeat |

The COB-ID constants are declared in `canopen.h`:

```c
#define CANOPEN_COBID_NMT             0x000U
#define CANOPEN_COBID_TPDO1_BASE      0x180U
#define CANOPEN_COBID_RPDO1_BASE      0x200U
#define CANOPEN_COBID_TPDO2_BASE      0x280U
#define CANOPEN_COBID_RPDO2_BASE      0x300U
#define CANOPEN_COBID_SDO_TX_BASE     0x580U
#define CANOPEN_COBID_SDO_RX_BASE     0x600U
#define CANOPEN_COBID_HEARTBEAT_BASE  0x700U
```

## 5. Startup and Communication Task

`CANFD_INIT()` configures the MCP2518FD, enables the receive FIFO, and
initializes the CANopen node in Pre-operational state. A CANopen boot-up
heartbeat with data `0x00` is sent on:

```text
0x700 + NodeID
```

The communication task is implemented in `Communication_Task()`:

1. Process pending MCP2518FD receive interrupts.
2. Drain the receive FIFO in thread context.
3. Run `MCP2518FD_Service1ms()`.
4. Run `CAN_Telemetry_Service1ms()`.

The service path provides:

- CANopen heartbeat producer
- TPDO event scheduling
- SYNC-triggered TPDO handling
- EMCY edge reporting
- legacy parameter and telemetry queue transmission only when
  `APP_USE_LEGACY_CAN_PROTOCOL = 1U`

The interrupt handler only signals the task. SPI transfers are performed
outside interrupt context.

## 6. NMT

NMT messages use COB-ID `0x000`:

```text
Byte 0: command
Byte 1: target node, 0 means broadcast
```

Supported commands:

| Command | Meaning |
| --- | --- |
| `0x01` | Start remote node, enter Operational |
| `0x02` | Stop remote node |
| `0x80` | Enter Pre-operational |
| `0x81` | Reset node, emit Boot-up, then enter Pre-operational |

RPDO processing is enabled only while the local NMT state is Operational.
SDO access is available while the node is Pre-operational and Operational.

## 7. Heartbeat

Heartbeat is produced on:

```text
0x700 + NodeID
```

Heartbeat payload:

| Value | Meaning |
| --- | --- |
| `0x00` | Boot-up |
| `0x04` | Stopped |
| `0x05` | Operational |
| `0x7F` | Pre-operational |

The producer period is configured through object `0x1017` in milliseconds.
The default is 1000 ms.

## 8. SYNC and EMCY

SYNC uses COB-ID `0x080`. The node accepts both zero-length SYNC frames and
SYNC frames containing a counter. TPDOs configured with synchronous
transmission types `1..240` are sent when SYNC is received.

EMCY uses:

```text
0x080 + NodeID
```

The current payload is 8 bytes:

| Byte | Meaning |
| --- | --- |
| `0..1` | emergency error code |
| `2` | error register |
| `3..7` | manufacturer-specific data, currently zero/axis error data |

An EMCY is queued when `g_axis.error` changes.

## 9. SDO

SDO request and response identifiers are:

```text
Request:  0x600 + NodeID
Response: 0x580 + NodeID
```

Important distinction:

- `0x601` is the CAN frame identifier for an SDO request to NodeID `1`.
- `0x1000`, `0x6040`, `0x6041` are object dictionary indices carried inside the
  SDO payload.

For example, writing controlword `0x6040:00` on NodeID `1` uses CAN ID
`0x601`, and bytes `1..3` of the payload contain index `0x6040` plus the
sub-index:

```text
CAN ID: 0x601
Data:   2B 40 60 00 06 00 00 00
        ^^
        SDO command specifier
           ^^^^^^^^
           index 0x6040, sub-index 0x00
                    ^^^^^
                    value 0x0006, little-endian
```

The implementation supports:

- expedited upload
- expedited download
- segmented upload
- segmented download
- toggle bit checking
- last-segment and unused-byte handling
- download length checking
- SDO abort responses for unsupported or invalid objects

The segmented session buffer is 255 bytes. Standard CiA 402 scalar objects
normally use expedited transfers. The device name object `0x1008` is
available as a segmented-upload example.
Segment upload requests use an 8-byte CAN data field; for example, the first
toggle-0 request is `60 00 00 00 00 00 00 00`.

Common SDO commands:

| Command | Meaning |
| --- | --- |
| `0x40` | initiate upload |
| `0x23` | expedited download, 4 bytes |
| `0x2B` | expedited download, 2 bytes |
| `0x2F` | expedited download, 1 byte |
| `0x21` | initiate segmented download with size |
| `0x60/0x70` | segmented upload request, toggle dependent |
| `0x00/0x10` | segmented download segment, toggle dependent |

All SDO frames use an 8-byte CAN data field. Multi-byte values are
little-endian. For NodeID 1, a valid `0x6040` write is:

```text
CAN ID: 0x601
Data:   2B 40 60 00 06 00 00 00
```

`0x6040` is an `UNSIGNED16` object. The expedited download command must
therefore be `0x2B`, and the value bytes are `06 00` for controlword
`0x0006`. A `0x23` four-byte download or a frame with fewer than 8 CAN data
bytes is rejected with abort code `0x06070010`.

The expected successful response is:

```text
CAN ID: 0x581
Data:   60 40 60 00 00 00 00 00
```

The following writes advance the CiA 402 state machine:

```text
2B 40 60 00 06 00 00 00   0x0006
2B 40 60 00 07 00 00 00   0x0007
2B 40 60 00 0F 00 00 00   0x000F
```

An incoming frame with COB-ID `0x300 + NodeID` is standard RPDO2 traffic.
It is not a legacy telemetry frame. In strict mode the device never emits
that identifier; TPDO2 uses `0x280 + NodeID` by default.

Do not send `00 06` in bytes 4 and 5. That is the little-endian value
`0x0600`, not `0x0006`, and the device intentionally does not swap it.

Typical SDO abort responses are:

| Abort code | Meaning in this firmware |
| --- | --- |
| `0x06070010` | Data length does not match the object |
| `0x06090030` | CiA 402 value or state transition is invalid |
| `0x06010002` | Object is unsupported or not writable |

## 10. PDO Communication Parameters

### 10.1 RPDO1

Communication parameters:

```text
0x1400:01  COB-ID
0x1400:02  Transmission type
```

Mapping parameters:

```text
0x1600:00  number of mapped objects
0x1600:01  mapping entry 1
0x1600:02  mapping entry 2
```

Default mapping:

```text
0x1600:01 = 0x60400010  Controlword, 16 bits
0x1600:02 = 0x60FF0020  Target velocity, 32 bits
```

### 10.2 RPDO2

Communication parameters:

```text
0x1401:01  COB-ID
0x1401:02  Transmission type
```

Mapping parameters:

```text
0x1601:00  number of mapped objects
0x1601:01  mapping entry 1
0x1601:02  mapping entry 2
```

Default mapping:

```text
0x1601:01 = 0x60600008  Modes of operation, 8 bits
0x1601:02 = 0x60710010  Target torque, 16 bits
```

### 10.3 TPDO1

Communication parameters:

```text
0x1800:01  COB-ID
0x1800:02  Transmission type
0x1800:03  Inhibit time, milliseconds
0x1800:05  Event timer, milliseconds
```

Mapping parameters:

```text
0x1A00:00  number of mapped objects
0x1A00:01  mapping entry 1
0x1A00:02  mapping entry 2
0x1A00:03  mapping entry 3
0x1A00:04  mapping entry 4
```

Default mapping:

```text
0x1A00:01 = 0x60410010  Statusword, 16 bits
0x1A00:02 = 0x606C0020  Actual velocity, 32 bits
0x1A00:03 = 0x60610008  Mode display, 8 bits
0x1A00:04 = 0x10010008  Error register, 8 bits
```

### 10.4 TPDO2

Communication parameters:

```text
0x1801:01  COB-ID
0x1801:02  Transmission type
0x1801:03  Inhibit time, milliseconds
0x1801:05  Event timer, milliseconds
```

Mapping parameters:

```text
0x1A01:00  number of mapped objects
0x1A01:01  mapping entry 1
0x1A01:02  mapping entry 2
```

Default mapping:

```text
0x1A01:01 = 0x60640020  Actual position, 32 bits
0x1A01:02 = 0x607A0020  Target position, 32 bits
```

Mapping entry format:

```text
bits 31..16: object index
bits 15..8 : sub-index
bits 7..0  : mapped length in bits
```

PDO mapping changes should follow the CANopen sequence:

1. Disable the PDO by setting bit 31 of its COB-ID.
2. Set mapping count to zero.
3. Write each mapping entry.
4. Set the final mapping count.
5. Configure the transmission type and timer parameters.
6. Re-enable the PDO by clearing bit 31.

## 11. CiA 402 Object Dictionary

The motor-control core exposes the following CiA 402 objects:

| Index | Object | Access | Current unit |
| --- | --- | --- | --- |
| `0x6040` | Controlword | read/write | bit mask |
| `0x6041` | Statusword | read-only | bit mask |
| `0x6060` | Modes of operation | read/write | CiA 402 mode |
| `0x6061` | Modes display | read-only | CiA 402 mode |
| `0x6064` | Position actual value | read-only | encoder counts |
| `0x606C` | Velocity actual value | read-only | rpm |
| `0x6071` | Target torque | read/write | mA |
| `0x607A` | Target position | read/write | encoder counts |
| `0x60FF` | Target velocity | read/write | rpm |

Communication objects currently exposed by the CAN layer include:

```text
0x1000 Device type
0x1001 Error register
0x1008 Device name
0x1017 Heartbeat producer time
0x1018 Identity (vendor, product, revision and serial entries)
0x1400/0x1401 RPDO communication
0x1600/0x1601 RPDO mapping
0x1800/0x1801 TPDO communication
0x1A00/0x1A01 TPDO mapping
```

## 12. Legacy Function-Code Protocol

The older custom protocol remains available for diagnostics and backward
compatibility. It uses the legacy identifier layout:

```text
SID = (function_code << 4) | (node_id & 0x0F)
```

Examples include:

```text
0x01  stop motor
0x02  start motor
0x06  set speed reference
0x0A  start calibration
0x0E  get node ID
0x0F  set node ID
```

These commands should not be used as the standard CiA 402 interface. Standard
CANopen masters must use the COB-IDs and object dictionary described above.

The legacy response and telemetry function codes are available only when
`APP_USE_LEGACY_CAN_PROTOCOL = 1U`:

| Function code | Meaning |
| --- | --- |
| `0x20` | command status response |
| `0x21` | parameter state response |
| `0x22` | parameter result block 1 |
| `0x23` | parameter result block 2 |
| `0x30..0x35` | legacy periodic telemetry, layout depends on `APP_USE_CAN_FD` |

The legacy identifier format masks the node ID to four bits. Therefore node
IDs above 15 are valid for standard CANopen COB-IDs but cannot be addressed
correctly through the legacy function-code path.

## 13. Current Scope and Validation Status

Implemented and source-reviewed:

- standard NMT basics
- heartbeat producer
- SYNC reception
- EMCY edge reporting
- expedited and segmented SDO sessions
- dynamic RPDO1/2 and TPDO1/2 mapping
- PDO COB-ID, transmission type, inhibit time and event timer handling
- CiA 402 velocity, torque and position objects

Not yet covered as a complete CANopen product:

- RPDO3/4 and TPDO3/4
- heartbeat consumer supervision
- full Node Guarding
- all SDO domain-object storage use cases
- complete CANopen conformance testing
- hardware CANopen master interoperability testing

The current host environment has no `arm-none-eabi-gcc` build tool or connected
CANopen master. `canopen.c` and the shared CiA 402 core pass host-side C syntax
validation in both strict and compatibility modes. The final acceptance test
must still be performed on target hardware with a standard CANopen master.
