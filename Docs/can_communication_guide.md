# CAN Communication Guide

## 1. Scope

This document describes the current MCP2518FD-based CAN implementation in the
project.

The preferred control path is now:

- standard CANopen transport
- CiA 402 drive profile objects

The older project-specific function-code protocol is still present as a
compatibility path, but it is not the recommended main control interface for a
CANopen master.

Primary source files:

- `Core/Inc/Communication/mcp2518fd/canopen.h`
- `Core/Src/Communication/mcp2518fd/canopen.c`
- `Core/Src/Communication/mcp2518fd/can_telemetry.c`
- `Core/Src/MotorControl/Core/mc_interface.c`
- `Core/Src/MotorControl/Core/mc_interface_ext.c`

## 2. CAN and CAN FD Build Options

Compile-time switches:

```c
#define APP_USE_CAN_FD 0
#define APP_USE_LEGACY_CAN_PROTOCOL 0U
```

Behavior:

- `APP_USE_CAN_FD = 0`
  - classical CAN
  - max payload 8 bytes
- `APP_USE_CAN_FD = 1`
  - CAN FD
  - max payload 16 bytes
  - same CANopen object model, wider frame capability

- `APP_USE_LEGACY_CAN_PROTOCOL = 0U`
  - only standard CANopen traffic is intended as the primary interface
- `APP_USE_LEGACY_CAN_PROTOCOL = 1U`
  - the legacy function-code command and telemetry path is also enabled

## 3. Node ID

The active CANopen node ID comes from the parameter module:

- `ParamId_GetCanNodeId()`
- `ParamId_SaveCanNodeIdToFlash()`

Valid local range:

```text
1..127
```

When the node ID changes, the code updates:

- RPDO1 default COB-ID
- TPDO1 default COB-ID
- RPDO2 default COB-ID
- TPDO2 default COB-ID
- RPDO3 default COB-ID
- TPDO3 default COB-ID
- RPDO4 default COB-ID
- TPDO4 default COB-ID

## 4. Standard COB-IDs

Implemented standard COB-ID bases:

| COB-ID | Direction | Function |
| --- | --- | --- |
| `0x000` | master -> all | NMT |
| `0x080` | master -> all | SYNC |
| `0x080 + NodeID` | device -> master | EMCY |
| `0x180 + NodeID` | device -> master | TPDO1 |
| `0x200 + NodeID` | master -> device | RPDO1 |
| `0x280 + NodeID` | device -> master | TPDO2 |
| `0x300 + NodeID` | master -> device | RPDO2 |
| `0x380 + NodeID` | device -> master | TPDO3 |
| `0x400 + NodeID` | master -> device | RPDO3 |
| `0x480 + NodeID` | device -> master | TPDO4 |
| `0x500 + NodeID` | master -> device | RPDO4 |
| `0x580 + NodeID` | device -> master | SDO response |
| `0x600 + NodeID` | master -> device | SDO request |
| `0x700 + NodeID` | device -> master | Heartbeat |

## 5. Startup and Runtime Flow

`CANFD_INIT()`:

- resets and configures the MCP2518FD
- configures RX FIFO and filters
- enters CAN normal mode
- initializes the local CANopen state to Pre-operational
- resets the CiA 402 core state
- initializes default PDO1..4 COB-IDs and mappings
- sends CANopen boot-up on `0x700 + NodeID`

`MCP2518FD_Service1ms()` currently handles:

- heartbeat producer timing
- EMCY edge reporting
- TPDO1 event scheduling
- TPDO2 event scheduling
- TPDO3 event scheduling
- TPDO4 event scheduling
- SYNC-triggered synchronous TPDO sending

## 6. NMT

Supported NMT commands:

| Command | Meaning |
| --- | --- |
| `0x01` | Start remote node |
| `0x02` | Stop remote node |
| `0x80` | Enter Pre-operational |
| `0x81` | Reset node |

Behavior:

- SDO access is available in Pre-operational and Operational
- RPDO handling is accepted only in Operational

## 7. Heartbeat and EMCY

Heartbeat producer:

- COB-ID: `0x700 + NodeID`
- boot-up data: `0x00`
- state values:
  - `0x04` stopped
  - `0x05` operational
  - `0x7F` pre-operational

Heartbeat producer time object:

- `0x1017`

EMCY:

- COB-ID: `0x080 + NodeID`
- emitted when the internal axis error changes

## 8. SDO

SDO request and response:

```text
Request:  0x600 + NodeID
Response: 0x580 + NodeID
```

Implemented SDO features:

- expedited upload
- expedited download
- segmented upload
- segmented download
- toggle validation
- length validation
- last-segment handling
- standard abort response generation

Examples for node 1:

Write `0x6040:00 = 0x0006`:

```text
CAN ID: 0x601
Data:   2B 40 60 00 06 00 00 00
```

Write `0x6060:00 = 3`:

```text
CAN ID: 0x601
Data:   2F 60 60 00 03 00 00 00
```

Write `0x60FF:00 = 1000`:

```text
CAN ID: 0x601
Data:   23 FF 60 00 E8 03 00 00
```

Read `0x6041:00`:

```text
CAN ID: 0x601
Data:   40 41 60 00 00 00 00 00
```

Common abort meanings in this firmware:

| Abort code | Meaning |
| --- | --- |
| `0x06070010` | wrong data size |
| `0x06090030` | invalid value or transition |
| `0x06010002` | unsupported or read-only object |
| `0x06020000` | object not found |

## 9. PDO Support

### 9.1 Default PDOs

| PDO | Objects |
| --- | --- |
| RPDO1 | `0x6040:00`, `0x60FF:00` |
| RPDO2 | `0x6060:00`, `0x6071:00` |
| RPDO3 | configurable |
| RPDO4 | configurable |
| TPDO1 | `0x6041:00`, `0x606C:00`, `0x6061:00`, `0x1001:00` |
| TPDO2 | `0x6064:00`, `0x607A:00` |
| TPDO3 | configurable |
| TPDO4 | configurable |

### 9.2 Communication objects

Implemented:

- `0x1400..0x1403`
- `0x1600..0x1603`
- `0x1800..0x1803`
- `0x1A00..0x1A03`

Supported fields:

- COB-ID
- transmission type
- inhibit time for TPDO
- event timer for TPDO
- mapping count
- mapping entries

### 9.3 Runtime behavior

Current code behavior:

- RPDO1 and RPDO2 have dedicated internal fast paths
- RPDO3 and RPDO4 are handled through the dynamic communication and mapping
  objects
- TPDO1 and TPDO2 have dedicated internal send paths
- TPDO3 and TPDO4 are sent through dynamic communication and mapping objects
- synchronous transmission types `1..240` are sent on SYNC reception
- event-driven types `254` and `255` are sent from the 1 ms service

## 10. CiA 402 Relation

The CAN layer is now mainly a transport shell. It does not own a separate
drive-state implementation.

Instead it forwards object access to the shared CiA 402 core. That means:

- the same object can be written over SDO, RPDO or CDC
- the same object-level validation is reused
- the same stateword and statusword logic is reused

## 11. Legacy Function-Code Path

The older command path still exists for compatibility builds.

Identifier format:

```text
SID = (function_code << 4) | (node_id & 0x0F)
```

Typical legacy commands:

- start motor
- stop motor
- set speed reference
- set node ID
- calibration related commands
- compatibility CiA 402 helper commands

Important limitation:

- the legacy path masks node ID to 4 bits
- standard CANopen node IDs above 15 are therefore not cleanly addressable
  through the legacy format

For standard master interoperability, use CANopen SDO and PDO instead of the
legacy command path.

## 12. Current Validation Summary

Code now covers:

- NMT
- heartbeat producer
- EMCY
- SYNC-triggered TPDO
- expedited and segmented SDO
- RPDO1..4 communication and mapping path
- TPDO1..4 communication and mapping path
- shared CiA 402 object dictionary forwarding

Still recommended on real hardware:

- full generic CANopen master SDO walk-through
- RPDO3/4 and TPDO3/4 remap verification
- timing verification for inhibit and event timers
- motor direction and sign verification for speed/torque/position objects
