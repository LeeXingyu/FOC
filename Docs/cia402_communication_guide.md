# CiA 402 Communication Guide

## 1. Scope

This document describes the current shared CiA 402 implementation used by the
CANopen transport and the CDC debug transport.

The protocol core lives in the motor-control layer:

```c
MC_Cia402_ReadObject()
MC_Cia402_WriteObject()
MC_Apply_Cia402_Controlword()
MC_Get_Cia402_Statusword()
```

Primary source files:

- `Core/Inc/MotorControl/Core/mc_interface.h`
- `Core/Inc/MotorControl/Core/mc_interface_ext.h`
- `Core/Src/MotorControl/Core/mc_interface.c`
- `Core/Src/MotorControl/Core/mc_interface_ext.c`
- `Core/Src/Communication/mcp2518fd/canopen.c`
- `USB_Device/App/usbd_cdc_if.c`

CANopen SDO/PDO and CDC `cia402` commands share the same object dictionary and
state-machine implementation. This means object access behavior is intended to
be transport-independent.

## 2. Current CiA 402 Scope

The firmware now exposes two layers:

- Standard CiA 402 main path
- FOC control mapping

The main path provides:

- standard `0x6040/0x6041` state machine objects
- standard `0x6060/0x6061` mode selection and feedback
- standard SDO access, including segmented upload and segmented download
- PDO communication and mapping objects for RPDO1..4 and TPDO1..4
- standard CANopen NMT, heartbeat, EMCY and SYNC interaction

The FOC mapping layer provides:

- speed reference mapping
- torque/current reference mapping
- position reference mapping
- actual speed, position and current feedback
- a larger set of CiA 402 parameters and placeholders for profile, homing,
  interpolation, touch probe, scaling and digital I/O related objects

Some objects already drive active FOC behavior. Others are currently exposed as
standard CiA 402-compatible parameter storage or feedback interfaces so a
standard master can access them using canonical indices and sub-indices.

## 3. CiA 402 State Machine

Implemented state set:

```text
Switch on disabled
Ready to switch on
Switched on
Operation enabled
Quick stop active
Fault
```

The state is reflected through `0x6041`.

### 3.1 Standard controlword sequence

| Controlword | Transition |
| --- | --- |
| `0x0006` | Switch on disabled -> Ready to switch on |
| `0x0007` | Ready to switch on -> Switched on |
| `0x000F` | Switched on -> Operation enabled |
| `0x0007` | Operation enabled -> Switched on |
| `0x0006` | Operation enabled -> Ready to switch on |
| `0x0002` or bit 2 cleared | Operation enabled -> Quick stop active |
| `0x0080` | Fault reset |

Entering Operation enabled uses the existing motor start path. Leaving the
enabled state resets active references through the motor-control core.

### 3.2 Statusword bits

The implementation currently generates the following commonly used bits:

| Bit | Meaning |
| --- | --- |
| 0 | Ready to switch on |
| 1 | Switched on |
| 2 | Operation enabled |
| 3 | Fault |
| 4 | Voltage enabled indication |
| 5 | Quick stop active |
| 6 | Switch on disabled |
| 8 | Torque mode indication |
| 10 | Target reached / velocity indication |
| 12 | Position mode indication |

## 4. Modes of Operation

`0x6060` selects the requested mode. `0x6061` reports the active mode display.

| Value | Mode | Internal mapping |
| --- | --- | --- |
| `1` | Profile Position | `CTRL_MODE_POSITION` |
| `3` | Profile Velocity | `CTRL_MODE_SPEED` |
| `4` | Profile Torque | `CTRL_MODE_TORQUE` |
| `8` | Cyclic Synchronous Position | `CTRL_MODE_POSITION` |
| `9` | Cyclic Synchronous Velocity | `CTRL_MODE_SPEED` |
| `10` | Cyclic Synchronous Torque | `CTRL_MODE_TORQUE` |

Unsupported mode values are rejected by the object write layer.

## 5. Object Dictionary Coverage

### 5.1 FOC-linked runtime objects

These objects already participate in the active control or feedback path:

| Index | Object | Access | Meaning |
| --- | --- | --- | --- |
| `0x6040` | Controlword | rw | CiA 402 state machine command |
| `0x6041` | Statusword | ro | CiA 402 state feedback |
| `0x6060` | Modes of operation | rw | requested mode |
| `0x6061` | Modes display | ro | active mode |
| `0x6064` | Position actual value | ro | current position feedback |
| `0x606C` | Velocity actual value | ro | current speed feedback in rpm |
| `0x6071` | Target torque | rw | torque/current reference |
| `0x6072` | Max torque | rw | torque limit parameter |
| `0x607A` | Target position | rw | position reference |
| `0x6081` | Profile velocity | rw | profile velocity parameter |
| `0x6083` | Profile acceleration | rw | mapped to speed ramp behavior |
| `0x6084` | Profile deceleration | rw | profile deceleration parameter |
| `0x60F4` | Following error actual value | ro | calculated following error |
| `0x60FF` | Target velocity | rw | speed reference in rpm |

### 5.2 Extended CiA 402 interface objects

These objects are implemented and SDO/PDO-accessible in the current codebase.
Depending on the object, they are either feedback, configuration storage or a
placeholder interface for future functional completion.

| Range | Notes |
| --- | --- |
| `0x603F` | Error code |
| `0x6062/0x6063` | internal position demand and internal position feedback |
| `0x6065..0x606B` | following error, position window and velocity feedback helpers |
| `0x6074..0x6079` | torque/current/dc-link related objects |
| `0x607B..0x6087` | position limits, polarity and motion profile objects |
| `0x608F..0x6095` | encoder resolution and unit-conversion factors |
| `0x6098..0x609A` | homing parameters |
| `0x60B8..0x60BD` | touch probe objects |
| `0x60C0..0x60C6` | interpolation and cyclic acceleration objects |
| `0x60E0/0x60E1` | positive and negative torque limit |
| `0x60FD/0x60FE` | digital input and output objects |
| `0x6502` | supported drive modes |
| `0x1200` | SDO server parameter |
| `0x1400..0x1403` | RPDO communication parameter |
| `0x1600..0x1603` | RPDO mapping parameter |
| `0x1800..0x1803` | TPDO communication parameter |
| `0x1A00..0x1A03` | TPDO mapping parameter |

## 6. FOC Mapping Summary

| CiA 402 object | Current firmware behavior |
| --- | --- |
| `0x6040` | `MC_Apply_Cia402_Controlword()` |
| `0x6060` | mode routing to existing control modes |
| `0x60FF` | `MC_Set_Speed_Reference()` |
| `0x6071` | `MC_Set_Torque_Reference()` |
| `0x607A` | position reference into existing position controller |
| `0x6064` | position feedback from axis estimator / encoder path |
| `0x606C` | speed feedback from axis estimator |
| `0x6077/0x6078` | actual torque/current style feedback from FOC current loop |
| `0x6079` | dc-link voltage feedback |
| `0x6083` | speed ramp related behavior |
| `0x60F4` | calculated following error based on active mode |

Objects not listed above are still useful for standard master compatibility,
parameter storage, future feature expansion or interoperability with generic
CANopen/CiA 402 tools.

## 7. CANopen Access

### 7.1 SDO

Request/response COB-IDs:

```text
Request:  0x600 + NodeID
Response: 0x580 + NodeID
```

Supported transfer types:

- expedited upload
- expedited download
- segmented upload
- segmented download

Segmented transfers are currently implemented for both directions. The
segmented session buffer is 255 bytes.

Important distinction:

- `0x601` is the SDO request CAN identifier for node 1
- `0x6040` is the controlword object index
- `0x1000` is the device type object index

Example: write `0x6040:00 = 0x0006` on node 1:

```text
CAN ID: 0x601
Data:   2B 40 60 00 06 00 00 00
```

Example: write `0x60FF:00 = 1000 rpm` on node 1:

```text
CAN ID: 0x601
Data:   23 FF 60 00 E8 03 00 00
```

Example: upload `0x6041:00` on node 1:

```text
CAN ID: 0x601
Data:   40 41 60 00 00 00 00 00
```

Typical abort codes used by the firmware:

| Abort code | Meaning |
| --- | --- |
| `0x06070010` | data size mismatch |
| `0x06090030` | invalid parameter value or invalid CiA 402 transition |
| `0x06010002` | unsupported or read-only object |
| `0x06020000` | object does not exist in the exposed dictionary |

### 7.2 RPDO / TPDO defaults

Default PDO layout:

| PDO | Default COB-ID | Default mapping |
| --- | --- | --- |
| RPDO1 | `0x200 + NodeID` | `0x6040:00` 16b, `0x60FF:00` 32b |
| RPDO2 | `0x300 + NodeID` | `0x6060:00` 8b, `0x6071:00` 16b |
| RPDO3 | `0x400 + NodeID` | configurable via `0x1402/0x1602` |
| RPDO4 | `0x500 + NodeID` | configurable via `0x1403/0x1603` |
| TPDO1 | `0x180 + NodeID` | `0x6041:00` 16b, `0x606C:00` 32b, `0x6061:00` 8b, `0x1001:00` 8b |
| TPDO2 | `0x280 + NodeID` | `0x6064:00` 32b, `0x607A:00` 32b |
| TPDO3 | `0x380 + NodeID` | configurable via `0x1802/0x1A02` |
| TPDO4 | `0x480 + NodeID` | configurable via `0x1803/0x1A03` |

The current code supports:

- dynamic COB-ID writes
- dynamic mapping writes
- transmission type updates
- inhibit time and event timer handling for TPDO1..4
- SYNC-triggered TPDO sending for synchronous transmission types
- RPDO processing only in NMT Operational

Recommended standard mapping update sequence:

1. Disable the PDO by setting bit 31 of the COB-ID.
2. Set sub-index `0` of the mapping object to `0`.
3. Write mapping entries.
4. Restore the final mapping count.
5. Configure transmission type and timers.
6. Clear bit 31 to enable the PDO.

## 8. NMT Sequence

Recommended standard-master flow:

1. Wait for boot-up heartbeat `0x700 + NodeID`, data `00`.
2. Put the node into Pre-operational if configuration is needed.
3. Configure SDO, PDO communication and PDO mapping.
4. Write `0x6060`.
5. Send the standard `0x6040` enable sequence.
6. Send NMT Start.
7. Use RPDOs or SDOs for cyclic command updates.

SDO access is allowed in Pre-operational and Operational. RPDO handling is
enabled only in Operational.

## 9. CDC Relationship

CDC remains useful for debug and brings no separate CiA 402 logic. The same
shared object dictionary is used underneath.

Therefore:

- CAN and CDC use the same state machine
- CAN and CDC use the same mode validation
- CAN and CDC use the same units for speed, torque and position
- CAN and CDC use the same object-side acceptance or rejection behavior

## 10. Validation Status

Code-level implementation now includes:

- standard CiA 402 state machine path
- standard SDO expedited and segmented access
- RPDO1..4 and TPDO1..4 communication/mapping objects
- runtime PDO3/4 handling in the CANopen transport
- heartbeat producer, EMCY edge reporting and SYNC-triggered PDO support

Still recommended on hardware:

- verify generic CANopen master SDO access against the full EDS
- verify RPDO3/4 and TPDO3/4 remapping with a standard host tool
- verify event timer and inhibit timing on target
- verify every FOC-linked object with the actual motor and encoder direction

Not claimed by this document:

- formal CiA conformance certification
- full CANopen product certification
- complete functional homing/touch-probe/interpolation behavior beyond the
  currently exposed interfaces
