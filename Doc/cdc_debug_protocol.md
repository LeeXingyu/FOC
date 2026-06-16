# CDC Debug Protocol

This project supports a CDC-only debug transport for Windows upper host tools.

## Transport

- USB CDC Virtual COM Port
- Line based ASCII commands
- Newline terminated

## Command set

### Basic control

- `help`
- `start`
- `stop`
- `speedmode`
- `posmode`
- `vfmode`

### Speed and PI

- `speed <rpm>`
- `kp <value>`
- `ki <value>`

### Motor / flash / calibration

- `pole <n>`
- `nodeget`
- `nodeset <0-15>`
- `flashread`
- `flashclear`
- `calib <0|5>`
- `calibstop`

### Generic CAN-like command

- `can <func_hex> <node> <hexbytes>`

Example:

```text
can 06 01 0000803F
```

This sends function `0x06`, node `1`, payload `1.0f` in little-endian hex.

## Telemetry

The firmware prints periodic telemetry lines in the following form:

```text
TEL POS=12.345 MECH=34.567 APP=78.901 SPEED=100.000 IQ=0.456
```

Fields:

- `POS`: rotor position in degrees
- `MECH`: offset-compensated mechanical angle in degrees
- `APP`: applied electrical angle in degrees
- `SPEED`: speed in rpm
- `IQ`: q-axis current in amperes

## Notes

- CDC-only mode is enabled by `APP_COMM_USE_CDC_ONLY`
- CAN and EtherCAT initialization stay disabled in that mode
- The host application only needs to parse `DBG` and `TEL` text lines
