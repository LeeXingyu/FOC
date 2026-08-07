# CAN 移植说明

本文用于说明当前工程中的 CAN / CANopen / CiA402 相关内容，如何移植到其他工程。

## 1. 需要移植的内容

- CAN 收发底层驱动
- CANopen 协议栈
- CiA402 对象字典
- FOC 控制映射层
- 相关配置宏和节点参数

## 2. 主要文件

- `Core/Src/Communication/mcp2518fd/canopen.c`
- `Core/Inc/Communication/mcp2518fd/canopen.h`
- `Core/Src/Communication/mcp2518fd/drv_canfdspi_api.c`
- `Core/Src/MotorControl/Core/mc_interface.c`
- `Core/Src/MotorControl/Core/mc_interface_ext.c`
- `Core/Inc/MotorControl/Core/mc_interface_ext.h`
- `Docs/LX_BLDC_CiA402.eds`
- `Docs/cia402_communication_guide.md`
- `Docs/can_communication_guide.md`

## 3. 移植顺序

1. 先移植 CAN 外设初始化和收发驱动。
2. 再移植 `canopen.c` 和 `canopen.h`。
3. 然后移植 CiA402 对象处理接口。
4. 最后移植 FOC 参数映射和 PDO 通信参数。

## 4. 必须保持一致的参数

- NodeID
- RPDO/TPDO COB-ID
- SDO 默认 COB-ID
- Heartbeat 周期
- RPDO1/2、TPDO1/2 默认映射

## 4.1 已暴露但未接控制链路的协议

当前文档中的部分 CiA402 对象已经在对象字典中开放，但它们的状态通常只是“可读/可写”，
并没有真正接入 FOC 的控制闭环、状态机或限位逻辑。可分为下面几类：

### 4.1.1 反馈类对象

这类对象会返回当前测量值，但通常只是把内部量直接暴露给上位机，不是独立控制链路。

- `0x6062` `Position Demand Value`
- `0x6063` `Position Actual Internal Value`
- `0x6069` `Velocity Sensor Actual Value`
- `0x606B` `Velocity Demand Value`
- `0x6074` `Torque Demand`
- `0x6077` `Torque Actual Value`
- `0x6078` `Current Actual Value`
- `0x6079` `DC Link Circuit Voltage`
- `0x60F4` `Following Error Actual Value`
- `0x603F` `Error Code`
- `0x1001` `Error Register`
- `0x6502` `Supported Drive Modes`

说明：

- 这些对象大多已经能 SDO 读出。
- 其中一部分值来自 `g_axis` 的实时量。
- 但它们本身不是独立控制入口，更多是“状态回传”。

### 4.1.2 参数存储类对象

这类对象目前主要用于保存上位机写入的参数，或者在读回时返回缓存值。
它们不一定已经进入速度环、位置环、转矩环的实际计算流程。

- `0x6067` `Position Window`
- `0x6068` `Position Window Time`
- `0x606A` `Velocity Sensor Selection`
- `0x6075` `Motor Rated Current`
- `0x6076` `Motor Rated Torque`
- `0x607B` `Position Range Limit`
- `0x607C` `Home Offset`
- `0x607D` `Software Position Limit`
- `0x607E` `Polarity`
- `0x607F` `Max Profile Velocity`
- `0x6080` `Max Motor Speed`
- `0x6082` `End Velocity`
- `0x6085` `Quick Stop Deceleration`
- `0x6086` `Motion Profile Type`
- `0x6087` `Torque Slope`
- `0x608F` `Position Encoder Resolution`
- `0x6090` `Velocity Encoder Resolution`
- `0x6091` `Gear Ratio`
- `0x6092` `Feed Constant`
- `0x6093` `Position Factor`
- `0x6094` `Velocity Factor`
- `0x6095` `Acceleration Factor`
- `0x6098` `Homing Method`
- `0x6099` `Speed during Search`
- `0x609A` `Homing Acceleration`
- `0x60B8` `Touch Probe Function`
- `0x60E0` `Positive Torque Limit`
- `0x60E1` `Negative Torque Limit`
- `0x60FE` `Physical Outputs / Bit Mask`

说明：

- 这些参数多数已经具备 SDO 读写接口。
- 但不少参数还只是“寄存器式存储”。
- 如果要真正生效，还需要把它们接到控制算法、限幅器、状态机判断里。

### 4.1.3 上位机显示类对象

这类对象主要用于让上位机看见“当前配置/当前状态”。
它们可以读到，但不一定驱动控制流程。

- `0x6061` `Modes of Operation Display`
- `0x6064` `Position Actual Value`
- `0x606C` `Velocity Actual Value`
- `0x60FD` `Digital Inputs`
- `0x60B9` `Touch Probe Status`
- `0x60BA..0x60BD` `Touch Probe Latch Values`

说明：

- 这些对象适合做监控、波形显示、日志分析。
- 但当前更偏“反馈展示层”，不是控制闭环本体。

### 4.1.4 预留 PDO / 通信参数

这些对象已经开放了接口，但很多场景里只是为了兼容标准配置流程，
不一定已经和所有控制链路完全打通。

- `0x1200` `SDO Server Parameter`
- `0x1402 / 0x1403` `RPDO3 / RPDO4 Communication Parameter`
- `0x1602 / 0x1603` `RPDO3 / RPDO4 Mapping Parameter`
- `0x1802 / 0x1803` `TPDO3 / TPDO4 Communication Parameter`
- `0x1A02 / 0x1A03` `TPDO3 / TPDO4 Mapping Parameter`

说明：

- 这些对象支持读写和保存。
- 但默认上不一定参与主控制链路。
- 若上位机未显式配置，它们通常不应影响主路径。

### 4.1.5 插补与回零类对象

这些对象已经暴露，但当前更多是协议兼容和参数留存，未必完整驱动设备动作。

- `0x60C0` `Interpolation Sub Mode`
- `0x60C1` `Interpolation Data Record`
- `0x60C2` `Interpolation Time Period Value / Index`
- `0x60C4` `Interpolation Data Configuration`

说明：

- 若当前工程不做 CSP/IP 的完整插补闭环，它们就是预留接口。
- 回零对象虽然能写入，但是否真正执行，还要看主状态机是否调用。

### 4.1.6 文档标注建议

建议在文档中把这类对象统一标成：

- `已支持对象接口`
- `未形成控制链路`
- `仅参数/反馈/预留`

这样上位机、移植工程和调试人员能一眼区分“能读写”与“能真正控制”。

## 5. 上电流程

- 初始化 CAN 控制器
- 初始化 CANopen
- 进入 PRE-OP
- 配置 SDO / PDO
- 切换到 OP
- 通过 RPDO 周期下发控制量
- 通过 TPDO 周期上报状态和反馈

## 6. 验证方法

- 读取 `0x1000`、`0x1018`、`0x6041`
- 写入 `0x6040`
- 写入 `0x6060`
- 写入 `0x60FF`
- 观察 `0x6041`、`0x6061`、`0x606C` 是否变化
- 观察 `0x701` 心跳是否正常

## 7. 常见问题

- 进不了 `OP`：优先检查 PDO 配置和 NodeID
- SDO 无响应：优先检查 `0x601/0x581`
- 速度不更新：优先检查 RPDO1 映射 `0x6040/0x60FF`
- 状态字不变：优先检查 `0x6040` 是否已正确下发

## 8. 备注

当前工程的 CAN 逻辑已经按 CiA402 主路径整理，移植到其他工程时，建议保持协议层与电机控制层分离。
