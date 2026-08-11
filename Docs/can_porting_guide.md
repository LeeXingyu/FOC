# CAN 移植说明

本文用于说明当前工程中的 `CAN / CANopen / CiA402` 相关内容，以及移植到其他工程时需要保留的关键链路。

## 1. 需要移植的内容

- CAN 收发底层驱动
- CANopen 协议栈入口
- CiA402 对象字典与状态机
- FOC 控制映射层
- NodeID、PDO、SDO、Heartbeat 等配置参数

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

当前默认链路中：

- RPDO1 默认映射为 `0x6040 + 0x607A`
  便于直接联调 CSP
- RPDO2 默认映射为 `0x6060 + 0x6071`
- TPDO1 默认用于状态字、速度、模式显示、错误寄存器
- TPDO2 默认用于位置反馈和目标位置回显

## 5. 上电流程

- 初始化 CAN 控制器
- 初始化 CANopen
- 进入 `PRE-OP`
- 配置 SDO / PDO
- 切换到 `OP`
- 通过 RPDO 周期下发控制量
- 通过 TPDO 周期上报状态和反馈

## 6. 验证方法

- 读取 `0x1000`、`0x1018`、`0x6041`
- 写入 `0x6040`
- 写入 `0x6060`
- 写入 `0x60FF` 或 `0x607A`
- 观察 `0x6041`、`0x6061`、`0x606C`、`0x6064` 是否变化
- 观察 `0x701 + NodeID` 心跳是否正常

## 7. 常见问题

- 进不了 `OP`：优先检查 PDO 配置和 NodeID
- SDO 无响应：优先检查 `0x601 / 0x581`
- 速度不更新：确认主站没有继续按旧配置把速度命令发到 RPDO1；当前默认 RPDO1 映射是 `0x6040 / 0x607A`
- 状态字不变：优先检查 `0x6040` 是否已正确下发
- CSP 不跟随：优先检查主站是否持续发送 `SYNC`

## 8. 备注

当前工程的 CAN 逻辑已经按 CiA402 主链路整理。移植到其他工程时，建议继续保持“协议层”和“电机控制层”分离：

- `CANopen / CiA402` 负责对象、PDO、状态机、同步
- `FOC / MotorControl` 负责速度环、位置环、转矩环和执行

## 9. 当前未完全接入 CAN CiA402 的功能说明

本节用于区分两件事：

- 对象已经存在于 CiA402 对象字典中，可以通过 SDO/PDO 访问
- 对象已经真正接入 FOC 控制、状态机、保护逻辑或硬件动作

当前 `0512_BLDC_STM32` 工程里，已经形成主链路的部分主要包括：

- `0x6040 / 0x6041` 状态机控制与状态反馈
- `0x6060 / 0x6061` 模式切换与显示
- `0x60FF` 速度给定
- `0x6071` 转矩/电流给定
- `0x607A` 位置给定
  PP 下立即生效，CSP 下在 SYNC 边沿锁存
- `0x6064` 位置反馈
- `0x606C` 速度反馈
- CANopen 的 `NMT / SDO / PDO / EMCY / Heartbeat / SYNC`

下面这些对象虽然已经暴露，但还没有完全接入 CAN CiA402 的实际功能链路。

### 9.1 参数已暴露，但仍偏“存储/兼容”的对象

这些对象目前大多已经支持 SDO 读写，但写入后还没有完整进入控制环、保护或限幅逻辑：

- `0x607B` Position Range Limit
- `0x607C` Home Offset
- `0x607D` Software Position Limit
- `0x607E` Polarity
- `0x607F` Max Profile Velocity
- `0x6080` Max Motor Speed
- `0x6082` End Velocity
- `0x6085` Quick Stop Deceleration
- `0x6086` Motion Profile Type
- `0x6087` Torque Slope
- `0x608F..0x6095` 编码器分辨率、齿轮比、单位换算因子
- `0x60E0 / 0x60E1` 正负转矩限制
- `0x60FE` Physical Outputs / Bit Mask

当前状态说明：

- 这些对象多数已经进入 `mc_interface_ext.c` 的对象读写分发。
- 但很多值还只是静态变量缓存。
- 还没有完整接到位置环、速度环、转矩环、限位器、GPIO 输出或状态机判断中。

### 9.2 Homing 相关对象尚未形成完整回零链路

以下对象已经暴露，但当前没有形成完整的回零功能：

- `0x6098` Homing Method
- `0x6099` Homing Speed
- `0x609A` Homing Acceleration

当前状态说明：

- 主站可以写入这些参数。
- 但当前工程中尚未看到由这些对象驱动的回零状态机、限位搜索、原点搜索、到位确认或 homing 完成状态反馈。

### 9.3 Interpolated Position 相关对象尚未接入实际运动

以下对象已经暴露，但目前还没有形成真正的插补执行链路：

- `0x60C0` Interpolation Sub Mode
- `0x60C1` Interpolation Data Record
- `0x60C2` Interpolation Time Period
- `0x60C4` Interpolation Data Configuration

当前状态说明：

- 对象可读写。
- 但当前代码主要还是把这些值保存下来。
- 还没有把它们转换成实时位置目标并驱动位置闭环。

### 9.4 CSV / CST 还不是严格意义上的同步模式

以下模式可以切换，但当前更像“沿用已有速度环/转矩环”的 CiA402 入口：

- `0x6060 = 9` CSV
- `0x6060 = 10` CST

当前状态说明：

- 目前 `SYNC` 的专用动作主要体现在 CSP 位置目标锁存上。
- CSV/CST 还没有形成严格的同步生效语义。
- 因此它们目前更接近“对象接口已接入”，而不是完整的 cyclic synchronous velocity / torque。

### 9.5 跟随误差对象已可反馈，但未完整闭环到故障处理

以下对象已经参与反馈：

- `0x6065` Following Error Window
- `0x6066` Following Error Time
- `0x60F4` Following Error Actual Value

当前状态说明：

- 已经可以计算并上报跟随误差。
- 当前位置模式下，也已经参与 target reached 一类的状态判断。
- 但还没有完整实现“超窗 + 超时 -> warning / fault / disable”的保护闭环。

### 9.6 Touch Probe / Digital IO 仍偏兼容接口

以下对象当前更偏向标准兼容接口和上位机显示接口：

- `0x60B8..0x60BD` Touch Probe
- `0x60FD` Digital Inputs
- `0x60FE` Physical Outputs / Bit Mask

当前状态说明：

- 触发锁存、输入捕获、GPIO 联动等硬件行为还没有完整接入。
- 因此它们当前更适合作为接口预留和显示对象，而不是完整功能对象。

### 9.7 PDO 通道已开放，但 RPDO3/4、TPDO3/4 仍偏“可配置通道”

以下 PDO 对象已经开放：

- `0x1400..0x1403`
- `0x1600..0x1603`
- `0x1800..0x1803`
- `0x1A00..0x1A03`

当前状态说明：

- RPDO1/2、TPDO1/2 已经形成主链路。
- RPDO3/4、TPDO3/4 更偏向动态配置和扩展用途。
- 如果主站不主动配置，它们通常不会自动形成完整控制业务流。

### 9.8 文档标注建议

后续继续补齐 CiA402 功能时，建议把对象按下面三类标注：

- `Fully connected`
  已经真正接入控制闭环、状态机或保护逻辑
- `Storage / feedback only`
  当前主要用于参数保存或状态反馈
- `Reserved / compatibility`
  当前主要用于标准兼容、扩展预留或后续功能补齐

这样在做主站联调、移植到新工程或补功能时，可以快速判断：

- 哪些对象已经能真正控制电机
- 哪些对象当前只能读写
- 哪些对象还需要继续向下接入 FOC / 保护 / 状态机
## 10. CSP startup behavior

Current `0512_BLDC_STM32` behavior for CiA402 CSP is now:

- Writing `0x6060 = 8` switches to CSP and immediately holds `fPosRef` at the current actual position.
- Entering `Operation Enabled` with controlword `0x000F` no longer starts motion by itself in CSP.
- In CSP, the axis is allowed to start only after at least one valid `0x607A` target position has been received.
- `0x607A` is still latched on `SYNC`; receiving the target alone only arms the pending position.

Recommended bring-up order:

1. Write `0x6060 = 8`
2. Drive CiA402 state machine with `0x6040 = 0x0006`, `0x0007`, `0x000F`
3. Cyclically send target position `0x607A`
4. Cyclically send `SYNC`

Expected effect:

- If the master only enters OP and does not send `0x607A`, the motor should stay still.
- If the master sends `0x607A` but no `SYNC`, the new CSP target remains pending and should not be applied yet.
- Motion should start only after target position traffic and `SYNC` are both present.
- If CSP is already running and `SYNC` stops for about 100 ms, the drive now freezes the position reference and waits for the next valid target/SYNC pair.

## 11. CSP feedback chain

For CSP to be directly commissionable, `0512_BLDC_STM32` now follows the same core idea used by the SG EtherCAT project:

- `0x607A` is the target position source.
- `SYNC` latches the pending CSP target into `g_axis.posCtrl.fPosRef`.
- The position loop uses `g_axis.posCtrl.iAbsRawPos - g_axis.posCtrl.iZeroAngle` as the measured position.
- `0x6064` and following error objects read back the same measured position source.

Implementation note:

- `iAbsRawPos` must be a continuously accumulated encoder count, not a one-time initialized value.
- Encoder calibration now also aligns `iZeroAngle` with the calibrated raw position so that CiA402 position feedback and the local position loop share the same zero reference.

If `0x607A` changes but `0x6064` stays almost constant, the first thing to check is no longer the CSP send rate, but whether the encoder feedback chain is really updating the continuous position count.
