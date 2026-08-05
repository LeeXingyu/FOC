# 1. 模块关系图

下面是最新版 0512 的模块关系梳理。

```mermaid
flowchart TB
    subgraph App[应用 / 调试入口]
        main[main.c / main.h]
        ui[上层命令 / 调试入口]
    end

    subgraph Task[任务层]
        mc_tasks[mc_tasks.c]
        tem_task[tem_task.c]
    end

    subgraph Interface[接口层]
        mc_interface[mc_interface.c]
        mc_interface_h[mc_interface.h]
    end

    subgraph Control[控制层]
        motor_control[motor_control.c]
        foc[foc.c]
        param_id[param_identify.c]
        curr_auto[curr_autotune.c]
        speed_auto[speed_autotune.c]
    end

    subgraph Fbdk[反馈 / FBDK]
        encoder[encoder.c]
        speed_pos[speed_pos_fbdk.c]
        curr_fbdk[curr_fbdk.c]
    end

    subgraph Safety[安全 / 保护]
        safety[safety_task.c]
        safety_type[safety_type.h]
    end

    main --> mc_tasks
    ui --> mc_interface
    mc_tasks --> mc_interface
    mc_tasks --> motor_control
    mc_tasks --> param_id
    mc_tasks --> curr_auto
    mc_tasks --> speed_auto

    mc_interface --> motor_control
    mc_interface --> speed_pos

    motor_control --> foc
    motor_control --> curr_fbdk
    motor_control --> encoder
    motor_control --> speed_pos
    motor_control --> mc_tasks

    param_id --> motor_control
    param_id --> curr_fbdk
    param_id --> foc
    param_id --> speed_pos

    curr_auto --> motor_control
    curr_auto --> curr_fbdk
    curr_auto --> encoder
    curr_auto --> foc
    curr_auto --> speed_pos

    speed_auto --> motor_control
    speed_auto --> curr_fbdk
    speed_auto --> speed_pos
    speed_auto --> mc_interface

    encoder --> curr_fbdk
    speed_pos --> encoder
    curr_fbdk --> safety
```

## 读取方式

- `mc_tasks.c` 是状态调度中心，决定当前走空闲、校准、整定、运行或故障。
- `mc_interface.c` 是对外操作入口，负责启停、状态字、控制字和基础配置。
- `motor_control.c` 是主控制逻辑，负责 FOC、速度环、电流环和开环运行。
- `curr_autotune.c` 与 `speed_autotune.c` 是 SG 风格核心增量，负责参数识别和闭环整定。
- `encoder.c` 与 `speed_pos_fbdk.c` 提供位置和速度反馈基础能力。
- `curr_fbdk.c` 负责 PWM、电流采样和驱动输出，是硬件抽象底座。
- `safety_task.c` 是保护和故障处理的外围支撑。

## 关键路径

1. 初始化：`Motor_Control_Init()`
2. 调度：`High_Frequency_Task()`
3. 运行：`FOC_Control()` -> `Curr_Control()` -> `Set_Phase_Duty()`
4. 整定：`MC_Calib_StartChain()` / `MC_Calib_StartParam()`
5. 恢复：`MC_Stop_Motor()` / `MC_Reset_Control_State()`

