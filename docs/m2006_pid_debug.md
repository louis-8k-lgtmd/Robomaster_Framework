# M2006 双环 PID 调试

当前配置为 CAN1 上的 C610 ID 1：反馈标准帧 `0x201`，控制标准帧 `0x200` 的第 1、2 个字节。电机线程每 10 ms 执行一次：接收反馈 → 位置 PID 生成速度目标 → 速度 PID 生成电流命令 → 打包并发送 CAN。

## 调整参数

- `modules/src/M2006.cpp` 顶部的 `kPositionKp/Ki/Kd` 是位置外环，`kPositionMaxSpeedRadS` 是外环输出的速度上限。
- 同一文件的 `kSpeedKp/Ki/Kd` 是速度内环，`kSpeedIntegralLimit` 是速度环积分限幅，`kCurrentCommandLimit` 是电流命令限幅。修改后重新构建、烧录。
- `modules/src/control_task.cpp` 顶部的 `kSineAmplitudeRad` 和 `kSinePeriodTicks` 控制正弦位置目标。当前以首次收到的输出轴角度为中心，幅值 ±0.5 rad、周期 6 s；位置参考为 `中心 + 0.5 sin(2πt/6)`，速度前馈为其导数，峰值约 0.524 rad/s。加速度前馈为二阶导数，峰值约 0.548 rad/s²。目标从当前角度开始，正反往复运动。

PID 的 `Ki` 和 `Kd` 针对 10 ms 调用周期整定，当前实现不自动乘除采样间隔。电流命令为 C610 协议数值，不是安培数；当前调试限幅为 ±2000。速度环试验参数 `Kp=225, Ki=2.5`，位置环 `Kp=10`；速度目标换向时清除上一方向的速度积分。位置模式还包含低速摩擦补偿、速度电流前馈 `kVelocityCurrentPerRadS=120`、正弦加速度电流前馈，以及实际速度接近零时最多约 ±200 的额外起转电流。速度前馈在当前正弦目标最大速度处贡献约 ±63 电流命令，减少为维持转速所需的持续速度误差。起转方向考虑未来约 0.4 秒的轨迹加速度，补偿随实际速度上升而消退。这些都是试验初值，尚未在实机上验证；若速度过冲或位置领先，先降低 `kVelocityCurrentPerRadS`，若换向抖动，先降低 `kBreakawayCurrent`，每次只改一个参数并比较曲线。

## 观察实时数值

在 VS Code 的 **Cortex Live Watch** 中添加以下全局变量字段。普通 Watch 在运行状态下可能显示 `not available`；Live Watch 显示数值，但不会自动生成曲线。

- `g_m2006_debug.feedback_frames`、`feedback_online`：确认电机反馈可用。
- `g_m2006_debug.position_target_rad`、`output_position_rad`、`position_error_rad`：位置外环。
- `g_m2006_debug.speed_target_active_rad_s`、`output_speed_rad_s`、`speed_error_rad_s`：速度内环。
- `g_m2006_debug.position_pid_speed_rad_s`、`speed_pid_current`、`current_command`：两级 PID 输出及发送电流。`current_command` 包含摩擦和加速度前馈补偿，因此与 `speed_pid_current` 可能不同。

如果 `feedback_online` 为 0，闭环会保持松开；先检查 `g_can_debug.last_rx_bus`、`last_rx_id`、`rx_m2006_frames`。

## 导出曲线

固件在 `g_m2006_trace` 中保留最近 512 个控制周期，约 5.12 秒。让电机运行几秒后**暂停**，在 Cortex-Debug 的 Debug Console 执行 GDB 命令：

```text
-exec dump binary memory D:/WorkSpace/Robomaster_Framework/m2006_trace.bin &g_m2006_trace[0] &g_m2006_trace[512]
```

随后在仓库根目录的 PowerShell 中运行（此路径为当前电脑已安装的 Python 运行时）：

```powershell
& 'C:\Users\louis\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe' scripts/plot_m2006_trace.py m2006_trace.bin
```

会生成 `m2006_trace.csv` 与 `m2006_trace.svg`。SVG 同时画出位置目标/反馈、速度目标/反馈及电流命令；CSV 还包含模式、反馈有效标志及两级 PID 输出。这个方法只读取暂停时保存的记录，不要求 SWO 或额外串口硬件。
