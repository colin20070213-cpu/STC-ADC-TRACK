# 代码阅读与修改指南

这份指南用于你自己维护当前双电感版本。应用文件统一采用 UTF-8 中文注释、4 空格缩进和独占一行的大括号；主要函数有用途说明，公开接口在头文件中说明参数、返回值、单位和限制。项目根目录的 `.editorconfig` 提供编辑器格式设置。原库的版权声明继续保留；生成的字体表由脚本维护。

库可以修改或新增，不能删除。Keil 工程中保留 `startup / user / boards / drivers` 的完整库引用；`application` 只存放新增循迹模块。
可选无线驱动 `Ci24R1` 与 `nrf24l01` 存在重名接口，两份源码均在 boards 分组显示，原模板编译选择保持为 nrf24l01。

## 先从哪里看

| 想做什么 | 打开哪个文件 | 主要入口 |
| --- | --- | --- |
| 改 PID 代码初值 | `USER/inc/track_config.h` | 顶部三个 `TRACK_PID_DEFAULT_*` 宏 |
| 看 PID 如何计算 | `USER/inc/track_pid.h`、`USER/src/track_pid.c` | `TrackPIDGains`、`TrackPIDState`、`TrackPID_Calculate()` |
| 看数据结构与单位 | `USER/inc/track_core.h` | `TrackParams`、`TrackSensors`、`TrackControl` |
| 改归一化与居中检查 | `USER/src/track_core.c` | `TrackNormalize()`、`TrackBodyCenter_*()` |
| 改循迹判断和降速 | `USER/src/track_core.c` | `TrackControl_Step()`、`update_crossing()`、`update_motor_duty()` |
| 改某个页面 | `USER/src/track_app.c` | `render_*()` 绘制，`handle_*_keys()` 操作 |
| 改启动/调度 | `USER/src/main.c`、`USER/src/track_app.c` | `main()`、`TrackApp_Init()`、`TrackApp_Task()` |
| 查接线和实际输出 | `USER/src/track_hw.c` | `sensor_pins`、`TrackHW_Init()`、`TrackHW_Set()` |
| 改方向设置 | `USER/src/track_app.c`、`USER/src/track_core.c` | `handle_direction_keys()`、`TrackParams_SetServoDirection()` |
| 查掉电存储 | `USER/src/track_store.c` | `TrackStore_Load()`、`TrackStore_Save()` |

## 一帧控制如何走

1. TIM0 中断每 1ms 维护时基、按键和截止保护，不执行 PID。
2. 主循环持续调用 `TrackApp_Task()`，到达 5ms 标称周期时调用 `control_task()`。
3. `TrackHW_Sample()` 读 ADC 并去极值平均，`TrackSensors_Update()` 更新归一化数据。
4. `TrackControl_Step()` 先检查失线，再做居中补偿、十字判断，然后调用独立 PID，最后算电机动力。
5. 应用层检查故障返回值。运行中短暂弱信号由核心层保持转向并限制动力，仍将受限命令交给 `TrackHW_Set()`；确认失线后停车。静态 PID 模式下 `signal_ok=0` 则立即关闭预览。
6. 只有运行模式允许传入电机使能；其他页面关闭电机。速度页、居中标定页还关闭舵机脉冲。

OLED 每次只发 8 字节，串口使用发送队列。不要在控制中加入整屏刷新、等待按键松开或长时间延时。

## PID 的参数和状态分开保存

`TrackParams` 的 `kp100/ki100/kd100` 是可保存参数，使用实际增益的 100 倍，避免屏幕和 Flash 处理小数。例如 4000 表示 Kp=40.00。

`TrackPIDGains` 是参与计算的实际浮点增益，核心控制层负责转换。

`TrackPIDState` 是运行历史，包括积分、上次误差、滤波微分及本次输出。它放在 `TrackControl.steering_pid` 中，不写 Flash。换模式或改变增益时调用重置，避免旧积分/微分状态影响新参数。

PID 输出为相对机械中值的偏移。核心层根据左右限位换算偏移范围，最后计算：

`舵机命令 = 机械中值 + 限幅后的 PID 偏移`。

它是位置式 PID；电机目前没有编码器反馈，不能把电机动力设置当成闭环车速。

转向目标由 `TRACK_STEERING_TARGET1000` 定义，默认 0；位置反馈通过 `TrackSensors_Feedback()` 获取，来自电感差比和及居中补偿。
原始小信号仍映射到 0～1000；`ratio_per_mille()` 用十轮商余递推计算千分比，`TrackSensors_Error()` 先算非负幅值再恢复符号，`center_average()` 显式求有符号均值，避免这些路径依赖目标芯片的 32 位乘除库。
`TrackSensors_MathCheck()` 在实际固件上电时执行；失败置 `TRACK_MATH_ERROR`，不能进入依赖电感计算的执行模式。不要通过把 E 固定为零或强行删除数学库来掩盖错误。
保留本车已有符号约定：普通循迹时 PID 误差为“反馈减目标”，最终乘舵机方向。十字策略处理后的误差保存在 `control.error`，原始位置反馈可独立读取。
蓝牙 CSV 末尾追加 `target1000`、`feedback1000` 两列，前 37 列顺序不变。UART 默认 9600 / 8N1，完整遥测为 250ms 一帧，PID 计算仍为 5ms 一次。

## 自己在代码里调 PID

1. 修改 `track_config.h` 顶部的三个宏，直接写实际值，不乘 100。
2. Kp 范围 0～500，Ki/Kd 范围 0～20，保持两位小数精度，并使用 `f` 后缀。当前用户设定的 Kp 初值为 200.00，允许上限为 500.00。
3. 编译、下载。若 Flash 有有效记录，继续使用该记录，不会因为宏变化自动覆盖。
4. 进入 PID 调参页面，上下选到第四项“恢复PID初值”。
5. 第一次中键载入宏初值到 RAM 预览；再次中键确认保存到 Flash；左键撤销。
6. 之后仍可用原有 0.01/0.10 步长微调并保存。

恢复初值只调用 `TrackParams_RestorePID()`，不应调用恢复全部参数的 `TrackParams_Default()`。这保证已经设置好的舵机和速度不被重置。

## 自己修改速度

现场用“电机设置 → 速度设定”，修改 `BASE` 和 `CURVE`。关系为 `0 ≤ CURVE ≤ BASE ≤ TRACK_DUTY_MAX`。

- `BASE=1000` 表示 10.00% 动力 PWM；`CURVE=600` 表示 6.00%。
- 速度页精调 10、粗调 100，分别对应 0.10/1.00 个百分点。
- `TRACK_BASE_DUTY`、`TRACK_CURVE_MIN_DUTY` 是无有效 Flash 时的默认值。
- `TRACK_DUTY_MAX` 是菜单校验和实际输出共用的上限，当前 4000，即 40.00%；提高上限不会自动改动已经保存的 BASE/CURVE。

改 PWM 不需要改 PWM 频率。频率由独立宏配置，属于硬件调试参数。

## 修改失线容错

`hold_weak_signal()` 在信号总和低于 `params.lost_sum` 时累计 `TrackControl.weak_ms`，普通弱信号持续约 100ms 才返回 `TRACK_LOST`。两路连续小于 15 时另累计 `zero_ms`，约 50ms 即返回失线。计时按标称 5ms 控制周期累计。

确认期间保留上次有效舵机命令，清除十字状态及旧 PID 历史；每轮动力取原值、CURVE、600 的最小值，禁止加速。总和恢复到 `lost_sum + 50` 才清零计时并恢复正常控制。`signal_ok=0` 表示反馈当前不适合正常 PID，并不意味着运行模式已经停车。

时长、恢复余量和确认期间动力上限分别集中在 `TRACK_LOST_CONFIRM_MS`、`TRACK_ZERO_CONFIRM_MS`、`TRACK_LOST_RECOVER_MARGIN`、`TRACK_LOST_DUTY_MAX`。这些是前台的信号判断，不改变中断的 20ms 输出更新截止保护；ADC 故障和手动停车仍沿用原处理。

## 菜单层级和方向设置

`MenuPage` 表示主菜单、标定、舵机和电机子菜单；`menu_selection[]` 为各层保留光标。
`root_labels` 管主菜单，`calibration_menu`、`servo_menu`、`motor_menu` 列出子菜单标签与目标模式。
`open_menu_page()` 切换层级；`enter_menu_item()` 进入叶页面；`return_menu()` 从叶页面回到原大类。
只有在主菜单“开始运行”才允许电机输出，方向页不试转。

方向保存在 `TrackParams.servo_left_sign`、`left_reverse`、`right_reverse` 中，宏仅是缺省值。
舵机 PID 使用运行时的 `servo_left_sign`；切换时必须调用 `TrackParams_SetServoDirection()`，
同时交换左右限位标签，不能只翻转符号而留下相互矛盾的限位顺序。
电机方向在启动读取参数和保存成功后，经 `TrackHW_SetMotorDirections()` 应用到驱动层；
该接口只接受 0/1，且在执行器仍 Arm 时拒绝配置。调参预览值未保存时不会替换驱动层方向。

## 页面参数的四份快照

应用层没有把所有临时值混在同一个变量中：

| 变量 | 用途 |
| --- | --- |
| `params` | 当前运行或预览的值 |
| `saved_params` | 最近一次成功保存值，用于 `*` 修改标记 |
| `entry_params` | 进入页面时的值，退出或故障恢复参考 |
| `edit_params` | 开始本次编辑时的值，左键撤销用 |

所有写 Flash 操作都经过 `save_params()`。保存失败不更新已保存快照，也不假装成功；仍可撤销或重试。

## 哪些数据会掉电保留

PID、舵机三值、三个方向、基础/弯道动力和失线阈值保存在同一条参数记录中。记录明确指定各字段的字节位置，没有直接把 C 结构体内存写入 Flash。

电感极值、居中参考、积分和微分历史只在 RAM 中。重新提交电感量程后，必须重新居中标定。

当前写入 v2：第 26 字节 bit0 为舵机反向、bit1 为左电机反向、bit2 为右电机反向，其余位必须为 0；记录仍为 32 字节。
兼容读取 v1 并沿用旧控制参数；由旧左右限位推断舵机方向，电机方向取代码初值。
读取不自动升级写入，下一次中键保存时才写 v2。正确的 IAP 槽位是 0x000000 和 0x000200，分别对应 CPU 地址 FE0000 和 FE0200；原程序区末尾地址属于错误配置，不再读写。
STC-ISP 需设置至少 1KB 用户 EEPROM，当前程序布局下不超过 64KB；是否在下载时擦除 EEPROM 取决于下载选项。正确槽位无有效记录时，默认舵机 C=760/L=840/R=670；已有有效记录仍优先。`TrackStore_LastError()` 返回最近一次保存失败的阶段。
标定仍使用最小跨度 100、居中单路强度 100、居中偏差限制 300 和偏差极差 60。当前用户配置的默认失线总强度为 40、双路近零门槛为 15。`TrackParams_Valid()` 与配置共用 `TRACK_LOST_SUM_MIN/MAX`，合法范围 1～500，允许阈值 40 随速度设置保存；禁止为 0。读取 EEPROM 时沿用已有迁移规则：旧弱信号默认 100 改为当前代码初值，其他保存值保持优先；读取不自动写回。
旧程序不识别 v2，回退旧程序时需重新核对参数。

## 改完怎么检查

运行 `tools/test.ps1` 做主机测试，运行 `tools/build.ps1` 做 Keil 全量编译和 HEX 边界检查。修改中文菜单时先运行 `tools/generate_fonts.ps1`；修改界面后可运行测试及 `tools/render_previews.py` 查看真实缓冲区预览。

软件验证通过后，还要架空检查纠偏方向、舵机机械限位和调参页电机禁止运行，再低速验证实际赛道。测试程序不会代替实物验证。
