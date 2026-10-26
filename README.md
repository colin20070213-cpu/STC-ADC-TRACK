# STC-ADC-TRACK 四电感与单编码器速度闭环

当前分支为 four-sensor-1ms。双电感历史完整版本保留在 main 和 [dual-stable-20261023](https://github.com/colin20070213-cpu/STC-ADC-TRACK/tree/dual-stable-20261023)，标签对应提交 1d4bbad2491c0c1bc7a7a1e43b5674c4dcc48344。

详细说明：[项目说明](Projects/0041.ADC_track/README.md)、[速度闭环](Projects/0041.ADC_track/SPEED_LOOP_GUIDE.md)、[四电感](Projects/0041.ADC_track/FOUR_SENSOR_GUIDE.md)、[发车步骤](Projects/0041.ADC_track/STARTUP_CHECKLIST.md)。
Keil 工程：Projects/0041.ADC_track/MDK/Project_Template.uvproj。
HEX：Projects/0041.ADC_track/MDK/Objects/Project_Template.hex。

- 外侧横电感 P13/P01；前方同平面纵电感当前 P10/P05，以实际接线核对宏。
- 论文 C4 加权偏差、四路独立归一化及每次上电重新标定。
- ADC 整帧触发 1ms，转向 PID 2ms，编码器快照 1ms，速度闭环 2ms；SPT5410LV 舵机载波 330Hz，历史舵机单位保持 2us。
- 右后轮 P04/TIM3 计数，P52 沿用原 Encoder 库方向电平；一套速度反馈产生共同 PWM，两电机同值，不差速转向。
- 主菜单 PID 调参包含转向 PID 和速度闭环调参；电机菜单包含速度设定、动力限幅、左右电机方向。速度目标初值为 0，首次需设置非零 BASE，否则显示 NEED SPEED；单位是 P04 脉冲/10ms。
- 代码初值：转向 Kp=130、Ki=0、Kd=8.15；速度 Kp=30、Ki=30、Kd=0。速度增益需实车整定。PWM 基础上限 3000、弯道上限 1500、允许最大 4000；有效 EEPROM 优先。
- 速度目标和速度 PID 支持掉电保存，原 32B 参数记录后扩展 32B SPD1，仍使用原两个 EEPROM 扇区；旧参数兼容读取，旧记录速度目标初始化为 0。
- 蓝牙 115200/8N1，每 500ms 只发送四列数字：转向目标×1000、转向反馈×1000、速度目标×100、速度反馈×100。无表头、参数快照或其他诊断列。
- 运行关闭 OLED，确认失线/故障/手动停止后亮屏。普通弱信号确认 250ms，四路近零确认 120ms。
- 43 个原库文件全部保留。原 Encoder 接口保留并增加稳定快照，处理计数回绕；原 58 个 Keil 引用保留，application 组新增速度模块后共 60 个引用。

完整主机回归通过，覆盖实际编码器库、速度闭环负载仿真、同值输出联锁、静态调参电机关闭、Flash 升级和全部 65 个擦写断点。Keil 全量编译 0 错误、0 警告，HEX 不进入 EEPROM 地址范围。软件测试不能替代实车编码器信号、采样吞吐、纠偏方向及速度增益验证。

在 Projects/0041.ADC_track 运行 tools/test.ps1、tools/build.ps1。BASELINE_MANIFEST.json 对应旧双路基线；FOUR_SENSOR_MANIFEST.json 对应当前分支文件。

2026-10-26 更新：TIM0 每 1ms 保存带时间戳的原始编码器快照，前台每 2ms 按实际间隔更新速度闭环。新增采集失败锁存和过期拒绝；采集函数只在 ISR 调用以避免 C251 非重入问题。速度与微分滤波按时间常数计算，速度标度和原 Flash 格式保持不变。SQ15S03E210BSAS1 完整型号的官方额定频率尚未核实，实际脉冲频率与运行中 2ms 执行耗时仍需上车测量。
