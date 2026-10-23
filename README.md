# STC-ADC-TRACK 四电感与 1ms ADC

当前分支为 four-sensor-1ms。改造前双电感完整版本保存在 main 和 [dual-stable-20261023](https://github.com/colin20070213-cpu/STC-ADC-TRACK/tree/dual-stable-20261023)，该标签始终对应提交 1d4bbad2491c0c1bc7a7a1e43b5674c4dcc48344。

新版说明：[四电感与 1ms 采样](Projects/0041.ADC_track/FOUR_SENSOR_GUIDE.md)。
Keil 工程：Projects/0041.ADC_track/MDK/Project_Template.uvproj。
新版 HEX：Projects/0041.ADC_track/MDK/Objects/Project_Template.hex。

- 两侧横电感 P13/P01 不变；中间同平面纵电感引脚宏必须按实际接线修改，当前 P16/P05 仅占位。
- 论文 C4 四路加权误差、独立归一化、居中参考和四路失线检测。
- TIM0 每 1ms 触发整帧非阻塞 ADC 扫描，ADC 中断推进；PID 保留 5ms。
- 舵机型号未确认，控制载波保持 50Hz；三值与载波解耦，1 单位始终为 2us。
- 当前宏初值 Kp=115、Ki=0、Kd=2.15、BASE=2000、CURVE=1000；EEPROM 优先。
- 蓝牙 9600/8N1、4Hz；保留前 39 列并增加 ADC 整帧/跨周期计数两列。
- 所有 43 个原库文件和 58 个 Keil 工程引用均保留，原库内容未改。

主机完整测试通过（含 6561 组独立公式比较、ADC 滤波/原子整帧/过期关闭、双/四路核心、Flash、菜单、PWM 保护及不同舵机载波换算）。Keil 全量编译 0 错误、0 警告，HEX 与 EEPROM 地址边界验证通过。尚未验证实物 1ms 吞吐及新舵机型号；先确认接线/载波、重新标定后再实车验证。

在 Projects/0041.ADC_track 目录运行 tools/test.ps1、tools/build.ps1；GCC/Keil 环境需正确配置。BASELINE_MANIFEST.json 对应旧标签原文件；FOUR_SENSOR_MANIFEST.json 对应当前分支内容。
