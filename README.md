# STC-ADC-TRACK 双电感基线

2026-10-23：四电感及 1ms 采样改造前的当前用户版本。

完整保留 Libraries（包括原版权、可选库及预编译库）和 Projects/0041.ADC_track 目录关系。
Keil 工程：Projects/0041.ADC_track/MDK/Project_Template.uvproj。
固件：Projects/0041.ADC_track/MDK/Objects/Project_Template.hex。
在项目目录运行 tools/build.ps1、tools/test.ps1；主机测试需要 GCC，Keil 路径可通过 build.ps1 的 -UV4 指定。

当前代码初值：Kp=115、Ki=0、Kd=2.15；BASE=2000、CURVE=1000；ADC/控制周期 5ms；舵机 PWM 50Hz；UART 9600。有效 EEPROM 参数仍优先。

验证：Keil 全量编译 0 错误、0 警告，HEX 地址检查通过；OLED、IAP、独立 PID、双/四路核心算法测试通过。现有应用测试第 284 行仍把 Kd 默认值写死为 0.15，而用户已改为 2.15，因此该断言失败，后续测试未继续；原测试原样备份。此基线保留当前源程序，不在上传前混入四电感修改。

BASELINE_MANIFEST.json 记录各文件 SHA-256，可用于核对还原。旧编辑器缓存、编译临时文件及历史压缩包未加入远端仓库；完整本地改造前压缩备份另存 MDK/dual-before-four-20261023.zip。
