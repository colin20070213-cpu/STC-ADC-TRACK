# STC-ADC-TRACK 四电感与单编码器速度闭环

当前分支 four-sensor-1ms。双电感完整基线保留在 main 和 [dual-stable-20261023](https://github.com/colin20070213-cpu/STC-ADC-TRACK/tree/dual-stable-20261023)，标签提交 1d4bbad2491c0c1bc7a7a1e43b5674c4dcc48344 不变。

- [项目说明](Projects/0041.ADC_track/README.md)
- [速度闭环与合并后的菜单](Projects/0041.ADC_track/SPEED_LOOP_GUIDE.md)
- [C4 权重调参](Projects/0041.ADC_track/C4_TUNING_GUIDE.md)
- [开始运行前操作](Projects/0041.ADC_track/STARTUP_CHECKLIST.md)

Keil 工程：Projects/0041.ADC_track/MDK/Project_Template.uvproj。
HEX：Projects/0041.ADC_track/MDK/Objects/Project_Template.hex。

当前电机设置只保留一组BASE/CURVE，默认2500/1400，按PWM刻度输入。按参考程序0.104/20ms换算速度目标，内部10ms目标为指令乘0.052；最终PWM=基础指令+速度PID修正，输出限幅0～4000，无目标/PWM慢爬升。原额外速度字段只作存储兼容。系数尚需本车实测。

外侧横电感P13/P01，前方同平面纵电感当前宏P10/P05。ADC整帧触发和编码器快照均为1ms，转向/速度PID均为2ms；SPT5410LV舵机载波330Hz，每历史舵机单位2us，代码三值C=734/L=820/R=650。原四路独立归一化、每次上电电感/居中标定、运行熄屏及故障亮屏保留。

右后轮P04/TIM3连续计数，P52原始电平；两电机共同PWM，不差速。两套PID和C4都支持0.01/0.10/1.00三档及掉电保存；当前保留用户速度宏50/0/0，已有有效Flash优先。SPD2总长80B，复用原两个512B扇区，EEPROM仍至少1KB；新保存使用PWM1标记，兼容旧v1/v2/SPD1/PPS1及无标记SPD2，旧原动力数字保留为PWM刻度指令，其他原参数保留。

蓝牙115200/8N1，只输出四列：转向目标×1000、转向反馈×1000、速度指令、速度反馈（换算后的PWM指令刻度）。上一帧发完即发送最新值，无额外表头和配置行。

主机回归通过，覆盖0.104/20ms换算、PWM前馈和正负积分、负载扰动、唯一指令菜单、旧数值迁移、第一帧立即响应、共同PWM、零目标/故障停止、C4、原ADC和各个81个擦写中断点。Keil编译0错误0警告，HEX与EEPROM边界检查通过。软件验证不能替代实际编码器、电磁信号、机械限位及速度PID实车整定。

全部43个原库文件及60个Keil引用保留。运行tools/test.ps1和tools/build.ps1进行验证；BASELINE_MANIFEST.json为旧双路基线，FOUR_SENSOR_MANIFEST.json为当前分支清单。
