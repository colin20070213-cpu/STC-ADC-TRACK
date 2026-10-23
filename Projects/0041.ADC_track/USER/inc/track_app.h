/** @file track_app.h
 *  @brief 应用入口：初始化一次，主循环持续调用任务函数。
 */
#ifndef TRACK_APP_H
#define TRACK_APP_H
/** @brief 读取持久参数、清空本次标定、进入菜单并配置看门狗。 */
void TrackApp_Init(void);
/** @brief 非阻塞执行按键、采样/控制、遥测和分段显示任务。 */
void TrackApp_Task(void);
#endif
