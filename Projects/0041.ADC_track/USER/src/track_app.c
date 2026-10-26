/**
 * @file    track_app.c
 * @brief   菜单、按键、显示、周期任务和故障处理。
 * @note    阅读顺序：页面绘制 -> 页面按键 -> 控制任务 -> 初始化/主循环。
 *          所有调参页均禁止电机运行，Flash 只在中键确认时写入。
 */
#include "track_app.h"
#include "track_hw.h"
#include "track_speed.h"
#include "track_font.h"
#ifdef TRACK_HOST_TEST
#include "track_app_test_hw.h"
#else
#include "LCD.h"
#include "Font.h"
#endif
#include <stdio.h>
#include <string.h>

/* 主机测试提供同名接口；固件由修正后的 LCD.h 声明列/页接口。 */
#ifdef TRACK_HOST_TEST
extern void LCD_Set_Pos(uint8_t x, uint8_t y);
extern void LCD_WrDat(uint8_t value);
extern void LCD_WrCmd(uint8_t command);
#endif

/** @brief 菜单层级；叶页面返回所属大类，子菜单左键返回主菜单。 */
typedef enum
{
    MENU_ROOT = 0,
    MENU_CALIBRATION,
    MENU_SERVO,
    MENU_MOTOR,
    MENU_PID,
    MENU_PAGE_COUNT
} MenuPage;

typedef enum
{
    ROOT_CALIBRATION = 0,
    ROOT_PID,
    ROOT_SERVO,
    ROOT_MOTOR,
    ROOT_RUN,
    ROOT_ITEM_COUNT
} RootMenuItem;

/** @brief 与字体生成脚本 labels 顺序对应的标签编号。 */
typedef enum
{
    LABEL_MAIN = 0, LABEL_SENSOR_CAL, LABEL_PID, LABEL_RUN,
    LABEL_SERVO_CENTER, LABEL_SERVO_LEFT, LABEL_SERVO_RIGHT,
    LABEL_BODY_CENTER, LABEL_SPEED, LABEL_PID_RESTORE,
    LABEL_CAL_GROUP, LABEL_SERVO_GROUP, LABEL_MOTOR_GROUP,
    LABEL_SERVO_DIRECTION, LABEL_LEFT_DIRECTION, LABEL_RIGHT_DIRECTION,
    LABEL_NORMAL, LABEL_REVERSED, LABEL_SPEED_PID, LABEL_PWM_LIMIT,
    LABEL_STEERING_TUNE, LABEL_SPEED_TUNE
} UILabel;

typedef struct
{
    uint8_t label;
    TrackMode target_mode;
} MenuEntry;

static const uint8_t root_labels[ROOT_ITEM_COUNT] =
{
    LABEL_CAL_GROUP, LABEL_PID, LABEL_SERVO_GROUP, LABEL_MOTOR_GROUP, LABEL_RUN
};
static const MenuEntry calibration_menu[] =
{
    {LABEL_SENSOR_CAL, TRACK_CAL},
    {LABEL_BODY_CENTER, TRACK_BODY_CENTER}
};
static const MenuEntry servo_menu[] =
{
    {LABEL_SERVO_CENTER, TRACK_CENTER},
    {LABEL_SERVO_LEFT, TRACK_LIMIT_LEFT},
    {LABEL_SERVO_RIGHT, TRACK_LIMIT_RIGHT},
    {LABEL_SERVO_DIRECTION, TRACK_SERVO_DIRECTION}
};
static const MenuEntry pid_menu[] =
{
    {LABEL_STEERING_TUNE, TRACK_PID},
    {LABEL_SPEED_TUNE, TRACK_SPEED_PID}
};
static const MenuEntry motor_menu[] =
{
    {LABEL_SPEED, TRACK_SPEED_TARGET},
    {LABEL_PWM_LIMIT, TRACK_SPEED},
    {LABEL_LEFT_DIRECTION, TRACK_LEFT_DIRECTION},
    {LABEL_RIGHT_DIRECTION, TRACK_RIGHT_DIRECTION}
};

#define PID_ITEM_RESTORE 3
#define PID_ITEM_COUNT   4

static TrackParams params;       /**< 当前运行/预览参数。 */
static TrackParams saved_params; /**< 最近一次成功保存的参数，用于显示是否有修改。 */
static TrackParams entry_params; /**< 进入页面时的参数，退出或故障时恢复。 */
static TrackParams edit_params;  /**< 开始一次编辑时的参数，左键撤销本次编辑。 */
static TrackSensors sensors;
static TrackControl control;
static TrackSpeedState speed_control;
static uint16_t encoder_last_count;
static uint16_t encoder_elapsed;
static uint8_t encoder_direction;
static uint8_t encoder_fresh;
static uint8_t encoder_ok;
static uint32_t last_encoder;
static TrackMode mode;
static TrackFault fault;

static MenuPage menu_page;
static uint8_t menu_item;
static uint8_t menu_selection[MENU_PAGE_COUNT]; /**< 每层分别记住光标。 */
static uint8_t pid_item;         /**< 0=Kp，1=Ki，2=Kd，3=恢复 PID 初值。 */
static uint8_t speed_item;       /**< 0=基础动力，1=弯道最低动力。 */
static uint8_t editing;
static uint8_t coarse_step;
static uint8_t defaults_loaded;
static uint8_t display_busy;
static uint8_t display_off;      /**< 运行中熄屏；停车后完整更新画面再亮屏。 */
static uint8_t cal_adc_error;
static uint8_t rail_count[TRACK_SENSOR_MAX];
static uint8_t startup_keys_locked;
static uint8_t sensor_math_ok;  /**< 上电在目标芯片执行整数计算自检。 */

static uint16_t sample_values[TRACK_SENSOR_MAX];
static uint16_t display_cursor;
static uint8_t framebuffer[1024]; /**< OLED 128×64，8 个竖向页。 */
static char text_line[48];
static char telemetry[64];
static char message[22];          /**< 6×8 字体一行最多显示 21 个 ASCII 字符。 */
static uint32_t last_control;
static uint32_t last_display;
static uint32_t last_telemetry;
static uint32_t startup_release_since;

/** @brief 将故障码转换成 OLED 可显示的短文本。 */
static const char *fault_name(TrackFault reason)
{
    switch (reason)
    {
        case TRACK_NO_CAL:      return "NEED CAL";
        case TRACK_ADC_ERROR:   return "ADC ERROR";
        case TRACK_LOST:        return "LOST";
        case TRACK_TIMEOUT:     return "TIMEOUT";
        case TRACK_MANUAL_STOP: return "STOPPED";
        case TRACK_BAD_PARAMS:  return "PARAM ERR";
        case TRACK_NO_CENTER:   return "NEED CENTER";
        case TRACK_MATH_ERROR:  return "MATH ERROR";
        case TRACK_NO_SPEED:    return "NEED SPEED";
        case TRACK_ENCODER_ERROR: return "ENC ERROR";
        case TRACK_ENCODER_STALL: return "ENC STALL";
        default:                return "OK";
    }
}

/** @brief 设置页面提示，自动截断到 OLED 单行宽度。 */
static void set_message(const char *text)
{
    strncpy(message, text, sizeof(message) - 1);
    message[sizeof(message) - 1] = 0;
}

/** @brief 请求下次空闲时立即重绘；不在按键处理中阻塞传输整屏。 */
static void request_display(void)
{
    last_display = TrackHW_Now() - TRACK_DISPLAY_PERIOD_MS;
}

/** @brief 参数相对已保存值是否变化；只作显示，不直接触发写 Flash。 */
static uint8_t params_dirty(void)
{
    return memcmp(&params, &saved_params, sizeof(params)) != 0;
}

/** @brief 获取当前车身居中采集的提示。 */
static const char *body_status(void)
{
    switch (sensors.center_status)
    {
        case TRACK_BODY_CAPTURING: return "HOLD STILL...";
        case TRACK_BODY_READY:     return "READY: OK TO SAVE";
        case TRACK_BODY_WEAK:      return "SIGNAL TOO WEAK";
        case TRACK_BODY_MOVING:    return "UNSTABLE: RETRY";
        case TRACK_BODY_OFFSET:    return "OFF CENTER: REALIGN";
        case TRACK_BODY_ADC:       return "ADC ERROR: RETRY";
        default: return sensors.center_valid ? "REFERENCE VALID" : "ALIGN BODY ON LINE";
    }
}

/** @brief 当前层级的项目数。 */
static uint8_t menu_count(void)
{
    switch (menu_page)
    {
        case MENU_CALIBRATION: return 2;
        case MENU_SERVO:       return 4;
        case MENU_MOTOR:       return 4;
        case MENU_PID:         return 2;
        default:               return ROOT_ITEM_COUNT;
    }
}

/** @brief 获取当前子菜单表；主菜单不使用返回结果。 */
static const MenuEntry *submenu_entries(void)
{
    switch (menu_page)
    {
        case MENU_SERVO: return servo_menu;
        case MENU_MOTOR: return motor_menu;
        case MENU_PID:   return pid_menu;
        default:         return calibration_menu;
    }
}

/** @brief 当前菜单层级的标题。 */
static uint8_t menu_title(void)
{
    switch (menu_page)
    {
        case MENU_CALIBRATION: return LABEL_CAL_GROUP;
        case MENU_SERVO:       return LABEL_SERVO_GROUP;
        case MENU_MOTOR:       return LABEL_MOTOR_GROUP;
        case MENU_PID:         return LABEL_PID;
        default:               return LABEL_MAIN;
    }
}

/** @brief 方向页只改参数，不输出舵机或电机。 */
static uint8_t is_direction_mode(void)
{
    return mode == TRACK_SERVO_DIRECTION ||
           mode == TRACK_LEFT_DIRECTION || mode == TRACK_RIGHT_DIRECTION;
}

/** @brief 当前方向页的反向标志：0=正向，1=反向。 */
static uint8_t direction_reversed(const TrackParams *values)
{
    if (mode == TRACK_SERVO_DIRECTION)
    {
        return values->servo_left_sign < 0;
    }
    return mode == TRACK_LEFT_DIRECTION ? values->left_reverse : values->right_reverse;
}

/** @brief 模式和标题独立映射，新增方向模式不改变旧标签编号。 */
static uint8_t mode_label(void)
{
    switch (mode)
    {
        case TRACK_CAL:             return LABEL_SENSOR_CAL;
        case TRACK_PID:             return LABEL_PID;
        case TRACK_RUN:             return LABEL_RUN;
        case TRACK_CENTER:          return LABEL_SERVO_CENTER;
        case TRACK_LIMIT_LEFT:      return LABEL_SERVO_LEFT;
        case TRACK_LIMIT_RIGHT:     return LABEL_SERVO_RIGHT;
        case TRACK_BODY_CENTER:     return LABEL_BODY_CENTER;
        case TRACK_SPEED:           return LABEL_PWM_LIMIT;
        case TRACK_SPEED_TARGET:    return LABEL_SPEED;
        case TRACK_SPEED_PID:       return LABEL_SPEED_PID;
        case TRACK_SERVO_DIRECTION: return LABEL_SERVO_DIRECTION;
        case TRACK_LEFT_DIRECTION:  return LABEL_LEFT_DIRECTION;
        case TRACK_RIGHT_DIRECTION: return LABEL_RIGHT_DIRECTION;
        default:                    return LABEL_MAIN;
    }
}

/* ==================== 页面绘制：只更新 framebuffer ==================== */

/** @brief 按像素 x、8 像素高的 page 坐标写 ASCII，超出屏幕则截断。 */
static void text_at(uint8_t x, uint8_t page, const char *text)
{
    uint8_t character;
    uint8_t column;

    while (*text && x <= 122 && page < 8)
    {
        character = (uint8_t)*text++;
        if (character < 32 || character > 126)
        {
            character = '?';
        }
        for (column = 0; column < 6; ++column)
        {
            framebuffer[(uint16_t)page * 128 + x++] = F6x8[character - 32][column];
        }
    }
}

/** @brief 画 16×16 中文标签；一个标签占两个 page，调用者须留出高度。 */
static void label_at(uint8_t x, uint8_t page, uint8_t label)
{
    uint8_t index;
    uint8_t glyph;
    uint8_t column;

    for (index = 0; index < 8 && x <= 112 && page < 7; ++index)
    {
        glyph = track_label_ids[label][index];
        if (glyph == 255)
        {
            break;
        }
        for (column = 0; column < 16; ++column)
        {
            framebuffer[(uint16_t)page * 128 + x + column] = track_glyphs[glyph][column];
            framebuffer[(uint16_t)(page + 1) * 128 + x + column] =
                track_glyphs[glyph][column + 16];
        }
        x += 16;
    }
}

/** @brief 反显一个菜单行（两页高）。 */
static void highlight_row(uint8_t page)
{
    uint16_t offset;

    for (offset = (uint16_t)page * 128; offset < (uint16_t)(page + 2) * 128; ++offset)
    {
        framebuffer[offset] ^= 0xFF;
    }
}

/** @brief 绘制三行滚动菜单；末屏显示最后三项。 */
static void render_menu(void)
{
    uint8_t first_item;
    uint8_t item;
    uint8_t page;
    uint8_t label;
    uint8_t count;
    const MenuEntry *entries;

    count = menu_count();
    entries = submenu_entries();
    label_at(0, 0, menu_title());
    sprintf(text_line, "%u/%u", (unsigned int)menu_item + 1, (unsigned int)count);
    text_at(104, 1, text_line);
    /* PID 大类标题有五字，状态文字从标题末尾之后开始，避免覆盖中文字模。 */
    text_at(menu_page == MENU_PID ? 86 : 70, 0, fault != TRACK_OK ? fault_name(fault) :
            (menu_page == MENU_ROOT ? (defaults_loaded ? "DEFAULT" : "READY") : "<:BACK"));
    first_item = (menu_item / 3) * 3;
    if (count > 3 && first_item > count - 3)
    {
        first_item = count - 3;
    }
    for (item = first_item; item < count && item < first_item + 3; ++item)
    {
        page = 2 + (item - first_item) * 2;
        sprintf(text_line, "%u", (unsigned int)item + 1);
        text_at(0, page, text_line);
        label = menu_page == MENU_ROOT ? root_labels[item] : entries[item].label;
        label_at(16, page, label);
        if (menu_item == item)
        {
            highlight_row(page);
        }
    }
}

/** @brief 极值标定页显示所有启用通道当前值/最小值/最大值。 */
static void render_sensor_calibration(void)
{
    uint8_t channel;

    for (channel = 0; channel < TRACK_SENSOR_COUNT; ++channel)
    {
        sprintf(text_line, "%u N%4u m%4u M%4u", (unsigned int)channel + 1,
                sensors.raw[channel], sensors.work_low[channel], sensors.work_high[channel]);
        text_at(0, (uint8_t)(channel + 2), text_line);
    }
    sprintf(text_line, "Samples:%u%s", sensors.samples, sensors.valid ? " OLD:OK" : "");
    text_at(0, 6, message[0] ? message : text_line);
    text_at(0, 7, "OK:save <:back >:new");
}

/** @brief 车身居中页显示原始差比和、参考和采样进度。 */
static void render_body_center(void)
{
    if (!sensors.valid)
    {
        sprintf(text_line, "RAW L:%u R:%u", sensors.raw[0], sensors.raw[1]);
        text_at(0, 2, text_line);
        text_at(0, 3, "1. NEED SENSOR CAL");
        label_at(0, 4, LABEL_SENSOR_CAL);
        text_at(0, 6, "MOTOR/SERVO OFF");
        text_at(0, 7, "OK:go cal <:back");
        return;
    }
    sprintf(text_line, "L:%4u R:%4u", (unsigned int)sensors.norm[0], (unsigned int)sensors.norm[1]);
    text_at(0, 2, text_line);
    sprintf(text_line, "E:%5d REF:%5d", (int)TrackSensors_Error(&sensors), (int)sensors.center_bias);
    text_at(0, 3, text_line);
    sprintf(text_line, "N:%u/%u %s", sensors.center_samples, (unsigned int)TRACK_BODY_SAMPLES,
            sensors.center_valid ? "VALID" : "UNSET");
    text_at(0, 4, text_line);
#if TRACK_SENSOR_COUNT == 4
    sprintf(text_line, "LM:%4u RM:%4u", (unsigned int)sensors.norm[2], (unsigned int)sensors.norm[3]);
#else
    sprintf(text_line, "ADC L:%4u R:%4u", (unsigned int)sensors.raw[0], (unsigned int)sensors.raw[1]);
#endif
    text_at(0, 5, text_line);
    text_at(0, 6, message[0] ? message : body_status());
    text_at(0, 7, sensors.center_status == TRACK_BODY_READY ?
            "OK:save >:new <:back" : "OK:start >:new <:back");
}

/** @brief 绘制恢复初值的独立选项页，第一次中键预览，第二次中键保存。 */
static void render_pid_restore(void)
{
    TrackParams preview;

    preview = params;
    if (!editing)
    {
        if (mode == TRACK_SPEED_PID) TrackParams_RestoreSpeedPID(&preview);
        else TrackParams_RestorePID(&preview);
    }
    if (mode == TRACK_SPEED_PID)
    {
        preview.kp100=preview.speed_kp100;preview.ki100=preview.speed_ki100;preview.kd100=preview.speed_kd100;
    }
    label_at(0, 2, LABEL_PID_RESTORE);
    sprintf(text_line, "Kp:%u.%02u", preview.kp100 / 100, preview.kp100 % 100);
    text_at(0, 4, text_line);
    sprintf(text_line, "Ki:%u.%02u", preview.ki100 / 100, preview.ki100 % 100);
    text_at(0, 5, text_line);
    sprintf(text_line, "Kd:%u.%02u", preview.kd100 / 100, preview.kd100 % 100);
    text_at(0, 6, text_line);
    text_at(0, 7, message[0] ? message :
            (editing ? "OK:save <:undo" : "OK:preview ^v:pick"));
}

/** @brief PID 前三项保持原布局，第四项单独显示恢复初值选项。 */
static void render_pid(void)
{
    uint8_t item;
    uint16_t gain;
    const char *gain_name;

    sprintf(text_line, "%u/4", (unsigned int)pid_item + 1);
    text_at(104, 1, text_line);
    if (pid_item == PID_ITEM_RESTORE)
    {
        render_pid_restore();
        return;
    }
    if (mode == TRACK_SPEED_PID)
    {
        sprintf(text_line, "V:%u/10ms MOTOR OFF", (unsigned int)(speed_control.feedback+0.5f));
        text_at(0, 2, encoder_ok ? text_line : "ENC ERROR / MOTOR OFF");
    }
    else text_at(0, 2, sensors.valid ?
            (control.signal_ok ? "MOTOR OFF / SIG OK" : "MOTOR OFF / NO SIGNAL") :
            "MOTOR OFF / NEED CAL");
    for (item = 0; item < 3; ++item)
    {
        if (mode == TRACK_SPEED_PID)
            gain = item == 0 ? params.speed_kp100 : item == 1 ? params.speed_ki100 : params.speed_kd100;
        else gain = item == 0 ? params.kp100 : item == 1 ? params.ki100 : params.kd100;
        gain_name = item == 0 ? "Kp" : item == 1 ? "Ki" : "Kd";
        sprintf(text_line, "%c%s:%3u.%02u %s", pid_item == item ? '>' : ' ', gain_name,
                gain / 100, gain % 100,
                editing && pid_item == item ? (coarse_step ? "EDIT .10" : "EDIT .01") : "");
        text_at(0, (uint8_t)(item + 3), text_line);
    }
    if (mode == TRACK_SPEED_PID)
        sprintf(text_line, "TARGET:%u PWM:%u%s", params.speed_base, speed_control.output, params_dirty() ? " *" : "");
    else sprintf(text_line, "E:%d S:%u%s%s", control.error, control.servo,
            control.saturated ? " LIM" : "", params_dirty() ? " *" : "");
    text_at(0, 6, text_line);
    text_at(0, 7, message[0] ? message :
            (editing ? "OK:save <:undo >:step" : "^v:pick >:edit <:back"));
}

/** @brief 速度页的百分比来自 PWM，不表示实测轮速；编辑期间输出保持关闭。 */
static void render_speed(void)
{
    if (mode == TRACK_SPEED_TARGET)
    {
        sprintf(text_line, "%cBASE:%u", speed_item == 0 ? '>' : ' ', params.speed_base);
        text_at(0, 2, text_line);
        sprintf(text_line, "%cCURVE:%u", speed_item == 1 ? '>' : ' ', params.speed_curve);
        text_at(0, 3, text_line);
        sprintf(text_line, "NOW:%u pulse/10ms", (unsigned int)(speed_control.feedback+0.5f));
        text_at(0, 4, text_line);
        sprintf(text_line, "STEP:%u OFF%s", coarse_step ? 10 : 1, params_dirty() ? " *" : "");
        text_at(0, 5, text_line);
        sprintf(text_line, "ENC:%u DIR:%u", encoder_last_count, (unsigned int)encoder_direction);
        text_at(0, 6, message[0] ? message : (encoder_ok ? text_line : "ENC ERROR"));
        text_at(0, 7, editing ? "OK:save <:undo >:step" : "^v:pick >:edit <:back");
        return;
    }
    sprintf(text_line, "%cBASE:%4u %2u.%02u%%", speed_item == 0 ? '>' : ' ',
            params.base_duty, params.base_duty / 100, params.base_duty % 100);
    text_at(0, 2, text_line);
    sprintf(text_line, "%cCURVE:%4u %2u.%02u%%", speed_item == 1 ? '>' : ' ',
            params.curve_duty, params.curve_duty / 100, params.curve_duty % 100);
    text_at(0, 3, text_line);
    sprintf(text_line, "LIMIT:%u = %u%%", (unsigned int)TRACK_DUTY_MAX,
            (unsigned int)TRACK_DUTY_MAX / 100);
    text_at(0, 4, text_line);
    sprintf(text_line, "STEP:%u%s", coarse_step ?
            (unsigned int)TRACK_SPEED_COARSE_STEP : (unsigned int)TRACK_SPEED_FINE_STEP,
            params_dirty() ? " *" : "");
    text_at(0, 5, text_line);
    text_at(0, 6, message[0] ? message : "MOTOR/SERVO OFF");
    text_at(0, 7, editing ? "OK:save <:undo >:step" : "^v:pick >:edit <:back");
}

/** @brief 显示实时循迹误差、PID 分量和输出。 */
static void render_run(void)
{
    sprintf(text_line, "L:%u R:%u", sensors.norm[0], sensors.norm[1]);
    text_at(0, 2, text_line);
    sprintf(text_line, "E:%d S:%u%s", control.error, control.servo,
            control.saturated ? " LIM" : "");
    text_at(0, 3, text_line);
    sprintf(text_line, "PWM:%u/%u", control.left_motor, control.right_motor);
    text_at(0, 4, text_line);
#if TRACK_SENSOR_COUNT == 4
    sprintf(text_line, "LM:%u RM:%u", sensors.norm[2], sensors.norm[3]);
#else
    sprintf(text_line, "P:%d I:%d D:%d", control.p_x10, control.i_x10, control.d_x10);
#endif
    text_at(0, 5, text_line);
    text_at(0, 6, control.crossing ? "CROSS: STRAIGHT" : "TRACKING");
    text_at(0, 7, "< or OK: STOP");
}

/** @brief 三个舵机页面共用显示；NOW 是预览值，SAVED 是 Flash 对应值。 */
static void render_servo(void)
{
    uint16_t current_value;
    uint16_t stored_value;

    current_value = mode == TRACK_CENTER ? params.center :
                    mode == TRACK_LIMIT_LEFT ? params.left : params.right;
    stored_value = mode == TRACK_CENTER ? saved_params.center :
                   mode == TRACK_LIMIT_LEFT ? saved_params.left : saved_params.right;
    sprintf(text_line, "NOW:%u%s", current_value, params_dirty() ? " *" : "");
    text_at(0, 2, text_line);
    sprintf(text_line, "SAVED:%u", stored_value);
    text_at(0, 3, text_line);
    sprintf(text_line, "C:%u L:%u R:%u", params.center, params.left, params.right);
    text_at(0, 4, text_line);
    sprintf(text_line, "STEP:%u  MOTOR OFF", coarse_step ? 10 : 1);
    text_at(0, 5, text_line);
    text_at(0, 6, message[0] ? message : "^v:adjust >:step");
    text_at(0, 7, "OK:save <:undo/back");
}

/** @brief 显示方向；设置期间不输出舵机脉冲或电机 PWM。 */
static void render_direction(void)
{
    label_at(0, 2, direction_reversed(&params) ? LABEL_REVERSED : LABEL_NORMAL);
    text_at(70, 2, params_dirty() ? "*" : "");
    sprintf(text_line, "SAVED:%s", direction_reversed(&saved_params) ? "REVERSE" : "NORMAL");
    text_at(0, 4, text_line);
    if (mode == TRACK_SERVO_DIRECTION)
    {
        sprintf(text_line, "C:%u L:%u R:%u", params.center, params.left, params.right);
        text_at(0, 5, text_line);
    }
    else
    {
        text_at(0, 5, "MOTOR/SERVO OFF");
    }
    text_at(0, 6, message[0] ? message : "OUTPUT OFF");
    text_at(0, 7, "^:N v:R OK:save <:out");
}

/** @brief 页面绘制总入口；每次先清空缓冲，避免上一页内容残留。 */
static void render_display(void)
{
    uint8_t title_label;

    memset(framebuffer, 0, sizeof(framebuffer));
    if (mode == TRACK_MENU)
    {
        render_menu();
        return;
    }
    title_label = mode_label();
    label_at(0, 0, title_label);
    switch (mode)
    {
        case TRACK_CAL:         render_sensor_calibration(); break;
        case TRACK_PID:
        case TRACK_SPEED_PID:   render_pid(); break;
        case TRACK_RUN:         render_run(); break;
        case TRACK_BODY_CENTER: render_body_center(); break;
        case TRACK_SPEED:
        case TRACK_SPEED_TARGET: render_speed(); break;
        case TRACK_SERVO_DIRECTION:
        case TRACK_LEFT_DIRECTION:
        case TRACK_RIGHT_DIRECTION:
            render_direction();
            break;
        default:                render_servo(); break;
    }
}

/** @brief 每次最多发送 8 字节 OLED 数据，避免整屏刷新阻塞控制。 */
static void display_task(uint32_t now)
{
    uint8_t byte_index;

    if (mode == TRACK_RUN)
    {
        return; /* 行驶中不绘制、不定位、不发送 OLED 数据或命令。 */
    }
    if (!display_busy && (display_off || (uint32_t)(now - last_display) >= TRACK_DISPLAY_PERIOD_MS))
    {
        render_display();
        display_cursor = 0;
        display_busy = 1;
        last_display = now;
    }
    if (display_busy)
    {
        LCD_Set_Pos((uint8_t)(display_cursor % 128), (uint8_t)(display_cursor / 128));
        for (byte_index = 0; byte_index < 8; ++byte_index)
        {
            LCD_WrDat(framebuffer[display_cursor++]);
        }
        if (display_cursor == sizeof(framebuffer))
        {
            display_busy = 0;
            if (display_off)
            {
                /* 先让故障/菜单整帧进入显存，避免亮屏时闪出旧画面。 */
                LCD_WrCmd(0xAF);
                display_off = 0;
            }
        }
    }
}

/* ==================== 页面切换与参数修改 ==================== */

/** @brief 切换主/子菜单，关闭输出并保存原层级光标。 */
static void open_menu_page(MenuPage page)
{
    TrackHW_Arm(0);
    menu_selection[menu_page] = menu_item;
    menu_page = page;
    menu_item = menu_selection[menu_page];
    mode = TRACK_MENU;
    fault = TRACK_OK;
    editing = 0;
    message[0] = 0;
    request_display();
}

/** @brief 关闭执行器并返回当前所属菜单；reason 用于显示停车原因。 */
static void return_menu(TrackFault reason)
{
    TrackHW_Arm(0);
    TrackSpeed_Stop(&speed_control);
    mode = TRACK_MENU;
    fault = reason;
    display_busy = 0;
    display_cursor = 0;
    control.left_motor = 0;
    control.right_motor = 0;
    editing = 0;
    message[0] = 0;
    request_display();
}

/** @brief 停止输出后写 Flash；失败不更新已保存快照，允许重试或撤销。 */
static uint8_t save_params(void)
{
    uint8_t stored;

    TrackHW_Arm(0);
    stored = TrackStore_Save(&params);
    last_control = TrackHW_Now();
    if (!stored)
    {
        switch (TrackStore_LastError())
        {
            case TRACK_STORE_PARAMS: set_message("FLASH PARAM ERROR"); break;
            case TRACK_STORE_BUSY:   set_message("FLASH OUTPUT ARMED"); break;
            case TRACK_STORE_ERASE:  set_message("FLASH ERASE FAILED"); break;
            case TRACK_STORE_WRITE:  set_message("FLASH WRITE FAILED"); break;
            case TRACK_STORE_READ:   set_message("FLASH READ FAILED"); break;
            case TRACK_STORE_VERIFY: set_message("FLASH VERIFY FAILED"); break;
            default:                set_message("FLASH SAVE FAILED"); break;
        }
        return 0;
    }
    saved_params = params;
    entry_params = params;
    defaults_loaded = 0;
    TrackHW_SetMotorDirections(params.left_reverse, params.right_reverse);
    set_message("SAVED OK");
    return 1;
}

/** @brief 参数变化后清空积分与微分历史，避免旧状态带入新参数。 */
static void reset_pid(void)
{
    TrackControl_Reset(&control, &params);
    TrackSpeed_Stop(&speed_control);
}

/** @brief 根据菜单索引进入页面；先关闭输出，再开启所需的舵机模式。 */
static void enter_menu_item(void)
{
    TrackMode target;
    const MenuEntry *entries;

    TrackHW_Arm(0);
    if (menu_page == MENU_ROOT)
    {
        switch (menu_item)
        {
            case ROOT_CALIBRATION: open_menu_page(MENU_CALIBRATION); return;
            case ROOT_SERVO:       open_menu_page(MENU_SERVO); return;
            case ROOT_MOTOR:       open_menu_page(MENU_MOTOR); return;
            case ROOT_PID:         open_menu_page(MENU_PID); return;
            case ROOT_RUN:         target = TRACK_RUN; break;
            default:               return;
        }
    }
    else
    {
        entries = submenu_entries();
        target = entries[menu_item].target_mode;
    }
    if (!sensor_math_ok &&
        (target == TRACK_PID || target == TRACK_RUN || target == TRACK_BODY_CENTER))
    {
        fault = TRACK_MATH_ERROR;
        request_display();
        return;
    }
    entry_params = params;
    editing = 0;
    coarse_step = 0;
    message[0] = 0;
    fault = TRACK_OK;
    reset_pid();
    memset(rail_count, 0, sizeof(rail_count));
    switch (target)
    {
        case TRACK_CAL:
            mode = TRACK_CAL;
            cal_adc_error = 0;
            TrackCal_Begin(&sensors);
            break;
        case TRACK_PID:
            mode = TRACK_PID;
            pid_item = 0;
            TrackHW_Arm(1);
            break;
        case TRACK_RUN:
            if (!sensors.valid)
            {
                fault = TRACK_NO_CAL;
                break;
            }
            if (!sensors.center_valid)
            {
                fault = TRACK_NO_CENTER;
                break;
            }
            if (!TrackParams_Valid(&params))
            {
                fault = TRACK_BAD_PARAMS;
                break;
            }
            if (params.speed_base == 0)
            {
                fault = TRACK_NO_SPEED;
                break;
            }
            if (!TrackHW_EncoderSample(&encoder_last_count, &encoder_direction, &last_encoder) ||
                (uint32_t)(TrackHW_Now() - last_encoder) > TRACK_ENCODER_STALE_MS)
            {
                fault = TRACK_ENCODER_ERROR;
                break;
            }
            encoder_ok = 1;
            encoder_fresh = 0;
            /* 先熄屏、取消未完成的分片刷新，再允许执行器工作。 */
            LCD_WrCmd(0xAE);
            display_off = 1;
            display_busy = 0;
            display_cursor = 0;
            mode = TRACK_RUN;
            TrackHW_Arm(1);
            break;
        case TRACK_CENTER:
        case TRACK_LIMIT_LEFT:
        case TRACK_LIMIT_RIGHT:
            mode = target;
            TrackHW_Arm(1);
            break;
        case TRACK_BODY_CENTER:
            mode = TRACK_BODY_CENTER;
            sensors.center_status = TRACK_BODY_IDLE;
            sensors.center_samples = 0;
            break;
        case TRACK_SPEED:
        case TRACK_SPEED_TARGET:
            mode = target;
            speed_item = 0;
            break;
        case TRACK_SPEED_PID:
            mode = target;
            pid_item = 0;
            break;
        case TRACK_SERVO_DIRECTION:
        case TRACK_LEFT_DIRECTION:
        case TRACK_RIGHT_DIRECTION:
            mode = target;
            break;
        default:
            fault = TRACK_BAD_PARAMS;
            break;
    }
    request_display();
}

/** @brief 在副本上调整当前字段；越界或关系不合法时不修改当前参数。 */
static void adjust_param(int8_t direction)
{
    TrackParams candidate;
    uint16_t *selected_value;
    uint16_t step;
    int32_t adjusted_value;

    candidate = params;
    step = coarse_step ? 10 : 1;
    if (mode == TRACK_SPEED_PID)
    {
        selected_value = pid_item == 0 ? &candidate.speed_kp100 :
                         pid_item == 1 ? &candidate.speed_ki100 : &candidate.speed_kd100;
    }
    else if (mode == TRACK_PID)
    {
        selected_value = pid_item == 0 ? &candidate.kp100 :
                         pid_item == 1 ? &candidate.ki100 : &candidate.kd100;
    }
    else if (mode == TRACK_SPEED_TARGET)
    {
        selected_value = speed_item == 0 ? &candidate.speed_base : &candidate.speed_curve;
        step = coarse_step ? 10 : 1;
    }
    else if (mode == TRACK_SPEED)
    {
        selected_value = speed_item == 0 ? &candidate.base_duty : &candidate.curve_duty;
        step = coarse_step ? TRACK_SPEED_COARSE_STEP : TRACK_SPEED_FINE_STEP;
    }
    else
    {
        selected_value = mode == TRACK_CENTER ? &candidate.center :
                         mode == TRACK_LIMIT_LEFT ? &candidate.left : &candidate.right;
    }
    adjusted_value = (int32_t)*selected_value + direction * (int16_t)step;
    if (adjusted_value < 0 || adjusted_value > 65535L)
    {
        set_message("LIMIT REACHED");
        return;
    }
    *selected_value = (uint16_t)adjusted_value;
    if (!TrackParams_Valid(&candidate))
    {
        set_message("LIMIT REACHED");
        return;
    }
    params = candidate;
    message[0] = 0;
    if (mode == TRACK_PID)
    {
        reset_pid();
    }
}

/** @brief 开始普通字段编辑，保存撤销快照。 */
static void begin_edit(void)
{
    edit_params = params;
    editing = 1;
    message[0] = 0;
}

/** @brief 将 PID 宏初值载入 RAM 预览；第二次中键才写 Flash。 */
static void preview_pid_defaults(void)
{
    TrackParams candidate;

    candidate = params;
    if (mode == TRACK_SPEED_PID) TrackParams_RestoreSpeedPID(&candidate);
    else TrackParams_RestorePID(&candidate);
    if (!TrackParams_Valid(&candidate))
    {
        set_message("PID DEFAULT INVALID");
        return;
    }
    begin_edit();
    params = candidate;
    reset_pid();
}

/** @brief PID 页按键；恢复选项不响应增减，避免误改默认预览。 */
static void handle_pid_keys(uint8_t keys)
{
    if (keys & KEY_LEFT_EVENT)
    {
        if (editing)
        {
            params = edit_params;
            editing = 0;
            reset_pid();
            set_message("EDIT CANCELLED");
        }
        else
        {
            return_menu(TRACK_OK);
        }
        return;
    }
    if (editing)
    {
        if (keys & KEY_OK_EVENT)
        {
            if (save_params())
            {
                editing = 0;
                reset_pid();
            }
            if (mode == TRACK_PID) TrackHW_Arm(1);
        }
        else if (pid_item != PID_ITEM_RESTORE)
        {
            if (keys & KEY_UP_EVENT)
            {
                adjust_param(1);
            }
            else if (keys & KEY_DOWN_EVENT)
            {
                adjust_param(-1);
            }
            else if (keys & KEY_RIGHT_EVENT)
            {
                coarse_step = !coarse_step;
            }
        }
        return;
    }
    if (keys & KEY_UP_EVENT)
    {
        pid_item = pid_item ? pid_item - 1 : PID_ITEM_COUNT - 1;
        message[0] = 0;
    }
    else if (keys & KEY_DOWN_EVENT)
    {
        pid_item = (pid_item + 1) % PID_ITEM_COUNT;
        message[0] = 0;
    }
    else if (pid_item == PID_ITEM_RESTORE && (keys & (KEY_OK_EVENT | KEY_RIGHT_EVENT)))
    {
        preview_pid_defaults();
    }
    else if (keys & KEY_RIGHT_EVENT)
    {
        begin_edit();
    }
    else if (keys & KEY_OK_EVENT)
    {
        save_params();
        if (mode == TRACK_PID) TrackHW_Arm(1);
    }
}

/** @brief 速度页沿用选择/编辑/撤销/保存方式；所有分支均不启动执行器。 */
static void handle_speed_keys(uint8_t keys)
{
    if (keys & KEY_LEFT_EVENT)
    {
        if (editing)
        {
            params = edit_params;
            editing = 0;
            set_message("EDIT CANCELLED");
        }
        else
        {
            return_menu(TRACK_OK);
        }
        return;
    }
    if (!editing)
    {
        if (keys & (KEY_UP_EVENT | KEY_DOWN_EVENT))
        {
            speed_item = !speed_item;
        }
        else if (keys & KEY_RIGHT_EVENT)
        {
            begin_edit();
        }
        else if (keys & KEY_OK_EVENT)
        {
            save_params();
        }
        return;
    }
    if (keys & KEY_UP_EVENT)
    {
        adjust_param(1);
    }
    else if (keys & KEY_DOWN_EVENT)
    {
        adjust_param(-1);
    }
    else if (keys & KEY_RIGHT_EVENT)
    {
        coarse_step = !coarse_step;
    }
    else if (keys & KEY_OK_EVENT)
    {
        if (save_params())
        {
            editing = 0;
        }
    }
}

/** @brief 极值采集按键；发生 ADC 错误后必须重新采集才能保存。 */
static void handle_calibration_keys(uint8_t keys)
{
    if (keys & KEY_LEFT_EVENT)
    {
        return_menu(TRACK_OK);
    }
    else if (keys & KEY_RIGHT_EVENT)
    {
        TrackCal_Begin(&sensors);
        cal_adc_error = 0;
        message[0] = 0;
    }
    else if (keys & KEY_OK_EVENT)
    {
        if (cal_adc_error)
        {
            set_message("ADC ERROR: > TO RESET");
        }
        else if (TrackCal_Commit(&sensors))
        {
            return_menu(TRACK_OK);
        }
        else if (sensors.bad_channel == 255)
        {
            set_message("MORE SAMPLES NEEDED");
        }
        else
        {
            sprintf(text_line, "CH%u SPAN/RAIL ERROR", (unsigned int)sensors.bad_channel);
            set_message(text_line);
        }
    }
}

/** @brief 居中采集按键；只有 READY 状态下的中键才提交参考。 */
static void handle_body_keys(uint8_t keys)
{
    if (keys & KEY_LEFT_EVENT)
    {
        return_menu(TRACK_OK);
    }
    else if (!sensors.valid)
    {
        /* 页面允许进入；缺少量程时，中键明确带到电感标定，不采集假参考。 */
        if (keys & KEY_OK_EVENT)
        {
            mode = TRACK_CAL;
            menu_item = 0;
            menu_selection[MENU_CALIBRATION] = 0;
            cal_adc_error = 0;
            message[0] = 0;
            TrackCal_Begin(&sensors);
        }
    }
    else if (keys & KEY_RIGHT_EVENT)
    {
        TrackBodyCenter_Begin(&sensors);
        message[0] = 0;
    }
    else if (keys & KEY_OK_EVENT)
    {
        if (sensors.center_status == TRACK_BODY_READY)
        {
            if (TrackBodyCenter_Commit(&sensors))
            {
                reset_pid();
                set_message("CENTER SAVED (RAM)");
            }
        }
        else if (sensors.center_status != TRACK_BODY_CAPTURING)
        {
            TrackBodyCenter_Begin(&sensors);
            message[0] = 0;
        }
    }
}

/** @brief 舵机三页共用按键；进入即预览，上下调整，中键保存，左键撤销退出。 */
static void handle_servo_keys(uint8_t keys)
{
    if (keys & KEY_LEFT_EVENT)
    {
        params = entry_params;
        return_menu(TRACK_OK);
    }
    else if (keys & KEY_UP_EVENT)
    {
        adjust_param(1);
    }
    else if (keys & KEY_DOWN_EVENT)
    {
        adjust_param(-1);
    }
    else if (keys & KEY_RIGHT_EVENT)
    {
        coarse_step = !coarse_step;
    }
    else if (keys & KEY_OK_EVENT)
    {
        save_params();
        TrackHW_Arm(1);
    }
}

/** @brief 方向页：上设正向、下设反向、右键切换，中键保存，左键撤销退出。 */
static void handle_direction_keys(uint8_t keys)
{
    TrackParams candidate;
    uint8_t reversed;

    if (keys & KEY_LEFT_EVENT)
    {
        params = entry_params;
        return_menu(TRACK_OK);
        return;
    }
    if (keys & KEY_OK_EVENT)
    {
        save_params();
        return;
    }
    if (!(keys & (KEY_UP_EVENT | KEY_DOWN_EVENT | KEY_RIGHT_EVENT)))
    {
        return;
    }
    TrackHW_Arm(0);
    candidate = params;
    reversed = direction_reversed(&candidate);
    if (keys & KEY_UP_EVENT)
    {
        reversed = 0;
    }
    else if (keys & KEY_DOWN_EVENT)
    {
        reversed = 1;
    }
    else
    {
        reversed = !reversed;
    }
    if (mode == TRACK_SERVO_DIRECTION)
    {
        if (!TrackParams_SetServoDirection(&candidate, reversed ? -1 : 1))
        {
            set_message("PARAM ERR");
            return;
        }
    }
    else if (mode == TRACK_LEFT_DIRECTION)
    {
        candidate.left_reverse = reversed;
    }
    else
    {
        candidate.right_reverse = reversed;
    }
    if (TrackParams_Valid(&candidate))
    {
        params = candidate;
        reset_pid();
        message[0] = 0;
    }
}

/** @brief 左键优先返回父菜单或撤销，主菜单左键不动作。 */
static void handle_keys(uint8_t keys)
{
    if (!keys)
    {
        return;
    }
    request_display();
    if (mode == TRACK_RUN)
    {
        if (keys & (KEY_LEFT_EVENT | KEY_OK_EVENT))
        {
            return_menu(TRACK_MANUAL_STOP);
        }
        return;
    }
    if (mode == TRACK_MENU)
    {
        if (keys & KEY_LEFT_EVENT)
        {
            if (menu_page != MENU_ROOT)
            {
                open_menu_page(MENU_ROOT);
            }
        }
        else if (keys & KEY_UP_EVENT)
        {
            menu_item = menu_item ? menu_item - 1 : menu_count() - 1;
        }
        else if (keys & KEY_DOWN_EVENT)
        {
            menu_item = (menu_item + 1) % menu_count();
        }
        else if (keys & KEY_OK_EVENT)
        {
            enter_menu_item();
        }
        return;
    }
    switch (mode)
    {
        case TRACK_CAL:         handle_calibration_keys(keys); break;
        case TRACK_BODY_CENTER: handle_body_keys(keys); break;
        case TRACK_PID:
        case TRACK_SPEED_PID:   handle_pid_keys(keys); break;
        case TRACK_SPEED:
        case TRACK_SPEED_TARGET: handle_speed_keys(keys); break;
        case TRACK_SERVO_DIRECTION:
        case TRACK_LEFT_DIRECTION:
        case TRACK_RIGHT_DIRECTION:
            handle_direction_keys(keys);
            break;
        default:                handle_servo_keys(keys); break;
    }
}

/* ==================== 前台周期任务 ==================== */

/** @brief 处理采样超时，静态标定保留旧数据，循迹模式立即停车。 */
static void handle_adc_failure(void)
{
    if (mode == TRACK_RUN || mode == TRACK_PID)
    {
        return_menu(TRACK_ADC_ERROR);
    }
    else if (mode == TRACK_CAL)
    {
        cal_adc_error = 1;
        set_message("ADC ERROR - NOT SAVED");
    }
    else if (mode == TRACK_BODY_CENTER)
    {
        sensors.center_status = TRACK_BODY_ADC;
        set_message("ADC ERROR: RETRY");
    }
}

/** @brief 每 2ms 消费最新 1ms 计数快照；换算单位仍为计数/10ms。 */
static void encoder_task(uint32_t now)
{
    uint16_t count;
    uint8_t direction;
    uint32_t sampled_ms;
    uint32_t elapsed = now - last_encoder;

    encoder_fresh = 0;
    if (elapsed < TRACK_SPEED_PERIOD_MS) return;
    if (!TrackHW_EncoderSample(&count, &direction, &sampled_ms) ||
        (uint32_t)(TrackHW_Now() - sampled_ms) > TRACK_ENCODER_STALE_MS)
    {
        encoder_ok = 0;
        last_encoder = now;
        return;
    }
    elapsed = sampled_ms - last_encoder;
    if (elapsed == 0) return; /* 同一计数快照不能重复积分。 */
    encoder_direction = direction;
    encoder_elapsed = elapsed > 65535UL ? 65535U : (uint16_t)elapsed;
    encoder_ok = TrackSpeed_Measure(&speed_control, (uint16_t)(count - encoder_last_count), encoder_elapsed);
    encoder_last_count = count;
    last_encoder = sampled_ms;
    encoder_fresh = encoder_ok;
}

/** @brief 2ms 控制帧读取最新 1ms ADC 扫描；电机使能只可能发生在 TRACK_RUN。 */
static void control_task(void)
{
    TrackFault result;
    uint16_t preview;
    uint8_t channel;

    if (track_guard_trip)
    {
        params = entry_params;
        return_menu(TRACK_TIMEOUT);
        return;
    }
    encoder_task(TrackHW_Now());
    if (mode == TRACK_RUN && !encoder_ok)
    {
        return_menu(TRACK_ENCODER_ERROR);
        return;
    }
    if (!TrackHW_Sample(sample_values))
    {
        handle_adc_failure();
        return;
    }
    TrackSensors_Update(&sensors, sample_values);
    if (mode == TRACK_CAL)
    {
        TrackCal_Sample(&sensors, sample_values);
        return;
    }
    if (mode == TRACK_BODY_CENTER)
    {
        TrackHW_Stop();
        TrackBodyCenter_Sample(&sensors, &params);
        return;
    }
    if (mode == TRACK_SPEED || mode == TRACK_SPEED_TARGET || mode == TRACK_SPEED_PID || is_direction_mode())
    {
        TrackHW_Stop();
        return;
    }
    if (mode == TRACK_PID || mode == TRACK_RUN)
    {
        for (channel = 0; channel < TRACK_SENSOR_COUNT; ++channel)
        {
            if (sensors.raw[channel] >= TRACK_ADC_RAIL)
            {
                if (++rail_count[channel] >= TRACK_ADC_RAIL_FRAMES)
                {
                    return_menu(TRACK_ADC_ERROR);
                    return;
                }
            }
            else
            {
                rail_count[channel] = 0;
            }
        }
        result = TrackControl_Step(&control, &sensors, &params, mode == TRACK_RUN);
        if (result != TRACK_OK || (mode == TRACK_PID && !control.signal_ok))
        {
            TrackHW_Stop();
            control.left_motor = 0;
            control.right_motor = 0;
            /* 静态 PID 弱信号立即关闭预览；运行模式由核心层限速确认后停车。 */
            if (mode == TRACK_PID &&
                (result == TRACK_NO_CAL || result == TRACK_LOST || result == TRACK_OK))
            {
                track_guard_age = 0;
                reset_pid();
                return;
            }
            if (result != TRACK_OK)
            {
                return_menu(result);
            }
            return;
        }
        if (mode == TRACK_RUN)
        {
            if (!control.signal_ok)
            {
                TrackSpeed_Stop(&speed_control); /* 弱信号不能被速度积分顶开限速。 */
            }
            else
            {
                uint16_t limit = TrackSpeed_Limit(&params, control.error, control.crossing);
                if (encoder_fresh)
                {
                    result = TrackSpeed_Calculate(&speed_control, &params,
                        TrackSpeed_Target(&params, control.error, control.crossing), limit, encoder_elapsed);
                    if (result != TRACK_OK)
                    {
                        return_menu(result);
                        return;
                    }
                }
                if (speed_control.output > limit) speed_control.output = limit;
                control.left_motor = speed_control.output;
                control.right_motor = speed_control.output; /* 单反馈、共同动力，禁止差速转向。 */
            }
        }
        TrackHW_Set(control.servo, control.left_motor, control.right_motor, mode == TRACK_RUN);
    }
    else if (mode == TRACK_CENTER || mode == TRACK_LIMIT_LEFT || mode == TRACK_LIMIT_RIGHT)
    {
        preview = mode == TRACK_CENTER ? params.center :
                  mode == TRACK_LIMIT_LEFT ? params.left : params.right;
        control.servo = preview;
        control.left_motor = 0;
        control.right_motor = 0;
        TrackHW_Set(preview, 0, 0, 0);
    }
}

/** @brief 只发送两组 PID 的目标和反馈四列；115200/8N1，每 500ms 一帧。
 *  @note 顺序：转向目标×1000、转向反馈×1000、速度目标×100、速度反馈×100。
 *        不发送表头、参数快照或其他诊断列；队列满时丢帧，不阻塞控制。
 */
static void telemetry_task(uint32_t now)
{
    if ((mode != TRACK_PID && mode != TRACK_SPEED_PID && mode != TRACK_RUN) ||
        (uint32_t)(now - last_telemetry) < TRACK_TELEMETRY_MS)
    {
        return;
    }
    last_telemetry = now;
    sprintf(telemetry, "%d,%d,%ld,%ld\r\n",
            (int)TRACK_STEERING_TARGET1000, TrackSensors_Feedback(&sensors),
            (long)(speed_control.target * 100.0f),
            (long)(speed_control.feedback * 100.0f));
    TrackUART_Queue(telemetry);
}

void TrackApp_Init(void)
{
    TrackHW_Arm(0);
    /* 方案 A：有效 Flash 优先，不能在读取后再用代码宏覆盖保存的 PID。 */
    defaults_loaded = !TrackStore_Load(&params);
    saved_params = params;
    entry_params = params;
    TrackHW_SetMotorDirections(params.left_reverse, params.right_reverse);
    memset(&sensors, 0, sizeof(sensors));
    mode = TRACK_MENU;
    sensor_math_ok = TrackSensors_MathCheck();
    fault = sensor_math_ok ? TRACK_OK : TRACK_MATH_ERROR;
    if (sensor_math_ok && !TrackParams_Valid(&params))
    {
        /* 启动时即提示参数配置错误，避免完成标定后才发现不能发车。 */
        fault = TRACK_BAD_PARAMS;
    }
    menu_page = MENU_ROOT;
    menu_item = 0;
    memset(menu_selection, 0, sizeof(menu_selection));
    editing = 0;
    display_busy = 0;
    display_off = 0;
    LCD_WrCmd(0xAF);
    reset_pid();
    startup_keys_locked = 1;
    startup_release_since = TrackHW_Now();
    TrackHW_Keys();
    last_control = TrackHW_Now();
    last_encoder = last_control;
    encoder_ok = TrackHW_EncoderSample(&encoder_last_count, &encoder_direction, &last_encoder);
    encoder_fresh = 0;
    speed_control.measured = 0;
    last_display = last_control - TRACK_DISPLAY_PERIOD_MS;
    last_telemetry = last_control;
    /* 33.1776MHz 下约 1.5s，看门狗只在前台主循环中喂。 */
    WDT_CONTR = 0x26;
}

void TrackApp_Task(void)
{
    uint32_t now;

    if (track_guard_trip)
    {
        params = entry_params;
        return_menu(TRACK_TIMEOUT);
        TrackHW_Keys();
    }
    now = TrackHW_Now();
    if (startup_keys_locked)
    {
        TrackHW_Keys();
        if (!P42 || !P46 || !P45 || !P41 || !P27)
        {
            startup_release_since = now;
        }
        else if ((uint32_t)(now - startup_release_since) >= 25)
        {
            startup_keys_locked = 0;
        }
    }
    else
    {
        handle_keys(TrackHW_Keys());
    }
    now = TrackHW_Now();
    if ((uint32_t)(now - last_control) >= TRACK_PERIOD_MS)
    {
        if (track_guard_armed && (uint32_t)(now - last_control) > 10)
        {
            return_menu(TRACK_TIMEOUT);
        }
        else
        {
            control_task();
        }
        last_control = now;
    }
    telemetry_task(TrackHW_Now());
    display_task(TrackHW_Now());
    TrackHW_FeedWatchdog();
}
