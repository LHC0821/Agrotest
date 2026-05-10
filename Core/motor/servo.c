#include "servo.h"
#include <string.h>

// ! ========================= 私 有 变 量 / 宏 定 义 ========================= ! //

#define RS06_TYPE_GET_ID           0x00U
#define RS06_TYPE_MOTION_CONTROL   0x01U
#define RS06_TYPE_FEEDBACK         0x02U
#define RS06_TYPE_ENABLE           0x03U
#define RS06_TYPE_STOP             0x04U
#define RS06_TYPE_SET_ZERO         0x06U
#define RS06_TYPE_SET_ID           0x07U
#define RS06_TYPE_READ_PARAM       0x11U
#define RS06_TYPE_WRITE_PARAM      0x12U
#define RS06_TYPE_SAVE             0x16U
#define RS06_TYPE_SET_BAUDRATE     0x17U
#define RS06_TYPE_ACTIVE_REPORT    0x18U
#define RS06_TYPE_PROTOCOL_SWITCH  0x19U

#define RS06_P_MIN                (-12.57f)
#define RS06_P_MAX                ( 12.57f)
#define RS06_V_MIN                (-50.0f)
#define RS06_V_MAX                ( 50.0f)
#define RS06_T_MIN                (-36.0f)
#define RS06_T_MAX                ( 36.0f)
#define RS06_KP_MIN               (0.0f)
#define RS06_KP_MAX               (5000.0f)
#define RS06_KD_MIN               (0.0f)
#define RS06_KD_MAX               (100.0f)

#define RS06_FRAME_LEN            8U

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static uint32_t rs06_make_ext_id(uint8_t type, uint16_t data16, uint8_t target_id);
static uint16_t rs06_host_field(uint8_t host_id);
static uint16_t rs06_u16_be(const uint8_t* data);
static uint16_t rs06_u16_le(const uint8_t* data);
static uint32_t rs06_u32_le(const uint8_t* data);
static uint16_t rs06_float_to_uint(float x, float min, float max, uint8_t bits);
static float rs06_uint_to_float(uint16_t x, float min, float max, uint8_t bits);
static rs06_status_t rs06_send_ext(const rs06_t* motor, uint32_t ext_id, const uint8_t data[RS06_FRAME_LEN]);
static rs06_status_t rs06_send_std(const rs06_t* motor, uint32_t std_id, const uint8_t data[RS06_FRAME_LEN]);
static void rs06_put_index(uint8_t data[RS06_FRAME_LEN], uint16_t index);
static uint8_t rs06_get_ext_type(const can_bus_frame_t* frame);
static rs06_status_t rs06_write_param_raw(const rs06_t* motor, uint16_t index, const uint8_t raw[4]);
static rs06_status_t rs06_write_param_u8_then_save(const rs06_t* motor, uint16_t index, uint8_t value, bool save);
static rs06_status_t rs06_write_param_u16_then_save(const rs06_t* motor, uint16_t index, uint16_t value, bool save);
static rs06_status_t rs06_write_param_u32_then_save(const rs06_t* motor, uint16_t index, uint32_t value, bool save);
static rs06_status_t rs06_write_param_float_then_save(const rs06_t* motor, uint16_t index, float value, bool save);
static void rs06_mit_fill_ff(uint8_t data[RS06_FRAME_LEN]);
static uint32_t rs06_mit_make_mode_std_id(uint8_t cmd, uint8_t motor_id);
static rs06_status_t rs06_mit_write_param_raw(const rs06_t* motor, uint16_t index, const uint8_t raw[4]);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 初始化 RS06 电机实例
 * @param motor RS06 电机实例
 * @param bus 所属 FDCAN 句柄
 * @param motor_id 电机 CAN ID
 * @param host_id 主机 ID；传 0 时使用 RS06_DEFAULT_HOST_ID
 */
void rs06_init(rs06_t* motor, FDCAN_HandleTypeDef* bus, uint8_t motor_id, uint8_t host_id) {
    if(motor == NULL) return;

    motor->bus = bus;
    motor->motor_id = motor_id;
    motor->host_id = (host_id == 0U) ? RS06_DEFAULT_HOST_ID : host_id;
    motor->tx_timeout_ms = RS06_DEFAULT_TX_TIMEOUT_MS;
}

/**
 * @brief 设置 RS06 实例的 CAN 发送等待超时时间
 * @param motor RS06 电机实例
 * @param timeout_ms 超时时间，单位 ms；0 表示不等待 TX FIFO 空位
 */
void rs06_set_tx_timeout(rs06_t* motor, uint32_t timeout_ms) {
    if(motor == NULL) return;
    motor->tx_timeout_ms = timeout_ms;
}

/**
 * @brief RS06 状态码转字符串
 * @param status RS06 状态码
 * @return 状态码字符串
 */
const char* rs06_status_to_str(rs06_status_t status) {
    switch(status) {
        case RS06_OK: return "RS06_OK";
        case RS06_CAN_ERROR: return "RS06_CAN_ERROR";
        case RS06_INVALID_ARG: return "RS06_INVALID_ARG";
        default: return "RS06_UNKNOWN";
    }
}

/**
 * @brief RS06 私有协议运行模式转字符串
 * @param mode 运行模式
 * @return 模式字符串
 */
const char* rs06_mode_to_str(rs06_mode_t mode) {
    switch(mode) {
        case RS06_MODE_MOTION: return "MOTION";
        case RS06_MODE_PP: return "PP";
        case RS06_MODE_SPEED: return "SPEED";
        case RS06_MODE_CURRENT: return "CURRENT";
        case RS06_MODE_CSP: return "CSP";
        default: return "UNKNOWN";
    }
}

/**
 * @brief 获取设备 ID 与 64 bit MCU 唯一标识
 * @param motor RS06 电机实例
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_get_device_id(const rs06_t* motor) {
    uint8_t data[RS06_FRAME_LEN] = { 0 };
    if(motor == NULL) return RS06_INVALID_ARG;

    return rs06_send_ext(motor,
        rs06_make_ext_id(RS06_TYPE_GET_ID, rs06_host_field(motor->host_id), motor->motor_id),
        data);
}

/**
 * @brief 读取电机版本号
 * @param motor RS06 电机实例
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_get_version(const rs06_t* motor) {
    uint8_t data[RS06_FRAME_LEN] = { 0 };
    if(motor == NULL) return RS06_INVALID_ARG;

    data[0] = 0x00U;
    data[1] = 0xC4U;

    return rs06_send_ext(motor,
        rs06_make_ext_id(RS06_TYPE_STOP, rs06_host_field(motor->host_id), motor->motor_id),
        data);
}

/**
 * @brief 使能电机运行
 * @param motor RS06 电机实例
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_enable(const rs06_t* motor) {
    uint8_t data[RS06_FRAME_LEN] = { 0 };
    if(motor == NULL) return RS06_INVALID_ARG;

    return rs06_send_ext(motor,
        rs06_make_ext_id(RS06_TYPE_ENABLE, rs06_host_field(motor->host_id), motor->motor_id),
        data);
}

/**
 * @brief 停止电机运行
 * @param motor RS06 电机实例
 * @param clear_error 是否同时清除故障
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_stop(const rs06_t* motor, bool clear_error) {
    uint8_t data[RS06_FRAME_LEN] = { 0 };
    if(motor == NULL) return RS06_INVALID_ARG;

    data[0] = clear_error ? 1U : 0U;
    return rs06_send_ext(motor,
        rs06_make_ext_id(RS06_TYPE_STOP, rs06_host_field(motor->host_id), motor->motor_id),
        data);
}

/**
 * @brief 清除电机错误
 * @param motor RS06 电机实例
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_clear_error(const rs06_t* motor) {
    return rs06_stop(motor, true);
}

/**
 * @brief 设置当前位置为机械零位
 * @note RS06 手册说明 PP 模式下标零会被屏蔽，建议在运控模式或 CSP 模式下标零
 * @param motor RS06 电机实例
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_set_mechanical_zero(const rs06_t* motor) {
    uint8_t data[RS06_FRAME_LEN] = { 1U, 0U, 0U, 0U, 0U, 0U, 0U, 0U };
    if(motor == NULL) return RS06_INVALID_ARG;

    return rs06_send_ext(motor,
        rs06_make_ext_id(RS06_TYPE_SET_ZERO, rs06_host_field(motor->host_id), motor->motor_id),
        data);
}

/**
 * @brief 修改电机 CAN ID
 * @param motor RS06 电机实例，motor_id 为当前 ID
 * @param new_motor_id 新电机 ID
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_set_can_id(const rs06_t* motor, uint8_t new_motor_id) {
    uint8_t data[RS06_FRAME_LEN] = { 0 };
    uint16_t data16;

    if(motor == NULL || new_motor_id == 0U || new_motor_id > 0x7FU) return RS06_INVALID_ARG;

    data16 = ((uint16_t)new_motor_id << 8) | rs06_host_field(motor->host_id);
    return rs06_send_ext(motor, rs06_make_ext_id(RS06_TYPE_SET_ID, data16, motor->motor_id), data);
}

/**
 * @brief 修改电机 CAN 波特率
 * @note 波特率切换类指令通常需要重新上电或重新初始化总线后验证
 * @param motor RS06 电机实例
 * @param baud 目标波特率枚举
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_set_baudrate(const rs06_t* motor, rs06_baud_t baud) {
    uint8_t data[RS06_FRAME_LEN] = { 1U, 2U, 3U, 4U, 5U, 6U, (uint8_t)baud, 0U };
    if(motor == NULL) return RS06_INVALID_ARG;
    return rs06_send_ext(motor,
        rs06_make_ext_id(RS06_TYPE_SET_BAUDRATE, rs06_host_field(motor->host_id), motor->motor_id),
        data);
}

/**
 * @brief 开启或关闭私有协议主动上报
 * @param motor RS06 电机实例
 * @param enable true 开启；false 关闭
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_set_active_report(const rs06_t* motor, bool enable) {
    uint8_t data[RS06_FRAME_LEN] = { 1U, 2U, 3U, 4U, 5U, 6U, 0U, 0U };
    if(motor == NULL) return RS06_INVALID_ARG;

    data[6] = enable ? 1U : 0U;
    return rs06_send_ext(motor,
        rs06_make_ext_id(RS06_TYPE_ACTIVE_REPORT, rs06_host_field(motor->host_id), motor->motor_id),
        data);
}

/**
 * @brief 保存当前电机参数
 * @param motor RS06 电机实例
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_save_config(const rs06_t* motor) {
    uint8_t data[RS06_FRAME_LEN] = { 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U };
    if(motor == NULL) return RS06_INVALID_ARG;

    return rs06_send_ext(motor,
        rs06_make_ext_id(RS06_TYPE_SAVE, rs06_host_field(motor->host_id), motor->motor_id),
        data);
}

/**
 * @brief 请求切换 RS06 协议
 * @note 协议切换通常需要电机重新上电后生效
 * @param motor RS06 电机实例
 * @param protocol 目标协议
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_set_protocol(const rs06_t* motor, rs06_protocol_t protocol) {
    uint8_t data[RS06_FRAME_LEN] = { 0 };
    if(motor == NULL) return RS06_INVALID_ARG;

    data[0] = (uint8_t)protocol;
    return rs06_send_ext(motor,
        rs06_make_ext_id(RS06_TYPE_PROTOCOL_SWITCH, rs06_host_field(motor->host_id), motor->motor_id),
        data);
}

/**
 * @brief 执行“停止 -> 运控模式 -> 标零 -> 保存”的完整标零保存流程
 * @param motor RS06 电机实例
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_zeroing_and_save(const rs06_t* motor) {
    rs06_status_t st;

    st = rs06_stop(motor, false);
    if(st != RS06_OK) return st;
    HAL_Delay(100U);

    st = rs06_set_mode(motor, RS06_MODE_MOTION);
    if(st != RS06_OK) return st;
    HAL_Delay(100U);

    st = rs06_set_mechanical_zero(motor);
    if(st != RS06_OK) return st;
    HAL_Delay(300U);

    st = rs06_save_config(motor);
    if(st != RS06_OK) return st;
    HAL_Delay(1000U);

    return RS06_OK;
}

/**
 * @brief 读取单个参数
 * @param motor RS06 电机实例
 * @param index 参数索引
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_read_param(const rs06_t* motor, uint16_t index) {
    uint8_t data[RS06_FRAME_LEN] = { 0 };
    if(motor == NULL) return RS06_INVALID_ARG;

    rs06_put_index(data, index);
    return rs06_send_ext(motor,
        rs06_make_ext_id(RS06_TYPE_READ_PARAM, rs06_host_field(motor->host_id), motor->motor_id),
        data);
}

/**
 * @brief 写入 uint8 参数，掉电默认不保存
 * @param motor RS06 电机实例
 * @param index 参数索引
 * @param value 参数值
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_write_param_u8(const rs06_t* motor, uint16_t index, uint8_t value) {
    uint8_t raw[4] = { value, 0U, 0U, 0U };
    return rs06_write_param_raw(motor, index, raw);
}

/**
 * @brief 写入 uint16 参数，掉电默认不保存
 * @param motor RS06 电机实例
 * @param index 参数索引
 * @param value 参数值，小端写入
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_write_param_u16(const rs06_t* motor, uint16_t index, uint16_t value) {
    uint8_t raw[4] = { (uint8_t)value, (uint8_t)(value >> 8), 0U, 0U };
    return rs06_write_param_raw(motor, index, raw);
}

/**
 * @brief 写入 uint32 参数，掉电默认不保存
 * @param motor RS06 电机实例
 * @param index 参数索引
 * @param value 参数值，小端写入
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_write_param_u32(const rs06_t* motor, uint16_t index, uint32_t value) {
    uint8_t raw[4] = { (uint8_t)value, (uint8_t)(value >> 8), (uint8_t)(value >> 16), (uint8_t)(value >> 24) };
    return rs06_write_param_raw(motor, index, raw);
}

/**
 * @brief 写入 float 参数，掉电默认不保存
 * @param motor RS06 电机实例
 * @param index 参数索引
 * @param value 参数值，小端写入
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_write_param_float(const rs06_t* motor, uint16_t index, float value) {
    uint8_t raw[4] = { 0 };
    memcpy(raw, &value, sizeof(float));
    return rs06_write_param_raw(motor, index, raw);
}

/**
 * @brief 设置私有协议运行模式
 * @param motor RS06 电机实例
 * @param mode 目标运行模式
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_set_mode(const rs06_t* motor, rs06_mode_t mode) {
    return rs06_write_param_u8(motor, RS06_PARAM_RUN_MODE, (uint8_t)mode);
}

/**
 * @brief 设置零点标志位
 * @param motor RS06 电机实例
 * @param zero_sta 0 表示上电位置 0 ~ 2pi；1 表示 -pi ~ pi
 * @param save 是否保存到电机参数区
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_set_zero_sta(const rs06_t* motor, uint8_t zero_sta, bool save) {
    return rs06_write_param_u8_then_save(motor, RS06_PARAM_ZERO_STA, zero_sta ? 1U : 0U, save);
}

/**
 * @brief 设置主动上报周期参数
 * @param motor RS06 电机实例
 * @param period_value 周期参数，1 表示 10ms，之后每 +1 增加 5ms
 * @param save 是否保存到电机参数区
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_set_report_period(const rs06_t* motor, uint16_t period_value, bool save) {
    return rs06_write_param_u16_then_save(motor, RS06_PARAM_REPORT_TIME, period_value, save);
}

/**
 * @brief 设置 CAN 超时阈值
 * @param motor RS06 电机实例
 * @param timeout_value 超时阈值，20000 约等于 1s，0 表示关闭超时保护
 * @param save 是否保存到电机参数区
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_set_can_timeout(const rs06_t* motor, uint32_t timeout_value, bool save) {
    return rs06_write_param_u32_then_save(motor, RS06_PARAM_CAN_TIMEOUT, timeout_value, save);
}

/**
 * @brief 设置未上电反驱阻尼开关
 * @param motor RS06 电机实例
 * @param enable true 开启取消阻尼；false 保持默认阻尼
 * @param save 是否保存到电机参数区
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_set_damper(const rs06_t* motor, bool enable, bool save) {
    return rs06_write_param_u8_then_save(motor, RS06_PARAM_DAMPER, enable ? 1U : 0U, save);
}

/**
 * @brief 设置附加零位偏置
 * @param motor RS06 电机实例
 * @param offset_rad 零位偏置，单位 rad
 * @param save 是否保存到电机参数区
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_set_add_offset(const rs06_t* motor, float offset_rad, bool save) {
    return rs06_write_param_float_then_save(motor, RS06_PARAM_ADD_OFFSET, offset_rad, save);
}

/**
 * @brief 设置齿槽补偿开关
 * @param motor RS06 电机实例
 * @param enable true 开启；false 关闭
 * @param save 是否保存到电机参数区
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_set_alveolous_comp(const rs06_t* motor, bool enable, bool save) {
    return rs06_write_param_u8_then_save(motor, RS06_PARAM_ALVEOLOUS_OPEN, enable ? 1U : 0U, save);
}

/**
 * @brief 设置初始化校准开关
 * @param motor RS06 电机实例
 * @param enable true 开启；false 关闭
 * @param save 是否保存到电机参数区
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_set_iq_test(const rs06_t* motor, bool enable, bool save) {
    return rs06_write_param_u8_then_save(motor, RS06_PARAM_IQ_TEST, enable ? 1U : 0U, save);
}

/**
 * @brief 设置电流目标
 * @param motor RS06 电机实例
 * @param current_a 目标 q 轴电流，单位 A
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_set_current_target(const rs06_t* motor, float current_a) {
    return rs06_write_param_float(motor, RS06_PARAM_IQ_REF, current_a);
}

/**
 * @brief 进入电流模式并下发电流目标
 * @param motor RS06 电机实例
 * @param current_a 目标 q 轴电流，单位 A
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_current_control(const rs06_t* motor, float current_a) {
    rs06_status_t st;

    st = rs06_stop(motor, false);
    if(st != RS06_OK) return st;
    HAL_Delay(20U);

    st = rs06_set_mode(motor, RS06_MODE_CURRENT);
    if(st != RS06_OK) return st;
    HAL_Delay(20U);

    st = rs06_enable(motor);
    if(st != RS06_OK) return st;
    HAL_Delay(20U);

    return rs06_set_current_target(motor, current_a);
}

/**
 * @brief 设置速度模式运行参数
 * @param motor RS06 电机实例
 * @param limit_cur_a 电流限制，单位 A
 * @param accel_rad_s2 加速度，单位 rad/s^2
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_set_speed_config(const rs06_t* motor, float limit_cur_a, float accel_rad_s2) {
    rs06_status_t st;

    st = rs06_write_param_float(motor, RS06_PARAM_LIMIT_CUR, limit_cur_a);
    if(st != RS06_OK) return st;
    HAL_Delay(2U);

    return rs06_write_param_float(motor, RS06_PARAM_SPEED_ACCEL, accel_rad_s2);
}

/**
 * @brief 设置速度目标
 * @param motor RS06 电机实例
 * @param speed_rad_s 目标速度，单位 rad/s
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_set_speed_target(const rs06_t* motor, float speed_rad_s) {
    return rs06_write_param_float(motor, RS06_PARAM_SPEED_REF, speed_rad_s);
}

/**
 * @brief 进入速度模式并下发速度目标
 * @param motor RS06 电机实例
 * @param speed_rad_s 目标速度，单位 rad/s
 * @param limit_cur_a 电流限制，单位 A
 * @param accel_rad_s2 加速度，单位 rad/s^2
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_speed_control(const rs06_t* motor, float speed_rad_s, float limit_cur_a, float accel_rad_s2) {
    rs06_status_t st;

    st = rs06_stop(motor, false);
    if(st != RS06_OK) return st;
    HAL_Delay(20U);

    st = rs06_set_mode(motor, RS06_MODE_SPEED);
    if(st != RS06_OK) return st;
    HAL_Delay(20U);

    st = rs06_enable(motor);
    if(st != RS06_OK) return st;
    HAL_Delay(20U);

    st = rs06_set_speed_config(motor, limit_cur_a, accel_rad_s2);
    if(st != RS06_OK) return st;
    HAL_Delay(20U);

    return rs06_set_speed_target(motor, speed_rad_s);
}

/**
 * @brief 设置 PP 位置模式速度和加速度
 * @param motor RS06 电机实例
 * @param speed_rad_s 速度限制，单位 rad/s
 * @param accel_rad_s2 加速度，单位 rad/s^2
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_set_pp_config(const rs06_t* motor, float speed_rad_s, float accel_rad_s2) {
    rs06_status_t st;

    st = rs06_write_param_float(motor, RS06_PARAM_PP_LIMIT_SPD, speed_rad_s);
    if(st != RS06_OK) return st;
    HAL_Delay(2U);

    return rs06_write_param_float(motor, RS06_PARAM_PP_ACCEL, accel_rad_s2);
}

/**
 * @brief 设置 PP 位置模式减速度
 * @param motor RS06 电机实例
 * @param decel_rad_s2 减速度，单位 rad/s^2
 * @param save 是否保存到电机参数区
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_set_pp_decel(const rs06_t* motor, float decel_rad_s2, bool save) {
    return rs06_write_param_float_then_save(motor, RS06_PARAM_PP_DECEL, decel_rad_s2, save);
}

/**
 * @brief 设置位置目标
 * @param motor RS06 电机实例
 * @param position_rad 目标位置，单位 rad
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_set_position_target(const rs06_t* motor, float position_rad) {
    return rs06_write_param_float(motor, RS06_PARAM_POSITION_REF, position_rad);
}

/**
 * @brief 进入 PP 位置模式并下发位置目标
 * @param motor RS06 电机实例
 * @param position_rad 目标位置，单位 rad
 * @param speed_rad_s 速度限制，单位 rad/s
 * @param accel_rad_s2 加速度，单位 rad/s^2
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_pp_goto(const rs06_t* motor, float position_rad, float speed_rad_s, float accel_rad_s2) {
    rs06_status_t st;

    st = rs06_stop(motor, false);
    if(st != RS06_OK) return st;
    HAL_Delay(20U);

    st = rs06_set_mode(motor, RS06_MODE_PP);
    if(st != RS06_OK) return st;
    HAL_Delay(20U);

    st = rs06_set_pp_config(motor, speed_rad_s, accel_rad_s2);
    if(st != RS06_OK) return st;
    HAL_Delay(20U);

    st = rs06_enable(motor);
    if(st != RS06_OK) return st;
    HAL_Delay(20U);

    return rs06_set_position_target(motor, position_rad);
}

/**
 * @brief 设置 CSP 位置模式速度限制
 * @param motor RS06 电机实例
 * @param speed_rad_s 速度限制，单位 rad/s
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_set_csp_config(const rs06_t* motor, float speed_rad_s) {
    return rs06_write_param_float(motor, RS06_PARAM_CSP_LIMIT_SPD, speed_rad_s);
}

/**
 * @brief 进入 CSP 位置模式并下发位置目标
 * @param motor RS06 电机实例
 * @param position_rad 目标位置，单位 rad
 * @param speed_rad_s 速度限制，单位 rad/s
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_csp_goto(const rs06_t* motor, float position_rad, float speed_rad_s) {
    rs06_status_t st;

    st = rs06_stop(motor, false);
    if(st != RS06_OK) return st;
    HAL_Delay(20U);

    st = rs06_set_mode(motor, RS06_MODE_CSP);
    if(st != RS06_OK) return st;
    HAL_Delay(20U);

    st = rs06_set_csp_config(motor, speed_rad_s);
    if(st != RS06_OK) return st;
    HAL_Delay(20U);

    st = rs06_enable(motor);
    if(st != RS06_OK) return st;
    HAL_Delay(20U);

    return rs06_set_position_target(motor, position_rad);
}

/**
 * @brief 私有协议运控模式控制
 * @param motor RS06 电机实例
 * @param torque_nm 前馈力矩，单位 N·m
 * @param position_rad 目标位置，单位 rad
 * @param velocity_rad_s 目标速度，单位 rad/s
 * @param kp 位置刚度系数
 * @param kd 速度阻尼系数
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_motion_control(const rs06_t* motor, float torque_nm, float position_rad, float velocity_rad_s, float kp, float kd) {
    uint8_t data[RS06_FRAME_LEN] = { 0 };
    uint16_t t;
    uint16_t p;
    uint16_t v;
    uint16_t kpi;
    uint16_t kdi;

    if(motor == NULL) return RS06_INVALID_ARG;

    t = rs06_float_to_uint(torque_nm, RS06_T_MIN, RS06_T_MAX, 16);
    p = rs06_float_to_uint(position_rad, RS06_P_MIN, RS06_P_MAX, 16);
    v = rs06_float_to_uint(velocity_rad_s, RS06_V_MIN, RS06_V_MAX, 16);
    kpi = rs06_float_to_uint(kp, RS06_KP_MIN, RS06_KP_MAX, 16);
    kdi = rs06_float_to_uint(kd, RS06_KD_MIN, RS06_KD_MAX, 16);

    data[0] = (uint8_t)(p >> 8);   data[1] = (uint8_t)p;
    data[2] = (uint8_t)(v >> 8);   data[3] = (uint8_t)v;
    data[4] = (uint8_t)(kpi >> 8); data[5] = (uint8_t)kpi;
    data[6] = (uint8_t)(kdi >> 8); data[7] = (uint8_t)kdi;

    return rs06_send_ext(motor, rs06_make_ext_id(RS06_TYPE_MOTION_CONTROL, t, motor->motor_id), data);
}

/**
 * @brief 解析 RS06 私有协议反馈帧
 * @param frame CAN 帧
 * @param feedback 输出反馈数据
 * @retval true 解析成功
 * @retval false 帧类型不匹配或参数无效
 */
bool rs06_parse_feedback(const can_bus_frame_t* frame, rs06_feedback_t* feedback) {
    if(frame == NULL || feedback == NULL || frame->id_type != CAN_BUS_ID_EXT || frame->len < RS06_FRAME_LEN) return false;
    if(rs06_get_ext_type(frame) != RS06_TYPE_FEEDBACK) return false;

    memset(feedback, 0, sizeof(*feedback));
    feedback->host_id = (uint8_t)(frame->id & 0xFFU);
    feedback->motor_id = (uint8_t)((frame->id >> 8) & 0xFFU);
    feedback->fault_bits = (uint8_t)((frame->id >> 16) & 0x3FU);
    feedback->pattern = (rs06_pattern_t)((frame->id >> 22) & 0x03U);
    feedback->position_rad = rs06_uint_to_float(rs06_u16_be(&frame->data[0]), RS06_P_MIN, RS06_P_MAX, 16);
    feedback->velocity_rad_s = rs06_uint_to_float(rs06_u16_be(&frame->data[2]), RS06_V_MIN, RS06_V_MAX, 16);
    feedback->torque_nm = rs06_uint_to_float(rs06_u16_be(&frame->data[4]), RS06_T_MIN, RS06_T_MAX, 16);
    feedback->temperature_c = ((float)rs06_u16_be(&frame->data[6])) * 0.1f;

    return true;
}

/**
 * @brief 解析私有协议参数读取 / 写入响应帧
 * @param frame CAN 帧
 * @param reply 输出参数响应
 * @retval true 解析成功
 * @retval false 帧类型不匹配或参数无效
 */
bool rs06_parse_param_reply(const can_bus_frame_t* frame, rs06_param_reply_t* reply) {
    if(frame == NULL || reply == NULL || frame->id_type != CAN_BUS_ID_EXT || frame->len < RS06_FRAME_LEN) return false;
    if(rs06_get_ext_type(frame) != RS06_TYPE_READ_PARAM && rs06_get_ext_type(frame) != RS06_TYPE_WRITE_PARAM) return false;

    memset(reply, 0, sizeof(*reply));
    reply->motor_id = (uint8_t)((frame->id >> 8) & 0xFFU);
    reply->host_id = (uint8_t)(frame->id & 0xFFU);
    reply->index = rs06_u16_le(&frame->data[0]);
    reply->value_u8 = frame->data[4];
    reply->value_u16 = rs06_u16_le(&frame->data[4]);
    reply->value_u32 = rs06_u32_le(&frame->data[4]);
    memcpy(&reply->value_float, &frame->data[4], sizeof(float));
    memcpy(reply->raw, frame->data, sizeof(reply->raw));

    return true;
}

/**
 * @brief 解析设备 ID 响应帧
 * @param frame CAN 帧
 * @param reply 输出设备 ID 响应
 * @retval true 解析成功
 * @retval false 帧类型不匹配或参数无效
 */
bool rs06_parse_device_id_reply(const can_bus_frame_t* frame, rs06_device_id_reply_t* reply) {
    if(frame == NULL || reply == NULL || frame->id_type != CAN_BUS_ID_EXT || frame->len < RS06_FRAME_LEN) return false;
    if(rs06_get_ext_type(frame) != RS06_TYPE_GET_ID) return false;

    memset(reply, 0, sizeof(*reply));
    reply->motor_id = (uint8_t)((frame->id >> 8) & 0xFFU);
    reply->target_id = (uint8_t)(frame->id & 0xFFU);
    memcpy(reply->unique_id, frame->data, sizeof(reply->unique_id));

    return true;
}

/**
 * @brief 解析版本号响应帧
 * @param frame CAN 帧
 * @param reply 输出版本号响应
 * @retval true 解析成功
 * @retval false 帧类型不匹配或参数无效
 */
bool rs06_parse_version_reply(const can_bus_frame_t* frame, rs06_version_reply_t* reply) {
    if(frame == NULL || reply == NULL || frame->id_type != CAN_BUS_ID_EXT || frame->len < 7U) return false;
    if(rs06_get_ext_type(frame) != RS06_TYPE_FEEDBACK) return false;
    if(frame->data[0] != 0x00U || frame->data[1] != 0xC4U || frame->data[2] != 0x56U) return false;

    memset(reply, 0, sizeof(*reply));
    reply->motor_id = (uint8_t)((frame->id >> 8) & 0xFFU);
    reply->host_id = (uint8_t)(frame->id & 0xFFU);
    reply->version = ((uint32_t)frame->data[3] << 24) |
        ((uint32_t)frame->data[4] << 16) |
        ((uint32_t)frame->data[5] << 8) |
        ((uint32_t)frame->data[6]);
    memcpy(reply->raw, frame->data, sizeof(reply->raw));

    return true;
}

/**
 * @brief MIT 协议使能电机
 * @param motor RS06 电机实例
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_mit_enable(const rs06_t* motor) {
    uint8_t data[RS06_FRAME_LEN];
    if(motor == NULL) return RS06_INVALID_ARG;

    rs06_mit_fill_ff(data);
    data[7] = 0xFCU;
    return rs06_send_std(motor, motor->motor_id, data);
}

/**
 * @brief MIT 协议失能电机
 * @param motor RS06 电机实例
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_mit_disable(const rs06_t* motor) {
    uint8_t data[RS06_FRAME_LEN];
    if(motor == NULL) return RS06_INVALID_ARG;

    rs06_mit_fill_ff(data);
    data[7] = 0xFDU;
    return rs06_send_std(motor, motor->motor_id, data);
}

/**
 * @brief MIT 协议设置当前位置为零点
 * @param motor RS06 电机实例
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_mit_set_zero_pos(const rs06_t* motor) {
    uint8_t data[RS06_FRAME_LEN];
    if(motor == NULL) return RS06_INVALID_ARG;

    rs06_mit_fill_ff(data);
    data[7] = 0xFEU;
    return rs06_send_std(motor, motor->motor_id, data);
}

/**
 * @brief MIT 协议清错或查询错误状态
 * @param motor RS06 电机实例
 * @param cmd 0xFF 表示清除当前异常；其他值用于查询并在应答中回传
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_mit_clear_or_check_error(const rs06_t* motor, uint8_t cmd) {
    uint8_t data[RS06_FRAME_LEN];
    if(motor == NULL) return RS06_INVALID_ARG;

    rs06_mit_fill_ff(data);
    data[6] = cmd;
    data[7] = 0xFBU;
    return rs06_send_std(motor, motor->motor_id, data);
}

/**
 * @brief MIT 协议设置运行模式
 * @param motor RS06 电机实例
 * @param mode 运行模式命令字
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_mit_set_run_mode(const rs06_t* motor, rs06_mit_run_mode_t mode) {
    uint8_t data[RS06_FRAME_LEN];
    if(motor == NULL) return RS06_INVALID_ARG;

    rs06_mit_fill_ff(data);
    data[6] = (uint8_t)mode;
    data[7] = 0xFCU;
    return rs06_send_std(motor, motor->motor_id, data);
}

/**
 * @brief MIT 协议修改电机 ID
 * @param motor RS06 电机实例
 * @param new_motor_id 新电机 ID
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_mit_set_motor_id(const rs06_t* motor, uint8_t new_motor_id) {
    uint8_t data[RS06_FRAME_LEN];
    if(motor == NULL || new_motor_id == 0U || new_motor_id > 0x7FU) return RS06_INVALID_ARG;

    rs06_mit_fill_ff(data);
    data[6] = new_motor_id;
    data[7] = 0xFAU;
    return rs06_send_std(motor, motor->motor_id, data);
}

/**
 * @brief MIT 协议切换电机协议
 * @note 协议切换通常需要重新上电后生效
 * @param motor RS06 电机实例
 * @param protocol 目标协议
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_mit_set_protocol(const rs06_t* motor, rs06_protocol_t protocol) {
    uint8_t data[RS06_FRAME_LEN];
    if(motor == NULL) return RS06_INVALID_ARG;

    rs06_mit_fill_ff(data);
    data[6] = (uint8_t)protocol;
    data[7] = 0xFDU;
    return rs06_send_std(motor, motor->motor_id, data);
}

/**
 * @brief MIT 协议修改主机 ID
 * @param motor RS06 电机实例
 * @param host_id 新主机 ID
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_mit_set_host_id(const rs06_t* motor, uint8_t host_id) {
    uint8_t data[RS06_FRAME_LEN];
    if(motor == NULL || host_id == 0U) return RS06_INVALID_ARG;

    rs06_mit_fill_ff(data);
    data[6] = host_id;
    data[7] = 0x01U;
    return rs06_send_std(motor, motor->motor_id, data);
}

/**
 * @brief MIT 协议保存电机数据
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_mit_save_config(const rs06_t* motor) {
    uint8_t data[RS06_FRAME_LEN];
    if(motor == NULL) return RS06_INVALID_ARG;

    rs06_mit_fill_ff(data);
    data[7] = 0xF8U;
    return rs06_send_std(motor, motor->motor_id, data);
}

/**
 * @brief MIT 协议开启或关闭主动上报
 * @param motor RS06 电机实例
 * @param enable true 开启；false 关闭
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_mit_set_active_report(const rs06_t* motor, bool enable) {
    uint8_t data[RS06_FRAME_LEN];
    if(motor == NULL) return RS06_INVALID_ARG;

    rs06_mit_fill_ff(data);
    data[6] = enable ? 1U : 0U;
    data[7] = 0xF9U;
    return rs06_send_std(motor, motor->motor_id, data);
}

/**
 * @brief MIT 协议读取参数
 * @param motor RS06 电机实例
 * @param index 参数索引
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_mit_read_param(const rs06_t* motor, uint16_t index) {
    uint8_t data[RS06_FRAME_LEN] = { 0 };
    if(motor == NULL) return RS06_INVALID_ARG;

    rs06_put_index(data, index);
    return rs06_send_std(motor, rs06_mit_make_mode_std_id(3U, motor->motor_id), data);
}

/**
 * @brief MIT 协议写入 uint8 参数
 * @param motor RS06 电机实例
 * @param index 参数索引
 * @param value 参数值
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_mit_write_param_u8(const rs06_t* motor, uint16_t index, uint8_t value) {
    uint8_t raw[4] = { value, 0U, 0U, 0U };
    return rs06_mit_write_param_raw(motor, index, raw);
}

/**
 * @brief MIT 协议写入 uint16 参数
 * @param motor RS06 电机实例
 * @param index 参数索引
 * @param value 参数值
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_mit_write_param_u16(const rs06_t* motor, uint16_t index, uint16_t value) {
    uint8_t raw[4] = { (uint8_t)value, (uint8_t)(value >> 8), 0U, 0U };
    return rs06_mit_write_param_raw(motor, index, raw);
}

/**
 * @brief MIT 协议写入 uint32 参数
 * @param motor RS06 电机实例
 * @param index 参数索引
 * @param value 参数值
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_mit_write_param_u32(const rs06_t* motor, uint16_t index, uint32_t value) {
    uint8_t raw[4] = { (uint8_t)value, (uint8_t)(value >> 8), (uint8_t)(value >> 16), (uint8_t)(value >> 24) };
    return rs06_mit_write_param_raw(motor, index, raw);
}

/**
 * @brief MIT 协议写入 float 参数
 * @param motor RS06 电机实例
 * @param index 参数索引
 * @param value 参数值
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_mit_write_param_float(const rs06_t* motor, uint16_t index, float value) {
    uint8_t raw[4] = { 0 };
    memcpy(raw, &value, sizeof(float));
    return rs06_mit_write_param_raw(motor, index, raw);
}

/**
 * @brief MIT 协议综合控制
 * @param motor RS06 电机实例
 * @param position_rad 目标位置，单位 rad
 * @param velocity_rad_s 目标速度，单位 rad/s
 * @param kp 位置刚度系数
 * @param kd 速度阻尼系数
 * @param torque_nm 前馈力矩，单位 N·m
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_mit_control(const rs06_t* motor, float position_rad, float velocity_rad_s, float kp, float kd, float torque_nm) {
    uint8_t data[RS06_FRAME_LEN] = { 0 };
    uint16_t p;
    uint16_t v;
    uint16_t kpi;
    uint16_t kdi;
    uint16_t t;

    if(motor == NULL) return RS06_INVALID_ARG;

    p = rs06_float_to_uint(position_rad, RS06_P_MIN, RS06_P_MAX, 16);
    v = rs06_float_to_uint(velocity_rad_s, RS06_V_MIN, RS06_V_MAX, 12);
    kpi = rs06_float_to_uint(kp, RS06_KP_MIN, RS06_KP_MAX, 12);
    kdi = rs06_float_to_uint(kd, RS06_KD_MIN, RS06_KD_MAX, 12);
    t = rs06_float_to_uint(torque_nm, RS06_T_MIN, RS06_T_MAX, 12);

    data[0] = (uint8_t)(p >> 8);
    data[1] = (uint8_t)p;
    data[2] = (uint8_t)(v >> 4);
    data[3] = (uint8_t)(((v & 0x0FU) << 4) | ((kpi >> 8) & 0x0FU));
    data[4] = (uint8_t)kpi;
    data[5] = (uint8_t)(kdi >> 4);
    data[6] = (uint8_t)(((kdi & 0x0FU) << 4) | ((t >> 8) & 0x0FU));
    data[7] = (uint8_t)t;

    return rs06_send_std(motor, motor->motor_id, data);
}

/**
 * @brief MIT 协议位置模式控制
 * @param motor RS06 电机实例
 * @param position_rad 目标位置，单位 rad
 * @param speed_rad_s 速度限制，单位 rad/s
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_mit_position_control(const rs06_t* motor, float position_rad, float speed_rad_s) {
    uint8_t data[RS06_FRAME_LEN] = { 0 };
    if(motor == NULL) return RS06_INVALID_ARG;

    memcpy(&data[0], &position_rad, sizeof(float));
    memcpy(&data[4], &speed_rad_s, sizeof(float));
    return rs06_send_std(motor, rs06_mit_make_mode_std_id(1U, motor->motor_id), data);
}

/**
 * @brief MIT 协议速度模式控制
 * @param motor RS06 电机实例
 * @param speed_rad_s 目标速度，单位 rad/s
 * @param current_limit_a 电流限制，单位 A
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_mit_speed_control(const rs06_t* motor, float speed_rad_s, float current_limit_a) {
    uint8_t data[RS06_FRAME_LEN] = { 0 };
    if(motor == NULL) return RS06_INVALID_ARG;

    memcpy(&data[0], &speed_rad_s, sizeof(float));
    memcpy(&data[4], &current_limit_a, sizeof(float));
    return rs06_send_std(motor, rs06_mit_make_mode_std_id(2U, motor->motor_id), data);
}

/**
 * @brief 兼容旧命名：MIT 协议设置运行模式
 * @param motor RS06 电机实例
 * @param mode_cmd 模式命令字
 * @return rs06_status_t 状态码
 */
rs06_status_t rs06_mit_set_motor_type(const rs06_t* motor, uint8_t mode_cmd) {
    return rs06_mit_set_run_mode(motor, (rs06_mit_run_mode_t)mode_cmd);
}

/**
 * @brief 解析 MIT 协议普通反馈帧
 * @param frame CAN 帧
 * @param feedback 输出反馈数据
 * @retval true 解析成功
 * @retval false 帧类型不匹配或参数无效
 */
bool rs06_mit_parse_feedback(const can_bus_frame_t* frame, rs06_mit_feedback_t* feedback) {
    uint16_t pos;
    uint16_t vel;
    uint16_t tq;
    uint16_t temp;

    if(frame == NULL || feedback == NULL || frame->id_type != CAN_BUS_ID_STD || frame->len < RS06_FRAME_LEN) return false;

    memset(feedback, 0, sizeof(*feedback));
    feedback->motor_id = frame->data[0];
    pos = ((uint16_t)frame->data[1] << 8) | frame->data[2];
    vel = ((uint16_t)frame->data[3] << 4) | (frame->data[4] >> 4);
    tq = (((uint16_t)(frame->data[4] & 0x0FU)) << 8) | frame->data[5];
    temp = (((uint16_t)(frame->data[6] & 0x0FU)) << 8) | frame->data[7];

    feedback->position_rad = rs06_uint_to_float(pos, RS06_P_MIN, RS06_P_MAX, 16);
    feedback->velocity_rad_s = rs06_uint_to_float(vel, RS06_V_MIN, RS06_V_MAX, 12);
    feedback->torque_nm = rs06_uint_to_float(tq, RS06_T_MIN, RS06_T_MAX, 12);
    feedback->pattern = (rs06_pattern_t)((frame->data[6] >> 6) & 0x03U);
    feedback->has_fault = (uint8_t)((frame->data[6] >> 5) & 0x01U);
    feedback->has_warning = (uint8_t)((frame->data[6] >> 4) & 0x01U);
    feedback->temperature_c = ((float)temp) * 0.1f;

    return true;
}

/**
 * @brief 解析 MIT 协议异常状态反馈帧
 * @param frame CAN 帧
 * @param reply 输出异常状态
 * @retval true 解析成功
 * @retval false 帧类型不匹配或参数无效
 */
bool rs06_mit_parse_fault_reply(const can_bus_frame_t* frame, rs06_mit_fault_reply_t* reply) {
    if(frame == NULL || reply == NULL || frame->id_type != CAN_BUS_ID_STD || frame->len < RS06_FRAME_LEN) return false;

    memset(reply, 0, sizeof(*reply));
    reply->motor_id = frame->data[0];
    reply->fault = ((uint32_t)frame->data[1]) |
        ((uint32_t)frame->data[2] << 8) |
        ((uint32_t)frame->data[3] << 16) |
        ((uint32_t)frame->data[4] << 24);
    reply->warning = ((uint32_t)frame->data[4]) |
        ((uint32_t)frame->data[5] << 8) |
        ((uint32_t)frame->data[6] << 16) |
        ((uint32_t)frame->data[7] << 24);
    memcpy(reply->raw, frame->data, sizeof(reply->raw));

    return true;
}

/**
 * @brief 解析 MIT 协议参数读写响应
 * @param frame CAN 帧
 * @param reply 输出参数响应
 * @retval true 解析成功
 * @retval false 帧类型不匹配或参数无效
 */
bool rs06_mit_parse_param_reply(const can_bus_frame_t* frame, rs06_param_reply_t* reply) {
    if(frame == NULL || reply == NULL || frame->id_type != CAN_BUS_ID_STD || frame->len < RS06_FRAME_LEN) return false;

    memset(reply, 0, sizeof(*reply));
    reply->motor_id = (uint8_t)(frame->id & 0xFFU);
    reply->host_id = 0U;
    reply->index = rs06_u16_le(&frame->data[0]);
    reply->value_u8 = frame->data[4];
    reply->value_u16 = rs06_u16_le(&frame->data[4]);
    reply->value_u32 = rs06_u32_le(&frame->data[4]);
    memcpy(&reply->value_float, &frame->data[4], sizeof(float));
    memcpy(reply->raw, frame->data, sizeof(reply->raw));

    return true;
}

/**
 * @brief 解析 MIT 协议设备 ID 响应
 * @param frame CAN 帧
 * @param reply 输出设备 ID 响应
 * @retval true 解析成功
 * @retval false 帧类型不匹配或参数无效
 */
bool rs06_mit_parse_device_id_reply(const can_bus_frame_t* frame, rs06_device_id_reply_t* reply) {
    if(frame == NULL || reply == NULL || frame->id_type != CAN_BUS_ID_STD || frame->len < RS06_FRAME_LEN) return false;

    memset(reply, 0, sizeof(*reply));
    reply->motor_id = (uint8_t)(frame->id & 0xFFU);
    reply->target_id = 0U;
    memcpy(reply->unique_id, frame->data, sizeof(reply->unique_id));

    return true;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 生成 RS06 私有协议 29 bit 扩展帧 ID
 */
static uint32_t rs06_make_ext_id(uint8_t type, uint16_t data16, uint8_t target_id) {
    return (((uint32_t)type & 0x1FU) << 24) |
        (((uint32_t)data16 & 0xFFFFU) << 8) |
        ((uint32_t)target_id);
}

/**
 * @brief 将主机 ID 扩展为私有协议 ID 中间字段
 */
static uint16_t rs06_host_field(uint8_t host_id) {
    return (uint16_t)host_id;
}

/**
 * @brief 读取大端 uint16
 */
static uint16_t rs06_u16_be(const uint8_t* data) {
    return ((uint16_t)data[0] << 8) | data[1];
}

/**
 * @brief 读取小端 uint16
 */
static uint16_t rs06_u16_le(const uint8_t* data) {
    return ((uint16_t)data[1] << 8) | data[0];
}

/**
 * @brief 读取小端 uint32
 */
static uint32_t rs06_u32_le(const uint8_t* data) {
    return ((uint32_t)data[0]) |
        ((uint32_t)data[1] << 8) |
        ((uint32_t)data[2] << 16) |
        ((uint32_t)data[3] << 24);
}

/**
 * @brief 浮点数线性映射到无符号整数
 */
static uint16_t rs06_float_to_uint(float x, float min, float max, uint8_t bits) {
    if(x < min) x = min;
    if(x > max) x = max;

    const float span = max - min;
    const float normalized = (x - min) / span;
    const uint32_t max_int = (1UL << bits) - 1UL;
    return (uint16_t)(normalized * (float)max_int + 0.5f);
}

/**
 * @brief 无符号整数线性映射到浮点数
 */
static float rs06_uint_to_float(uint16_t x, float min, float max, uint8_t bits) {
    const uint32_t max_int = (1UL << bits) - 1UL;
    x &= (uint16_t)max_int;
    return ((float)x) * (max - min) / (float)max_int + min;
}

/**
 * @brief 通过扩展帧发送 8 字节 RS06 私有协议数据
 */
static rs06_status_t rs06_send_ext(const rs06_t* motor, uint32_t ext_id, const uint8_t data[RS06_FRAME_LEN]) {
    if(motor == NULL || motor->bus == NULL || data == NULL) return RS06_INVALID_ARG;

    can_bus_status_t st = can_bus_send_ext(motor->bus, ext_id, data, RS06_FRAME_LEN, motor->tx_timeout_ms);
    return (st == CAN_BUS_OK) ? RS06_OK : RS06_CAN_ERROR;
}

/**
 * @brief 通过标准帧发送 8 字节 RS06 MIT 协议数据
 */
static rs06_status_t rs06_send_std(const rs06_t* motor, uint32_t std_id, const uint8_t data[RS06_FRAME_LEN]) {
    if(motor == NULL || motor->bus == NULL || data == NULL) return RS06_INVALID_ARG;

    can_bus_status_t st = can_bus_send_std(motor->bus, std_id, data, RS06_FRAME_LEN, motor->tx_timeout_ms);
    return (st == CAN_BUS_OK) ? RS06_OK : RS06_CAN_ERROR;
}

/**
 * @brief 写入参数索引到数据区 Byte0 ~ 1
 */
static void rs06_put_index(uint8_t data[RS06_FRAME_LEN], uint16_t index) {
    data[0] = (uint8_t)(index & 0xFFU);
    data[1] = (uint8_t)((index >> 8) & 0xFFU);
    data[2] = 0U;
    data[3] = 0U;
}

/**
 * @brief 获取扩展帧通信类型字段
 */
static uint8_t rs06_get_ext_type(const can_bus_frame_t* frame) {
    return (uint8_t)((frame->id >> 24) & 0x1FU);
}

/**
 * @brief 写入 4 字节原始参数值
 */
static rs06_status_t rs06_write_param_raw(const rs06_t* motor, uint16_t index, const uint8_t raw[4]) {
    uint8_t data[RS06_FRAME_LEN] = { 0 };
    if(motor == NULL || raw == NULL) return RS06_INVALID_ARG;

    rs06_put_index(data, index);
    memcpy(&data[4], raw, 4U);

    return rs06_send_ext(motor,
        rs06_make_ext_id(RS06_TYPE_WRITE_PARAM, rs06_host_field(motor->host_id), motor->motor_id),
        data);
}

/**
 * @brief 写入 uint8 参数并按需保存
 */
static rs06_status_t rs06_write_param_u8_then_save(const rs06_t* motor, uint16_t index, uint8_t value, bool save) {
    rs06_status_t st = rs06_write_param_u8(motor, index, value);
    if(st != RS06_OK || !save) return st;
    HAL_Delay(100U);
    return rs06_save_config(motor);
}

/**
 * @brief 写入 uint16 参数并按需保存
 */
static rs06_status_t rs06_write_param_u16_then_save(const rs06_t* motor, uint16_t index, uint16_t value, bool save) {
    rs06_status_t st = rs06_write_param_u16(motor, index, value);
    if(st != RS06_OK || !save) return st;
    HAL_Delay(100U);
    return rs06_save_config(motor);
}

/**
 * @brief 写入 uint32 参数并按需保存
 */
static rs06_status_t rs06_write_param_u32_then_save(const rs06_t* motor, uint16_t index, uint32_t value, bool save) {
    rs06_status_t st = rs06_write_param_u32(motor, index, value);
    if(st != RS06_OK || !save) return st;
    HAL_Delay(100U);
    return rs06_save_config(motor);
}

/**
 * @brief 写入 float 参数并按需保存
 */
static rs06_status_t rs06_write_param_float_then_save(const rs06_t* motor, uint16_t index, float value, bool save) {
    rs06_status_t st = rs06_write_param_float(motor, index, value);
    if(st != RS06_OK || !save) return st;
    HAL_Delay(100U);
    return rs06_save_config(motor);
}

/**
 * @brief 填充 MIT 协议固定 0xFF 前缀
 */
static void rs06_mit_fill_ff(uint8_t data[RS06_FRAME_LEN]) {
    for(uint8_t i = 0U; i < RS06_FRAME_LEN; ++i) data[i] = 0xFFU;
}

/**
 * @brief 生成 MIT 位置、速度、读写参数等模式类标准帧 ID
 */
static uint32_t rs06_mit_make_mode_std_id(uint8_t cmd, uint8_t motor_id) {
    return (((uint32_t)cmd & 0x07U) << 8) | motor_id;
}

/**
 * @brief MIT 协议写入 4 字节原始参数值
 */
static rs06_status_t rs06_mit_write_param_raw(const rs06_t* motor, uint16_t index, const uint8_t raw[4]) {
    uint8_t data[RS06_FRAME_LEN] = { 0 };
    if(motor == NULL || raw == NULL) return RS06_INVALID_ARG;

    rs06_put_index(data, index);
    memcpy(&data[4], raw, 4U);

    return rs06_send_std(motor, rs06_mit_make_mode_std_id(4U, motor->motor_id), data);
}
