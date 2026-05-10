#include "servo.h"
#include <string.h>

#define RS06_TYPE_GET_ID          0x00U
#define RS06_TYPE_MOTION_CONTROL  0x01U
#define RS06_TYPE_FEEDBACK        0x02U
#define RS06_TYPE_ENABLE          0x03U
#define RS06_TYPE_STOP            0x04U
#define RS06_TYPE_SET_ZERO        0x06U
#define RS06_TYPE_SET_ID          0x07U
#define RS06_TYPE_READ_PARAM      0x11U
#define RS06_TYPE_WRITE_PARAM     0x12U
#define RS06_TYPE_SAVE            0x16U

#define RS06_PARAM_RUN_MODE       0x7005U
#define RS06_PARAM_IQ_REF         0x7006U
#define RS06_PARAM_SPEED_REF      0x700AU
#define RS06_PARAM_POSITION_REF   0x7016U
#define RS06_PARAM_CSP_LIMIT_SPD  0x7017U
#define RS06_PARAM_LIMIT_CUR      0x7018U
#define RS06_PARAM_PP_LIMIT_SPD   0x7024U
#define RS06_PARAM_PP_ACCEL       0x7025U
#define RS06_PARAM_ZERO_STA       0x2023U

#define RS06_P_MIN               (-12.57f)
#define RS06_P_MAX               ( 12.57f)
#define RS06_V_MIN               (-50.0f)
#define RS06_V_MAX               ( 50.0f)
#define RS06_T_MIN               (-36.0f)
#define RS06_T_MAX               ( 36.0f)
#define RS06_KP_MIN              (0.0f)
#define RS06_KP_MAX              (5000.0f)
#define RS06_KD_MIN              (0.0f)
#define RS06_KD_MAX              (100.0f)

static uint32_t rs06_make_ext_id(uint8_t type, uint16_t data16, uint8_t target_id) {
    return (((uint32_t)type & 0x1FU) << 24) |
        (((uint32_t)data16 & 0xFFFFU) << 8) |
        ((uint32_t)target_id);
}

static uint16_t rs06_host_field(uint8_t host_id) {
    return (uint16_t)host_id;
}

static uint16_t rs06_u16_be(const uint8_t* data) {
    return ((uint16_t)data[0] << 8) | data[1];
}

static uint16_t rs06_float_to_uint(float x, float min, float max, uint8_t bits) {
    if(x < min) x = min;
    if(x > max) x = max;

    const float span = max - min;
    const float normalized = (x - min) / span;
    const uint32_t max_int = (1UL << bits) - 1UL;
    return (uint16_t)(normalized * (float)max_int + 0.5f);
}

static float rs06_uint_to_float(uint16_t x, float min, float max, uint8_t bits) {
    const uint32_t max_int = (1UL << bits) - 1UL;
    return ((float)x) * (max - min) / (float)max_int + min;
}

static rs06_status_t rs06_send_ext(const rs06_t* motor, uint32_t ext_id, const uint8_t data[8]) {
    if(motor == NULL || motor->bus == NULL) {
        return RS06_INVALID_ARG;
    }

    can_bus_status_t st = can_bus_send_ext(motor->bus,
        ext_id,
        data,
        8U,
        motor->tx_timeout_ms);
    return (st == CAN_BUS_OK) ? RS06_OK : RS06_CAN_ERROR;
}

static void rs06_put_index(uint8_t data[8], uint16_t index) {
    data[0] = (uint8_t)(index & 0xFFU);
    data[1] = (uint8_t)((index >> 8) & 0xFFU);
    data[2] = 0U;
    data[3] = 0U;
}

void rs06_init(rs06_t* motor,
    FDCAN_HandleTypeDef* bus,
    uint8_t motor_id,
    uint8_t host_id) {
    if(motor == NULL) {
        return;
    }

    motor->bus = bus;
    motor->motor_id = motor_id;
    motor->host_id = (host_id == 0U) ? RS06_DEFAULT_HOST_ID : host_id;
    motor->tx_timeout_ms = CAN_BUS_TX_TIMEOUT_MS;
}

rs06_status_t rs06_enable(const rs06_t* motor) {
    uint8_t data[8] = { 0 };
    uint32_t id = rs06_make_ext_id(RS06_TYPE_ENABLE,
        rs06_host_field(motor->host_id),
        motor->motor_id);
    return rs06_send_ext(motor, id, data);
}

rs06_status_t rs06_stop(const rs06_t* motor, bool clear_error) {
    uint8_t data[8] = { 0 };
    data[0] = clear_error ? 1U : 0U;
    uint32_t id = rs06_make_ext_id(RS06_TYPE_STOP,
        rs06_host_field(motor->host_id),
        motor->motor_id);
    return rs06_send_ext(motor, id, data);
}

rs06_status_t rs06_set_mechanical_zero(const rs06_t* motor) {
    uint8_t data[8] = { 1U, 0U, 0U, 0U, 0U, 0U, 0U, 0U };
    uint32_t id = rs06_make_ext_id(RS06_TYPE_SET_ZERO,
        rs06_host_field(motor->host_id),
        motor->motor_id);
    return rs06_send_ext(motor, id, data);
}

rs06_status_t rs06_save_config(const rs06_t* motor) {
    uint8_t data[8] = { 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U };
    uint32_t id = rs06_make_ext_id(RS06_TYPE_SAVE,
        rs06_host_field(motor->host_id),
        motor->motor_id);
    return rs06_send_ext(motor, id, data);
}

rs06_status_t rs06_zeroing_and_save(const rs06_t* motor) {
    rs06_status_t st;

    /* Do not zero in PP mode. Stop first, then switch to motion mode. */
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

rs06_status_t rs06_read_param(const rs06_t* motor, uint16_t index) {
    uint8_t data[8] = { 0 };
    rs06_put_index(data, index);
    uint32_t id = rs06_make_ext_id(RS06_TYPE_READ_PARAM,
        rs06_host_field(motor->host_id),
        motor->motor_id);
    return rs06_send_ext(motor, id, data);
}

rs06_status_t rs06_write_param_u8(const rs06_t* motor, uint16_t index, uint8_t value) {
    uint8_t data[8] = { 0 };
    rs06_put_index(data, index);
    data[4] = value;
    uint32_t id = rs06_make_ext_id(RS06_TYPE_WRITE_PARAM,
        rs06_host_field(motor->host_id),
        motor->motor_id);
    return rs06_send_ext(motor, id, data);
}

rs06_status_t rs06_write_param_float(const rs06_t* motor, uint16_t index, float value) {
    uint8_t data[8] = { 0 };
    rs06_put_index(data, index);
    memcpy(&data[4], &value, sizeof(float));
    uint32_t id = rs06_make_ext_id(RS06_TYPE_WRITE_PARAM,
        rs06_host_field(motor->host_id),
        motor->motor_id);
    return rs06_send_ext(motor, id, data);
}

rs06_status_t rs06_set_mode(const rs06_t* motor, rs06_mode_t mode) {
    return rs06_write_param_u8(motor, RS06_PARAM_RUN_MODE, (uint8_t)mode);
}

rs06_status_t rs06_set_zero_sta(const rs06_t* motor, uint8_t zero_sta, bool save) {
    rs06_status_t st = rs06_write_param_u8(motor, RS06_PARAM_ZERO_STA, zero_sta ? 1U : 0U);
    if(st != RS06_OK || !save) {
        return st;
    }
    HAL_Delay(100U);
    return rs06_save_config(motor);
}

rs06_status_t rs06_set_pp_config(const rs06_t* motor, float speed_rad_s, float accel_rad_s2) {
    rs06_status_t st = rs06_write_param_float(motor, RS06_PARAM_PP_LIMIT_SPD, speed_rad_s);
    if(st != RS06_OK) return st;
    HAL_Delay(2U);
    return rs06_write_param_float(motor, RS06_PARAM_PP_ACCEL, accel_rad_s2);
}

rs06_status_t rs06_set_csp_config(const rs06_t* motor, float speed_rad_s) {
    return rs06_write_param_float(motor, RS06_PARAM_CSP_LIMIT_SPD, speed_rad_s);
}

rs06_status_t rs06_set_position_target(const rs06_t* motor, float position_rad) {
    return rs06_write_param_float(motor, RS06_PARAM_POSITION_REF, position_rad);
}

rs06_status_t rs06_pp_goto(const rs06_t* motor,
    float position_rad,
    float speed_rad_s,
    float accel_rad_s2) {
    rs06_status_t st = rs06_stop(motor, false);
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

rs06_status_t rs06_csp_goto(const rs06_t* motor,
    float position_rad,
    float speed_rad_s) {
    rs06_status_t st = rs06_stop(motor, false);
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

rs06_status_t rs06_motion_control(const rs06_t* motor,
    float torque_nm,
    float position_rad,
    float velocity_rad_s,
    float kp,
    float kd) {
    uint8_t data[8] = { 0 };
    uint16_t t = rs06_float_to_uint(torque_nm, RS06_T_MIN, RS06_T_MAX, 16);
    uint16_t p = rs06_float_to_uint(position_rad, RS06_P_MIN, RS06_P_MAX, 16);
    uint16_t v = rs06_float_to_uint(velocity_rad_s, RS06_V_MIN, RS06_V_MAX, 16);
    uint16_t kpi = rs06_float_to_uint(kp, RS06_KP_MIN, RS06_KP_MAX, 16);
    uint16_t kdi = rs06_float_to_uint(kd, RS06_KD_MIN, RS06_KD_MAX, 16);

    data[0] = (uint8_t)(p >> 8); data[1] = (uint8_t)p;
    data[2] = (uint8_t)(v >> 8); data[3] = (uint8_t)v;
    data[4] = (uint8_t)(kpi >> 8); data[5] = (uint8_t)kpi;
    data[6] = (uint8_t)(kdi >> 8); data[7] = (uint8_t)kdi;

    uint32_t id = rs06_make_ext_id(RS06_TYPE_MOTION_CONTROL, t, motor->motor_id);
    return rs06_send_ext(motor, id, data);
}

bool rs06_parse_feedback(const can_bus_frame_t* frame, rs06_feedback_t* feedback) {
    if(frame == NULL || feedback == NULL || frame->id_type != CAN_BUS_ID_EXT || frame->len < 8U) {
        return false;
    }

    uint8_t type = (uint8_t)((frame->id >> 24) & 0x1FU);
    if(type != RS06_TYPE_FEEDBACK) {
        return false;
    }

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
