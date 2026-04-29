/**
 * @file    delay.h
 * @brief   延时服务 (阻塞 & 非阻塞)
 */
#ifndef _delay_h_
#define _delay_h_

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 毫秒微秒时间类型定义
 */
typedef uint32_t ms_t;
typedef uint32_t us_t;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 初始化毫秒延时服务，注册获取时间的回调函数
 * @param get_ms 获取当前毫秒时间的回调函数指针
 * @return 无
 */
void delay_ms_init(ms_t(*get_ms)(void));
/**
 * @brief 阻塞式延时 (毫秒)
 * @param ms 延时时长 (毫秒)
 * @return 无
 */
void delay_ms(ms_t ms);
/**
 * @brief 阻塞式延时 (秒)
 * @param s 延时时长 (秒)
 * @return 无
 */
void delay_s(ms_t s);
/**
 * @brief 非阻塞式延时检测 (毫秒)
 * @param start 延时起始时间的存储地址
 * @param interval_ms 延时间隔 (毫秒)
 * @return 延时是否完成 (true: 完成; false: 未完成)
 */
bool delay_nb_ms(ms_t* start, ms_t interval_ms);

/**
 * @brief 初始化微秒延时服务，注册获取时间的回调函数
 * @param get_us 获取当前微秒时间的回调函数指针
 * @return 无
 */
void delay_us_init(us_t(*get_us)(void));
/**
 * @brief 阻塞式延时 (微秒)
 * @param us 延时时长 (微秒)
 * @return 无
 */
void delay_us(us_t us);
/**
 * @brief 非阻塞式延时检测 (微秒)
 * @param start 延时起始时间的存储地址
 * @param interval_us 延时间隔 (微秒)
 * @return 延时是否完成 (true: 完成; false: 未完成)
 */
bool delay_nb_us(us_t* start, us_t interval_us);

#endif
