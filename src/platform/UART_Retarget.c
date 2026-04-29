#include <stdio.h>
#include "main.h"

extern UART_HandleTypeDef huart1;

/* ==================== 配置开关 ==================== */
// 如果启用了DMA接收（如 ReceiveToIdle_DMA）, 则定义此宏, 否则注释
#define UART_USE_DMA_RX
/* ================================================== */

/* ==================== 重 定 向 printf / scanf ==================== */

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define GETCHAR_PROTOTYPE int __io_getchar(void)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif

/* printf 单字符输出：统一使用阻塞发送 */
PUTCHAR_PROTOTYPE
{
    uint8_t ch8 = (uint8_t)ch;
    HAL_UART_Transmit(&huart1, &ch8, 1, HAL_MAX_DELAY);
    return ch;
}

/* scanf 单字符输入：仅在未启用DMA接收时允许阻塞读取 */
GETCHAR_PROTOTYPE
{
#ifdef UART_USE_DMA_RX
    // DMA接收模式下禁止阻塞接收，防止中断DMA流程
    return EOF;
#else
    uint8_t ch8;
    if(HAL_UART_Receive(&huart1, &ch8, 1, HAL_MAX_DELAY) == HAL_OK){
        return (int)ch8;
    }
    return EOF;
#endif
}

