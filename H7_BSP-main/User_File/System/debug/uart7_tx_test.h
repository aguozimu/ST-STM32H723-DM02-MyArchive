/**
 * @file    uart7_tx_test.h
 * @brief   UART7 发送链路测试：TIM7 10 kHz 中断内直接向 TDR 写递增字节
 * @author  zzm
 * @date    2026-09-20
 */

#ifndef UART7_TX_TEST_H
#define UART7_TX_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief 启动 UART7 10 kHz 发送测试
 * @note  需在 MX_TIM7_Init() 与 MX_UART7_Init() 之后调用，内部会重新配置并启动 TIM7
 */
void UART7_Tx_Test_Init(void);

/**
 * @brief 停止测试并关闭 TIM7 中断
 */
void UART7_Tx_Test_Stop(void);

/**
 * @brief TIM7 周期中断处理，由 TIM_PeriodElapsedCallback() 分发
 */
void UART7_Tx_Test_PeriodElapsedCallback(void);

/**
 * @brief 取发送统计
 * @param Sent    已发出字节数，可传 NULL
 * @param Dropped 因 TXE 未就绪丢弃的次数，可传 NULL
 */
void UART7_Tx_Test_Get_Stats(uint32_t *Sent, uint32_t *Dropped);

#ifdef __cplusplus
}
#endif

#endif /* UART7_TX_TEST_H */
