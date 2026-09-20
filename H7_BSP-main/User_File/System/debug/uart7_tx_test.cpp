/**
 * @file    uart7_tx_test.cpp
 * @brief   UART7 发送链路测试
 * @author  zzm
 * @date    2026-09-20
 *
 * @details TIM7 以 10 kHz 触发周期中断，每次中断向 UART7 发出 1 字节递增计数
 *          （0x00, 0x01, ... 0xFF, 0x00 ...），用于验证 PE7/PE8 到接收端的链路。
 *
 *          不走 bsp_uart 的 UART_Transmit_Data()：
 *            - 1 字节走 DMA 的中断开销远大于数据本身；
 *            - UART_Transmit_Data() 没有发送忙判断，上一帧未发完时直接返回
 *              HAL_BUSY 且无人检查返回值，连续高频调用会静默丢帧。
 *          这里直接判 TXE 后写 TDR，中断内没有等待，不会被线路拖住。
 *
 * @note    波特率 115200 8N1 时，1 字节占 10 bit = 86.8 us；10 kHz 周期为
 *          100 us，线路占用率约 86.8%，余量只有 13.2 us。
 *          若 Dropped 持续增长，把测试速率降到 5 kHz，或把 UART7 波特率提到
 *          230400（需 DAPLink 侧一致）。
 */

/* Includes ------------------------------------------------------------------*/

#include "uart7_tx_test.h"

#include "main.h"
#include "tim.h"
#include "usart.h"

/* Private macros ------------------------------------------------------------*/

// TIM7 计数时钟 = 240 MHz / 240 = 1 MHz，Period = 100 得 10 kHz
#define UART7_TEST_TIM_PRESCALER (240U - 1U)
#define UART7_TEST_TIM_PERIOD    (100U - 1U)

/* Private variables ---------------------------------------------------------*/

static volatile uint32_t Tx_Byte = 0U;
static volatile uint32_t Tx_Sent_Count = 0U;
static volatile uint32_t Tx_Drop_Count = 0U;

/* Function prototypes -------------------------------------------------------*/

extern "C" void UART7_Tx_Test_Init(void)
{
    // 运行时改写 TIM7 的周期，而不是去改 CubeMX 生成的 tim.c，
    // 否则下次重新生成代码时会被覆盖回 65535。
    htim7.Init.Prescaler = UART7_TEST_TIM_PRESCALER;
    htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim7.Init.Period = UART7_TEST_TIM_PERIOD;
    htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
    {
        Error_Handler();
    }

    Tx_Byte = 0U;
    Tx_Sent_Count = 0U;
    Tx_Drop_Count = 0U;

    HAL_TIM_Base_Start_IT(&htim7);
}

extern "C" void UART7_Tx_Test_Stop(void)
{
    HAL_TIM_Base_Stop_IT(&htim7);
}

extern "C" void UART7_Tx_Test_PeriodElapsedCallback(void)
{
    // TXE 置位表示 TDR 已空，可以写入下一字节。
    // 若未置位，说明上一字节还没被取走（线路跟不上），此时丢弃并计数；
    // 不在中断里自旋等待，避免拖住调度。
    if ((UART7->ISR & USART_ISR_TXE_TXFNF) != 0U)
    {
        UART7->TDR = Tx_Byte;
        Tx_Byte = (Tx_Byte + 1U) & 0xFFU;
        Tx_Sent_Count++;
    }
    else
    {
        Tx_Drop_Count++;
    }
}

extern "C" void UART7_Tx_Test_Get_Stats(uint32_t *Sent, uint32_t *Dropped)
{
    if (Sent != nullptr)
    {
        *Sent = Tx_Sent_Count;
    }
    if (Dropped != nullptr)
    {
        *Dropped = Tx_Drop_Count;
    }
}
