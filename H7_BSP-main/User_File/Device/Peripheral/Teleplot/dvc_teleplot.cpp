/**
 * @file    dvc_teleplot.cpp
 * @brief   teleplot 波形输出（VS Code teleplot 插件，ASCII 文本协议）
 * @version 0.1
 * @date    2026-09-21 0.1 新建：给 ADRC 调参看波形用
 *
 * @copyright USTC-RoboWalker (c) 2026
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_teleplot.h"

#include <math.h>
#include <string.h>

/* Exported variables --------------------------------------------------------*/

Class_Teleplot Teleplot;

#if TELEPLOT_ENABLE

/* Private macros ------------------------------------------------------------*/

/**
 * 一行里除通道名之外的固定开销，按最坏情况算：
 * '>' + ':' + 数值 + '\n' + 1 字节余量。
 * 数值最长是 "-42949.00000"（Format_Float 的幅值上限）= 12 字符，共 16。
 * 这个常量参与缓冲区越界判断，只能放大不能缩小 —— 写小了，
 * 通道一多就会把 DMA 缓冲区写越界，HAL 也会按超长长度去发。
 */
#define TELEPLOT_LINE_OVERHEAD (16U)

/** 小数位数 */
#define TELEPLOT_DECIMAL_DIGITS (5U)

/* Private variables ---------------------------------------------------------*/

/**
 * @brief 发送缓冲区
 * @note  必须在 .dma_buffer 段：D-Cache 打开时只有 RAM_DMA(0x24000000) 是
 *        non-cacheable（MPU region 0 + 链接脚本 h7_memory.ld），否则 DMA
 *        会把缓存里的旧数据发出去。
 */
__attribute__((section(".dma_buffer"), aligned(32)))
static uint8_t Teleplot_Tx_Buffer[TELEPLOT_TX_BUFFER_SIZE];

/* Private function declarations ---------------------------------------------*/

static uint16_t Format_Uint(char *dst, uint32_t value);
static uint16_t Format_Float(char *dst, float value);

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 无符号整数转十进制文本（不含结束符）
 */
static uint16_t Format_Uint(char *dst, uint32_t value)
{
    char reverse[10];
    uint16_t count = 0U;

    do
    {
        reverse[count] = (char)('0' + (value % 10U));
        value /= 10U;
        count++;
    } while ((value != 0U) && (count < sizeof(reverse)));

    for (uint16_t i = 0U; i < count; i++)
    {
        dst[i] = reverse[count - 1U - i];
    }

    return (count);
}

/**
 * @brief 浮点转十进制文本（不含结束符），固定 TELEPLOT_DECIMAL_DIGITS 位小数
 * @note  不用 snprintf("%f")：本工程只链了 --specs=nano.specs，没开
 *        -u _printf_float，nano 的 printf 打 %f 会输出空值。
 *        这里纯整数运算，没有 libc 浮点依赖。
 */
static uint16_t Format_Float(char *dst, float value)
{
    if (!isfinite(value))
    {
        dst[0] = 'n';
        dst[1] = 'a';
        dst[2] = 'n';
        return (3U);
    }

    uint16_t length = 0U;

    if (value < 0.0f)
    {
        dst[length] = '-';
        length++;
        value = -value;
    }

    /* 防止后面 value*100000 溢出 uint32 */
    if (value > 42949.0f)
    {
        value = 42949.0f;
    }

    uint32_t integer = (uint32_t)value;
    uint32_t fraction = (uint32_t)((value - (float)integer) * 100000.0f + 0.5f);
    if (fraction >= 100000U)
    {
        /* 小数四舍五入进位到整数 */
        integer++;
        fraction = 0U;
    }

    length += Format_Uint(&dst[length], integer);
    dst[length] = '.';
    length++;

    uint32_t divisor = 10000U;
    for (uint8_t i = 0U; i < TELEPLOT_DECIMAL_DIGITS; i++)
    {
        dst[length] = (char)('0' + ((fraction / divisor) % 10U));
        length++;
        divisor /= 10U;
    }

    return (length);
}

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 绑定发送口
 */
void Class_Teleplot::Init(UART_HandleTypeDef *huart, uint32_t period_ms)
{
    UART_Handler = huart;
    Period_Ms = (period_ms == 0U) ? TELEPLOT_PERIOD_DEFAULT_MS : period_ms;
    Tick_Counter = 0U;
}

/**
 * @brief 注册一路通道
 */
int Class_Teleplot::Channel_Add(const char *name, Teleplot_Sample_Function sample)
{
    if ((name == nullptr) || (sample == nullptr) || (Channel_Number >= TELEPLOT_CHANNEL_MAX))
    {
        return (-1);
    }

    Channel_Name[Channel_Number] = name;
    Channel_Sample[Channel_Number] = sample;
    Channel_Number++;

    return ((int)Channel_Number - 1);
}

/**
 * @brief 周期服务：每 1 ms 调一次，满 period_ms 才真正发一帧
 */
void Class_Teleplot::TIM_1ms_Tick()
{
    if ((UART_Handler == nullptr) || (Channel_Number == 0U) || (!Enabled))
    {
        return;
    }

    Tick_Counter++;
    if (Tick_Counter < Period_Ms)
    {
        return;
    }
    Tick_Counter = 0U;

    Output();
}

/**
 * @brief 组一帧（每路一行 `>name:value\n`）并启动 DMA 发送
 * @note  先确认上一帧发完了再动缓冲区：Transmit_DMA 只认 gState==READY，
 *        若先把新数据写进缓冲区再发现 BUSY，正在传输的 DMA 会读到半新半旧
 *        的内容。所以"先判忙、后填buf"。
 */
void Class_Teleplot::Output()
{
    if (UART_Handler->gState != HAL_UART_STATE_READY)
    {
        Tx_Drop_Count++;
        return;
    }

    uint16_t length = 0U;

    for (uint8_t i = 0U; i < Channel_Number; i++)
    {
        const char *name = Channel_Name[i];
        const uint16_t name_length = (uint16_t)strlen(name);

        if (((uint32_t)length + (uint32_t)name_length + TELEPLOT_LINE_OVERHEAD) >
            TELEPLOT_TX_BUFFER_SIZE)
        {
            /* 缓冲区放不下这一路：整帧放弃，宁可丢帧也不要发半帧 */
            Tx_Drop_Count++;
            return;
        }

        Teleplot_Tx_Buffer[length] = '>';
        length++;
        memcpy(&Teleplot_Tx_Buffer[length], name, name_length);
        length += name_length;
        Teleplot_Tx_Buffer[length] = ':';
        length++;

        length += Format_Float((char *)&Teleplot_Tx_Buffer[length], Channel_Sample[i]());

        Teleplot_Tx_Buffer[length] = '\n';
        length++;
    }

    if (HAL_UART_Transmit_DMA(UART_Handler, Teleplot_Tx_Buffer, length) != HAL_OK)
    {
        Tx_Drop_Count++;
        return;
    }

    Tx_Frame_Count++;
}

#else /* TELEPLOT_ENABLE */

/* 空实现：接口保留，不产生任何代码 */
void Class_Teleplot::Init(UART_HandleTypeDef *huart, uint32_t period_ms)
{
    (void)huart;
    (void)period_ms;
}

int Class_Teleplot::Channel_Add(const char *name, Teleplot_Sample_Function sample)
{
    (void)name;
    (void)sample;
    return (-1);
}

void Class_Teleplot::TIM_1ms_Tick() {}

void Class_Teleplot::Output() {}

#endif /* TELEPLOT_ENABLE */

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
