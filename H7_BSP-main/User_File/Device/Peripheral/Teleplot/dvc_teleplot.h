/**
 * @file    dvc_teleplot.h
 * @brief   teleplot 波形输出（VS Code teleplot 插件，ASCII 文本协议）
 * @version 0.1
 * @date    2026-09-21 0.1 新建：给 ADRC 调参看波形用
 *
 * @details
 * 上行协议：每路一行 `>name:value\n`，多路拼成一帧、一次 DMA 发出去。
 *          teleplot 插件按 \n 切行，所以行尾只发 \n，不能发 \r\n。
 * 通道注册：Channel_Add() 传一个取值回调，不存变量指针 —— 回调读的是
 *          Class_ADRC 那种 Get_XXX() 接口，没有指针失效/生命周期问题。
 * 预留：    TELEPLOT_ENABLE 置 0 时全部实现变空函数，调用点不用改；
 *          换/加通道只改注册处（本工程在 TransportTask.cpp 的 RegisterTeleplotChannels()）。
 *
 * @note    两个工程约定别踩：
 *          1. 发送缓冲区必须放 .dma_buffer 段 —— D-Cache 开着，只有
 *             RAM_DMA(0x24000000) 被 MPU 配成 non-cacheable，否则 DMA
 *             发出去的是缓存里的旧数据（README「开发约定」）。
 *          2. newlib-nano 没开 -u _printf_float，snprintf("%f") 会打出空值，
 *             所以本模块自带定点格式化，不依赖 libc 的浮点 printf。
 *
 * @copyright USTC-RoboWalker (c) 2026
 */

#ifndef __DVC_TELEPLOT_H
#define __DVC_TELEPLOT_H

/* Includes ------------------------------------------------------------------*/

#include "bsp_uart.h" /* UART_HandleTypeDef（含 usart.h） */
#include <stdint.h>

/* Exported macros -----------------------------------------------------------*/

/** 总开关：置 0 时本模块退化为空实现，接口保留（"预留"用这个） */
#define TELEPLOT_ENABLE (1)

/** 最多注册多少路通道 */
#define TELEPLOT_CHANNEL_MAX (48)

/** 发送缓冲区长度。当前诊断通道约 850 B/帧，2048 留足余量。 */
#define TELEPLOT_TX_BUFFER_SIZE (2048)

/** 默认发送周期(ms)，50 → 20 Hz；适配 UART7 的 115200 波特率。 */
#define TELEPLOT_PERIOD_DEFAULT_MS (50U)

/**
 * 模块自身的上电默认值（本工程保持 0，不由这里开）。
 * UART7 在 TransportTask 注册完通道后显式调用 Set_Enabled(true)，
 * 避免 TIM1msTask 在通道尚未注册时抢先启动输出。
 */
#define TELEPLOT_DEFAULT_ENABLED (0)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 通道取值回调
 * @return 该通道当前值
 * @note   在任务上下文调用，不要做阻塞操作
 */
typedef float (*Teleplot_Sample_Function)(void);

/**
 * @brief teleplot 文本波形输出
 * @note  单例对象 @ref Teleplot。发送走 HAL_UART_Transmit_DMA，
 *        上一帧没发完就丢帧（Tx_Drop_Count 累加），绝不阻塞调用任务。
 */
class Class_Teleplot
{
public:
    /**
     * @brief 绑定发送口
     * @param huart     发送用的 UART 句柄，必须带 TX DMA（本工程用 &huart7）
     * @param period_ms 发送周期(ms)，默认 20 → 50 Hz
     */
    void Init(UART_HandleTypeDef *huart, uint32_t period_ms = TELEPLOT_PERIOD_DEFAULT_MS);

    /**
     * @brief 注册一路通道
     * @param name   波形名字，必须是静态存储期（字符串字面量即可）
     * @param sample 取值回调
     * @return 通道序号(0 起)，-1 表示已满
     */
    int Channel_Add(const char *name, Teleplot_Sample_Function sample);

    /** @brief 开关波形输出。打开时重置周期计数，等满一个周期才发第一帧 */
    void Set_Enabled(bool enabled);
    void Toggle_Enabled();
    bool Get_Enabled() const;

    /**
     * @brief 周期服务，每 1 ms 调一次，内部按 period_ms 分频
     * @note  必须在任务上下文调用（内部会启 DMA 发送）
     */
    void TIM_1ms_Tick();

    /* 诊断量 ----------------------------------------------------------------*/

    uint32_t Get_Tx_Frame_Count() const; /*!< 成功交给 DMA 的帧数 */
    uint32_t Get_Tx_Drop_Count() const;  /*!< 因上一帧未发完而丢掉的帧数 */
    uint8_t Get_Channel_Number() const;  /*!< 已注册通道数 */

protected:
    /** @brief 组一帧并启动 DMA 发送 */
    void Output();

    UART_HandleTypeDef *UART_Handler = nullptr;

    const char *Channel_Name[TELEPLOT_CHANNEL_MAX] = {nullptr};
    Teleplot_Sample_Function Channel_Sample[TELEPLOT_CHANNEL_MAX] = {nullptr};
    uint8_t Channel_Number = 0U;

    uint32_t Period_Ms = TELEPLOT_PERIOD_DEFAULT_MS;
    uint32_t Tick_Counter = 0U;

    bool Enabled = (TELEPLOT_DEFAULT_ENABLED != 0);

    uint32_t Tx_Frame_Count = 0U;
    uint32_t Tx_Drop_Count = 0U;
};

/* Exported variables --------------------------------------------------------*/

extern Class_Teleplot Teleplot;

/* Exported function declarations --------------------------------------------*/

inline void Class_Teleplot::Set_Enabled(bool enabled)
{
    Enabled = enabled;
    Tick_Counter = 0U;
}

inline void Class_Teleplot::Toggle_Enabled()
{
    Set_Enabled(!Enabled);
}

inline bool Class_Teleplot::Get_Enabled() const
{
    return (Enabled);
}

inline uint32_t Class_Teleplot::Get_Tx_Frame_Count() const
{
    return (Tx_Frame_Count);
}

inline uint32_t Class_Teleplot::Get_Tx_Drop_Count() const
{
    return (Tx_Drop_Count);
}

inline uint8_t Class_Teleplot::Get_Channel_Number() const
{
    return (Channel_Number);
}

#endif // !__DVC_TELEPLOT_H

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
