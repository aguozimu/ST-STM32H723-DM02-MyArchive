/**
 ******************************************************************************
 * @file    bsp_can.h
 * @brief   板级支持包：CAN 总线驱动
 * @details
 * 接收方向使用 (FDCAN 句柄, 帧类型, CAN ID) 作为回调注册键。
 * 发送方向保留两条通道：
 * - CAN_Tx_Submit：插入队列，每次提交都按顺序发送。
 * - CAN_Tx_Perform：周期缓冲，同一 (FDCAN, 帧类型, CAN ID) 只保留最新数据。
 * @author  zzm
 * @date    2026-05-18
 * @version v2.1
 ******************************************************************************
 */
#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "fdcan.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/** Classic CAN 单帧允许的最大数据长度。 */
#define FDCAN_MAX_PAYLOAD (8)

/** CAN 标识符类型；零值保持为标准帧，兼容现有消息的零初始化。 */
typedef enum
{
    CAN_ID_STANDARD = 0U,
    CAN_ID_EXTENDED = 1U,
} Enum_CAN_Id_Type;

/**
 * @brief CAN 接收回调类型。
 * @param hfdcan 实际收到该帧的 FDCAN 句柄。
 * @param id 接收到的 CAN ID。
 * @param data 接收数据，只在本次回调执行期间有效。
 * @param len 接收数据长度。
 * @param context 注册时保存的设备实例或用户数据。
 * @note 回调运行在 FDCAN 接收中断中，不应阻塞。
 */
typedef void (*CAN_RxCallback_t)(FDCAN_HandleTypeDef *hfdcan,
                                 uint32_t id,
                                 uint8_t *data,
                                 uint32_t len,
                                 void *context);

/** CAN 原始接收诊断，用于区分“总线无帧”和“软件回调未匹配”。 */
typedef struct
{
    uint32_t rx_count;
    uint32_t last_id;
    uint32_t rx_read_error_count;
    uint8_t last_length;
    uint8_t last_data[FDCAN_MAX_PAYLOAD];
} Struct_CAN_Rx_Diagnostic;

/**
 * @brief 一帧完整的 Classic CAN 发送数据。
 * @note 队列和周期槽都会复制结构体内容，调用者可以使用局部变量。
 */
typedef struct
{
    FDCAN_HandleTypeDef *hfdcan;     /*!< 目标 FDCAN 总线句柄。 */
    uint32_t id;                     /*!< 标准帧为 0x000~0x7FF；扩展帧为 0x00000000~0x1FFFFFFF。 */
    uint8_t data[FDCAN_MAX_PAYLOAD]; /*!< 待发送的数据。 */
    uint8_t len;                     /*!< 有效数据长度，取值范围为 1~8。 */
    Enum_CAN_Id_Type id_type;        /*!< CAN ID 类型；零初始化时为标准帧。 */
} Struct_CAN_Tx_Msg;

/**
 * @brief 初始化三条 FDCAN 总线、插入队列和周期发送槽，清除待重试插入消息。
 * @note 应在 RTOS 内核初始化完成后、创建 CAN 发送任务前调用一次。
 */
void BSP_CAN_ConfigInit(void);

/** 读取指定 FDCAN 总线收到的原始帧诊断快照。 */
bool BSP_CAN_GetRxDiagnostic(FDCAN_HandleTypeDef *hfdcan,
                            Struct_CAN_Rx_Diagnostic *diagnostic);

/**
 * @brief 注册一个接收回调。
 * @param can_id 要监听的标准帧 ID。
 * @param hfdcan 要监听的 FDCAN 总线句柄。
 * @param callback 收到匹配报文时调用的函数。
 * @param context 回调对应的设备实例或用户数据，BSP 不解析其内容。
 * @return true 注册成功；false 注册表已满或相同键已经注册。
 * @note 注册键是 (hfdcan, can_id)，因此不同总线可以注册相同 ID。
 * @note callback 在 FDCAN 接收中断中执行，不应进行阻塞操作。
 */
bool BSP_CAN_RegisterCallback(uint32_t can_id,
                              FDCAN_HandleTypeDef *hfdcan,
                              CAN_RxCallback_t callback,
                              void *context);

/**
 * @brief 按帧类型注册一个接收回调。
 * @param can_id 要监听的 CAN ID。
 * @param id_type 标准帧或扩展帧。
 * @param hfdcan 要监听的 FDCAN 总线句柄。
 * @param callback 收到匹配报文时调用的函数。
 * @param context 回调对应的设备实例或用户数据，BSP 不解析其内容。
 * @return true 注册成功；false 参数无效、注册表已满或相同键已经注册。
 * @note 注册键是 (hfdcan, id_type, can_id)。callback 在 FDCAN 接收中断中执行。
 */
bool BSP_CAN_RegisterCallbackEx(uint32_t can_id,
                                Enum_CAN_Id_Type id_type,
                                FDCAN_HandleTypeDef *hfdcan,
                                CAN_RxCallback_t callback,
                                void *context);

/**
 * @brief 把一帧消息插入发送队列。
 * @param tx_msg 要插入队列的完整 CAN 消息。
 * @return true 已成功复制到队列；false 参数无效、队列未创建或队列已满。
 * @note 每次提交都会按队列顺序处理，适合使能、失能、复位、回零等命令。
 * @note 函数会复制消息内容，返回后调用者可以继续修改或释放原变量。
 */
bool CAN_Tx_Submit(const Struct_CAN_Tx_Msg *tx_msg);

/**
 * @brief 将多帧消息作为一个连续批次插入发送队列。
 * @param tx_msgs 完整 CAN 消息数组。
 * @param count 数组元素数量，必须为 1~CAN 发送队列容量。
 * @return true 整批已连续入队；false 参数无效、队列空间不足或互斥锁不可用。
 * @details 所有生产者共用提交互斥锁，批次内部不会被其他任务提交的消息穿插，
 *          适合设备协议的长命令分包。调用者必须处于任务上下文。
 */
bool CAN_Tx_SubmitBatch(const Struct_CAN_Tx_Msg *tx_msgs, size_t count);

/**
 * @brief 更新周期发送缓冲。
 * @param tx_msg 要发布的最新周期消息。
 * @return true 已更新或分配周期槽；false 参数无效或周期槽已满。
 * @note BSP 自动按 (hfdcan, id_type, id) 查找或分配槽，不需要设备层保存槽号。
 * @note 相同键只保留最新数据；不同总线、帧类型或 ID 使用不同槽。
 * @note 本函数只更新内存中的周期槽，不直接调用 HAL 发送。
 */
bool CAN_Tx_Perform(const Struct_CAN_Tx_Msg *tx_msg);

/**
 * @brief 按先进先出顺序处理插入发送队列。
 * @details FIFO 满或 HAL 写入失败时保留当前帧并立即返回，
 *          下次调用先重试该帧，成功后才继续处理后续消息。
 * @note 只能由同一个 CAN 发送任务周期调用，不支持并发或重入。
 * @note 三条总线共享队列；队首失败会阻止所有后续插入消息越过，
 *       包括其他总线的消息。BSP_CAN_SendPer 可继续独立处理周期缓冲。
 */
void BSP_CAN_SendAsync(void);

/**
 * @brief 周期检查所有发送槽，并发送尚未处理的最新数据。
 * @return true 所有待处理槽均已写入硬件 FIFO，或当前没有待处理数据；
 *         false 至少有一个待处理槽暂时未能写入硬件 FIFO。
 * @note 通常由 CAN 发送任务周期调用。
 */
bool BSP_CAN_SendPer(void);

#ifdef __cplusplus
}
#endif

#endif
