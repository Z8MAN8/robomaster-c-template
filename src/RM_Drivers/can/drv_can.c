 /*
 * Change Logs:
 * Date            Author          Notes
 * 2023-08-23      ChuShicheng     first version
 */
#include "drv_can.h"

#define DBG_TAG           "bsp.can"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

/* can object ptrs storage, used for recv callback */
// 在CAN产生接收中断会遍历数组,选出hcan和rxid与发生中断的实例相同的那个,调用其回调函数
// @todo: 后续为每个CAN总线单独添加一个can_object指针数组,提高回调查找的性能
static can_obj_t *can_object[CAN_REGISTER_MAX] = {NULL};
static uint8_t idx; // 全局CAN实例索引,每次有新的模块注册会自增

/**
 * @brief 添加过滤器以实现对特定 id 的报文的接收
 *
 * @note 这里将其配置为前14个过滤器给CAN1使用,后14个被CAN2使用
 *       初始化时，CAN1使用FIFO0，CAN2使用FIFO1
 *
 * @param object can object owned by specific module
 */
static void can_add_filter(can_obj_t *object)
{
    CAN_FilterTypeDef can_filter_conf;
    static uint8_t can1_filter_idx = 0, can2_filter_idx = 14;

    can_filter_conf.FilterMode = CAN_FILTERMODE_IDLIST;
    can_filter_conf.FilterScale = CAN_FILTERSCALE_16BIT;                                                      // 使用16位 id 模式,即只有低16位有效
    can_filter_conf.FilterFIFOAssignment = object->can_handle == &hcan1 ? CAN_RX_FIFO0 : CAN_RX_FIFO1;
    can_filter_conf.SlaveStartFilterBank = 14;                                                                // 从第14个过滤器开始配置从机过滤器(在 STM32 的 BxCAN 控制器中 CAN2 是 CAN1 的从机)
    can_filter_conf.FilterIdLow = object->rx_id << 5;                                                         // 过滤器寄存器的低16位,因为使用 STDID ,所以只有低11位有效,高5位要填0
    can_filter_conf.FilterBank = object->can_handle == &hcan1 ? (can1_filter_idx++) : (can2_filter_idx++);
    can_filter_conf.FilterActivation = CAN_FILTER_ENABLE;

    HAL_CAN_ConfigFilter(object->can_handle, &can_filter_conf);
}

/**
 * @brief 在第一个 CAN 实例初始化的时候会自动调用此函数,启动 CAN 服务
 *
 * @note 此函数会启动 CAN1 和 CAN2 ,开启 FIFO0 & FIFO1 接收通知
 *
 */
static void can_service_init()
{
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
}

can_obj_t *can_register(can_config_t *config)
{
    if (!idx)
    {
        can_service_init(); // 第一次注册,先进行硬件初始化
        LOG_I("CAN Service Init");
    }
    if (idx >= CAN_REGISTER_MAX) // 超过最大实例数
    {
        while (1)
            LOG_E("CAN object exceeded MAX num, consider balance the load of CAN bus");
    }
    for (size_t i = 0; i < idx; i++)
    { // 重复注册 | id重复
        if (can_object[i]->rx_id == config->rx_id && can_object[i]->can_handle == config->can_handle)
        {
            while (1)
                LOG_E("CAN id crash ,tx [%d] or rx [%d] already registered", &config->tx_id, &config->rx_id);
        }
    }
    
    can_obj_t *object = (can_obj_t *)rt_malloc(sizeof(can_obj_t)); // 分配空间
    rt_memset(object, 0, sizeof(can_obj_t));                       // 分配的空间未必是0,所以要先清空
    // 进行发送报文的配置
    object->txconf.StdId = config->tx_id; // 发送id
    object->txconf.IDE = CAN_ID_STD;      // 使用标准id
    object->txconf.RTR = CAN_RTR_DATA;    // 发送数据帧
    object->txconf.DLC = 0x08;            // 默认发送长度为8
    // 设置回调函数和接收发送id
    object->can_handle = config->can_handle;
    object->tx_id = config->tx_id;
    object->rx_id = config->rx_id;
    object->can_module_callback = config->can_module_callback;
    object->object_name = config->object_name;

    can_add_filter(object);         // 添加 CAN 过滤器
    can_object[idx++] = object;     // 将实例保存到can_object中

    return object; // 返回can实例指针
}

/* 如果让CANobject保存txbuff,会增加一次复制的开销 */
uint8_t can_transmit(can_obj_t *object, float timeout)
{
    static uint32_t busy_count;
    static volatile float wait_time __attribute__((unused)); // for cancel warning
    uint32_t send_mail_box;
    float dwt_start = dwt_get_time_ms();
    while (HAL_CAN_GetTxMailboxesFreeLevel(object->can_handle) == 0) // 等待邮箱空闲
    {
        if (dwt_get_time_ms() - dwt_start > timeout) // 超时
        {
            LOG_W("CAN MAILbox full! failed to add msg to mailbox. Cnt [%d]", busy_count);
            busy_count++;
            return 0;
        }
    }
    wait_time = dwt_get_time_ms() - dwt_start;

    if (HAL_CAN_AddTxMessage(object->can_handle, &object->txconf, object->tx_buff, &send_mail_box))
    {
        LOG_W("CAN bus BUS! cnt:%d", busy_count);
        busy_count++;
        return 0;
    }
    return 1; // 发送成功
}

void can_set_dlc(can_obj_t *object, uint8_t length)
{
    if (length > 8 || length == 0)
        while (1)
            LOG_E("CAN DLC error! check your code or wild pointer");
    object->txconf.DLC = length;
}

/* -----------------------belows are callback definitions--------------------------*/

/**
 * @brief rx fifo callback. Once FIFO_0 is full,this func would be called
 *
 * @param hcan CAN handle indicate which device the oddest mesg in FIFO_0 comes from
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    static CAN_RxHeaderTypeDef rxconf;
    uint8_t can_rx_buff[8];
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0))
    {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxconf, can_rx_buff);
        for (size_t i = 0; i < idx; ++i)
        {
            if (hcan == can_object[i]->can_handle && rxconf.StdId == can_object[i]->rx_id)  // 搜索对应 CAN 实例
            {
                if (can_object[i]->can_module_callback != NULL)
                {
                    can_object[i]->rx_len = rxconf.DLC;
                    rt_memcpy(can_object[i]->rx_buff, can_rx_buff, rxconf.DLC); // 消息拷贝到对应实例
                    can_object[i]->can_module_callback(can_object[i]);          // 触发回调进行数据解析处理
                }
                return;
            }
        }
    }
}

/**
 * @brief rx fifo callback. Once FIFO_1 is full,this func would be called
 *
 * @param hcan CAN handle indicate which device the oddest mesg in FIFO_1 comes from
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    static CAN_RxHeaderTypeDef rxconf;
    uint8_t can_rx_buff[8];
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO1))
    {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rxconf, can_rx_buff);
        for (size_t i = 0; i < idx; ++i)
        {
            if (hcan == can_object[i]->can_handle && rxconf.StdId == can_object[i]->rx_id)  // 搜索对应 CAN 实例
            {
                if (can_object[i]->can_module_callback != NULL)
                {
                    can_object[i]->rx_len = rxconf.DLC;
                    rt_memcpy(can_object[i]->rx_buff, can_rx_buff, rxconf.DLC); // 消息拷贝到对应实例
                    can_object[i]->can_module_callback(can_object[i]);          // 触发回调进行数据解析处理
                }
                return;
            }
        }
    }
}
