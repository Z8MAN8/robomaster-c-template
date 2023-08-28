 /*
 * Change Logs:
 * Date            Author          Notes
 * 2023-08-23      ChuShicheng     first version
 */
#ifndef _DRV_CAN_H
#define _DRV_CAN_H

#include "rm_bsp.h"

#define CAN_REGISTER_MAX (2 * 7)  // 这个数量取决于CAN总线的负载

 extern CAN_HandleTypeDef hcan1;
 extern CAN_HandleTypeDef hcan2;

/* can object typedef, every module registered to CAN should have this variable */
typedef struct can_object
{
    CAN_HandleTypeDef *can_handle; // CAN句柄
    CAN_TxHeaderTypeDef txconf;    // CAN报文发送配置
    uint32_t tx_id;                // 发送id
    uint32_t rx_id;                // 接收id
    uint8_t tx_buff[8];            // 发送缓存,发送消息长度可以通过can_set_dlc()设定,最大为8
    uint8_t rx_buff[8];            // 接收缓存,最大消息长度为8
    uint8_t rx_len;                // 接收长度,可能为0-8
    // 接收的回调函数,用于解析接收到的数据
    void (*can_module_callback)(struct can_object *); // callback needs an object to tell among registered ones
    const char *object_name;       // 拥有CAN实例的模块名称
}can_obj_t;

/* CAN实例初始化结构体,将此结构体指针传入注册函数 */
typedef struct
{
    CAN_HandleTypeDef *can_handle;              // CAAN句柄
    uint32_t tx_id;                             // 发送id(主发)
    uint32_t rx_id;                             // 接收id(主收)
    void (*can_module_callback)(struct can_object *);   // 处理接收数据的回调函数
    const char *object_name;                    // 拥有CAN实例的模块名称
}can_config_t;

/**
 * @brief Register a module to CAN service,remember to call this before using a CAN device
 * @param config init config
 * @return can_obj_t* can object owned by module
 */
can_obj_t *can_register(can_config_t *config);

/**
 * @brief 修改CAN发送报文的数据帧长度;注意最大长度为8,在没有进行修改的时候,默认长度为8
 *
 * @param object 要修改长度的can实例
 * @param length    设定长度
 */
void can_set_dlc(can_obj_t *object, uint8_t length);

/**
 * @brief transmit mesg through CAN device,通过CAN实例发送消息
 *        发送前需要向CAN实例的tx_buff写入发送数据
 * 
 * @attention 超时时间不应该超过调用此函数的任务的周期,否则会导致任务阻塞
 * 
 * @param timeout 超时时间,单位为ms;后续改为us,获得更精确的控制
 * @param object* can object owned by module
 */
uint8_t can_transmit(can_obj_t *object,float timeout);

#endif /* _DRV_CAN_H */
