 /*
 * Change Logs:
 * Date            Author          Notes
 * 2023-08-23      ChuShicheng     first version
 */
#ifndef _RM_BSP_H
#define _RM_BSP_H

#include <rtthread.h>
#include "main.h"

#ifdef BSP_USING_CAN
#include "drv_can.h"
#endif /* BSP_USING_CAN */

#ifdef BSP_USING_DWT
#include "drv_dwt.h"
#endif /* BSP_USING_DWT */

#ifdef BSP_USING_MOTOR
//#include "motor_task.h"
#endif /* BSP_USING_MOTOR */

#endif /* _RM_BSP_H */
