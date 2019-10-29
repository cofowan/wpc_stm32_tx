/*
 * Copyright (c) 2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-08-26     sogwms       The first version
 */

#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__
#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOSConfig.h"
#include "task.h"

//#include <rtthread.h>
/* RT-Thread error code definitions */
#define RT_EOK                          0               /**< There is no error */
#define RT_ERROR                        1               /**< A generic error happens */
#define RT_ETIMEOUT                     2               /**< Timed out */
#define RT_EFULL                        3               /**< The resource is full */
#define RT_EEMPTY                       4               /**< The resource is empty */
#define RT_ENOMEM                       5               /**< No memory */
#define RT_ENOSYS                       6               /**< No system */
#define RT_EBUSY                        7               /**< Busy */
#define RT_EIO                          8               /**< IO error */
#define RT_EINTR                        9               /**< Interrupted system call */
#define RT_EINVAL                       10              /**< Invalid argument */
#define RT_NULL                         (0)
typedef signed   char                   rt_int8_t;      /**<  8bit integer type */
typedef signed   short                  rt_int16_t;     /**< 16bit integer type */
typedef signed   int                    rt_int32_t;     /**< 32bit integer type */
typedef unsigned char                   rt_uint8_t;     /**<  8bit unsigned integer type */
typedef unsigned short                  rt_uint16_t;    /**< 16bit unsigned integer type */
typedef unsigned int                    rt_uint32_t;    /**< 32bit unsigned integer type */


typedef int                             rt_bool_t;      /**< boolean type */
typedef long                            rt_base_t;      /**< Nbit CPU related date type */
typedef unsigned long                   rt_ubase_t;     /**< Nbit unsigned CPU related data type */

typedef rt_base_t                       rt_err_t;       /**< Type for error number */
typedef rt_uint32_t                     rt_time_t;      /**< Type for time stamp */
typedef rt_uint32_t                     rt_tick_t;      /**< Type for tick count */
typedef rt_base_t                       rt_flag_t;      /**< Type for flags */
typedef rt_ubase_t                      rt_size_t;      /**< Type for size number */
typedef rt_ubase_t                      rt_dev_t;       /**< Type for device */
typedef rt_base_t                       rt_off_t;       /**< Type for offset */


/* boolean type definitions */
#define RT_TRUE                         1               /**< boolean true  */
#define RT_FALSE                        0               /**< boolean fails */
/**@}*/
struct controller_pid_param
{
    float kp;
    float ki;
    float kd;
};

struct controller_param
{
    union
    {
        struct controller_pid_param pid;
    } data;
};

typedef struct controller_param *controller_param_t;

struct controller
{
    float           target;
    float           output;
    rt_uint16_t     sample_time;    // unit:ms
    int             enable;

    rt_err_t        (*update)(void *controller, float current_point);
    rt_err_t        (*reset)(void *controller);
    rt_err_t        (*destroy)(void *controller);
    rt_err_t        (*set_param)(void *controller, controller_param_t param);
    rt_err_t        (*get_param)(void *controller, controller_param_t param);
};

typedef struct controller *controller_t;

controller_t        controller_create(rt_size_t size, rt_uint16_t sample_time);
rt_err_t            controller_update(controller_t controller, float current_point);
rt_err_t            controller_destroy(controller_t controller);
rt_err_t            controller_enable(controller_t controller);
rt_err_t            controller_disable(controller_t controller);
rt_err_t            controller_reset(controller_t controller);
rt_err_t            controller_set_target(controller_t controller, rt_int16_t target);
rt_err_t            controller_set_sample_time(controller_t controller, rt_uint16_t sample_time);
rt_err_t            controller_set_param(controller_t controller, controller_param_t param);
rt_err_t            controller_get_param(controller_t controller, controller_param_t param);

#endif // __CONTROLLER_H__
