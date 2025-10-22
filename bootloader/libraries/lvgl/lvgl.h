#ifndef LV_CONF_H
#define LV_CONF_H

#include <stdint.h>

/*====================
   COLOR SETTINGS
 *====================*/
#define LV_COLOR_DEPTH 16
#define LV_COLOR_16_SWAP 0

/*====================
   DISPLAY SETTINGS
 *====================*/
#define LV_HOR_RES_MAX 240
#define LV_VER_RES_MAX 240

/*====================
   MEMORY SETTINGS
 *====================*/
#define LV_MEM_CUSTOM 0
#define LV_MEM_SIZE (32U * 1024U)

/*====================
   FEATURE CONFIGURATION
 *====================*/
#define LV_USE_LOG 1
#define LV_USE_GPU 0
#define LV_USE_PERF_MONITOR 0

/*====================
   MISC
 *====================*/
#define LV_TICK_CUSTOM 1
#if LV_TICK_CUSTOM
    #define LV_TICK_CUSTOM_INCLUDE "Arduino.h"
    #define LV_TICK_CUSTOM_SYS_TIME_EXPR (millis())
#endif

#endif /*LV_CONF_H*/
