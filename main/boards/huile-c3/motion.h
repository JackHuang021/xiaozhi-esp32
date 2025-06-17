/**
 * @file motion.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-06-18
 * 
 * @copyright Copyright (c) 2025
 * 
 */

 #pragma once

 #include "esp_err.h"
 #include "driver/ledc.h"

#ifdef __cplusplus
extern "C" {
#endif

enum motion_state {
    STATE_DANCE,
    STATE_IDLE,
};

#ifdef __cplusplus
}
#endif
