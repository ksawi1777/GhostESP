#pragma once

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t jammer_start(void);
esp_err_t jammer_stop(void);
bool      jammer_is_running(void);

#ifdef __cplusplus
}
#endif
