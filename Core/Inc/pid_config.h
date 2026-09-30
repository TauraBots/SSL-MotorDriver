#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  float kp;
  float ki;
  float kd;
  uint32_t crc;
} PIDConfig;

#ifdef __cplusplus
}
#endif
