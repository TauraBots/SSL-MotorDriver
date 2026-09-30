#pragma once

#include <stdint.h>

typedef enum
{
  COMM_MODE_BENCH = 0,
  COMM_MODE_MATCH = 1
} CommMode;

#ifndef TAURA_COMM_MODE
#define TAURA_COMM_MODE COMM_MODE_BENCH
#endif

typedef enum
{
  MATCH_TRANSPORT_CHANNELS = 0,
  MATCH_TRANSPORT_TEAMFRAME = 1
} MatchTransport;

#ifndef TAURA_MATCH_TRANSPORT
#define TAURA_MATCH_TRANSPORT MATCH_TRANSPORT_CHANNELS
#endif

