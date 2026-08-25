#include "firmware_runtime.h"

#include "main.h"

void FirmwareRuntime_WatchdogInit(void)
{
  /* About 1.5 s at the nominal 40 kHz LSI, including oscillator tolerance. */
  IWDG->KR = 0x5555U;
  IWDG->PR = 6U;
  IWDG->RLR = 234U;
  IWDG->KR = 0xAAAAU;
  IWDG->KR = 0xCCCCU;
}

void FirmwareRuntime_WatchdogRefresh(void)
{
  IWDG->KR = 0xAAAAU;
}
