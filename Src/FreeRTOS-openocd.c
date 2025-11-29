#include "FreeRTOS.h"

#ifdef __GNUC__
#define USED __attribute__((used))
#else
#define USED
#endif

/* 使用 volatile 防止被優化掉 */
const volatile int USED uxTopUsedPriority = configMAX_PRIORITIES - 1;
