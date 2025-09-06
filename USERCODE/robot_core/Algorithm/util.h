#ifndef UTIL_H_
#define UTIL_H_

#include "CommonMacro.h"
void IntervalExecute(void (*func)(void), uint32_t time, uint32_t *staticval,
                     uint32_t (*millis_func)(void));

void LoopExecute(void (*func)(void), uint32_t n);

#endif // !UTIL_H_
