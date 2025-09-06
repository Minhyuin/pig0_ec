#ifndef LOG_H_
#define LOG_H_

#include <stdarg.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

// 定义函数指针类型
typedef int (*putc_func_t)(int);

void log_init(putc_func_t pfputc);
void logging(const char *level_str, const char *format, ...);

#define __LOG(level, format, ...) logging(level, format, ##__VA_ARGS__)

#define LOG(format, ...) __LOG("[INFO] ", format, ##__VA_ARGS__)

#define LOG_WARN(format, ...) __LOG("[WARN] " format, ##__VA_ARGS__)
#define LOG_ERROR(format, ...) __LOG("[ERROR] " format, ##__VA_ARGS__)
#define LOG_FATAL(format, ...) __LOG("[FATAL] " format, ##__VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif // !LOG_H_
