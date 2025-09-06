#include "log.h"

// 全局变量存储函数指针
static putc_func_t fputc_func = NULL;

// 初始化函数
void log_init(putc_func_t pfputc) { fputc_func = pfputc; }

// 重定向 fputc 函数
int fputc(int ch, FILE *f) {
  if (fputc_func != NULL) {
    return fputc_func(ch); // 调用传入的函数指针
  }
  return ch; // 如果未初始化，直接返回字符
}

// 日志函数
void logging(const char *level_str, const char *format, ...) {
  if (level_str == NULL)
    return;
  printf("[%s] ", level_str);
  va_list args;
  va_start(args, format);
  vprintf(format, args); // 使用重定向后的 vprintf
  va_end(args);
}
