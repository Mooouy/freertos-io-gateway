#ifndef TEST_ASSERT_H
#define TEST_ASSERT_H

/*
 * 极简零依赖断言：比较两个整数值(16 位 / 32 位)。
 * 不相等时打印 file:line、被测表达式与期望/实际值, 返回 1; 相等返回 0。
 * 各测试函数把返回值累加, 非 0 即表示有断言失败。
 */

#include <stdint.h>
#include <stdio.h>

static inline int test_eq_u16(uint16_t expected, uint16_t actual,
                              const char *expr, const char *file, int line)
{
    if (expected != actual)
    {
        printf("FAIL %s:%d: %s expected 0x%04X, got 0x%04X\n",
               file, line, expr, (unsigned)expected, (unsigned)actual);
        return 1;
    }
    return 0;
}

static inline int test_eq_u32(uint32_t expected, uint32_t actual,
                              const char *expr, const char *file, int line)
{
    if (expected != actual)
    {
        printf("FAIL %s:%d: %s expected 0x%08X, got 0x%08X\n",
               file, line, expr, (unsigned)expected, (unsigned)actual);
        return 1;
    }
    return 0;
}

#define TEST_EQ_U16(expected, actual) \
    test_eq_u16((uint16_t)(expected), (uint16_t)(actual), #actual, __FILE__, __LINE__)

#define TEST_EQ_U32(expected, actual) \
    test_eq_u32((uint32_t)(expected), (uint32_t)(actual), #actual, __FILE__, __LINE__)

#endif /* TEST_ASSERT_H */
