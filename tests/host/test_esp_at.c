#include "esp_at.h"
#include "test_assert.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* 本文件局部断言(不改共享 test_assert.h): 字符串相等与定长字节相等。 */
static int test_eq_str(const char *expected, const char *actual, const char *expr,
                       const char *file, int line)
{
    if (strcmp(expected, actual) != 0)
    {
        printf("FAIL %s:%d: %s expected \"%s\", got \"%s\"\n", file, line, expr,
               expected, actual);
        return 1;
    }
    return 0;
}
#define TEST_STREQ(expected, actual) \
    test_eq_str((expected), (actual), #actual, __FILE__, __LINE__)

static int test_eq_mem(const void *expected, const void *actual, size_t n,
                       const char *expr, const char *file, int line)
{
    if (memcmp(expected, actual, n) != 0)
    {
        printf("FAIL %s:%d: %s: %u-byte content mismatch\n", file, line, expr,
               (unsigned)n);
        return 1;
    }
    return 0;
}
#define TEST_EQ_MEM(expected, actual, n) \
    test_eq_mem((expected), (actual), (n), #actual, __FILE__, __LINE__)

/* 喂入一段字面字节(不含结尾 NUL), 返回最后一次 Feed 的状态。 */
static EspAtStatus_t feed_str(EspAtParser_t *p, const char *s)
{
    EspAtStatus_t st = ESP_AT_NEED_MORE;
    for (const char *c = s; *c != '\0'; c++)
    {
        st = EspAt_Feed(p, (uint8_t)*c);
    }
    return st;
}

/* 分片 "OK\r\n": 逐字节 NEED_MORE, 换行时产出完整行, 分类为 OK。 */
static int test_feed_fragmented_ok(void)
{
    char buf[32];
    EspAtParser_t p;
    int fails = 0;
    EspAt_Init(&p, buf, (uint16_t)sizeof buf);

    fails += TEST_EQ_U16(ESP_AT_NEED_MORE, EspAt_Feed(&p, (uint8_t)'O'));
    fails += TEST_EQ_U16(ESP_AT_NEED_MORE, EspAt_Feed(&p, (uint8_t)'K'));
    fails += TEST_EQ_U16(ESP_AT_NEED_MORE, EspAt_Feed(&p, (uint8_t)'\r'));
    fails += TEST_EQ_U16(ESP_AT_LINE, EspAt_Feed(&p, (uint8_t)'\n'));
    fails += TEST_STREQ("OK", buf);
    fails += TEST_EQ_U16(ESP_AT_LINE_OK, EspAt_Classify(buf));
    return fails;
}

/* "ERROR\r\n" 完整行, 分类为 ERROR。 */
static int test_feed_error(void)
{
    char buf[32];
    EspAtParser_t p;
    int fails = 0;
    EspAt_Init(&p, buf, (uint16_t)sizeof buf);

    fails += TEST_EQ_U16(ESP_AT_LINE, feed_str(&p, "ERROR\r\n"));
    fails += TEST_STREQ("ERROR", buf);
    fails += TEST_EQ_U16(ESP_AT_LINE_ERROR, EspAt_Classify(buf));
    return fails;
}

/* 仅 "\r\n" 为空行; 裸 "\n" 亦为空行。 */
static int test_feed_empty_line(void)
{
    char buf[32];
    EspAtParser_t p;
    int fails = 0;

    EspAt_Init(&p, buf, (uint16_t)sizeof buf);
    fails += TEST_EQ_U16(ESP_AT_LINE, feed_str(&p, "\r\n"));
    fails += TEST_STREQ("", buf);
    fails += TEST_EQ_U16(ESP_AT_LINE_EMPTY, EspAt_Classify(buf));

    EspAt_Init(&p, buf, (uint16_t)sizeof buf);
    fails += TEST_EQ_U16(ESP_AT_LINE, EspAt_Feed(&p, (uint8_t)'\n'));
    fails += TEST_EQ_U16(ESP_AT_LINE_EMPTY, EspAt_Classify(buf));
    return fails;
}

/* 断连通知(Wi-Fi / MQTT)先归为明确事件。 */
static int test_disconnect_notifications(void)
{
    char buf[64];
    EspAtParser_t p;
    int fails = 0;

    EspAt_Init(&p, buf, (uint16_t)sizeof buf);
    fails += TEST_EQ_U16(ESP_AT_LINE, feed_str(&p, "WIFI DISCONNECT\r\n"));
    fails += TEST_EQ_U16(ESP_AT_LINE_WIFI_DISCONNECT, EspAt_Classify(buf));

    EspAt_Init(&p, buf, (uint16_t)sizeof buf);
    fails += TEST_EQ_U16(ESP_AT_LINE, feed_str(&p, "+MQTTDISCONNECTED:0\r\n"));
    fails += TEST_EQ_U16(ESP_AT_LINE_MQTT_DISCONNECT, EspAt_Classify(buf));
    return fails;
}

/* 命令回显等未知行归为 OTHER, 不误判。 */
static int test_classify_other(void)
{
    return TEST_EQ_U16(ESP_AT_LINE_OTHER, EspAt_Classify("AT+CWMODE=1"));
}

/* 空指针输入返回显式 invalid, 与"可识别但未知"的 OTHER 区分。 */
static int test_classify_invalid(void)
{
    return TEST_EQ_U16(ESP_AT_LINE_INVALID, EspAt_Classify(NULL));
}

/* 参考风格订阅行: 提取 topic 与按长度截取的 payload。 */
static int test_subrecv_basic(void)
{
    char buf[64];
    char topic[16] = {0};
    uint8_t payload[16] = {0};
    EspAtParser_t p;
    EspAtSubrecvBuffers_t bufs = {topic, (uint16_t)sizeof topic, payload,
                                  (uint16_t)sizeof payload};
    EspAtSubrecvResult_t r = {0};
    int fails = 0;

    EspAt_Init(&p, buf, (uint16_t)sizeof buf);
    fails += TEST_EQ_U16(ESP_AT_LINE, feed_str(&p, "+MQTTSUBRECV:0,\"abc\",3,123\r\n"));
    fails += TEST_EQ_U16(ESP_AT_LINE_MQTT_SUBRECV, EspAt_Classify(buf));

    fails += TEST_EQ_U16(ESP_AT_PARSE_OK, EspAt_ParseSubrecv(buf, &bufs, &r));
    fails += TEST_STREQ("abc", topic);
    fails += TEST_EQ_U16(3, r.payload_len);
    fails += TEST_EQ_MEM("123", payload, 3);
    return fails;
}

/* payload 内含逗号: 必须按 length 截取, 不能在逗号处截断。 */
static int test_subrecv_payload_with_commas(void)
{
    char buf[64];
    char topic[16] = {0};
    uint8_t payload[16] = {0};
    EspAtParser_t p;
    EspAtSubrecvBuffers_t bufs = {topic, (uint16_t)sizeof topic, payload,
                                  (uint16_t)sizeof payload};
    EspAtSubrecvResult_t r = {0};
    int fails = 0;

    EspAt_Init(&p, buf, (uint16_t)sizeof buf);
    fails += TEST_EQ_U16(ESP_AT_LINE, feed_str(&p, "+MQTTSUBRECV:0,\"abc\",5,1,2,3\r\n"));
    fails += TEST_EQ_U16(ESP_AT_PARSE_OK, EspAt_ParseSubrecv(buf, &bufs, &r));
    fails += TEST_STREQ("abc", topic);
    fails += TEST_EQ_U16(5, r.payload_len);
    fails += TEST_EQ_MEM("1,2,3", payload, 5);
    return fails;
}

/* payload 缓冲过小: 按容量截断并返回 TRUNCATED, 声明长度仍如实回填。 */
static int test_subrecv_truncated_payload(void)
{
    char buf[64];
    char topic[16] = {0};
    uint8_t payload[2] = {0};
    EspAtParser_t p;
    EspAtSubrecvBuffers_t bufs = {topic, (uint16_t)sizeof topic, payload,
                                  (uint16_t)sizeof payload};
    EspAtSubrecvResult_t r = {0};
    int fails = 0;

    EspAt_Init(&p, buf, (uint16_t)sizeof buf);
    (void)feed_str(&p, "+MQTTSUBRECV:0,\"abc\",3,123\r\n");
    fails += TEST_EQ_U16(ESP_AT_PARSE_TRUNCATED, EspAt_ParseSubrecv(buf, &bufs, &r));
    fails += TEST_EQ_U16(3, r.payload_len); /* 声明长度 */
    fails += TEST_EQ_MEM("12", payload, 2); /* 只拷入容量内 2 字节 */
    return fails;
}

/* topic 缓冲过小: 按容量截断(留 NUL)并返回 TRUNCATED。 */
static int test_subrecv_truncated_topic(void)
{
    char buf[64];
    char topic[2] = {0};
    uint8_t payload[16] = {0};
    EspAtParser_t p;
    EspAtSubrecvBuffers_t bufs = {topic, (uint16_t)sizeof topic, payload,
                                  (uint16_t)sizeof payload};
    EspAtSubrecvResult_t r = {0};
    int fails = 0;

    EspAt_Init(&p, buf, (uint16_t)sizeof buf);
    (void)feed_str(&p, "+MQTTSUBRECV:0,\"abc\",3,123\r\n");
    fails += TEST_EQ_U16(ESP_AT_PARSE_TRUNCATED, EspAt_ParseSubrecv(buf, &bufs, &r));
    fails += TEST_STREQ("a", topic); /* cap-1=1 字节 + NUL */
    return fails;
}

/* 各类非法/畸形订阅行均返回 INVALID, 不越界。 */
static int test_subrecv_invalid(void)
{
    char topic[16] = {0};
    uint8_t payload[16] = {0};
    EspAtSubrecvBuffers_t bufs = {topic, (uint16_t)sizeof topic, payload,
                                  (uint16_t)sizeof payload};
    EspAtSubrecvResult_t r = {0};
    int fails = 0;

    /* 非 +MQTTSUBRECV 行 */
    fails += TEST_EQ_U16(ESP_AT_PARSE_INVALID, EspAt_ParseSubrecv("OK", &bufs, &r));
    /* 缺第二个引号 */
    fails += TEST_EQ_U16(ESP_AT_PARSE_INVALID,
                         EspAt_ParseSubrecv("+MQTTSUBRECV:0,\"abc,3,123", &bufs, &r));
    /* 长度字段无数字 */
    fails += TEST_EQ_U16(ESP_AT_PARSE_INVALID,
                         EspAt_ParseSubrecv("+MQTTSUBRECV:0,\"abc\",,123", &bufs, &r));
    /* 声明长度大于行内实际字节 */
    fails += TEST_EQ_U16(ESP_AT_PARSE_INVALID,
                         EspAt_ParseSubrecv("+MQTTSUBRECV:0,\"abc\",9,12", &bufs, &r));
    /* 缺第二个逗号(无 payload 起点) */
    fails += TEST_EQ_U16(ESP_AT_PARSE_INVALID,
                         EspAt_ParseSubrecv("+MQTTSUBRECV:0,\"abc\",3", &bufs, &r));
    /* 长度字段后混入非数字: 数字后必须紧跟逗号 */
    fails += TEST_EQ_U16(ESP_AT_PARSE_INVALID,
                         EspAt_ParseSubrecv("+MQTTSUBRECV:0,\"abc\",3x,123", &bufs, &r));
    fails += TEST_EQ_U16(ESP_AT_PARSE_INVALID,
                         EspAt_ParseSubrecv("+MQTTSUBRECV:0,\"abc\",3 ,123", &bufs, &r));
    return fails;
}

/* 超长行返回 OVERFLOW, 不覆盖 guard 区, 且之后能干净复位继续解析。 */
static int test_overflow_then_reset(void)
{
    char storage[16];
    EspAtParser_t p;
    EspAtStatus_t st = ESP_AT_NEED_MORE;
    const uint16_t cap = 8;
    int fails = 0;

    memset(storage, 0x5A, sizeof storage); /* guard 填充 */
    EspAt_Init(&p, storage, cap);

    for (int i = 0; i < 20; i++) /* 20 字节内容, 远超 cap-1 */
    {
        st = EspAt_Feed(&p, (uint8_t)'A');
        fails += TEST_EQ_U16(ESP_AT_NEED_MORE, st); /* 溢出期间持续消费 */
    }
    fails += TEST_EQ_U16(ESP_AT_NEED_MORE, EspAt_Feed(&p, (uint8_t)'\r'));
    fails += TEST_EQ_U16(ESP_AT_OVERFLOW, EspAt_Feed(&p, (uint8_t)'\n'));

    /* 缓冲之外的 guard 区[cap..末尾]必须原封不动。 */
    for (size_t i = cap; i < sizeof storage; i++)
    {
        fails += TEST_EQ_U16(0x5A, (uint8_t)storage[i]);
    }

    /* 干净复位: 溢出后紧接一条正常行应正确解析。 */
    fails += TEST_EQ_U16(ESP_AT_LINE, feed_str(&p, "OK\r\n"));
    fails += TEST_STREQ("OK", storage);
    fails += TEST_EQ_U16(ESP_AT_LINE_OK, EspAt_Classify(storage));
    return fails;
}

/* 恰好填满容量(cap-1 内容)不溢出; 多一字节才溢出(边界)。 */
static int test_exact_fit_boundary(void)
{
    char buf[5]; /* 容量 5 -> 内容上限 4 */
    EspAtParser_t p;
    int fails = 0;

    EspAt_Init(&p, buf, (uint16_t)sizeof buf);
    fails += TEST_EQ_U16(ESP_AT_LINE, feed_str(&p, "ABCD\r\n")); /* 4 字节内容: 恰好 */
    fails += TEST_STREQ("ABCD", buf);

    EspAt_Init(&p, buf, (uint16_t)sizeof buf);
    fails += TEST_EQ_U16(ESP_AT_OVERFLOW, feed_str(&p, "ABCDE\r\n")); /* 5 字节: 溢出 */
    return fails;
}

/* 连续两行: 解析器在行间干净复位, 复用同一缓冲。 */
static int test_two_consecutive_lines(void)
{
    char buf[32];
    EspAtParser_t p;
    int fails = 0;

    EspAt_Init(&p, buf, (uint16_t)sizeof buf);
    fails += TEST_EQ_U16(ESP_AT_LINE, feed_str(&p, "OK\r\n"));
    fails += TEST_EQ_U16(ESP_AT_LINE_OK, EspAt_Classify(buf));
    fails += TEST_EQ_U16(ESP_AT_LINE, feed_str(&p, "ERROR\r\n"));
    fails += TEST_EQ_U16(ESP_AT_LINE_ERROR, EspAt_Classify(buf));
    return fails;
}

/* 参数非法: 空 parser / 空缓冲返回 INVALID。 */
static int test_feed_invalid_args(void)
{
    EspAtParser_t p;
    int fails = 0;

    EspAt_Init(&p, NULL, 0); /* 空缓冲, 容量 0 */
    fails += TEST_EQ_U16(ESP_AT_INVALID, EspAt_Feed(&p, (uint8_t)'A'));
    fails += TEST_EQ_U16(ESP_AT_INVALID, EspAt_Feed(NULL, (uint8_t)'A'));
    return fails;
}

int main(void)
{
    int fails = 0;

    fails += test_feed_fragmented_ok();
    fails += test_feed_error();
    fails += test_feed_empty_line();
    fails += test_disconnect_notifications();
    fails += test_classify_other();
    fails += test_classify_invalid();
    fails += test_subrecv_basic();
    fails += test_subrecv_payload_with_commas();
    fails += test_subrecv_truncated_payload();
    fails += test_subrecv_truncated_topic();
    fails += test_subrecv_invalid();
    fails += test_overflow_then_reset();
    fails += test_exact_fit_boundary();
    fails += test_two_consecutive_lines();
    fails += test_feed_invalid_args();

    if (fails == 0)
    {
        printf("All ESP AT parser tests passed\n");
    }
    else
    {
        printf("%d ESP AT assertion(s) failed\n", fails);
    }
    return (fails == 0) ? 0 : 1;
}
