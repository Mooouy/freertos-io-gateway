#ifndef ESP_AT_H
#define ESP_AT_H

/*
 * ESP8266 AT 响应行解析器(纯逻辑, 不依赖 HAL/FreeRTOS)。
 *
 * 职责拆成三步, 均为可单测的纯函数/值语义:
 *   1) EspAt_Feed        —— 逐字节把 UART 流装配成"完整一行"(以 \n 结尾, \r 作为帧符不入缓冲);
 *                           行缓冲与容量由调用者提供, 本层不持有任何可写全局缓冲、不做动态分配。
 *   2) EspAt_Classify    —— 对已 NUL 结尾的完整行做分类(OK/ERROR/断连 URC/+MQTTSUBRECV/其它)。
 *   3) EspAt_ParseSubrecv—— 从 +MQTTSUBRECV 行按"长度前缀"提取 topic 与 payload。
 *
 * 关键安全约束(见设计规格 §3.4/§4, 及参考文档 docs/references/esp8266-mqtt-reference.md):
 *   - 绝不对"正被 ISR 写入、未 NUL 结尾"的流直接 strstr; 字符串函数只作用于已完整的一行。
 *   - payload 按行内声明的 length 截取, 不依赖逗号或 CRLF(payload 内可含逗号)。
 *   - 行超出容量时返回 ESP_AT_OVERFLOW 并干净复位, 绝不越界写 guard 区。
 *
 * 局限(本层只做行级框定): payload 内若含 \n/CRLF 或 NUL 需上层做长度感知的分帧,
 * 不属本层(与参考工程一致)。
 */

#include <stdint.h>

/* 逐字节喂入结果。 */
typedef enum
{
    ESP_AT_NEED_MORE = 0, /* 字节已消费, 当前行尚未结束 */
    ESP_AT_LINE,          /* 完整一行就绪: 读 parser->buf(已去 CRLF 且 NUL 结尾) */
    ESP_AT_OVERFLOW,      /* 行超出缓冲容量: 已丢弃该行并复位, 未越界写 */
    ESP_AT_INVALID,       /* 参数非法(空指针 / 容量为 0) */
} EspAtStatus_t;

/* 完整行的分类。 */
typedef enum
{
    ESP_AT_LINE_INVALID = 0,     /* 非法输入(空指针): 显式无效态, 亦为零初始化默认值 */
    ESP_AT_LINE_EMPTY,           /* 空行(仅 CRLF) */
    ESP_AT_LINE_OK,              /* "OK" */
    ESP_AT_LINE_ERROR,           /* "ERROR" */
    ESP_AT_LINE_MQTT_SUBRECV,    /* "+MQTTSUBRECV:..." 订阅消息 */
    ESP_AT_LINE_WIFI_DISCONNECT, /* "WIFI DISCONNECT" URC */
    ESP_AT_LINE_MQTT_DISCONNECT, /* "+MQTTDISCONNECTED:..." URC */
    ESP_AT_LINE_OTHER,           /* 可识别为一行但非以上类别 */
} EspAtLineType_t;

/* +MQTTSUBRECV 解析结果状态。 */
typedef enum
{
    ESP_AT_PARSE_OK = 0,
    ESP_AT_PARSE_INVALID,   /* 非 +MQTTSUBRECV 行 / 结构非法 / 行内实际字节不足声明长度 */
    ESP_AT_PARSE_TRUNCATED, /* topic 或 payload 超出调用者缓冲, 已按容量截断 */
} EspAtParseStatus_t;

/*
 * 逐字节行装配器状态。缓冲区与容量由调用者提供(栈上或静态), 本结构不含隐藏可写全局缓冲。
 * 收到 ESP_AT_LINE 后, 调用者应在下一次 EspAt_Feed 之前读取 buf。
 */
typedef struct
{
    char *buf;         /* 调用者拥有的行缓冲 */
    uint16_t capacity; /* buf 容量(字节, 含末尾 NUL 位) */
    uint16_t len;      /* 当前已累计内容长度(不含 NUL) */
    uint8_t overflow;  /* 粘滞标志: 当前行已溢出, 丢弃剩余字节直到换行 */
} EspAtParser_t;

/* +MQTTSUBRECV 提取所用的调用者缓冲(值语义, 不保留调用者指针)。 */
typedef struct
{
    char *topic_buf;      /* 接收 NUL 结尾 topic */
    uint16_t topic_cap;   /* topic_buf 容量(含 NUL) */
    uint8_t *payload_buf; /* 接收 payload 原始字节(不追加 NUL) */
    uint16_t payload_cap; /* payload_buf 容量 */
} EspAtSubrecvBuffers_t;

typedef struct
{
    uint16_t payload_len; /* 行内声明的 payload 长度(即使截断也回填声明长度) */
} EspAtSubrecvResult_t;

/* 初始化行装配器, 绑定调用者缓冲与容量。 */
void EspAt_Init(EspAtParser_t *parser, char *buf, uint16_t capacity);

/* 喂入一个字节, 返回是否已装配出完整一行 / 溢出 / 参数非法。 */
EspAtStatus_t EspAt_Feed(EspAtParser_t *parser, uint8_t byte);

/* 对已 NUL 结尾的完整行分类。 */
EspAtLineType_t EspAt_Classify(const char *line);

/*
 * 从 +MQTTSUBRECV 行提取 topic(首个引号串)与 payload(长度前缀截取)。
 * 规则: topic = 第一个引号串; length = topic 后第一个逗号之后的十进制整数;
 *       payload = 再下一个逗号之后按 length 截取(不依赖逗号/CRLF)。
 */
EspAtParseStatus_t EspAt_ParseSubrecv(const char *line,
                                      const EspAtSubrecvBuffers_t *bufs,
                                      EspAtSubrecvResult_t *result);

#endif /* ESP_AT_H */
