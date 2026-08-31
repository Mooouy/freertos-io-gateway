#include "esp_at.h"

#include <string.h>

/* 分类用前缀(含冒号者表示带参数的 URC)。用 sizeof-1 避免手数长度。 */
#define ESP_AT_SUBRECV_PREFIX "+MQTTSUBRECV:"
#define ESP_AT_MQTT_DISC_PREFIX "+MQTTDISCONNECTED"

void EspAt_Init(EspAtParser_t *parser, char *buf, uint16_t capacity)
{
    if (parser == NULL)
    {
        return;
    }
    parser->buf = buf;
    parser->capacity = capacity;
    parser->len = 0;
    parser->overflow = 0;
}

EspAtStatus_t EspAt_Feed(EspAtParser_t *parser, uint8_t byte)
{
    if (parser == NULL || parser->buf == NULL || parser->capacity == 0U)
    {
        return ESP_AT_INVALID;
    }

    if (byte == (uint8_t)'\n')
    {
        /* 溢出行: 丢弃并干净复位, 不产出行内容。 */
        if (parser->overflow)
        {
            parser->len = 0;
            parser->overflow = 0;
            return ESP_AT_OVERFLOW;
        }
        parser->buf[parser->len] = '\0';
        parser->len = 0; /* 复位计数; buf 内容留给调用者在下次 Feed 前读取 */
        return ESP_AT_LINE;
    }

    /* '\r' 属 CRLF 帧终止符, 不计入行内容(本层按文本行框定); 故"可用内容 = 容量-1"。 */
    if (byte == (uint8_t)'\r')
    {
        return ESP_AT_NEED_MORE;
    }

    /* 已溢出: 继续消费剩余字节但不写, 直到换行才复位。 */
    if (parser->overflow)
    {
        return ESP_AT_NEED_MORE;
    }

    /* 末字节为 NUL 预留: 内容最多 capacity-1 字节。 */
    if (parser->len < (uint16_t)(parser->capacity - 1U))
    {
        parser->buf[parser->len] = (char)byte;
        parser->len++;
        return ESP_AT_NEED_MORE;
    }

    /* 容量已满: 标记溢出, 不写 guard 区(索引不超过 capacity-1)。 */
    parser->overflow = 1;
    return ESP_AT_NEED_MORE;
}

EspAtLineType_t EspAt_Classify(const char *line)
{
    if (line == NULL)
    {
        return ESP_AT_LINE_INVALID;
    }
    if (line[0] == '\0')
    {
        return ESP_AT_LINE_EMPTY;
    }
    if (strcmp(line, "OK") == 0)
    {
        return ESP_AT_LINE_OK;
    }
    if (strcmp(line, "ERROR") == 0)
    {
        return ESP_AT_LINE_ERROR;
    }
    if (strncmp(line, ESP_AT_SUBRECV_PREFIX, sizeof(ESP_AT_SUBRECV_PREFIX) - 1U) == 0)
    {
        return ESP_AT_LINE_MQTT_SUBRECV;
    }
    if (strncmp(line, ESP_AT_MQTT_DISC_PREFIX, sizeof(ESP_AT_MQTT_DISC_PREFIX) - 1U) == 0)
    {
        return ESP_AT_LINE_MQTT_DISCONNECT;
    }
    if (strcmp(line, "WIFI DISCONNECT") == 0)
    {
        return ESP_AT_LINE_WIFI_DISCONNECT;
    }
    return ESP_AT_LINE_OTHER;
}

EspAtParseStatus_t EspAt_ParseSubrecv(const char *line,
                                      const EspAtSubrecvBuffers_t *bufs,
                                      EspAtSubrecvResult_t *result)
{
    if (line == NULL || bufs == NULL || result == NULL)
    {
        return ESP_AT_PARSE_INVALID;
    }
    result->payload_len = 0;

    if (strncmp(line, ESP_AT_SUBRECV_PREFIX, sizeof(ESP_AT_SUBRECV_PREFIX) - 1U) != 0)
    {
        return ESP_AT_PARSE_INVALID;
    }

    /* topic = 第一个引号串。 */
    const char *q1 = strchr(line, '"');
    if (q1 == NULL)
    {
        return ESP_AT_PARSE_INVALID;
    }
    const char *q2 = strchr(q1 + 1, '"');
    if (q2 == NULL)
    {
        return ESP_AT_PARSE_INVALID;
    }
    size_t topic_len = (size_t)(q2 - q1 - 1);

    /* length = topic 闭合引号之后第一个逗号后的十进制整数(至少一位)。 */
    const char *comma1 = strchr(q2 + 1, ',');
    if (comma1 == NULL)
    {
        return ESP_AT_PARSE_INVALID;
    }
    const char *digits = comma1 + 1;
    if (*digits < '0' || *digits > '9')
    {
        return ESP_AT_PARSE_INVALID;
    }
    uint32_t declared = 0;
    while (*digits >= '0' && *digits <= '9')
    {
        declared = declared * 10U + (uint32_t)(*digits - '0');
        if (declared > 0xFFFFU) /* 长度不合理, 拒绝以防溢出/异常帧 */
        {
            return ESP_AT_PARSE_INVALID;
        }
        digits++;
    }
    /*
     * 长度字段后必须紧跟逗号: 拒绝 "3x" / "3 " 等混入非数字的长度字段;
     * 且 payload 起点就用数字之后的这个逗号, 不再 strchr 向后搜任意逗号。
     */
    if (*digits != ',')
    {
        return ESP_AT_PARSE_INVALID;
    }
    const char *payload = digits + 1;
    size_t available = strlen(payload);
    if ((size_t)declared > available) /* 行内实际不足声明长度: 视为非法/不完整帧 */
    {
        return ESP_AT_PARSE_INVALID;
    }

    result->payload_len = (uint16_t)declared;

    EspAtParseStatus_t status = ESP_AT_PARSE_OK;

    /* 拷贝 topic(有界, NUL 结尾)。 */
    if (bufs->topic_buf != NULL && bufs->topic_cap > 0U)
    {
        if (topic_len >= (size_t)bufs->topic_cap)
        {
            size_t n = (size_t)bufs->topic_cap - 1U;
            memcpy(bufs->topic_buf, q1 + 1, n);
            bufs->topic_buf[n] = '\0';
            status = ESP_AT_PARSE_TRUNCATED;
        }
        else
        {
            memcpy(bufs->topic_buf, q1 + 1, topic_len);
            bufs->topic_buf[topic_len] = '\0';
        }
    }

    /* 拷贝 payload(按 length 截取, 有界; 不依赖逗号/CRLF)。 */
    if (bufs->payload_buf != NULL && bufs->payload_cap > 0U)
    {
        size_t n = (size_t)declared;
        if (n > (size_t)bufs->payload_cap)
        {
            n = (size_t)bufs->payload_cap;
            status = ESP_AT_PARSE_TRUNCATED;
        }
        memcpy(bufs->payload_buf, payload, n);
    }

    return status;
}
