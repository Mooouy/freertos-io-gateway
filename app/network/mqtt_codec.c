#include "mqtt_codec.h"

#include <stdio.h>
#include <string.h>

/* ----------------------------------------------------------------- 解码辅助 */

static void skip_ws(const char **pp)
{
    const char *p = *pp;
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n')
    {
        p++;
    }
    *pp = p;
}

/*
 * 读取一个 JSON 字符串 "..."(不处理转义, 本项目固定格式内无转义)。
 * 进入时 *pp 指向起始引号。成功: 复制到 out(NUL 结尾)、*pp 越过闭合引号、返回 true。
 * 未闭合或超出 out_cap 返回 false。
 */
static bool parse_json_string(const char **pp, char *out, uint16_t out_cap)
{
    const char *p = *pp;
    if (*p != '"')
    {
        return false;
    }
    p++;
    uint16_t n = 0;
    while (*p != '"')
    {
        if (*p == '\0')
        {
            return false; /* 未闭合 */
        }
        if (n >= (uint16_t)(out_cap - 1U))
        {
            return false; /* 超出目标容量 */
        }
        out[n++] = *p++;
    }
    out[n] = '\0';
    *pp = p + 1; /* 越过闭合引号 */
    return true;
}

/*
 * 读取一个 JSON 整数(可带负号)。要求至少一位数字, 结果须落在 int32 范围, 否则 false。
 * 成功: *out 写入值, *pp 指向数字后的第一个字符, 返回 true。
 */
static bool parse_json_int(const char **pp, int32_t *out)
{
    const char *p = *pp;
    bool neg = false;
    if (*p == '-')
    {
        neg = true;
        p++;
    }
    if (*p < '0' || *p > '9')
    {
        return false; /* 无数字 */
    }
    int64_t acc = 0;
    while (*p >= '0' && *p <= '9')
    {
        acc = acc * 10 + (int64_t)(*p - '0');
        if (acc > 2147483648LL) /* 超出 int32 可表示范围(含负边界余量) */
        {
            return false;
        }
        p++;
    }
    if (neg)
    {
        acc = -acc;
    }
    if (acc > 2147483647LL || acc < -2147483648LL)
    {
        return false;
    }
    *out = (int32_t)acc;
    *pp = p;
    return true;
}

/* ----------------------------------------------------------------- 解码命令 */

MqttCodecStatus_t MqttCodec_DecodeCommand(const char *json, MqttCommand_t *out)
{
    if (out == NULL)
    {
        return MQTT_CODEC_INVALID; /* 无出参可写, 只能返回错误 */
    }
    /* 先复位 out, 保证契约: 任何失败后 out->type == MQTT_CMD_INVALID。 */
    memset(out, 0, sizeof *out); /* type = MQTT_CMD_INVALID */
    if (json == NULL)
    {
        return MQTT_CODEC_INVALID;
    }

    bool has_id = false, has_op = false, has_channel = false, has_value = false,
         has_mode = false;
    int32_t id = 0, channel = 0, value = 0;
    char op[16] = {0};
    char mode[12] = {0};

    const char *p = json;
    skip_ws(&p);
    if (*p != '{')
    {
        return MQTT_CODEC_INVALID;
    }
    p++;
    skip_ws(&p);
    if (*p != '}') /* 非空对象 */
    {
        for (;;)
        {
            skip_ws(&p);
            char key[12];
            if (!parse_json_string(&p, key, (uint16_t)sizeof key))
            {
                return MQTT_CODEC_INVALID;
            }
            skip_ws(&p);
            if (*p != ':')
            {
                return MQTT_CODEC_INVALID;
            }
            p++;
            skip_ws(&p);

            if (strcmp(key, "id") == 0)
            {
                if (has_id)
                {
                    return MQTT_CODEC_DUPLICATE_FIELD;
                }
                if (*p == '"')
                {
                    return MQTT_CODEC_TYPE_ERROR;
                }
                if (!parse_json_int(&p, &id))
                {
                    return MQTT_CODEC_INVALID;
                }
                has_id = true;
            }
            else if (strcmp(key, "channel") == 0)
            {
                if (has_channel)
                {
                    return MQTT_CODEC_DUPLICATE_FIELD;
                }
                if (*p == '"')
                {
                    return MQTT_CODEC_TYPE_ERROR;
                }
                if (!parse_json_int(&p, &channel))
                {
                    return MQTT_CODEC_INVALID;
                }
                has_channel = true;
            }
            else if (strcmp(key, "value") == 0)
            {
                if (has_value)
                {
                    return MQTT_CODEC_DUPLICATE_FIELD;
                }
                if (*p == '"')
                {
                    return MQTT_CODEC_TYPE_ERROR;
                }
                if (!parse_json_int(&p, &value))
                {
                    return MQTT_CODEC_INVALID;
                }
                has_value = true;
            }
            else if (strcmp(key, "op") == 0)
            {
                if (has_op)
                {
                    return MQTT_CODEC_DUPLICATE_FIELD;
                }
                if (*p != '"')
                {
                    return MQTT_CODEC_TYPE_ERROR;
                }
                if (!parse_json_string(&p, op, (uint16_t)sizeof op))
                {
                    return MQTT_CODEC_INVALID;
                }
                has_op = true;
            }
            else if (strcmp(key, "mode") == 0)
            {
                if (has_mode)
                {
                    return MQTT_CODEC_DUPLICATE_FIELD;
                }
                if (*p != '"')
                {
                    return MQTT_CODEC_TYPE_ERROR;
                }
                if (!parse_json_string(&p, mode, (uint16_t)sizeof mode))
                {
                    return MQTT_CODEC_INVALID;
                }
                has_mode = true;
            }
            else
            {
                return MQTT_CODEC_INVALID; /* 未知键 */
            }

            skip_ws(&p);
            if (*p == ',')
            {
                p++;
                continue;
            }
            if (*p == '}')
            {
                break;
            }
            return MQTT_CODEC_INVALID;
        }
    }
    p++; /* 越过 '}' */
    skip_ws(&p);
    if (*p != '\0')
    {
        return MQTT_CODEC_INVALID; /* 尾部多余内容 */
    }

    /* 必需的公共字段 */
    if (!has_id || !has_op)
    {
        return MQTT_CODEC_MISSING_FIELD;
    }
    /* id 必须为正整数: 0 保留作命令发起方"无需回复"哨兵(request_id==0), 负数非法。 */
    if (id <= 0)
    {
        return MQTT_CODEC_INVALID_VALUE;
    }

    MqttCmdType_t type;
    if (strcmp(op, "relay_set") == 0)
    {
        type = MQTT_CMD_RELAY_SET;
    }
    else if (strcmp(op, "ao_set") == 0)
    {
        type = MQTT_CMD_AO_SET;
    }
    else if (strcmp(op, "ai_mode_set") == 0)
    {
        type = MQTT_CMD_AI_MODE_SET;
    }
    else
    {
        return MQTT_CODEC_UNKNOWN_OP;
    }

    /* 固定格式: 每类命令必须且只能带其规定字段(拒绝缺字段与多余已知字段)。 */
    uint8_t ch_index;
    uint8_t relay_value = 0;
    uint16_t ao_raw = 0;
    GatewayAiMode_t ai_mode = GATEWAY_AI_MODE_INVALID;

    if (type == MQTT_CMD_RELAY_SET || type == MQTT_CMD_AO_SET)
    {
        if (!has_channel || !has_value)
        {
            return MQTT_CODEC_MISSING_FIELD;
        }
        if (has_mode)
        {
            return MQTT_CODEC_INVALID; /* 多余字段 */
        }
        if (channel < 1)
        {
            return MQTT_CODEC_INVALID_VALUE;
        }
        if (type == MQTT_CMD_RELAY_SET)
        {
            if (channel > (int32_t)BSM_CHANNEL_COUNT)
            {
                return MQTT_CODEC_INVALID_VALUE;
            }
            if (value != 0 && value != 1)
            {
                return MQTT_CODEC_INVALID_VALUE;
            }
            relay_value = (uint8_t)value;
        }
        else /* AO_SET */
        {
            if (channel > (int32_t)BSM_AO_COUNT)
            {
                return MQTT_CODEC_INVALID_VALUE;
            }
            if (channel == 1) /* DA1: 电压 mV, 0..10000 */
            {
                if (value < 0 || value > (int32_t)BSM_VOLTAGE_MAX_MV ||
                    !Bsm_MillivoltsToRaw((uint16_t)value, &ao_raw))
                {
                    return MQTT_CODEC_INVALID_VALUE;
                }
            }
            else /* channel == 2, DA2: 电流 uA, 0..20000 */
            {
                if (value < 0 || value > (int32_t)BSM_CURRENT_MAX_UA ||
                    !Bsm_MicroampsToRaw((uint16_t)value, &ao_raw))
                {
                    return MQTT_CODEC_INVALID_VALUE;
                }
            }
        }
    }
    else /* AI_MODE_SET */
    {
        if (!has_channel || !has_mode)
        {
            return MQTT_CODEC_MISSING_FIELD;
        }
        if (has_value)
        {
            return MQTT_CODEC_INVALID; /* 多余字段 */
        }
        if (channel < 1 || channel > (int32_t)BSM_AI_COUNT)
        {
            return MQTT_CODEC_INVALID_VALUE;
        }
        if (strcmp(mode, "voltage") == 0)
        {
            ai_mode = GATEWAY_AI_MODE_VOLTAGE;
        }
        else if (strcmp(mode, "current") == 0)
        {
            ai_mode = GATEWAY_AI_MODE_CURRENT;
        }
        else
        {
            return MQTT_CODEC_INVALID_VALUE;
        }
    }

    ch_index = (uint8_t)(channel - 1);

    out->type = type;
    out->id = id;
    out->channel_index = ch_index;
    if (type == MQTT_CMD_RELAY_SET)
    {
        out->payload.relay.value = relay_value;
    }
    else if (type == MQTT_CMD_AO_SET)
    {
        out->payload.ao.value_raw = ao_raw;
    }
    else
    {
        out->payload.ai.mode = ai_mode;
    }
    return MQTT_CODEC_OK;
}

/* ----------------------------------------------------------------- 编码辅助 */

static MqttCodecStatus_t finish_encode(int n, uint16_t cap, uint16_t *out_len)
{
    if (n < 0)
    {
        return MQTT_CODEC_INVALID;
    }
    if ((size_t)n >= (size_t)cap)
    {
        return MQTT_CODEC_BUFFER_TOO_SMALL;
    }
    if (out_len != NULL)
    {
        *out_len = (uint16_t)n;
    }
    return MQTT_CODEC_OK;
}

static const char *ai_mode_str(GatewayAiMode_t mode)
{
    switch (mode)
    {
    case GATEWAY_AI_MODE_VOLTAGE:
        return "voltage";
    case GATEWAY_AI_MODE_CURRENT:
        return "current";
    case GATEWAY_AI_MODE_INVALID:
        return "invalid";
    default:
        return NULL;
    }
}

static const char *link_state_str(GatewayLinkState_t state)
{
    switch (state)
    {
    case GATEWAY_LINK_INIT:
        return "init";
    case GATEWAY_LINK_ONLINE:
        return "online";
    case GATEWAY_LINK_DEGRADED:
        return "degraded";
    case GATEWAY_LINK_OFFLINE:
        return "offline";
    case GATEWAY_LINK_INVALID:
        return "invalid";
    default:
        return NULL;
    }
}

static const char *reply_result_str(MqttReplyResult_t result)
{
    switch (result)
    {
    case MQTT_REPLY_OK:
        return "ok";
    case MQTT_REPLY_TIMEOUT:
        return "timeout";
    case MQTT_REPLY_OFFLINE:
        return "offline";
    case MQTT_REPLY_INVALID_PARAM:
        return "invalid_parameter";
    case MQTT_REPLY_QUEUE_FULL:
        return "queue_full";
    case MQTT_REPLY_ERROR:
        return "error";
    default:
        return NULL;
    }
}

/* ----------------------------------------------------------------- 编码报文 */

MqttCodecStatus_t MqttCodec_EncodeTelemetry(char *out, uint16_t cap,
                                            const MqttTelemetry_t *telemetry,
                                            uint16_t *out_len)
{
    if (out == NULL || cap == 0U || telemetry == NULL)
    {
        return MQTT_CODEC_INVALID;
    }
    const char *link = link_state_str(telemetry->link_state);
    const char *m0 = ai_mode_str(telemetry->ai[0].mode);
    const char *m1 = ai_mode_str(telemetry->ai[1].mode);
    if (link == NULL || m0 == NULL || m1 == NULL)
    {
        return MQTT_CODEC_INVALID;
    }

    int n = snprintf(
        out, cap,
        "{\"seq\":%lu,\"link\":\"%s\",\"di\":%u,\"relay\":%u,"
        "\"ai1\":{\"mode\":\"%s\",\"raw\":%u,\"value\":%ld},"
        "\"ai2\":{\"mode\":\"%s\",\"raw\":%u,\"value\":%ld},"
        "\"ao1\":{\"raw\":%u,\"mv\":%ld},\"ao2\":{\"raw\":%u,\"ua\":%ld}}",
        (unsigned long)telemetry->sequence, link, (unsigned)telemetry->di_bits,
        (unsigned)telemetry->relay_bits, m0, (unsigned)telemetry->ai[0].raw,
        (long)telemetry->ai[0].value, m1, (unsigned)telemetry->ai[1].raw,
        (long)telemetry->ai[1].value, (unsigned)telemetry->da1_raw,
        (long)telemetry->da1_millivolts, (unsigned)telemetry->da2_raw,
        (long)telemetry->da2_microamps);
    return finish_encode(n, cap, out_len);
}

MqttCodecStatus_t MqttCodec_EncodeReply(char *out, uint16_t cap, int32_t id,
                                        MqttReplyResult_t result, int32_t actual,
                                        uint16_t *out_len)
{
    if (out == NULL || cap == 0U)
    {
        return MQTT_CODEC_INVALID;
    }
    const char *rs = reply_result_str(result);
    if (rs == NULL)
    {
        return MQTT_CODEC_INVALID;
    }
    int n = snprintf(out, cap, "{\"id\":%ld,\"result\":\"%s\",\"actual\":%ld}",
                     (long)id, rs, (long)actual);
    return finish_encode(n, cap, out_len);
}

MqttCodecStatus_t MqttCodec_EncodeStatus(char *out, uint16_t cap,
                                         const char *device_id, bool online,
                                         uint16_t *out_len)
{
    if (out == NULL || cap == 0U || device_id == NULL)
    {
        return MQTT_CODEC_INVALID;
    }
    int n = snprintf(out, cap, "{\"device\":\"%s\",\"status\":\"%s\"}", device_id,
                     online ? "online" : "offline");
    return finish_encode(n, cap, out_len);
}
