#ifndef MQTT_CODEC_H
#define MQTT_CODEC_H

/*
 * MQTT 报文编解码(纯逻辑, 不依赖 HAL/FreeRTOS, 无动态分配, 无隐藏全局可写缓冲)。
 *
 * 只做本项目固定格式(规格 §7), 不是通用 JSON 库, 但对入参做严格边界检查:
 *  - 解码命令 relay_set / ao_set / ai_mode_set: 要求正整数 id(id<=0 判 INVALID_VALUE,
 *    以便调用方用 request_id==0 作"无需回复"哨兵)、校验通道与取值类型/范围,
 *    拒绝 unknown op、缺字段、类型错误、重复字段(尤其 duplicate id)与未知/多余字段。
 *  - 编码 telemetry / reply / status: 一律 snprintf 有界写出, 截断返回
 *    MQTT_CODEC_BUFFER_TOO_SMALL。
 *
 * 单位约定(规格 §7.2): 电压 mV, 电流 uA, 时间 ms。JSON 的 channel 为外部 1-based,
 * 解码结果转成 0-based `channel_index`。复用 gateway_model 的 AI 模式/链路状态枚举,
 * 使解码结果与遥测视图可直接被 GatewayTask 消费, 不重复定义枚举。
 */

#include <stdbool.h>
#include <stdint.h>

#include "gateway_model.h" /* GatewayAiMode_t, GatewayLinkState_t */
#include "bsm0404.h"       /* BSM_AI_COUNT, BSM_AO_COUNT, 量程常量与换算函数 */

/* 编解码统一状态。含多个显式错误值(满足"每个枚举具备 invalid/error 值")。 */
typedef enum
{
    MQTT_CODEC_OK = 0,
    MQTT_CODEC_INVALID,          /* 结构/格式非法: 非固定格式 JSON、未知键、多余字段、数字不可表示 */
    MQTT_CODEC_MISSING_FIELD,    /* 缺必需字段 */
    MQTT_CODEC_DUPLICATE_FIELD,  /* 同一字段重复出现(含 duplicate id) */
    MQTT_CODEC_TYPE_ERROR,       /* 字段 JSON 类型不符(数字/字符串互换) */
    MQTT_CODEC_UNKNOWN_OP,       /* op 值不在支持集合 */
    MQTT_CODEC_INVALID_VALUE,    /* 取值越界/非法: 通道范围、relay≠0/1、AO 越界、mode 非法 */
    MQTT_CODEC_BUFFER_TOO_SMALL, /* 编码输出被截断 */
} MqttCodecStatus_t;

/* 支持的命令类型。零初始化即落 INVALID(显式无效态)。 */
typedef enum
{
    MQTT_CMD_INVALID = 0,
    MQTT_CMD_RELAY_SET,
    MQTT_CMD_AO_SET,
    MQTT_CMD_AI_MODE_SET,
} MqttCmdType_t;

/* 解码后的命令(值语义)。channel_index 为 0-based; 每类命令只用其对应的 payload 成员。 */
typedef struct
{
    MqttCmdType_t type;
    int32_t id;            /* 命令唯一整数 id, 回复时原样回显 */
    uint8_t channel_index; /* 0-based: JSON channel(1-based) 减 1 */
    union
    {
        struct
        {
            uint8_t value; /* relay_set: 仅 0/1 */
        } relay;
        struct
        {
            uint16_t value_raw; /* ao_set: 0..4000, 由 DA1 电压 mV / DA2 电流 uA 校验并换算 */
        } ao;
        struct
        {
            GatewayAiMode_t mode; /* ai_mode_set: VOLTAGE / CURRENT */
        } ai;
    } payload;
} MqttCommand_t;

/* 遥测中单路 AI 视图(规格 §2.2: 同时上报模式/原始值/换算物理量)。 */
typedef struct
{
    GatewayAiMode_t mode; /* voltage / current / invalid */
    uint16_t raw;         /* 0..4000 */
    int32_t value;        /* 物理量: voltage→mV, current→uA; 无效→-1 */
} MqttAiPoint_t;

/* 遥测快照视图(由 GatewayTask 从 gateway_model 填充, 与内部模型解耦)。 */
typedef struct
{
    uint32_t sequence;             /* 成功采样序号 */
    GatewayLinkState_t link_state; /* Modbus 链路状态 */
    uint8_t di_bits;               /* X1-X4 输入位图, 0..15 */
    uint8_t relay_bits;            /* Y1-Y4 继电器位图, 0..15 */
    MqttAiPoint_t ai[BSM_AI_COUNT]; /* ai[0]=AI1, ai[1]=AI2 */
    uint16_t da1_raw;              /* DA1 原始值 0..4000 */
    int32_t da1_millivolts;        /* DA1 固定为电压输出(0-10V), 单位 mV */
    uint16_t da2_raw;              /* DA2 原始值 0..4000 */
    int32_t da2_microamps;         /* DA2 固定为电流输出(0-20mA), 单位 uA */
} MqttTelemetry_t;

/* 命令回复结果(规格 §7.2)。MQTT_REPLY_ERROR 为显式通用错误值。 */
typedef enum
{
    MQTT_REPLY_OK = 0,        /* "ok" */
    MQTT_REPLY_TIMEOUT,       /* "timeout" */
    MQTT_REPLY_OFFLINE,       /* "offline" */
    MQTT_REPLY_INVALID_PARAM, /* "invalid_parameter" */
    MQTT_REPLY_QUEUE_FULL,    /* "queue_full" */
    MQTT_REPLY_ERROR,         /* "error" */
} MqttReplyResult_t;

/*
 * 解码一条命令 JSON(NUL 结尾)到 out。成功返回 MQTT_CODEC_OK 并填充 out;
 * 任何失败返回对应错误码且 out->type 保持 MQTT_CMD_INVALID。
 */
MqttCodecStatus_t MqttCodec_DecodeCommand(const char *json, MqttCommand_t *out);

/* 以下编码函数一律有界写出; 截断返回 MQTT_CODEC_BUFFER_TOO_SMALL; 成功时可选回填 out_len。 */
MqttCodecStatus_t MqttCodec_EncodeTelemetry(char *out, uint16_t cap,
                                            const MqttTelemetry_t *telemetry,
                                            uint16_t *out_len);
MqttCodecStatus_t MqttCodec_EncodeReply(char *out, uint16_t cap,
                                        int32_t id,
                                        MqttReplyResult_t result,
                                        int32_t actual,
                                        uint16_t *out_len);
MqttCodecStatus_t MqttCodec_EncodeStatus(char *out, uint16_t cap,
                                         const char *device_id,
                                         bool online,
                                         uint16_t *out_len);

#endif /* MQTT_CODEC_H */
