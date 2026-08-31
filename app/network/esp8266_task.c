/*
 * ESP8266 Wi-Fi/MQTT 协处理器任务实现(v0.5.0)。见 esp8266_task.h 与
 * docs/references/esp8266-mqtt-reference.md。
 *
 * 非阻塞状态机复现参考接入顺序, 每条 AT 命令带 deadline+retry(不用 HAL_Delay);
 * USART2 ISR 只把字节投递到 Stream Buffer, 行装配/分类/JSON 全在任务上下文完成;
 * 全部缓冲/队列/任务栈静态分配(FreeRTOS heap 仅 3072B, 不能动态占用)。
 */

#include "esp8266_task.h"

#if __has_include("network_config.h")
#include "network_config.h" /* 本地真实凭据，不入库 */
#else
#include "network_config.example.h" /* 公开快照使用安全占位配置 */
#endif
#include "esp_at.h"
#include "mqtt_codec.h"
#include "gateway_task.h"
#include "gateway_config.h" /* GATEWAY_ESP_DIAG_LOG_ENABLED */

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "stream_buffer.h"
#include "task.h"

#include "main.h"  /* ESP_RST_Pin / ESP_RST_GPIO_Port */
#include "usart.h" /* huart2 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/*
 * ESP/MQTT 下行诊断日志宏: 由 GATEWAY_ESP_DIAG_LOG_ENABLED(gateway_config.h)控制, 默认关闭。
 * 关闭时展开为空语句, 完全移除对应 printf(参数无副作用, 不改变业务逻辑); 开启可复现现场命令
 * 下行诊断。核心状态日志(task started / online / backoff)直接 printf, 不走此宏, 始终打印。
 */
#if GATEWAY_ESP_DIAG_LOG_ENABLED
#define ESP_DIAG(...) printf(__VA_ARGS__)
#else
#define ESP_DIAG(...) ((void)0)
#endif

/* ----------------------------------------------------------------- 配置常量 */

/* MQTT 主题(编译期由 network_config 的 topic base 拼出, 见规格 §7.1)。 */
#define ESP_TOPIC_COMMAND NET_MQTT_TOPIC_BASE "/command"
#define ESP_TOPIC_TELEMETRY NET_MQTT_TOPIC_BASE "/telemetry"
#define ESP_TOPIC_REPLY NET_MQTT_TOPIC_BASE "/reply"
#define ESP_TOPIC_STATUS NET_MQTT_TOPIC_BASE "/status"

#define ESP_RX_STREAM_SIZE 256U /* ISR->任务 字节流缓冲 */
#define ESP_LINE_BUF_SIZE 192U  /* 单行(含 +MQTTSUBRECV) */
#define ESP_AT_BUF_SIZE 512U    /* AT 命令 TX(容纳转义后的遥测) */
#define ESP_JSON_BUF_SIZE 256U  /* 编码 JSON(转义前) */
#define ESP_TOPIC_BUF_SIZE 64U
#define ESP_PAYLOAD_BUF_SIZE 128U
#define ESP_RX_CHUNK 32U

#define ESP_AT_RETRY_LIMIT 3U  /* 每条连接类 AT 命令重试次数 */
#define ESP_PUB_RETRY_LIMIT 2U /* 每条发布重试次数 */
#define ESP_TX_TIMEOUT_MS 200U /* 有界 TX 超时(短命令, 非盲等) */
#define ESP_LOOP_TICK_MS 50U   /* 无 RX 时的状态机检查节拍 */

#define ESP_RESET_LOW_MS 50U       /* 复位脚拉低时长 */
#define ESP_RESET_BOOT_MS 1500U    /* 释放复位后等 ESP 启动 */
#define ESP_CWMODE_TIMEOUT_MS 2000U
#define ESP_CWJAP_TIMEOUT_MS 15000U /* Wi-Fi 接入较慢 */
#define ESP_USERCFG_TIMEOUT_MS 3000U
#define ESP_CONN_TIMEOUT_MS 10000U /* MQTT 连接较慢 */
#define ESP_SUB_TIMEOUT_MS 5000U
#define ESP_PUB_TIMEOUT_MS 5000U

#define ESP_TELEMETRY_PERIOD_MS 5000U
#define ESP_BACKOFF_MIN_MS 2000U
#define ESP_BACKOFF_MAX_MS 30000U

/* MQTT 遗嘱(LWT)/在线状态策略: will 与在线 retained 状态同 topic/QoS/retain。 */
#define ESP_MQTT_KEEPALIVE_S 60U
#define ESP_STATUS_QOS 1
#define ESP_STATUS_RETAIN 1

#define ESP_TASK_STACK_BYTES 1536U /* 状态机 + snprintf(遥测 14 参) + printf, 留栈余量 */
#define ESP_REPLY_RING_LEN 8U
#define ESP_CMD_CACHE_LEN 8U

/* ----------------------------------------------------------------- 类型/状态 */

typedef enum
{
    ESP_ST_RESET = 0,
    ESP_ST_SET_STATION_MODE,
    ESP_ST_WIFI_CONNECTING,
    ESP_ST_MQTT_CONFIGURING,
    ESP_ST_MQTT_WILL, /* 连接前配置遗嘱(AT+MQTTCONNCFG), best-effort */
    ESP_ST_MQTT_CONNECTING,
    ESP_ST_SUBSCRIBING,
    ESP_ST_ONLINE,
    ESP_ST_BACKOFF,
} EspState_t;

typedef enum
{
    ESP_PUB_NONE = 0,
    ESP_PUB_STATUS,
    ESP_PUB_REPLY,
    ESP_PUB_TELEMETRY,
} EspPubKind_t;

typedef struct
{
    uint32_t id;
    MqttReplyResult_t result;
    int32_t actual;
} EspReply_t;

typedef struct
{
    bool used;
    bool done;
    uint32_t id;
    MqttReplyResult_t result;
    int32_t actual;
} EspCmdCache_t;

/* RX: Stream Buffer(静态) + HAL 单字节接收目标 + 行解析器。 */
static StreamBufferHandle_t rx_stream;
static StaticStreamBuffer_t rx_stream_control;
static uint8_t rx_stream_storage[ESP_RX_STREAM_SIZE + 1U]; /* FreeRTOS 要求 +1 */
static uint8_t esp_rx_byte;
static EspAtParser_t line_parser;
static char line_buf[ESP_LINE_BUF_SIZE];

/* 状态机运行态。 */
static EspState_t state;
static uint32_t state_deadline;
static uint8_t at_retries;
static bool at_sent;
static bool at_ok;
static bool at_error;
static bool link_dropped;
static bool reset_released;
static uint32_t backoff_ms = ESP_BACKOFF_MIN_MS;

/* ONLINE 出站。 */
static uint32_t next_telemetry_ms;
static bool status_pending;
static bool status_online;
static bool telemetry_due;
static bool pub_in_flight;
static uint8_t pub_retries;
static EspPubKind_t pub_kind;

/* 回复环(ESP 任务本地, 无并发)。 */
static EspReply_t reply_ring[ESP_REPLY_RING_LEN];
static uint8_t reply_head;
static uint8_t reply_tail;

/* 命令去重缓存(id -> 结果; 重复命令不重复执行非幂等动作)。 */
static EspCmdCache_t cmd_cache[ESP_CMD_CACHE_LEN];
static uint8_t cmd_cache_next;

/* 工作缓冲(单任务使用, 静态以省栈)。 */
static char at_buf[ESP_AT_BUF_SIZE];
static char json_buf[ESP_JSON_BUF_SIZE];
static char subrecv_topic[ESP_TOPIC_BUF_SIZE];
static uint8_t subrecv_payload[ESP_PAYLOAD_BUF_SIZE];
static GatewayModel_t tele_model;
static MqttTelemetry_t tele;

/* 任务静态分配(不占 FreeRTOS heap)。 */
static StaticTask_t esp_task_tcb;
static uint32_t esp_task_stack[ESP_TASK_STACK_BYTES / 4U];
static osThreadId_t esp_task_handle;

/* ----------------------------------------------------------------- 小工具 */

static bool deadline_reached(uint32_t now, uint32_t dl)
{
    return (int32_t)(now - dl) >= 0;
}

/* 有界 TX(非 HAL_MAX_DELAY 盲等); RX 由 ISR 独立捕获, 不受此短阻塞影响。 */
static void esp_uart_write(const char *s)
{
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)s, (uint16_t)strlen(s), ESP_TX_TIMEOUT_MS);
}

/* ----------------------------------------------------------------- 回复/去重 */

static void esp_reply_push(uint32_t id, MqttReplyResult_t result, int32_t actual)
{
    uint8_t next = (uint8_t)((reply_head + 1U) % ESP_REPLY_RING_LEN);
    if (next == reply_tail)
    {
        return; /* 环满, 丢弃(有界, 不无限堆积) */
    }
    reply_ring[reply_head].id = id;
    reply_ring[reply_head].result = result;
    reply_ring[reply_head].actual = actual;
    reply_head = next;
}

static int esp_cache_find(uint32_t id)
{
    for (uint8_t i = 0; i < ESP_CMD_CACHE_LEN; i++)
    {
        if (cmd_cache[i].used && (cmd_cache[i].id == id))
        {
            return (int)i;
        }
    }
    return -1;
}

static void esp_cache_add(uint32_t id)
{
    cmd_cache[cmd_cache_next].used = true;
    cmd_cache[cmd_cache_next].done = false;
    cmd_cache[cmd_cache_next].id = id;
    cmd_cache_next = (uint8_t)((cmd_cache_next + 1U) % ESP_CMD_CACHE_LEN);
}

static void esp_cache_mark_done(uint32_t id, MqttReplyResult_t result, int32_t actual)
{
    int idx = esp_cache_find(id);
    if (idx < 0)
    {
        return;
    }
    cmd_cache[idx].done = true;
    cmd_cache[idx].result = result;
    cmd_cache[idx].actual = actual;
}

static MqttReplyResult_t esp_map_result(GatewayCmdResult_t r)
{
    switch (r)
    {
    case GATEWAY_RESULT_OK:
        return MQTT_REPLY_OK;
    case GATEWAY_RESULT_REJECTED:
        return MQTT_REPLY_INVALID_PARAM;
    case GATEWAY_RESULT_NOT_READY:
        return MQTT_REPLY_OFFLINE;
    case GATEWAY_RESULT_FAILED:
    default:
        return MQTT_REPLY_ERROR;
    }
}

/* ----------------------------------------------------------------- 命令入站 */

static void esp_handle_command(const char *json)
{
    MqttCommand_t cmd;
    MqttCodecStatus_t dst = MqttCodec_DecodeCommand(json, &cmd);
    /* 诊断: decode 后打印状态与 id/type(失败时 codec 已将 out 清零, 两者均为 0)。 */
    ESP_DIAG("ESP decode st=%d type=%d id=%ld\r\n", (int)dst, (int)cmd.type, (long)cmd.id);
    if (dst != MQTT_CODEC_OK)
    {
        /* 非法命令: codec 不在失败时回传 id, 无法回显真实 id -> 用 0 表示"收到非法命令"。 */
        esp_reply_push(0U, MQTT_REPLY_INVALID_PARAM, 0);
        return;
    }

    uint32_t id = (uint32_t)cmd.id;
    int idx = esp_cache_find(id);
    if (idx >= 0)
    {
        /* 重复命令: 不重复执行非幂等动作; 若此前已完成则重发原结果。 */
        if (cmd_cache[idx].done)
        {
            esp_reply_push(id, cmd_cache[idx].result, cmd_cache[idx].actual);
        }
        return;
    }

    GatewayCommand_t gc = {0};
    gc.request_id = id;
    switch (cmd.type)
    {
    case MQTT_CMD_RELAY_SET:
        gc.type = GATEWAY_CMD_RELAY_SET;
        gc.channel = cmd.channel_index;
        gc.value = cmd.payload.relay.value;
        break;
    case MQTT_CMD_AO_SET:
        gc.type = GATEWAY_CMD_AO_SET;
        gc.channel = cmd.channel_index;
        gc.value = cmd.payload.ao.value_raw;
        break;
    case MQTT_CMD_AI_MODE_SET:
        gc.type = GATEWAY_CMD_AI_MODE_SET;
        gc.channel = cmd.channel_index;
        gc.value = (uint16_t)cmd.payload.ai.mode;
        break;
    default:
        esp_reply_push(id, MQTT_REPLY_INVALID_PARAM, 0);
        return;
    }

    esp_cache_add(id); /* 先登记再提交: 提交与结果之间的重复命令按 in-flight 去重 */
    GatewaySubmitResult_t sr = GatewayTask_SubmitCommand(&gc);
    ESP_DIAG("ESP submit rc=%d id=%lu\r\n", (int)sr, (unsigned long)id); /* 诊断: 提交结果 */
    if (sr == GATEWAY_SUBMIT_OK)
    {
        return; /* reply 由 GatewayTask 结果队列稍后回传后发布 */
    }
    MqttReplyResult_t rr = (sr == GATEWAY_SUBMIT_FULL) ? MQTT_REPLY_QUEUE_FULL : MQTT_REPLY_INVALID_PARAM;
    esp_cache_mark_done(id, rr, 0);
    esp_reply_push(id, rr, 0);
}

static void esp_handle_subrecv(const char *line)
{
    EspAtSubrecvBuffers_t bufs = {subrecv_topic, (uint16_t)sizeof subrecv_topic, subrecv_payload,
                                  (uint16_t)sizeof subrecv_payload};
    EspAtSubrecvResult_t r = {0};

    EspAtParseStatus_t ps = EspAt_ParseSubrecv(line, &bufs, &r);
    if (ps != ESP_AT_PARSE_OK)
    {
        /* 诊断: 解析失败时打印状态与原始行(限长 160B 防刷屏)。+MQTTSUBRECV 行只含
         * 订阅消息(command topic + JSON 命令), 不含 Wi-Fi/MQTT 密码。 */
        ESP_DIAG("ESP subrecv parse=%d\r\n", (int)ps);
        ESP_DIAG("ESP subrecv raw=%.160s\r\n", line);
        return;
    }
    /* 诊断: 解析成功后打印 topic 与 payload 长度(命令 topic/JSON 均不含凭据)。 */
    ESP_DIAG("ESP subrecv topic=%s len=%u\r\n", subrecv_topic, (unsigned)r.payload_len);
#if GATEWAY_ESP_DIAG_LOG_ENABLED
    /* 诊断: topic 与命令 topic 不符时打印; 仅提示, 不跳过以保持原处理路径不变
     * (设备只订阅 command topic, 正常不会触发此分支)。整块随开关移除, 关闭时不做多余 strcmp。 */
    if (strcmp(subrecv_topic, ESP_TOPIC_COMMAND) != 0)
    {
        printf("ESP ignored topic=%s\r\n", subrecv_topic);
    }
#endif
    if (r.payload_len >= (uint16_t)sizeof subrecv_payload)
    {
        return; /* payload 超缓冲, 视为非法(命令报文很短, 正常不会触发) */
    }
    subrecv_payload[r.payload_len] = '\0'; /* codec 需 NUL 结尾 C 串 */
    esp_handle_command((const char *)subrecv_payload);
}

static void esp_handle_line(const char *line)
{
    switch (EspAt_Classify(line))
    {
    case ESP_AT_LINE_OK:
        at_ok = true;
        break;
    case ESP_AT_LINE_ERROR:
        at_error = true;
        break;
    case ESP_AT_LINE_WIFI_DISCONNECT:
    case ESP_AT_LINE_MQTT_DISCONNECT:
        link_dropped = true;
        break;
    case ESP_AT_LINE_MQTT_SUBRECV:
        ESP_DIAG("ESP subrecv line\r\n"); /* 诊断: 收到 +MQTTSUBRECV 行 */
        esp_handle_subrecv(line);
        break;
    default:
        break; /* EMPTY / OTHER(命令回显等) 忽略 */
    }
}

static void esp_feed_byte(uint8_t b)
{
    EspAtStatus_t st = EspAt_Feed(&line_parser, b);
    if (st == ESP_AT_LINE)
    {
        esp_handle_line(line_buf);
    }
    /* ESP_AT_OVERFLOW: 丢弃超长行, 解析器已干净复位 */
}

/* ----------------------------------------------------------------- 出站发布 */

/* 把 src 转义(ESP-AT 数据字段的 " , \ 需前置 \)追加到 dst[*pk..); 溢出返回 false。
 * cap 应由调用方预留后缀余量(如 ",q,r\r\n")。 */
static bool esp_escape_append(char *dst, size_t cap, size_t *pk, const char *src)
{
    size_t k = *pk;
    for (const char *p = src; *p != '\0'; p++)
    {
        if (k + 2U >= cap)
        {
            return false;
        }
        char c = *p;
        if (c == '"' || c == ',' || c == '\\')
        {
            dst[k++] = '\\';
        }
        dst[k++] = c;
    }
    *pk = k;
    return true;
}

/* 把 JSON 转义后套入 AT+MQTTPUB(单命令, 避免 PUBRAW 的 '>' 握手); 溢出返回 false。 */
static bool esp_format_pub(const char *topic, const char *json, int qos, int retain)
{
    int prefix = snprintf(at_buf, sizeof at_buf, "AT+MQTTPUB=0,\"%s\",\"", topic);
    if (prefix < 0 || (size_t)prefix >= sizeof at_buf)
    {
        return false;
    }
    size_t k = (size_t)prefix;
    if (!esp_escape_append(at_buf, sizeof at_buf - 12U, &k, json)) /* 留后缀余量 */
    {
        return false;
    }
    int suffix = snprintf(at_buf + k, sizeof at_buf - k, "\",%d,%d\r\n", qos, retain);
    return (suffix > 0) && ((size_t)suffix < sizeof at_buf - k);
}

/*
 * AT+MQTTCONNCFG=0,<keepalive>,0,"<status topic>","<escaped offline status>",<qos>,<retain>
 * 在 MQTT 连接前配置遗嘱(LWT): broker 于客户端异常掉线时代发 offline 状态到 status 主题。
 * will 的 topic/QoS/retain 与在线 retained 状态一致, payload 由 mqtt_codec 按 offline 编码,
 * 与在线状态同语义(只是 status 字段为 offline)。溢出/编码失败返回 false。
 */
static bool esp_format_conncfg(void)
{
    if (MqttCodec_EncodeStatus(json_buf, (uint16_t)sizeof json_buf, NET_MQTT_DEVICE_ID, false,
                               NULL) != MQTT_CODEC_OK)
    {
        return false;
    }
    int prefix = snprintf(at_buf, sizeof at_buf, "AT+MQTTCONNCFG=0,%u,0,\"%s\",\"",
                          (unsigned)ESP_MQTT_KEEPALIVE_S, ESP_TOPIC_STATUS);
    if (prefix < 0 || (size_t)prefix >= sizeof at_buf)
    {
        return false;
    }
    size_t k = (size_t)prefix;
    if (!esp_escape_append(at_buf, sizeof at_buf - 12U, &k, json_buf))
    {
        return false;
    }
    int suffix = snprintf(at_buf + k, sizeof at_buf - k, "\",%d,%d\r\n", ESP_STATUS_QOS,
                          ESP_STATUS_RETAIN);
    return (suffix > 0) && ((size_t)suffix < sizeof at_buf - k);
}

static void esp_model_to_telemetry(const GatewayModel_t *m, MqttTelemetry_t *t)
{
    memset(t, 0, sizeof *t);
    t->sequence = m->sequence;
    t->link_state = m->link_state;
    t->di_bits = m->snapshot.di_bits;
    t->relay_bits = m->snapshot.relay_bits;
    for (uint8_t i = 0; i < BSM_AI_COUNT; i++)
    {
        t->ai[i].mode = m->ai_mode[i];
        t->ai[i].raw = m->snapshot.ai_raw[i];
        t->ai[i].value = m->ai_value[i];
    }
    t->da1_raw = m->snapshot.ao_raw[0];
    t->da1_millivolts = m->ao_value[0];
    t->da2_raw = m->snapshot.ao_raw[1];
    t->da2_microamps = m->ao_value[1];
}

/* 按优先级(status > reply > telemetry)组装下一条待发布报文到 at_buf; 无则返回 NONE。 */
static EspPubKind_t esp_build_next_pub(void)
{
    if (status_pending)
    {
        if (MqttCodec_EncodeStatus(json_buf, (uint16_t)sizeof json_buf, NET_MQTT_DEVICE_ID,
                                   status_online, NULL) != MQTT_CODEC_OK ||
            !esp_format_pub(ESP_TOPIC_STATUS, json_buf, ESP_STATUS_QOS,
                            ESP_STATUS_RETAIN)) /* retained online, 与 will 一致 */
        {
            status_pending = false; /* 编码失败: 放弃这条, 避免卡死 */
            return ESP_PUB_NONE;
        }
        return ESP_PUB_STATUS;
    }
    if (reply_head != reply_tail)
    {
        const EspReply_t *rp = &reply_ring[reply_tail];
        if (MqttCodec_EncodeReply(json_buf, (uint16_t)sizeof json_buf, (int32_t)rp->id, rp->result,
                                  rp->actual, NULL) != MQTT_CODEC_OK ||
            !esp_format_pub(ESP_TOPIC_REPLY, json_buf, 1, 0))
        {
            reply_tail = (uint8_t)((reply_tail + 1U) % ESP_REPLY_RING_LEN); /* 丢弃坏条目 */
            return ESP_PUB_NONE;
        }
        return ESP_PUB_REPLY;
    }
    if (telemetry_due)
    {
        GatewayTask_CopyModel(&tele_model);
        esp_model_to_telemetry(&tele_model, &tele);
        if (MqttCodec_EncodeTelemetry(json_buf, (uint16_t)sizeof json_buf, &tele, NULL) !=
                MQTT_CODEC_OK ||
            !esp_format_pub(ESP_TOPIC_TELEMETRY, json_buf, 0, 0))
        {
            telemetry_due = false;
            return ESP_PUB_NONE;
        }
        return ESP_PUB_TELEMETRY;
    }
    return ESP_PUB_NONE;
}

static void esp_confirm_pub(void)
{
    switch (pub_kind)
    {
    case ESP_PUB_STATUS:
        status_pending = false;
        break;
    case ESP_PUB_REPLY:
        reply_tail = (uint8_t)((reply_tail + 1U) % ESP_REPLY_RING_LEN);
        break;
    case ESP_PUB_TELEMETRY:
        telemetry_due = false;
        break;
    default:
        break;
    }
    pub_in_flight = false;
    pub_kind = ESP_PUB_NONE;
}

static void esp_online_publish(uint32_t now)
{
    if (pub_in_flight)
    {
        if (at_ok)
        {
            esp_confirm_pub();
        }
        else if (at_error || deadline_reached(now, state_deadline))
        {
            if (pub_retries > 0U)
            {
                pub_retries--;
                esp_uart_write(at_buf);
                at_ok = false;
                at_error = false;
                state_deadline = now + ESP_PUB_TIMEOUT_MS;
            }
            else
            {
                esp_confirm_pub(); /* 重试耗尽: 丢弃该条, 保持在线(断线由 URC 触发重连) */
            }
        }
        return;
    }

    pub_kind = esp_build_next_pub();
    if (pub_kind == ESP_PUB_NONE)
    {
        return;
    }
    esp_uart_write(at_buf);
    pub_in_flight = true;
    pub_retries = ESP_PUB_RETRY_LIMIT;
    at_ok = false;
    at_error = false;
    state_deadline = now + ESP_PUB_TIMEOUT_MS;
}

static void esp_process_results(void)
{
    GatewayCommandResult_t res;
    while (GatewayTask_ReceiveResult(&res, 0))
    {
        MqttReplyResult_t rr = esp_map_result(res.result);
        esp_cache_mark_done(res.request_id, rr, res.actual);
        esp_reply_push(res.request_id, rr, res.actual);
    }
}

/* ----------------------------------------------------------------- 状态机 */

static void esp_enter(EspState_t s, uint32_t now)
{
    state = s;
    at_sent = false;
    at_ok = false;
    at_error = false;
    at_retries = ESP_AT_RETRY_LIMIT;

    if (s == ESP_ST_ONLINE)
    {
        link_dropped = false;
        pub_in_flight = false;
        pub_kind = ESP_PUB_NONE;
        status_pending = true; /* 上线发布 retained online 状态 */
        status_online = true;
        telemetry_due = true; /* (重)连后尽快重发当前快照 */
        next_telemetry_ms = now + ESP_TELEMETRY_PERIOD_MS;
        backoff_ms = ESP_BACKOFF_MIN_MS; /* 成功后复位退避 */
        printf("ESP online\r\n");
    }
    else if (s == ESP_ST_BACKOFF)
    {
        state_deadline = now + backoff_ms;
        printf("ESP backoff %lums\r\n", (unsigned long)backoff_ms);
        backoff_ms = (backoff_ms < ESP_BACKOFF_MAX_MS) ? (backoff_ms * 2U) : ESP_BACKOFF_MAX_MS;
    }
}

/* 连接类状态通用: 未发则发, 收 OK 前进, ERROR/超时重试, 重试耗尽退避。 */
static void esp_at_step(EspState_t next_on_ok, const char *cmd, uint32_t timeout, uint32_t now)
{
    if (!at_sent)
    {
        esp_uart_write(cmd);
        at_sent = true;
        at_ok = false;
        at_error = false;
        state_deadline = now + timeout;
    }
    else if (at_ok)
    {
        esp_enter(next_on_ok, now);
    }
    else if (at_error || deadline_reached(now, state_deadline))
    {
        if (at_retries > 0U)
        {
            at_retries--;
            at_sent = false; /* 下一拍重发同一命令 */
        }
        else
        {
            esp_enter(ESP_ST_BACKOFF, now);
        }
    }
}

static void esp_step(uint32_t now)
{
    switch (state)
    {
    case ESP_ST_RESET:
        if (!at_sent)
        {
            HAL_GPIO_WritePin(ESP_RST_GPIO_Port, ESP_RST_Pin, GPIO_PIN_RESET); /* 拉低复位 */
            /*
             * 任务态排空残留字节 + 复位行解析器。不裸调 xStreamBufferReset: 它与 USART2 ISR 的
             * xStreamBufferSendFromISR 并发不安全; 而单生产者(ISR)-单消费者(本任务)下,
             * xStreamBufferReceive 与 SendFromISR 并发是安全的。复位期间 ESP 被拉低不发送,
             * 残留字节读空即可; 个别在途字节留待复位后作为启动噪声(归 OTHER)被忽略。
             */
            {
                uint8_t drop[ESP_RX_CHUNK];
                while (xStreamBufferReceive(rx_stream, drop, sizeof drop, 0) > 0U)
                {
                }
            }
            EspAt_Init(&line_parser, line_buf, (uint16_t)sizeof line_buf);
            reset_released = false;
            state_deadline = now + ESP_RESET_LOW_MS;
            at_sent = true;
        }
        else if (!reset_released)
        {
            if (deadline_reached(now, state_deadline))
            {
                HAL_GPIO_WritePin(ESP_RST_GPIO_Port, ESP_RST_Pin, GPIO_PIN_SET); /* 释放复位 */
                reset_released = true;
                state_deadline = now + ESP_RESET_BOOT_MS;
            }
        }
        else if (deadline_reached(now, state_deadline))
        {
            esp_enter(ESP_ST_SET_STATION_MODE, now);
        }
        break;

    case ESP_ST_SET_STATION_MODE:
        esp_at_step(ESP_ST_WIFI_CONNECTING, "AT+CWMODE=1\r\n", ESP_CWMODE_TIMEOUT_MS, now);
        break;

    case ESP_ST_WIFI_CONNECTING:
        if (!at_sent)
        {
            (void)snprintf(at_buf, sizeof at_buf, "AT+CWJAP=\"%s\",\"%s\"\r\n", NET_WIFI_SSID,
                           NET_WIFI_PASSWORD);
        }
        esp_at_step(ESP_ST_MQTT_CONFIGURING, at_buf, ESP_CWJAP_TIMEOUT_MS, now);
        break;

    case ESP_ST_MQTT_CONFIGURING:
        if (!at_sent)
        {
            (void)snprintf(at_buf, sizeof at_buf,
                           "AT+MQTTUSERCFG=0,1,\"%s\",\"%s\",\"%s\",0,0,\"\"\r\n",
                           NET_MQTT_CLIENT_ID, NET_MQTT_USERNAME, NET_MQTT_PASSWORD);
        }
        esp_at_step(ESP_ST_MQTT_WILL, at_buf, ESP_USERCFG_TIMEOUT_MS, now);
        break;

    case ESP_ST_MQTT_WILL:
        /* 配置遗嘱(best-effort): 编码/发送失败或 ERROR/超时都继续连接, 兼容不支持
         * MQTTCONNCFG 的固件(此时无 broker 端 LWT, 但连接与在线状态仍正常)。 */
        if (!at_sent)
        {
            if (!esp_format_conncfg())
            {
                esp_enter(ESP_ST_MQTT_CONNECTING, now);
                break;
            }
            esp_uart_write(at_buf);
            at_sent = true;
            at_ok = false;
            at_error = false;
            state_deadline = now + ESP_USERCFG_TIMEOUT_MS;
        }
        else if (at_ok || at_error || deadline_reached(now, state_deadline))
        {
            esp_enter(ESP_ST_MQTT_CONNECTING, now);
        }
        break;

    case ESP_ST_MQTT_CONNECTING:
        if (!at_sent)
        {
            (void)snprintf(at_buf, sizeof at_buf, "AT+MQTTCONN=0,\"%s\",%u,1\r\n",
                           NET_MQTT_BROKER_HOST, (unsigned)NET_MQTT_BROKER_PORT);
        }
        esp_at_step(ESP_ST_SUBSCRIBING, at_buf, ESP_CONN_TIMEOUT_MS, now);
        break;

    case ESP_ST_SUBSCRIBING:
        if (!at_sent)
        {
            (void)snprintf(at_buf, sizeof at_buf, "AT+MQTTSUB=0,\"%s\",1\r\n", ESP_TOPIC_COMMAND);
        }
        esp_at_step(ESP_ST_ONLINE, at_buf, ESP_SUB_TIMEOUT_MS, now);
        break;

    case ESP_ST_ONLINE:
        if (link_dropped)
        {
            esp_enter(ESP_ST_BACKOFF, now);
            break;
        }
        esp_process_results();
        if (deadline_reached(now, next_telemetry_ms))
        {
            telemetry_due = true;
            next_telemetry_ms = now + ESP_TELEMETRY_PERIOD_MS;
        }
        esp_online_publish(now);
        break;

    case ESP_ST_BACKOFF:
    default:
        if (deadline_reached(now, state_deadline))
        {
            esp_enter(ESP_ST_RESET, now); /* 退避后硬件复位重来(规格 §8.2) */
        }
        break;
    }
}

/* ----------------------------------------------------------------- 任务/初始化 */

static void StartEsp8266Task(void *argument)
{
    (void)argument;
    printf("ESP8266 task started\r\n");
    esp_enter(ESP_ST_RESET, osKernelGetTickCount());

    for (;;)
    {
        uint8_t chunk[ESP_RX_CHUNK];
        /* 阻塞等待 RX 至多一个节拍, 保持事件驱动且能按时检查 deadline。 */
        size_t n = xStreamBufferReceive(rx_stream, chunk, sizeof chunk, pdMS_TO_TICKS(ESP_LOOP_TICK_MS));
        while (n > 0U)
        {
            for (size_t i = 0; i < n; i++)
            {
                esp_feed_byte(chunk[i]);
            }
            n = xStreamBufferReceive(rx_stream, chunk, sizeof chunk, 0); /* 排空剩余 */
        }
        esp_step(osKernelGetTickCount());
    }
}

void Esp8266_Init(void)
{
    rx_stream = xStreamBufferCreateStatic(ESP_RX_STREAM_SIZE, 1U, rx_stream_storage,
                                          &rx_stream_control);
    EspAt_Init(&line_parser, line_buf, (uint16_t)sizeof line_buf);
    state = ESP_ST_RESET;
    at_sent = false;
    (void)HAL_UART_Receive_IT(&huart2, &esp_rx_byte, 1U); /* 武装单字节接收 */
}

void Esp8266_StartTask(void)
{
    const osThreadAttr_t attr = {
        .name = "esp8266Task",
        .cb_mem = &esp_task_tcb,
        .cb_size = sizeof(esp_task_tcb),
        .stack_mem = esp_task_stack,
        .stack_size = sizeof(esp_task_stack),
        .priority = (osPriority_t)osPriorityBelowNormal,
    };
    esp_task_handle = osThreadNew(StartEsp8266Task, NULL, &attr);
}

#ifdef USE_HAL_DRIVER
void Esp8266_OnUartRxCplt(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART2)
    {
        return;
    }
    BaseType_t woken = pdFALSE;
    (void)xStreamBufferSendFromISR(rx_stream, &esp_rx_byte, 1U, &woken);
    (void)HAL_UART_Receive_IT(&huart2, &esp_rx_byte, 1U); /* 重新武装 */
    portYIELD_FROM_ISR(woken);
}
#endif /* USE_HAL_DRIVER */
