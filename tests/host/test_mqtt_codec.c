#include "mqtt_codec.h"
#include "test_assert.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* 本文件局部字符串相等断言(不改共享 test_assert.h)。 */
static int test_eq_str(const char *expected, const char *actual, const char *expr,
                       const char *file, int line)
{
    if (strcmp(expected, actual) != 0)
    {
        printf("FAIL %s:%d: %s\n  expected \"%s\"\n  got      \"%s\"\n", file, line,
               expr, expected, actual);
        return 1;
    }
    return 0;
}
#define TEST_STREQ(expected, actual) \
    test_eq_str((expected), (actual), #actual, __FILE__, __LINE__)

/* 便捷: 解码并返回状态。 */
static MqttCodecStatus_t decode(const char *json, MqttCommand_t *cmd)
{
    return MqttCodec_DecodeCommand(json, cmd);
}

/* ---- 解码: 三种有效命令 ---- */

static int test_decode_relay_set(void)
{
    MqttCommand_t c;
    int fails = 0;
    fails += TEST_EQ_U16(MQTT_CODEC_OK,
                         decode("{\"id\":1001,\"op\":\"relay_set\",\"channel\":1,\"value\":1}", &c));
    fails += TEST_EQ_U16(MQTT_CMD_RELAY_SET, c.type);
    fails += TEST_EQ_U32(1001, (uint32_t)c.id);
    fails += TEST_EQ_U16(0, c.channel_index); /* 1-based 1 -> 0-based 0 */
    fails += TEST_EQ_U16(1, c.payload.relay.value);
    return fails;
}

static int test_decode_ao_set(void)
{
    MqttCommand_t c;
    int fails = 0;
    /* DA1: 6500 mV -> raw 6500*4000/10000 = 2600 */
    fails += TEST_EQ_U16(MQTT_CODEC_OK,
                         decode("{\"id\":1002,\"op\":\"ao_set\",\"channel\":1,\"value\":6500}", &c));
    fails += TEST_EQ_U16(MQTT_CMD_AO_SET, c.type);
    fails += TEST_EQ_U32(1002, (uint32_t)c.id);
    fails += TEST_EQ_U16(0, c.channel_index);
    fails += TEST_EQ_U16(2600, c.payload.ao.value_raw);
    /* DA2 边界: 20000 uA -> raw 4000 */
    fails += TEST_EQ_U16(MQTT_CODEC_OK,
                         decode("{\"id\":7,\"op\":\"ao_set\",\"channel\":2,\"value\":20000}", &c));
    fails += TEST_EQ_U16(1, c.channel_index);
    fails += TEST_EQ_U16(4000, c.payload.ao.value_raw);
    return fails;
}

static int test_decode_ai_mode_set(void)
{
    MqttCommand_t c;
    int fails = 0;
    fails += TEST_EQ_U16(MQTT_CODEC_OK,
                         decode("{\"id\":1003,\"op\":\"ai_mode_set\",\"channel\":1,\"mode\":\"voltage\"}", &c));
    fails += TEST_EQ_U16(MQTT_CMD_AI_MODE_SET, c.type);
    fails += TEST_EQ_U16(GATEWAY_AI_MODE_VOLTAGE, c.payload.ai.mode);
    fails += TEST_EQ_U16(0, c.channel_index);
    fails += TEST_EQ_U16(MQTT_CODEC_OK,
                         decode("{\"id\":4,\"op\":\"ai_mode_set\",\"channel\":2,\"mode\":\"current\"}", &c));
    fails += TEST_EQ_U16(GATEWAY_AI_MODE_CURRENT, c.payload.ai.mode);
    fails += TEST_EQ_U16(1, c.channel_index);
    return fails;
}

/* ---- 解码: 各类拒绝 ---- */

static int test_decode_missing_field(void)
{
    MqttCommand_t c;
    int fails = 0;
    /* relay_set 缺 value */
    fails += TEST_EQ_U16(MQTT_CODEC_MISSING_FIELD,
                         decode("{\"id\":1,\"op\":\"relay_set\",\"channel\":1}", &c));
    /* 缺 id */
    fails += TEST_EQ_U16(MQTT_CODEC_MISSING_FIELD,
                         decode("{\"op\":\"relay_set\",\"channel\":1,\"value\":1}", &c));
    /* ai_mode_set 缺 mode */
    fails += TEST_EQ_U16(MQTT_CODEC_MISSING_FIELD,
                         decode("{\"id\":1,\"op\":\"ai_mode_set\",\"channel\":1}", &c));
    /* 失败后 type 保持 INVALID */
    fails += TEST_EQ_U16(MQTT_CMD_INVALID, c.type);
    return fails;
}

static int test_decode_unknown_op(void)
{
    MqttCommand_t c;
    return TEST_EQ_U16(MQTT_CODEC_UNKNOWN_OP,
                       decode("{\"id\":1,\"op\":\"foo\",\"channel\":1,\"value\":1}", &c));
}

/* id 必须为正整数: 0/负数判 INVALID_VALUE(保证 request_id==0 可作"无需回复"哨兵)。 */
static int test_decode_id_must_be_positive(void)
{
    MqttCommand_t c;
    int fails = 0;
    fails += TEST_EQ_U16(MQTT_CODEC_INVALID_VALUE,
                         decode("{\"id\":0,\"op\":\"relay_set\",\"channel\":1,\"value\":1}", &c));
    fails += TEST_EQ_U16(MQTT_CODEC_INVALID_VALUE,
                         decode("{\"id\":-5,\"op\":\"relay_set\",\"channel\":1,\"value\":1}", &c));
    fails += TEST_EQ_U16(MQTT_CMD_INVALID, c.type); /* 失败后 type 复位 */
    return fails;
}

static int test_decode_type_error(void)
{
    MqttCommand_t c;
    int fails = 0;
    /* id 应为整数, 给字符串 */
    fails += TEST_EQ_U16(MQTT_CODEC_TYPE_ERROR,
                         decode("{\"id\":\"x\",\"op\":\"relay_set\",\"channel\":1,\"value\":1}", &c));
    /* op 应为字符串, 给整数 */
    fails += TEST_EQ_U16(MQTT_CODEC_TYPE_ERROR,
                         decode("{\"id\":1,\"op\":5,\"channel\":1,\"value\":1}", &c));
    /* value 应为整数, 给字符串 */
    fails += TEST_EQ_U16(MQTT_CODEC_TYPE_ERROR,
                         decode("{\"id\":1,\"op\":\"relay_set\",\"channel\":1,\"value\":\"x\"}", &c));
    return fails;
}

static int test_decode_duplicate_field(void)
{
    MqttCommand_t c;
    int fails = 0;
    /* duplicate id */
    fails += TEST_EQ_U16(MQTT_CODEC_DUPLICATE_FIELD,
                         decode("{\"id\":1,\"id\":2,\"op\":\"relay_set\",\"channel\":1,\"value\":1}", &c));
    /* duplicate channel */
    fails += TEST_EQ_U16(MQTT_CODEC_DUPLICATE_FIELD,
                         decode("{\"id\":1,\"op\":\"relay_set\",\"channel\":1,\"channel\":2,\"value\":1}", &c));
    return fails;
}

static int test_decode_channel_out_of_range(void)
{
    MqttCommand_t c;
    int fails = 0;
    fails += TEST_EQ_U16(MQTT_CODEC_INVALID_VALUE,
                         decode("{\"id\":1,\"op\":\"relay_set\",\"channel\":5,\"value\":1}", &c));
    fails += TEST_EQ_U16(MQTT_CODEC_INVALID_VALUE,
                         decode("{\"id\":1,\"op\":\"relay_set\",\"channel\":0,\"value\":1}", &c));
    fails += TEST_EQ_U16(MQTT_CODEC_INVALID_VALUE,
                         decode("{\"id\":1,\"op\":\"ao_set\",\"channel\":3,\"value\":5000}", &c));
    fails += TEST_EQ_U16(MQTT_CODEC_INVALID_VALUE,
                         decode("{\"id\":1,\"op\":\"ai_mode_set\",\"channel\":3,\"mode\":\"voltage\"}", &c));
    return fails;
}

static int test_decode_relay_value_invalid(void)
{
    MqttCommand_t c;
    int fails = 0;
    fails += TEST_EQ_U16(MQTT_CODEC_INVALID_VALUE,
                         decode("{\"id\":1,\"op\":\"relay_set\",\"channel\":1,\"value\":2}", &c));
    fails += TEST_EQ_U16(MQTT_CODEC_INVALID_VALUE,
                         decode("{\"id\":1,\"op\":\"relay_set\",\"channel\":1,\"value\":-1}", &c));
    return fails;
}

static int test_decode_ao_value_out_of_range(void)
{
    MqttCommand_t c;
    int fails = 0;
    /* DA1 电压 > 10000 mV */
    fails += TEST_EQ_U16(MQTT_CODEC_INVALID_VALUE,
                         decode("{\"id\":1,\"op\":\"ao_set\",\"channel\":1,\"value\":10001}", &c));
    /* DA2 电流 > 20000 uA */
    fails += TEST_EQ_U16(MQTT_CODEC_INVALID_VALUE,
                         decode("{\"id\":1,\"op\":\"ao_set\",\"channel\":2,\"value\":20001}", &c));
    /* 负值 */
    fails += TEST_EQ_U16(MQTT_CODEC_INVALID_VALUE,
                         decode("{\"id\":1,\"op\":\"ao_set\",\"channel\":1,\"value\":-1}", &c));
    return fails;
}

static int test_decode_mode_invalid(void)
{
    MqttCommand_t c;
    return TEST_EQ_U16(MQTT_CODEC_INVALID_VALUE,
                       decode("{\"id\":1,\"op\":\"ai_mode_set\",\"channel\":1,\"mode\":\"foo\"}", &c));
}

static int test_decode_malformed_and_extras(void)
{
    MqttCommand_t c;
    int fails = 0;
    /* 未知键 */
    fails += TEST_EQ_U16(MQTT_CODEC_INVALID,
                         decode("{\"id\":1,\"op\":\"relay_set\",\"channel\":1,\"value\":1,\"x\":9}", &c));
    /* 多余已知字段(relay_set 带 mode) */
    fails += TEST_EQ_U16(MQTT_CODEC_INVALID,
                         decode("{\"id\":1,\"op\":\"relay_set\",\"channel\":1,\"value\":1,\"mode\":\"voltage\"}", &c));
    /* 尾部多余内容 */
    fails += TEST_EQ_U16(MQTT_CODEC_INVALID,
                         decode("{\"id\":1,\"op\":\"relay_set\",\"channel\":1,\"value\":1} x", &c));
    /* 非对象 */
    fails += TEST_EQ_U16(MQTT_CODEC_INVALID, decode("[1,2,3]", &c));
    /* 空指针 */
    fails += TEST_EQ_U16(MQTT_CODEC_INVALID, decode(NULL, &c));
    return fails;
}

/* 契约回归: 对已成功解码的 cmd 再传 NULL json, 必须返回 INVALID 且把 type 复位。 */
static int test_decode_null_resets_command(void)
{
    MqttCommand_t c;
    int fails = 0;
    /* 先成功解码, 使 c.type != MQTT_CMD_INVALID */
    fails += TEST_EQ_U16(MQTT_CODEC_OK,
                         decode("{\"id\":1,\"op\":\"relay_set\",\"channel\":1,\"value\":1}", &c));
    fails += TEST_EQ_U16(MQTT_CMD_RELAY_SET, c.type);
    /* 再传 NULL json: 返回 INVALID 且 type 复位为 INVALID(header 契约) */
    fails += TEST_EQ_U16(MQTT_CODEC_INVALID, MqttCodec_DecodeCommand(NULL, &c));
    fails += TEST_EQ_U16(MQTT_CMD_INVALID, c.type);
    return fails;
}

/* ---- 编码: exact output ---- */

static int test_encode_reply(void)
{
    char buf[64];
    uint16_t len = 0;
    int fails = 0;
    fails += TEST_EQ_U16(MQTT_CODEC_OK,
                         MqttCodec_EncodeReply(buf, (uint16_t)sizeof buf, 1001, MQTT_REPLY_OK, 1, &len));
    fails += TEST_STREQ("{\"id\":1001,\"result\":\"ok\",\"actual\":1}", buf);
    fails += TEST_EQ_U16((uint16_t)strlen(buf), len);
    fails += TEST_EQ_U16(MQTT_CODEC_OK,
                         MqttCodec_EncodeReply(buf, (uint16_t)sizeof buf, 1002, MQTT_REPLY_INVALID_PARAM, 0, &len));
    fails += TEST_STREQ("{\"id\":1002,\"result\":\"invalid_parameter\",\"actual\":0}", buf);
    return fails;
}

static int test_encode_status(void)
{
    char buf[64];
    int fails = 0;
    fails += TEST_EQ_U16(MQTT_CODEC_OK,
                         MqttCodec_EncodeStatus(buf, (uint16_t)sizeof buf, "gw-demo-0001", true, NULL));
    fails += TEST_STREQ("{\"device\":\"gw-demo-0001\",\"status\":\"online\"}", buf);
    fails += TEST_EQ_U16(MQTT_CODEC_OK,
                         MqttCodec_EncodeStatus(buf, (uint16_t)sizeof buf, "gw-demo-0001", false, NULL));
    fails += TEST_STREQ("{\"device\":\"gw-demo-0001\",\"status\":\"offline\"}", buf);
    return fails;
}

static int test_encode_telemetry(void)
{
    char buf[256];
    MqttTelemetry_t t = {0};
    int fails = 0;

    t.sequence = 5;
    t.link_state = GATEWAY_LINK_ONLINE;
    t.di_bits = 3;
    t.relay_bits = 1;
    t.ai[0].mode = GATEWAY_AI_MODE_VOLTAGE;
    t.ai[0].raw = 2000;
    t.ai[0].value = 5000;
    t.ai[1].mode = GATEWAY_AI_MODE_CURRENT;
    t.ai[1].raw = 1600;
    t.ai[1].value = 8000;
    t.da1_raw = 2000;
    t.da1_millivolts = 5000;
    t.da2_raw = 1600;
    t.da2_microamps = 8000;

    fails += TEST_EQ_U16(MQTT_CODEC_OK,
                         MqttCodec_EncodeTelemetry(buf, (uint16_t)sizeof buf, &t, NULL));
    fails += TEST_STREQ(
        "{\"seq\":5,\"link\":\"online\",\"di\":3,\"relay\":1,"
        "\"ai1\":{\"mode\":\"voltage\",\"raw\":2000,\"value\":5000},"
        "\"ai2\":{\"mode\":\"current\",\"raw\":1600,\"value\":8000},"
        "\"ao1\":{\"raw\":2000,\"mv\":5000},\"ao2\":{\"raw\":1600,\"ua\":8000}}",
        buf);
    return fails;
}

static int test_encode_buffer_too_small(void)
{
    char small[16];
    MqttTelemetry_t t = {0};
    int fails = 0;

    t.link_state = GATEWAY_LINK_ONLINE;
    t.ai[0].mode = GATEWAY_AI_MODE_VOLTAGE;
    t.ai[1].mode = GATEWAY_AI_MODE_CURRENT;

    fails += TEST_EQ_U16(MQTT_CODEC_BUFFER_TOO_SMALL,
                         MqttCodec_EncodeReply(small, (uint16_t)sizeof small, 1001, MQTT_REPLY_OK, 1, NULL));
    fails += TEST_EQ_U16(MQTT_CODEC_BUFFER_TOO_SMALL,
                         MqttCodec_EncodeTelemetry(small, (uint16_t)sizeof small, &t, NULL));
    return fails;
}

int main(void)
{
    int fails = 0;

    fails += test_decode_relay_set();
    fails += test_decode_ao_set();
    fails += test_decode_ai_mode_set();
    fails += test_decode_missing_field();
    fails += test_decode_unknown_op();
    fails += test_decode_id_must_be_positive();
    fails += test_decode_type_error();
    fails += test_decode_duplicate_field();
    fails += test_decode_channel_out_of_range();
    fails += test_decode_relay_value_invalid();
    fails += test_decode_ao_value_out_of_range();
    fails += test_decode_mode_invalid();
    fails += test_decode_malformed_and_extras();
    fails += test_decode_null_resets_command();
    fails += test_encode_reply();
    fails += test_encode_status();
    fails += test_encode_telemetry();
    fails += test_encode_buffer_too_small();

    if (fails == 0)
    {
        printf("All MQTT codec tests passed\n");
    }
    else
    {
        printf("%d MQTT codec assertion(s) failed\n", fails);
    }
    return (fails == 0) ? 0 : 1;
}
