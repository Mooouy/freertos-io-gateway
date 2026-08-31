# 原生主机测试：只编译与 HAL/RTOS 解耦的纯协议模块, 用主机编译器构建。
add_executable(test_modbus_rtu
    test_modbus_rtu.c
    ${CMAKE_SOURCE_DIR}/app/modbus/modbus_rtu.c
)
target_include_directories(test_modbus_rtu PRIVATE
    ${CMAKE_SOURCE_DIR}/app/modbus
)

add_test(NAME modbus_rtu COMMAND test_modbus_rtu)

# Modbus service 契约测试: 纯头文件类型契约, 无需额外 .c 源。
add_executable(test_modbus_types
    test_modbus_types.c
)
target_include_directories(test_modbus_types PRIVATE
    ${CMAKE_SOURCE_DIR}/app/modbus
)

add_test(NAME modbus_types COMMAND test_modbus_types)

# BSM-0404 设备驱动测试: 纯逻辑(位图/计数器/模拟量换算), 编译驱动源。
add_executable(test_bsm0404
    test_bsm0404.c
    ${CMAKE_SOURCE_DIR}/app/devices/bsm0404.c
)
target_include_directories(test_bsm0404 PRIVATE
    ${CMAKE_SOURCE_DIR}/app/devices
)

add_test(NAME bsm0404 COMMAND test_bsm0404)

# 网关数据模型测试: 纯逻辑(快照/时序/诊断/状态), 编译模型与 BSM 换算源。
add_executable(test_gateway_model
    test_gateway_model.c
    ${CMAKE_SOURCE_DIR}/app/gateway/gateway_model.c
    ${CMAKE_SOURCE_DIR}/app/devices/bsm0404.c
)
target_include_directories(test_gateway_model PRIVATE
    ${CMAKE_SOURCE_DIR}/app/gateway
    ${CMAKE_SOURCE_DIR}/app/config
    ${CMAKE_SOURCE_DIR}/app/devices
    ${CMAKE_SOURCE_DIR}/app/modbus
)

add_test(NAME gateway_model COMMAND test_gateway_model)

# 网关本地规则测试: 纯逻辑(滞回/互锁/点动), 仅需规则源(用到 BSM 通道数宏)。
add_executable(test_gateway_rules
    test_gateway_rules.c
    ${CMAKE_SOURCE_DIR}/app/gateway/gateway_rules.c
)
target_include_directories(test_gateway_rules PRIVATE
    ${CMAKE_SOURCE_DIR}/app/gateway
    ${CMAKE_SOURCE_DIR}/app/devices
)

add_test(NAME gateway_rules COMMAND test_gateway_rules)

# ESP8266 AT 行解析器测试: 纯逻辑(逐字节行装配 + +MQTTSUBRECV 长度前缀提取), 编译 parser 源。
add_executable(test_esp_at
    test_esp_at.c
    ${CMAKE_SOURCE_DIR}/app/network/esp_at.c
)
target_include_directories(test_esp_at PRIVATE
    ${CMAKE_SOURCE_DIR}/app/network
)

add_test(NAME esp_at COMMAND test_esp_at)

# MQTT 编解码测试: 纯逻辑(固定格式命令解码 + telemetry/reply/status 编码),
# 复用 BSM 换算(链接 bsm0404.c)与 gateway_model/bsm 枚举(仅头)。
add_executable(test_mqtt_codec
    test_mqtt_codec.c
    ${CMAKE_SOURCE_DIR}/app/network/mqtt_codec.c
    ${CMAKE_SOURCE_DIR}/app/devices/bsm0404.c
)
target_include_directories(test_mqtt_codec PRIVATE
    ${CMAKE_SOURCE_DIR}/app/network
    ${CMAKE_SOURCE_DIR}/app/gateway
    ${CMAKE_SOURCE_DIR}/app/devices
    ${CMAKE_SOURCE_DIR}/app/modbus
)

add_test(NAME mqtt_codec COMMAND test_mqtt_codec)
