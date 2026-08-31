#ifndef NETWORK_CONFIG_H
#define NETWORK_CONFIG_H

/*
 * ESP8266 Wi-Fi / MQTT 连接配置模板(v0.5.0 起)。
 *
 * 用法: 复制本文件为同目录下的 `network_config.h` 并填入真实凭据。
 *   cp app/config/network_config.example.h app/config/network_config.h
 * `network_config.h` 已加入 .gitignore, 切勿提交真实 SSID/密码/Broker 凭据;
 * 仓库只提交本模板(.example.h), 其中全部为安全占位假值。
 *
 * 约定:
 *   - device_id 用于组装 MQTT 主题 gateway/{device_id}/...(见设计规格 §7.1)。
 *   - 端口为十进制整数; 首版明文 1883, TLS 不属首版(规格 §12)。
 *   - 这些值由 ESP8266Task(Task 5)在 AT+CWJAP / AT+MQTTUSERCFG / AT+MQTTCONN 中使用。
 *   - 本文件只放连接凭据/标识; UART 引脚等硬件配置在 CubeMX(.ioc)与 gateway_config.h。
 */

/* ---- Wi-Fi 接入点(AT+CWJAP)---- */
#define NET_WIFI_SSID "your-wifi-ssid"
#define NET_WIFI_PASSWORD "your-wifi-password"

/* ---- MQTT Broker(AT+MQTTUSERCFG / AT+MQTTCONN)---- */
#define NET_MQTT_BROKER_HOST "broker.example.com"
#define NET_MQTT_BROKER_PORT 1883
#define NET_MQTT_USERNAME "mqtt-user"
#define NET_MQTT_PASSWORD "mqtt-password"

/* ---- 设备标识与主题基名(规格 §7.1: gateway/{device_id}/...)---- */
#define NET_MQTT_DEVICE_ID "gw-demo-0001"
#define NET_MQTT_CLIENT_ID "gw-demo-0001"
#define NET_MQTT_TOPIC_BASE "gateway/gw-demo-0001"

#endif /* NETWORK_CONFIG_H */
