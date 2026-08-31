/*
 * Modbus service: RS485(USART3)总线的唯一拥有者。
 *
 * 设计要点(对应 v0.3.0 Task 2):
 * - 只有本文件的 service 任务(StartModbusRxTask)直接访问 USART3 收发;
 *   调用方通过请求队列按值提交 ModbusRequest_t, service 串行执行总线事务。
 * - 中断只做两件事: USART3 收完 1 字节存入缓冲并续挂接收; TIM2 帧间隔到点
 *   把整帧用 FromISR API 投递到帧队列。中断中不打印、不解析、不阻塞。
 * - 等待响应时, 路由不匹配(从站/功能码不符)的帧被忽略并继续等待, 不会提前
 *   结束当前请求(修复 P2); 只有路由命中的帧才做完整解析并决定结果。
 * - 正式队列由本文件静态创建; 不再使用 CubeMX 生成的占位队列(修复 P3)。
 *
 * 纯 RTU 组帧/校验/路由判断在 modbus_rtu.c, 与 HAL/RTOS 解耦, 可主机测试。
 */

#include "modbus_service.h"

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "tim.h"
#include "usart.h"

#include "modbus_rtu.h"

#include <stddef.h>
#include <stdio.h>

#define MODBUS_RX_BUFFER_SIZE 256U
#define MODBUS_FRAME_MAX MODBUS_RX_BUFFER_SIZE
#define MODBUS_FRAME_QUEUE_LEN 3U
#define MODBUS_REQUEST_QUEUE_LEN 4U
#define MODBUS_FRAME_GAP_MS 5U
#define MODBUS_TX_TIMEOUT_MS 100U
#define MODBUS_TX_BUFFER_SIZE 256U
#define MODBUS_REQUEST_RETRY_COUNT 1U      /* 失败(超时/CRC)后再重试 1 次 */
#define MODBUS_SUBMIT_GUARD_MARGIN_MS 200U /* 调用方等待结果的守卫余量 */

/* ISR -> service 任务的原始整帧(service 内部类型, 不进入对外契约)。 */
typedef struct
{
    uint16_t len;
    uint8_t data[MODBUS_FRAME_MAX];
} ModbusFrame_t;

/* 中断字节收集状态。 */
static uint8_t rx_buffer[MODBUS_RX_BUFFER_SIZE];
static volatile uint16_t rx_index;
static volatile uint32_t last_rx_time_ms;
static uint8_t rx_byte;

/* ISR -> service 任务的整帧队列(静态分配)。 */
static QueueHandle_t frame_queue;
static StaticQueue_t frame_queue_control;
static uint8_t frame_queue_storage[MODBUS_FRAME_QUEUE_LEN * sizeof(ModbusFrame_t)];
static ModbusFrame_t isr_frame; /* ISR 内组帧暂存 */
static ModbusFrame_t rx_frame;  /* service 任务收帧暂存 */

/* 调用方 -> service 任务的请求队列(静态分配)。 */
static QueueHandle_t request_queue;
static StaticQueue_t request_queue_control;
static uint8_t request_queue_storage[MODBUS_REQUEST_QUEUE_LEN * sizeof(ModbusRequest_t)];

/*
 * 调用方串行化锁 + 单结果槽。submit_mutex 保证同一时刻只有一个调用方在途,
 * 因此单个 result_slot 足够安全: service 写满结果槽后通知请求方任务取走。
 */
static SemaphoreHandle_t submit_mutex;
static StaticSemaphore_t submit_mutex_control;
static ModbusResult_t result_slot;

/* USART3 发送缓冲(仅 service 任务使用)。 */
static uint8_t tx_frame[MODBUS_TX_BUFFER_SIZE];

/*
 * 调试串口重定向到 USART1。printf 仅在任务上下文使用, 不在中断中调用。
 */
int __io_putchar(int ch)
{
    uint8_t byte = (uint8_t)ch;
    (void)HAL_UART_Transmit(&huart1, &byte, 1, 10);
    return ch;
}

/* 调试打印一段帧字节, 只在任务上下文调用。 */
static void Modbus_PrintHex(const char *label, const uint8_t *data, uint16_t len)
{
    printf("%s (%u bytes):", label, (unsigned int)len);
    for (uint16_t i = 0; i < len; i++)
    {
        printf(" %02X", data[i]);
    }
    printf("\r\n");
}

/* 操作类型对应的 Modbus 功能码; 未知操作返回 0。 */
static uint8_t Modbus_FuncForOp(ModbusOperation_t op)
{
    switch (op)
    {
        case MODBUS_OP_READ_HOLDING:
            return MODBUS_RTU_FUNC_READ_HOLDING_REGISTERS;
        case MODBUS_OP_WRITE_SINGLE:
            return MODBUS_RTU_FUNC_WRITE_SINGLE_REGISTER;
        case MODBUS_OP_WRITE_MULTIPLE:
            return MODBUS_RTU_FUNC_WRITE_MULTIPLE_REGISTERS;
        default:
            return 0U;
    }
}

/* 按请求构造 RTU 发送帧到 tx_frame, 返回长度; 非法请求返回 0。 */
static uint16_t Modbus_BuildRequestFrame(const ModbusRequest_t *req)
{
    switch (req->operation)
    {
        case MODBUS_OP_READ_HOLDING:
            return Modbus_RTU_BuildRead03Request(req->slave, req->address, req->count,
                                                 tx_frame, (uint16_t)sizeof tx_frame);
        case MODBUS_OP_WRITE_SINGLE:
            return Modbus_RTU_BuildWrite06Request(req->slave, req->address, req->values[0],
                                                  tx_frame, (uint16_t)sizeof tx_frame);
        case MODBUS_OP_WRITE_MULTIPLE:
            return Modbus_RTU_BuildWrite16Request(req->slave, req->address, req->count,
                                                  req->values, tx_frame,
                                                  (uint16_t)sizeof tx_frame);
        default:
            return 0U;
    }
}

static ModbusStatus_t Modbus_MapRtuStatus(ModbusRtuStatus_t st)
{
    switch (st)
    {
        case MODBUS_RTU_OK:
            return MODBUS_STATUS_OK;
        case MODBUS_RTU_CRC_ERROR:
            return MODBUS_STATUS_CRC_ERROR;
        case MODBUS_RTU_EXCEPTION:
            return MODBUS_STATUS_EXCEPTION;
        case MODBUS_RTU_INVALID_RESPONSE:
        case MODBUS_RTU_BUFFER_TOO_SMALL:
        default:
            return MODBUS_STATUS_INVALID_RESPONSE;
    }
}

/* 清空帧队列中本次请求之前残留的旧帧。 */
static void Modbus_DrainFrameQueue(void)
{
    while (xQueueReceive(frame_queue, &rx_frame, 0) == pdPASS)
    {
    }
}

/* rx_frame 已通过路由匹配; 这里做完整协议解析并回填结果。 */
static ModbusStatus_t Modbus_ParseMatched(const ModbusRequest_t *req, ModbusResult_t *result)
{
    ModbusRtuStatus_t st;

    switch (req->operation)
    {
        case MODBUS_OP_READ_HOLDING:
        {
            ModbusRtuRead03Context_t ctx;
            ModbusRtuRead03Result_t rtu = {0};
            uint8_t data[MODBUS_RESULT_MAX_REGISTERS * 2U];

            ctx.slave_addr = req->slave;
            ctx.reg_count = req->count;
            ctx.user_buf = data;
            ctx.user_buf_len = (uint16_t)sizeof data;

            st = Modbus_RTU_ParseRead03Response(rx_frame.data, rx_frame.len, &ctx, &rtu);
            if (st == MODBUS_RTU_OK)
            {
                uint8_t regs = (uint8_t)(rtu.byte_count / 2U);
                result->register_count = regs;
                for (uint8_t i = 0; i < regs; i++)
                {
                    result->registers[i] = (uint16_t)(((uint16_t)data[i * 2U] << 8) |
                                                      data[(i * 2U) + 1U]);
                }
            }
            break;
        }
        case MODBUS_OP_WRITE_SINGLE:
        {
            ModbusRtuWrite06Context_t ctx;
            ctx.slave_addr = req->slave;
            ctx.reg_addr = req->address;
            ctx.value = req->values[0];
            st = Modbus_RTU_ParseWrite06Response(rx_frame.data, rx_frame.len, &ctx);
            break;
        }
        case MODBUS_OP_WRITE_MULTIPLE:
        {
            ModbusRtuWrite16Context_t ctx;
            ctx.slave_addr = req->slave;
            ctx.start_addr = req->address;
            ctx.reg_count = req->count;
            st = Modbus_RTU_ParseWrite16Response(rx_frame.data, rx_frame.len, &ctx);
            break;
        }
        default:
            return MODBUS_STATUS_INVALID_RESPONSE;
    }

    if (st == MODBUS_RTU_EXCEPTION)
    {
        /* 路由+解析已确认是当前功能码且长度为 5 的异常帧, data[2] 即异常码。 */
        result->exception_code = rx_frame.data[2];
    }
    return Modbus_MapRtuStatus(st);
}

/*
 * 在 timeout_ms 内等待一个路由匹配的响应帧。
 * 路由不匹配(从站/功能码不符)的帧被忽略并继续等待, 不会结束当前请求(P2);
 * 只有路由命中的帧才解析并返回最终结果。无任何匹配帧到点则返回 TIMEOUT。
 */
static ModbusStatus_t Modbus_WaitForMatch(const ModbusRequest_t *req, ModbusResult_t *result)
{
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(req->timeout_ms);
    uint8_t func = Modbus_FuncForOp(req->operation);

    for (;;)
    {
        TickType_t now = xTaskGetTickCount();
        if ((int32_t)(deadline - now) <= 0)
        {
            return MODBUS_STATUS_TIMEOUT;
        }

        if (xQueueReceive(frame_queue, &rx_frame, deadline - now) != pdPASS)
        {
            return MODBUS_STATUS_TIMEOUT;
        }

        if (Modbus_RTU_FrameMatches(rx_frame.data, rx_frame.len, req->slave, func) == 0)
        {
            Modbus_PrintHex("Modbus RX ignored", rx_frame.data, rx_frame.len);
            continue; /* P2: 不相关帧不结束请求, 继续等待 */
        }

        Modbus_PrintHex("Modbus RX", rx_frame.data, rx_frame.len);
        return Modbus_ParseMatched(req, result);
    }
}

/* 一次事务尝试: 组帧 -> 清旧帧 -> 发送 -> 等待匹配响应。 */
static ModbusStatus_t Modbus_RunAttempt(const ModbusRequest_t *req, ModbusResult_t *result)
{
    uint16_t tx_len = Modbus_BuildRequestFrame(req);
    if (tx_len == 0U)
    {
        return MODBUS_STATUS_INVALID_RESPONSE;
    }

    Modbus_DrainFrameQueue();

    Modbus_PrintHex("Modbus TX", tx_frame, tx_len);
    if (HAL_UART_Transmit(&huart3, tx_frame, tx_len, MODBUS_TX_TIMEOUT_MS) != HAL_OK)
    {
        return MODBUS_STATUS_TIMEOUT;
    }

    return Modbus_WaitForMatch(req, result);
}

/* 完整事务: 尝试一次, 超时/CRC 错误再重试 MODBUS_REQUEST_RETRY_COUNT 次。 */
static ModbusStatus_t Modbus_RunTransaction(const ModbusRequest_t *req, ModbusResult_t *result)
{
    ModbusStatus_t status = MODBUS_STATUS_TIMEOUT;

    for (uint8_t attempt = 0U; attempt <= MODBUS_REQUEST_RETRY_COUNT; attempt++)
    {
        result->register_count = 0U;
        result->exception_code = 0U;
        status = Modbus_RunAttempt(req, result);
        if ((status != MODBUS_STATUS_TIMEOUT) && (status != MODBUS_STATUS_CRC_ERROR))
        {
            break;
        }
    }
    return status;
}

/* 提交前的请求参数校验; 非法返回 INVALID_RESPONSE。 */
static ModbusStatus_t Modbus_ValidateRequest(const ModbusRequest_t *req)
{
    if (Modbus_FuncForOp(req->operation) == 0U)
    {
        return MODBUS_STATUS_INVALID_RESPONSE;
    }

    if (req->operation == MODBUS_OP_READ_HOLDING)
    {
        if ((req->count == 0U) || (req->count > MODBUS_RESULT_MAX_REGISTERS))
        {
            return MODBUS_STATUS_INVALID_RESPONSE;
        }
    }
    else if (req->operation == MODBUS_OP_WRITE_MULTIPLE)
    {
        if ((req->count == 0U) || (req->count > MODBUS_REQUEST_MAX_VALUES))
        {
            return MODBUS_STATUS_INVALID_RESPONSE;
        }
    }

    return MODBUS_STATUS_OK;
}

/* 启动 USART3 字节接收与 TIM2 帧间隔检测; 仅 service 任务调用。 */
static void Modbus_Service_StartRx(void)
{
    rx_index = 0U;
    last_rx_time_ms = HAL_GetTick();
    (void)HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
    (void)HAL_TIM_Base_Start_IT(&htim2);
}

void Modbus_Service_Init(void)
{
    frame_queue = xQueueCreateStatic(MODBUS_FRAME_QUEUE_LEN, sizeof(ModbusFrame_t),
                                     frame_queue_storage, &frame_queue_control);
    request_queue = xQueueCreateStatic(MODBUS_REQUEST_QUEUE_LEN, sizeof(ModbusRequest_t),
                                       request_queue_storage, &request_queue_control);
    submit_mutex = xSemaphoreCreateMutexStatic(&submit_mutex_control);
}

/*
 * 同步提交一次 Modbus 请求并等待结果。
 * 用 submit_mutex 串行化调用方, 把请求按值送入队列, 阻塞等待 service 通知,
 * 再按值取回结果。全程不向 service 传递调用方指针。
 * 调用方不得同时使用 osThreadFlags(与底层任务通知共用 index 0)。
 */
ModbusStatus_t Modbus_Service_Submit(const ModbusRequest_t *request, ModbusResult_t *result)
{
    ModbusRequest_t local;
    ModbusStatus_t status;
    uint32_t guard_ms;

    if ((request == NULL) || (result == NULL))
    {
        return MODBUS_STATUS_INVALID_RESPONSE;
    }

    if ((request_queue == NULL) || (submit_mutex == NULL))
    {
        return MODBUS_STATUS_OFFLINE;
    }

    status = Modbus_ValidateRequest(request);
    if (status != MODBUS_STATUS_OK)
    {
        return status;
    }

    local = *request; /* 值拷贝, 不保留调用方指针 */
    local.requester = (ModbusRequester_t)xTaskGetCurrentTaskHandle();

    (void)xSemaphoreTake(submit_mutex, portMAX_DELAY);

    (void)ulTaskNotifyTake(pdTRUE, 0); /* 丢弃可能残留的旧通知 */

    if (xQueueSend(request_queue, &local, 0) != pdPASS)
    {
        (void)xSemaphoreGive(submit_mutex);
        return MODBUS_STATUS_QUEUE_FULL;
    }

    guard_ms = (local.timeout_ms * (MODBUS_REQUEST_RETRY_COUNT + 1U)) +
               MODBUS_SUBMIT_GUARD_MARGIN_MS;
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(guard_ms)) == 0U)
    {
        /* service 未在守卫时间内完成(异常情况)。 */
        (void)xSemaphoreGive(submit_mutex);
        return MODBUS_STATUS_TIMEOUT;
    }

    *result = result_slot; /* 值拷贝取回结果 */
    (void)xSemaphoreGive(submit_mutex);
    return result->status;
}

/* USART3 接收完成中断转发目标; 运行在中断上下文。 */
void Modbus_Service_OnUartRxCplt(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART3)
    {
        return;
    }

    last_rx_time_ms = HAL_GetTick();

    if (rx_index < MODBUS_RX_BUFFER_SIZE)
    {
        rx_buffer[rx_index] = rx_byte;
        rx_index++;
    }
    else
    {
        rx_index = 0U;
    }

    (void)HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
}

/* TIM2 帧间隔中断转发目标; 运行在中断上下文。 */
void Modbus_Service_OnTimPeriodElapsed(TIM_HandleTypeDef *htim)
{
    uint16_t len;
    BaseType_t woken = pdFALSE;

    if (htim->Instance != TIM2)
    {
        return;
    }

    if (frame_queue == NULL)
    {
        return;
    }

    if ((HAL_GetTick() - last_rx_time_ms) < MODBUS_FRAME_GAP_MS)
    {
        return;
    }

    if (rx_index == 0U)
    {
        return;
    }

    len = rx_index;
    if (len > MODBUS_FRAME_MAX)
    {
        len = MODBUS_FRAME_MAX;
    }

    isr_frame.len = len;
    for (uint16_t i = 0; i < len; i++)
    {
        isr_frame.data[i] = rx_buffer[i];
    }
    rx_index = 0U;

    (void)xQueueSendFromISR(frame_queue, &isr_frame, &woken);
    portYIELD_FROM_ISR(woken);
}

/*
 * service 任务(CubeMX mbRxTask 入口): RS485 总线唯一拥有者。
 * 启动接收后, 串行处理请求队列: 执行事务 -> 写结果槽 -> 通知请求方任务。
 */
void StartModbusRxTask(void *argument)
{
    ModbusRequest_t req;
    ModbusResult_t result;

    (void)argument;

    if ((frame_queue == NULL) || (request_queue == NULL) || (submit_mutex == NULL))
    {
        printf("Modbus service not initialized\r\n");
        for (;;)
        {
            osDelay(1000);
        }
    }

    Modbus_Service_StartRx();
    printf("Modbus service started\r\n");

    for (;;)
    {
        if (xQueueReceive(request_queue, &req, portMAX_DELAY) == pdPASS)
        {
            /* Modbus_RunTransaction 每次尝试都会先清零 register_count/exception_code。 */
            result.status = Modbus_RunTransaction(&req, &result);

            result_slot = result;
            if (req.requester != NULL)
            {
                (void)xTaskNotifyGive((TaskHandle_t)req.requester);
            }
        }
    }
}
