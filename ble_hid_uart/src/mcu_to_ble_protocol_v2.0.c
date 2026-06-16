//增加了私有通信协议 后续在此基础上进行代码开发

#define LOG_TAG_CONST MCU2BLE
#define LOG_TAG "[MCU2BLE]"
#define LOG_ERROR_ENABLE
#define LOG_DEBUG_ENABLE
#define LOG_INFO_ENABLE



#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "mcu_to_ble_protocol_v2.0.h"
#include "kernel.h"
#include "dev_led.h"
//#include "system/debug.h"

#ifdef LOG_INFO_ENABLE
const char log_tag_const_i_MCU2BLE = 1;
#endif
#ifdef LOG_DEBUG_ENABLE
const char log_tag_const_d_MCU2BLE = 1;
#endif
#ifdef LOG_ERROR_ENABLE
const char log_tag_const_e_MCU2BLE = 1;
#endif


#define MCU_MAX_LEN          128          // 蓝牙芯片与主控收发数据最大长度

#define BLE_RESEND_MAX_COUNT  3
#define BLE_RESEND_TIMEOUT_MS 2000

extern int handshake_retry_count; //ble向主控发送握手指令的次数计数
//----------------------MCU发送消息----------------------------------------
static void *g_protocol_event_ctx = NULL;  // 协议事件回调用户上下文

//-----------app to mcu data-------------
typedef void (*ble_uart_send_callback_t)(const uint8_t *data, uint16_t len);

static ble_uart_send_callback_t g_ble_uart_send_cb = NULL;

void app_to_ble_register_send_callback(ble_uart_send_callback_t cb)
{
    g_ble_uart_send_cb = cb;
}
//--------------BLE主动上报--------------------
static ble_protocol_send_func_t g_ble_send_func = NULL;

typedef struct {
    char last_send_buf[64];     // 保存上一次发送的数据
    uint16_t last_send_len;     // 缓存数据长度
    uint32_t last_send_time;    // 上一次发送的时间
    uint8_t resend_count;       // 重发次数
    uint8_t wait_ack;           // 等待ACK
} ble_resend_ctx_t;
static ble_resend_ctx_t ble_resend_ctx = {0};

//-----------时间获取-------------------
static inline uint32_t timer_get_ms(void)
{
    return k_uptime_get_32();
}
//-----------控制器相关状态-----

controler_state_t g_controler_state= {
    .handshake_ok  = false,
    .active_ok     = false,
    .last_cmd_type = -1       // 初始化为-1表示未发送过指令
};

// BLE主动上报接口实现
void ble_to_mcu_report_link_state(int state)
{
    if (g_ble_send_func) //g_ble_send_func = 1 说明存在回调函数
    {
        char buf[32] = {0};
        // 2=主动上报已连接，3=主动上报已断开
        snprintf(buf, sizeof(buf), "BLE|LNK|%s", state ? "DISCON" : "CON");
        g_ble_send_func(buf, strlen(buf)); // 发送数据

        // 保存重发信息
        memcpy(ble_resend_ctx.last_send_buf, buf, strlen(buf));
        ble_resend_ctx.last_send_len = strlen(buf);
        ble_resend_ctx.last_send_time = timer_get_ms();
        ble_resend_ctx.resend_count = 0;
        ble_resend_ctx.wait_ack = 1;
    }
}

//--------------app发来数据，蓝牙分包处理-----------------
#define UART_FRAME_HEAD_0  0xAA
#define UART_FRAME_HEAD_1  0x55
#define UART_RX_CACHE_SIZE 512

extern void TGT_SendMultiData(const uint8_t *data, size_t size);

//---------------------------------------------------------------
void ble_to_mcu_resend_check(void) // 蓝牙主动上报数据重发
{
    if (ble_resend_ctx.wait_ack)
    {
        if (timer_get_ms() - ble_resend_ctx.last_send_time > BLE_RESEND_TIMEOUT_MS)
        {
            if (ble_resend_ctx.resend_count < BLE_RESEND_MAX_COUNT)
            {
                if (g_ble_send_func)
                {
                    g_ble_send_func(ble_resend_ctx.last_send_buf, ble_resend_ctx.last_send_len);
                }
                ble_resend_ctx.last_send_time = timer_get_ms();
                ble_resend_ctx.resend_count++;
                log_info("重发BLE主动上报数据,第%d次\n", ble_resend_ctx.resend_count);
            }
            else
            {
                log_info("BLE主动上报数据重发三次失败,报错！");
                ble_resend_ctx.wait_ack = 0;
                // 可通知上层错误
            }
        }
    }
}

// 注册发送函数
void ble_to_mcu_register_send_func(ble_protocol_send_func_t func)
{
    g_ble_send_func = func;
}

/*
 * fun: Judge_stringisdata
 * param: char *data:入参数据指针
 * return: void
 * description: 判断字符串数据后能否用atoi转换
 */
bool Judge_stringisdata(const char *data)
{
    if((*data)>='0' && (*data)<='9')
    {
        return true;//该数据在0-9之间
    }
    else
    {
        return false;//数据不合法，直接返回
    }
}
//-----------------------新版协议-----------------------------------------
#define BLE_PROTOCOL_CMD_LEN 20
//握手指令
const uint8_t HANDSHAKE_CMD[BLE_PROTOCOL_CMD_LEN] ={
    0x5A, 0xA5 ,0x00, 0x00 ,0x00 ,0x00 ,0x00 ,0x00, 0x00, 0x00 ,
    0x00 ,0x00 ,0x00, 0x00 ,0x3C, 0x00 ,0x00, 0x00, 0x0C, 0x03
};
//控制器故障查询指令
const uint8_t FAULT_QUERY_CMD[BLE_PROTOCOL_CMD_LEN] = {
    0x5A, 0xA5 ,0x00, 0x00 ,0x00 ,0x00 ,0x00 ,0x00, 0x00, 0x00 ,
    0x00 ,0x00 ,0x00, 0x00 ,0x3B, 0x00 ,0x00, 0x00, 0x0C, 0x02
};
// 解锁状态查询指令
const uint8_t UNLOCK_STATUS_CMD[BLE_PROTOCOL_CMD_LEN] = {
    0x5A, 0xA5 ,0x00, 0x00 ,0x00 ,0x00 ,0x00 ,0x00, 0x00, 0x00 ,
    0x00 ,0x00 ,0x00, 0x00 ,0xCC, 0x00 ,0x00, 0x00, 0x5C, 0x0A
};
//解锁指令
const uint8_t UNLOCK_CMD[BLE_PROTOCOL_CMD_LEN] = {
    0x5A, 0xA5 ,0x00, 0x00 ,0x00 ,0x00 ,0x00 ,0x00, 0x00, 0x00 ,
    0x00 ,0x00 ,0x00, 0x00 ,0x33, 0x00 ,0x00, 0x00, 0x97, 0x02
};
//锁定指令
const uint8_t LOCK_CMD[BLE_PROTOCOL_CMD_LEN] = {
    0x5A, 0xA5 ,0x00, 0x00 ,0x00 ,0x00 ,0x00 ,0x00, 0x00, 0x00 ,
    0x00 ,0x00 ,0x00, 0x00 ,0xC3, 0x00 ,0x00, 0x00, 0xE7, 0x09
};



cmd_wait_t cmd_wait[5] = {0}; // 0:握手 1:查询 2:解锁 3:锁定 4:故障查询

/*
 * fun:  BLE_send_protocol_cmd
 * param:  ble_cmd_type_t cmd_type:指令类型
 * return: void
 * description:  发送BLE协议指令
 */
// void reply_app_status(app_cmd_type_t cmd, uint8_t result_code)
// {
//     uint8_t reply[4] = {0xAA, cmd, result_code, 0x55};
//     bt_ven_notify(reply, 4);
// }
void reply_app_status(app_cmd_type_t cmd, uint8_t result_code)
{
    uint8_t reply[4] = {0xAA, cmd, result_code, 0x55};

    // 打印即将发送的数据内容
    printk("reply_app_status: cmd=%d, result_code=%02X, data=", cmd, result_code);
    for (int i = 0; i < 4; i++) {
        printk("%02X ", reply[i]);
    }
    printk("\n");

    // 发送BLE通知，并打印返回值
    int ret = bt_ven_notify(reply, 4);
    printk("bt_ven_notify return: %d\n", ret);
}
/*
 * fun: Handle_mcu_replydata
 * param: char *data:入参数据指针
 * return: void
 * description: 处理主控返回的数据
 */
static void Handle_mcu_replydata(const char *data, uint16_t datalen, ble_cmd_type_t cmd_type)
{
//     printk("Handle_mcu_replydata: datalen=%d, cmd_type=%d\n", datalen, cmd_type);
    // 1. 数据长度判断
    if (datalen > MCU_MAX_LEN)
    {
        return; // 超出最大长度，直接返回
    }

    // 2. 处理主控几种应答
    if (datalen == 2)
    {
        //printk("Handle cmd of mcu reply \n");

        const uint8_t *pdata = (const uint8_t *)data;

        // 正常收到主控回复，清除等待标志
        if (cmd_type >= 0 && cmd_type < BLE_CMD_MAX && cmd_wait[cmd_type].waiting)
        {
            uint32_t now = k_uptime_get_32();
            if (now - cmd_wait[cmd_type].send_time_ms <= 198)
            {
                cmd_wait[cmd_type].waiting = false;
                // 处理正常回复逻辑
                printk("MCU reply received for cmd %d within 2s\n", cmd_type);
            }
            // 超时放在定时检测中检查
        }

        //---------------对指令的处理------------------
        if (cmd_type == BLE_CMD_HANDSHAKE_RSP) //握手指令
        {
            if (pdata[0] == 0x5A && pdata[1] == 0xA5) // // 握手应成功
            {
                g_controler_state.handshake_ok = true; // 握手成功，停止定时器

                handshake_retry_count = 0;

                log_info("BLE and Controler HANDSHAKE succeed");
                set_led_state(LED_STATE_GREEN_ON);   // 绿灯常亮
                return;
            }
            else
            {
                log_info("BLE and Controler HANDSHAKE fail");
                set_led_state(LED_STATE_RED_FAST_BLINK); //红灯0.5s快闪  和控制器连接失败
                return;
            }
        }
        else if (cmd_type == BLE_CMD_UNLOCK_STATUS_RSP)//查询解锁状态指令
        {
            log_info("Controler Status is here\r");
            //查询解锁状态
            if (pdata[0] == 0x69 && pdata[1] == 0x96)  // 解锁状态：已解锁
            {
                g_controler_state.active_ok = true; // 设备已处于激活状态
                uint8_t result_code = 0x00; //解锁状态
                reply_app_status(APP_CMD_QUERY_UNLOCK_RSP, result_code);//回复app
                log_info("Controler Status is Unlock\r");
                return;
            }
            if (pdata[0] == 0x96 && pdata[1] == 0x69) // 解锁状态：未解锁
            {
                g_controler_state.active_ok = false; // 设备处于未激活状态，需要激活
                uint8_t result_code = 0x01; //未解锁状态
                reply_app_status(APP_CMD_QUERY_UNLOCK_RSP, result_code);//回复app
                log_info("Controler Status is lock\r");
                return;
            }
            set_led_state(LED_STATE_GREEN_ON); // 绿灯常亮
        }
        else if (cmd_type == BLE_CMD_UNLOCK_RSP)//解锁指令
        {
            if (pdata[0] == 0x5A && pdata[1] == 0xA5) // // 解锁成功，需要将结果发给app
            {
                // 发送解锁成功的应答给APP
                uint8_t result_code = 0x00; //解锁成功
                reply_app_status(APP_CMD_UNLOCK_RSP, result_code);//回复app
                log_info("Controler Unlock Success");
                return;
            }
        }
        else if (cmd_type == BLE_CMD_LOCK_RSP)//锁定指令
        {
            if (pdata[0] == 0x5A && pdata[1] == 0xA5) // // 锁定成功，需要将结果发给app
            {
                // 发送锁定成功的应答给APP
                uint8_t result_code = 0x00; //锁定成功
                reply_app_status(APP_CMD_LOCK_RSP, result_code);//回复app
                log_info("Controler Lock Success");
                return;
            }
        }
        else if (cmd_type == BLE_CMD_FAULT_QUERY_RSP)//控制器故障查询指令
        {
            // 主控返回2字节，直接转发给APP，格式：AA 05 Byte0 Byte1 55
            uint8_t reply[5] = {0xAA, APP_CMD_FAULT_QUERY_RSP, pdata[0], pdata[1], 0x55};
            // 打印即将发送的数据内容
            user_print_auto("reply_app_fault_query", reply, 5);
            int ret = bt_ven_notify(reply, 5);
            printk("bt_ven_notify return: %d\n", ret);
            log_info("Controler Fault Query: Byte0=0x%02X, Byte1=0x%02X\n", pdata[0], pdata[1]);

            // 检查是否有故障
            if (!(pdata[0] == 0x00 && pdata[1] == 0x00))
            {
                set_led_state(LED_STATE_RED_ON); // 红灯常亮
            }
            return;

        }

    }
    set_led_state(LED_STATE_GREEN_ON); // 绿灯常亮
    // 3. 其他数据处理（如有需要，可转发或忽略）
//     user_print_auto("MCU Send Other Data", (const uint8_t *)data, datalen);
}
/*
 * fun:  Handle_protocol_data
 * param:  const char *data:入参数据指针,
 *         uint16_t len:数据长度,
 *         void (*user_ble_notify_fun)(const char *data, uint16_t len):用户回调函数
 * return: void
 * description: 主控数据处理函数
 */
void Handle_protocol_data(const char *data, uint16_t len)
{
    // 数据不符合协议要求，直接返回
    if (data == NULL || len == 0 || len > MCU_MAX_LEN)
    {
        return;
    }
//     printk("Controler send Data to BLE");
//     show_reg3(data,len);
    /// 处理控制器回复数据
    Handle_mcu_replydata(data, len, g_controler_state.last_cmd_type);
}

/**
 * @brief 发送新协议指令到主控
 * @param cmd_type 0=握手指令，1=查询解锁状态指令，2=解锁指令 3=锁定指令
 *
 */
void BLE_send_protocol_cmd(ble_cmd_type_t cmd_type)
{
    // 只有握手成功后，才允许发送除握手包以外的指令
    if (cmd_type != BLE_CMD_HANDSHAKE_RSP && !g_controler_state.handshake_ok)
    {
        log_info("Handshake not completed, cannot send cmd %d\n", cmd_type);
        return;
    }
    uint8_t buf[BLE_PROTOCOL_CMD_LEN] = {0};
    switch (cmd_type)
    {
        case BLE_CMD_HANDSHAKE_RSP: // 握手指令
            memcpy(buf, HANDSHAKE_CMD, BLE_PROTOCOL_CMD_LEN);
            break;
        case BLE_CMD_FAULT_QUERY_RSP: // 控制器故障查询指令
            memcpy(buf, FAULT_QUERY_CMD, BLE_PROTOCOL_CMD_LEN);
            break;
        case BLE_CMD_UNLOCK_STATUS_RSP: // 查询解锁状态指令
            memcpy(buf, UNLOCK_STATUS_CMD, BLE_PROTOCOL_CMD_LEN);
            break;
        case BLE_CMD_UNLOCK_RSP: // 解锁指令
            memcpy(buf, UNLOCK_CMD, BLE_PROTOCOL_CMD_LEN);
            break;
        case BLE_CMD_LOCK_RSP: // 锁定指令
            memcpy(buf, LOCK_CMD, BLE_PROTOCOL_CMD_LEN);
            break;
        default:
            return;
    }

    //记录最近一次发送的指令类型
    g_controler_state.last_cmd_type = cmd_type;

    cmd_wait[cmd_type].waiting = true;
    cmd_wait[cmd_type].send_time_ms = k_uptime_get_32(); //记录当前时间



    //--打印发送给控制器的数据---
    user_print_auto("BLE send data to Contoler", (const uint8_t *)buf, BLE_PROTOCOL_CMD_LEN);

    //发送指令到MCU控制器
    //void ble_send_data(const char *data, uint16_t len);
    ble_send_data(buf, BLE_PROTOCOL_CMD_LEN);
    //TGT_SendMultiData(buf, BLE_PROTOCOL_CMD_LEN);

}

void user_parameter_init(void)
{
    for (int i = 0; i < 5; i++) {
        cmd_wait[i].waiting = false;
        cmd_wait[i].timeout_state = false;
        cmd_wait[i].send_time_ms = 0;
    }
}
