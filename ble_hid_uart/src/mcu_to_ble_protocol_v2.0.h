#ifndef MCU_TO_BLE_PROTOCOL_V2_H
#define MCU_TO_BLE_PROTOCOL_V2_H

#include <stdint.h>
#include <stdbool.h>
#define MCUMSG_BLE          "MCU|"
#define MCUMSG_FUN_LINK     "LNK|"
#define MCUMSG_FUN_ADV      "ADV|"
#define MCUMSG_FUN_GTH      "GTH|"     //蓝牙连接强度信号
#define MCUMSG_FUN_MAC      "MAC|"
#define MCUMSG_FUN_READY    "READY|"

#define MCUMSG_ASK          "ASK"
#define MCUMSG_ACK          "ACK"
#define MCUMSG_OPEN         "OPEN"
#define MCUMSG_CLOSE        "CLOSE"
#define MCUMSG_CON          "CON|"
#define MCUMSG_DISCON       "DISCON|"

// 协议命令字符串定义
#define CMD_ADV_OUTCOME_SUCC   "BLE|ADV|0"   // 广播操作成功
#define CMD_ADV_OUTCOME_FAIL   "BLE|ADV|1"   // 广播操作失败

#define CMD_LNK_DATA_CON       "BLE|LNK|0"   // 已连接
#define CMD_LNK_DATA_DISCON    "BLE|LNK|1"   // 断开连接

#define CMD_BLE_READY       "BLE|READY|SUC"   // 蓝牙已就绪
#define CMD_BLE_NOT_READY   "BLE|READY|FAIL"  // 蓝牙未就绪

//-----------log -----------------------
#ifndef LOG_INFO
#define LOG_INFO(fmt, ...)   printf(fmt, ##__VA_ARGS__)
#endif

#ifndef LOG_DEBUG
#define LOG_DEBUG(fmt, ...)  printf(fmt, ##__VA_ARGS__)
#endif

#define log_info LOG_INFO
#define log_debug LOG_DEBUG

#define DEBUG_PRINT_ENABLE 1  //调试打印使能  只在调试时打开
//------------OTA-------------------------
#define OTA_ENABLED 1 // OTA功能使能

//------控制器和蓝牙模拟通信-----
#define BLE_CONTROLER_SIMU_ENABLED  0 // 蓝牙和控制器模拟通信使能
//---------------参数------------------------------

#define UART_FRAME_HEAD_0  0xAA
#define UART_FRAME_HEAD_1  0x55
#define UART_RX_CACHE_SIZE 512

//对app发送的数据进行粘包分帧处理，并逐帧发送到主控 MCU
#define APP_FRAME_HEAD 0x55
#define APP_FRAME_TRAIL 0xAA
#define APP_RX_CACHE_SIZE 256

typedef enum {
    BLE_CMD_HANDSHAKE_RSP = 0,   // 握手指令
    BLE_CMD_UNLOCK_STATUS_RSP,   // 查询解锁状态
    BLE_CMD_UNLOCK_RSP,          // 解锁
    BLE_CMD_LOCK_RSP ,           // 锁定
    BLE_CMD_FAULT_QUERY_RSP,     // 控制器故障查询指令
    BLE_CMD_MAX                  // 最大指令数  用来对指令数量进行计数
} ble_cmd_type_t;

//app发送数据的指令编号
typedef enum {
    APP_CMD_HANDSHAKE_RSP = 1,   // 握手指令
    APP_CMD_QUERY_UNLOCK_RSP,    // 查询解锁状态
    APP_CMD_UNLOCK_RSP,          // 解锁
    APP_CMD_LOCK_RSP,            // 锁定
    APP_CMD_FAULT_QUERY_RSP,     // 控制器故障查询
    APP_CMD_QUERY_MAC_RSP,       // 查询MAC地址
    APP_CMD_SET_BLE_NAME,        // 修改蓝牙名称
    APP_CMD_MAX                  // 最大指令数  用来对指令数量进行计数
} app_cmd_type_t;

typedef struct {
    bool waiting;
    uint32_t send_time_ms;
    bool timeout_state;
} cmd_wait_t;
extern cmd_wait_t cmd_wait[5]; // 0:握手 1:查询 2:解锁 3:锁定 4:故障查询
//----------------------------------------
typedef struct
{
    const char *msg_content;
    /* data */
}blemsg_fun_str;


typedef enum
{
    MCUMSG_ENUM_NULL= 0,
    MCUMSG_ENUM_LINK,
    MCUMSG_ENUM_ADV,
    MCUMSG_ENUM_GTH,
    MCUMSG_ENUM_MAC,
    MCUMSG_ENUM_READY,
    MCUMSG_ENUM_ADV_ALLNUM
}blemsg_fun_enum;


// 协议命令类型枚举
typedef enum {
    PROTO_CMD_NONE = 0,
    PROTO_CMD_ADV_SUCC,
    PROTO_CMD_ADV_FAIL,
    PROTO_CMD_LNK_CON,
    PROTO_CMD_LNK_DISCON,
    PROTO_CMD_UNKNOWN
} proto_cmd_e;

// 协议解析结果结构体
typedef struct {
    proto_cmd_e cmd_type;
    const char *raw_str;
    uint32_t raw_len;
} proto_parse_result_t;

typedef struct {
    bool handshake_ok;            // 握手成功状态
    bool active_ok;               // 激活状态
    ble_cmd_type_t last_cmd_type; // 最近一次发送的指令类型
} controler_state_t;

//---------------------------------------------------------
typedef int (*ble_get_conn_state_cb_t)(void); // 获取连接状态回调函数
typedef void (*ble_send_data_cb_t)(const char *data);// 发送数据回调函数
typedef int (*ble_get_adv_state_cb_t)(void);//广播状态获取回调函数
typedef int (*ble_adv_ctrl_cb_t)(void); //广播控制回调函数 返回1成功，0失败

// 协议解析函数
proto_cmd_e mcu_to_ble_parse_cmd(const uint8_t *data, uint32_t len);

// 协议组包函数（如有需要）
uint32_t mcu_to_ble_pack_cmd(proto_cmd_e cmd, uint8_t *out_buf, uint32_t buf_size);

//--------------BLE主动上报--------------------
typedef void (*ble_protocol_send_func_t)(const char *data, uint16_t len);
// 注册BLE协议主动上报的发送函数
void ble_to_mcu_register_send_func(ble_protocol_send_func_t func);
// BLE主动上报接口
void ble_to_mcu_report_link_state(int state); // 0=连接，1=断开
//----------------------------------------------

// 其它协议相关接口可继续扩展...
// 注册回调

void mcu_to_ble_register_adv_open_cb(ble_adv_ctrl_cb_t cb); //注册打开广播回调函数
void mcu_to_ble_register_adv_close_cb(ble_adv_ctrl_cb_t cb);//注册关闭广播回调函数

void ble_to_mcu_resend_check(void); // 蓝牙主动上报数据重发
bool Judge_stringisdata(const char *data);

//对外部接口函数
void Handle_protocol_data(const char *data, uint16_t len);
void BLE_send_protocol_cmd(ble_cmd_type_t cmd_type);
void reply_app_status(app_cmd_type_t cmd, uint8_t result_code);

void user_parameter_init(void);

extern controler_state_t g_controler_state;
extern void ble_send_data(const char *data, uint16_t len);
#endif // MCU_TO_BLE_PROTOCOL_V2_H



