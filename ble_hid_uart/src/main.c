/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */


 //后面以此版本作为开发基础

 #include <zephyr/types.h>
 #include <stddef.h>
 #include <string.h>
 #include <errno.h>
 #include <sys/printk.h>
 #include <sys/byteorder.h>
 #include <zephyr.h>
 #include <sys/reboot.h>

 #include <stdio.h>

 #include <settings/settings.h>

 #include <bluetooth/bluetooth.h>
 #include <bluetooth/hci.h>
 #include <bluetooth/conn.h>
 #include <bluetooth/uuid.h>
 #include <bluetooth/gatt.h>
 #include <bluetooth/services/bas.h>
 #include <bluetooth/services/hrs.h>

 #include <drivers/gpio.h>
 #include "PanSeries.h"
 #include "comm_prf.h"

 #include <device.h>
 #include <devicetree.h>
 #include <drivers/uart.h>
 #include <host/hci_core.h>
 #include "pan_fmc.h"
 #include <drivers/flash.h>

 #include "soc.h"
 #include "ancs_c.h"
 #include <pan_ble_stack.h>

 #include <bluetooth/buf.h>
 #include <host/conn_internal.h> // 需要暴露内部结构体

 #include "mcu_to_ble_protocol_v2.0.h"
 #include <host/adv.h>
 #include <ctype.h>
#include "dev_led.h"

#include "mcu_to_ble_protocol_v3.0.h"
/******************  OTA INCLUDE *******************/
#if OTA_ENABLED
#ifdef CONFIG_MCUMGR_CMD_OS_MGMT
#include "os_mgmt/os_mgmt.h"
#endif
#ifdef CONFIG_MCUMGR_CMD_IMG_MGMT
#include "img_mgmt/img_mgmt.h"
#endif
#ifdef CONFIG_MCUMGR_CMD_STAT_MGMT
#include "stat_mgmt/stat_mgmt.h"
#endif
#ifdef CONFIG_MCUMGR_SMP_BT
#include <mgmt/mcumgr/smp_bt.h>
#endif
#endif
/*****************************************************/

 //base demo
 struct bt_conn *connect_handle;

 //////软件定时器开启//////
 static struct k_timer ancs_timer;
 //////软件定时器开启//////

 ///************************************** HOG.H***********************************************////
 #define CONSUMER_KEY_VAL_AC_HOME                0x0001  /* AC Home */
 #define CONSUMER_KEY_VAL_BRIGHTNESS_DOWN        0x0002  /* Brightness Down */
 #define CONSUMER_KEY_VAL_BRIGHTNESS_UP          0x0004  /* Brightness Up */
 #define CONSUMER_KEY_VAL_KEYBOARD_LAYOUT        0x0008  /* Keyboard Layout */
 #define CONSUMER_KEY_VAL_AC_SEARCH              0x0010  /* AC Search */
 #define CONSUMER_KEY_VAL_SCAN_PREVIOUS_TRACK    0x0100  /* Scan Previous Track */
 #define CONSUMER_KEY_VAL_PLAY_PAUSE             0x0200  /* Play/Pause */
 #define CONSUMER_KEY_VAL_SCAN_NEXT_TRACK        0x0400  /* Scan Next Track */
 #define CONSUMER_KEY_VAL_MUTE                   0x0800  /* Mute */
 #define CONSUMER_KEY_VAL_VOLUME_DECREMENT       0x1000  /* Volume Decrement */
 #define CONSUMER_KEY_VAL_VOLUME_INCREMENT       0x2000  /* Volume Increment */
 #define CONSUMER_KEY_VAL_POWER                  0x4000  /* Power */
 #define CONSUMER_KEY_VAL_REALEASED              0x0000  /* Realeased */


 #define BT_LE_ADV_CONN_NAME_APP BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE | \
						 BT_LE_ADV_OPT_USE_NAME,	    \
						 0x140,			    \
						 0x140, NULL)



 extern void panchip_prf_set_tx_pwr(int8_t tx_pwr);
 extern int bt_write_tx_power_level(int8_t tx_power_level);

 //----------用户声明---------------
 static bool ble_init_ok = false;
 static struct k_work adv_stop;
 static struct k_work adv_start;

 static struct k_work rssi_work;

 // 全局变量，记录广播状态
 static bool adv_is_open = false;
 static bool adv_busy = false;


 static int app_ble_adv_open(void);
 static int app_ble_adv_close(void);
 static void ble_notify_ready_status(int ble_err, int adv_ret);
 void adv_stop_work(struct k_work *work);

 void ble_send_data(const char *data, uint16_t len);
 static void uart_real_send(const uint8_t *data, uint16_t len);

 static void rssi_work_handler(struct k_work *work);

 int bt_ven_notify(uint8_t *data, uint32_t len);

 //////////MAC的广播存入数组//////////
 //////////MAC的广播存入数组//////////

 ///************************************** HOG.H***********************************************////

 ///////////////////////ANCS/////////////////////////
 volatile uint8_t event_flag;
 ///////////////////////ANCS/////////////////////////

 typedef struct {
	 uint8_t rssi;
	 uint8_t rssi_flag;
 } rssi_data;
 rssi_data rssi_set;

 ////// ANCS配置 //////
 uint8_t  ANCS_FIRST_FLAG = 0;
 uint8_t	 ANCS_FLAG = 2;
 ////// ANCS配置 //////

 ///************************************** HOG.C***********************************************////
 enum {
	 HIDS_REMOTE_WAKE                = BIT(0),
	 HIDS_NORMALLY_CONNECTABLE       = BIT(1),
 };

 struct hids_info {
	 uint16_t version;       /* version number of base USB HID Specification */
	 uint8_t code;           /* country HID Device hardware is localized for. */
	 uint8_t flags;
 } __packed;

 struct hids_report {
	 uint8_t id;     /* report id */
	 uint8_t type;   /* report type */
 } __packed;

 struct hids_info info = {
	 .version = 0x0000,
	 .code = 0x00,
	 .flags = HIDS_NORMALLY_CONNECTABLE,
 };

 enum {
	 HIDS_INPUT      = 0x01,
	 HIDS_OUTPUT     = 0x02,
	 HIDS_FEATURE    = 0x03,
 };

 struct hids_report input = {
	 .id = 0x01,
	 .type = HIDS_INPUT,
 };

 struct hids_report input_consumer = {
	 .id = 0x02,
	 .type = HIDS_INPUT,
 };

 uint8_t simulate_input;
 uint8_t simulate_input_consumer;
 uint8_t ctrl_point;
 static unsigned int fixed_passkey = BT_PASSKEY_INVALID;


 static uint8_t report_map[] = {
	 0x05, 0x0C,         // Usage Page (Consumer)
	 0x09, 0x01,         // Usage (Consumer Control)
	 0xA1, 0x01,         // Collection (Application)
	 0x85, 0x01,         //      Report Id (1)
	 0x15, 0x00,         //      Logical minimum (0)
	 0x25, 0x01,         //      Logical maximum (1)
	 0x75, 0x01,         //      Report Size (1)
	 0x95, 0x01,         //      Report Count (1)
	 0x09, 0x94,         //      (Quit)
	 0x81, 0x06,         //      Input (Data,Value,Relative,Bit Field)
	 0x09, 0x95,         //      (Help)
	 0x81, 0x06,         //      Input (Data,Value,Relative,Bit Field)
	 0x09, 0xEA,         //      (Volume Down)
	 0x81, 0x06,         //      Input (Data,Value,Relative,Bit Field)
	 0x09, 0xE9,         //      (Volume Up)
	 0x81, 0x06,         //      Input (Data,Value,Relative,Bit Field)
	 0x09, 0xCB,         //      (Tracking Decrement)
	 0x81, 0x06,         //      Input (Data,Value,Relative,Bit Field)
	 0x09, 0xCA,         //      (tracking Increment)
	 0x81, 0x06,         //      Input (Data,Value,Relative,Bit Field)
	 0x09, 0xB6,         //      (Scan Previous Track)
	 0x81, 0x06,         //      Input (Data,Value,Relative,Bit Field)
	 0x09, 0xB5,         //      (Scan Next Track)
	 0x81, 0x06,         //      Input (Data,Value,Relative,Bit Field)
	 0x09, 0xB1,         //      (Pause)
	 0x81, 0x06,         //      Input (Data,Value,Relative,Bit Field)
	 0x09, 0xB0,         //      (Play)
	 0x81, 0x06,         //      Input (Data,Value,Relative,Bit Field)
	 0x75, 0x01,         //      Report Size (1)
	 0x95, 0x06,         //      Report Count (6)
	 0x81, 0x07,         //      Input (Data,Value,Relative,Bit Field)
	 0xC0                // End Collection
 };

 //----------用户函数声明------------------

 void user_print_auto(const char *prefix, const uint8_t *data, uint16_t len);
 //-----------用户变量声明----------------------------------
 #define HANDSHAKE_MAX_RETRY 50
 int handshake_retry_count = 0;

 //--------------------log 函数----------------------

 //--------------------------------------------------

 extern void panchip_prf_set_tx_pwr(int8_t tx_pwr);

 static ssize_t read_info(struct bt_conn *conn,
			  const struct bt_gatt_attr *attr, void *buf,
			  uint16_t len, uint16_t offset)
 {
	 return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data,
				  sizeof(struct hids_info));
 }

 static ssize_t read_report_map(struct bt_conn *conn,
					const struct bt_gatt_attr *attr, void *buf,
					uint16_t len, uint16_t offset)
 {
	 return bt_gatt_attr_read(conn, attr, buf, len, offset, report_map,
				  sizeof(report_map));
 }

 static ssize_t read_report(struct bt_conn *conn,
				const struct bt_gatt_attr *attr, void *buf,
				uint16_t len, uint16_t offset)
 {
	 return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data,
				  sizeof(struct hids_report));
 }

 static void input_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
 {
	 simulate_input = (value == BT_GATT_CCC_NOTIFY) ? 1 : 0;
	 printk("ccc %04X\n", value);
 }

 static ssize_t read_input_report(struct bt_conn *conn,
				  const struct bt_gatt_attr *attr, void *buf,
				  uint16_t len, uint16_t offset)
 {
	 return bt_gatt_attr_read(conn, attr, buf, len, offset, NULL, 0);
 }

 static ssize_t read_report_consumer(struct bt_conn *conn,
					 const struct bt_gatt_attr *attr, void *buf,
					 uint16_t len, uint16_t offset)
 {
	 return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data,
				  sizeof(struct hids_report));
 }

 static void input_ccc_changed_consumer(const struct bt_gatt_attr *attr, uint16_t value)
 {
	 simulate_input_consumer = (value == BT_GATT_CCC_NOTIFY) ? 1 : 0;
	 printk("ccc consumer %04X\n", value);
 }

 static ssize_t read_input_report_consumer(struct bt_conn *conn,
					   const struct bt_gatt_attr *attr, void *buf,
					   uint16_t len, uint16_t offset)
 {
	 return bt_gatt_attr_read(conn, attr, buf, len, offset, NULL, 0);
 }

 static ssize_t write_ctrl_point(struct bt_conn *conn,
				 const struct bt_gatt_attr *attr,
				 const void *buf, uint16_t len, uint16_t offset,
				 uint8_t flags)
 {
	 uint8_t *value = attr->user_data;

	 if (offset + len > sizeof(ctrl_point)) {
		 return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	 }

	 memcpy(value + offset, buf, len);

	 return len;
 }




 ///************************************** HOG.C***********************************************////


 typedef struct {
	 uint32_t baudrate;
	 uint8_t Blue_mac[6];
	 uint8_t device_name[28];
	 uint8_t name_length;
	 uint32_t rst_flag;
	 uint8_t own_mac[6];
	 uint8_t bond_mac[6];
	 uint32_t passkey;

 } fmc_data;

 bool uart_running;
 extern int z_clock_hw_cycles_per_sec;


 #define VND_MAX_LEN               200
 #define BAUDRATE                  9600
 #define TGT_UART                  UART1
 #define DEFAULT_CONFIG_DATA_ADD   0x0007c000

 uint8_t Default_mac_addr[6] = { 0x11, 0x22, 0x33, 0x44, 0x66, 0x88 };

 uint8_t vnd_value[VND_MAX_LEN + 1] = { 'V', 'e', 'n', 'd', 'o', 'r', '1' };
 static uint8_t vnd_auth_value[VND_MAX_LEN + 1] = { 'V', 'e', 'n', 'd', 'o', 'r', '2' };
 uint8_t vnd_wwr_value[VND_MAX_LEN + 1] = { 'V', 'e', 'n', 'd', 'o', 'r', '3' };

 uint8_t simulate_vnd;
 static uint8_t indicating;
 struct bt_gatt_indicate_params ind_params;


 /* The devicetree node identifier for the "led0" alias. PAN1080 AFLD Board LED pin is P21.*/
 #define GPIO_NODE DT_ALIAS(led0)

 #if DT_NODE_EXISTS(GPIO_NODE)
 #define PORT    DT_GPIO_LABEL(GPIO_NODE, gpios)
 #define PIN     DT_GPIO_PIN(GPIO_NODE, gpios)
 #define FLAGS   DT_GPIO_FLAGS(GPIO_NODE, gpios)
 #else
 /* A build error here means your board isn't set up to blink an LED. */
 #error "Unsupported board: led0 devicetree alias is not defined"
 #define LED0    ""
 #define PIN     0
 #define FLAGS   0
 #endif

 /*
  * When DT_CHOSEN_ZEPHYR_FLASH_CONTROLLER_LABEL is available, we use it here.
  * Otherwise the device can be set at runtime with the set_device command.
  */
 #ifndef DT_CHOSEN_ZEPHYR_FLASH_CONTROLLER_LABEL
 #define DT_CHOSEN_ZEPHYR_FLASH_CONTROLLER_LABEL ""
 #endif

 static const struct device *flash_device;


 // void show_reg3(uint8_t const *data, uint32_t len)
 // {
 // 	uint32_t i = 0;

 // 	if (len == 0) {
 // 		return;
 // 	}
 // 	for (; i < len; i++) {
 // 		printf("0x%02X ", data[i]);
 // 	}
 // 	printf("\n");
 // }
 void show_reg3(uint8_t const *data, uint32_t len)
 {
	 uint32_t i = 0;

	 if (len == 0) {
		 return;
	 }
	 for (; i < len; i++) {
		 printk("0x%02X ", data[i]);
	 }
	 printk("\n");
 }
 void show_reg(uint8_t const *data, uint32_t len)
 {
	 uint32_t i;

	 for (i = 0; i < len; i++) {
		 printk("M0x%02X ", data[i]);
	 }
	 printk("\n");
 }

 //---------------自定义打印函数--------------------
 void user_print_auto(const char *prefix, const uint8_t *data, uint16_t len)
 {
	 bool is_printable = true;
	 for (uint16_t i = 0; i < len; i++) {
		 if (!isprint(data[i]) && data[i] != '\r' && data[i] != '\n')
		 {
			 is_printable = false;
			 break;
		 }
	 }
	 if (is_printable)
	 {
		 printk("%s (string, len=%d): %.*s\r", prefix, len, len, data);
	 }
	 else
	 {
		 printk("%s (hex, len=%d): ", prefix, len);
		 for (uint16_t i = 0; i < len; i++) {
			 printk("%02X ", data[i]);
		 }
		 printk("\r");
	 }
 }
 /////////////通过串口1发送给MCU串口的函数///////////////
 /* Send multiple data to target test Uart */
 void TGT_SendMultiData(const uint8_t *data, size_t size)
 {
	// printk("TGT_SendMultiData: ");
	// for (size_t i = 0; i < size; i++) {
	// 	printk("%02X ", data[i]);
	// }
	// printk("\r\n");

	 while (size--) {
		 TGT_UART->RBR_THR_DLL = *(data++);
		 /* Wait until THR is empty to avoid data lost */
		 while (!(TGT_UART->LSR & UART_LSR_TEMT_Msk)) {
		 }
	 }
 }



//处理app数据并回复app
void Handle_app_cmd_and_reply(uint8_t app_cmd ,const uint8_t *data, uint16_t len)
{
    // 1. 握手指令
    if (app_cmd == APP_CMD_HANDSHAKE_RSP)
	{
		// 只有解码器与控制器握手成功，才允许APP与解码器握手
		if (!g_controler_state.handshake_ok)
		{
			// 回复APP握手失败
			//小程序校验蓝牙回复结果失败（数据校验失败、超时未回复），会重发两次，都失败后认为握手失败。
			//所以，此处直接不回复，app重发两次后，即会认为握手失败。
			//协议是：app与蓝牙握手，若蓝牙回复的数据不对或者未回复，
			//app发送两次指令后仍不对，则认为握手失败，但是这逻辑，蓝牙无法判断呀，除非修改协议。
			printk("Controler handshake not completed, APP handshake failed\n");
			set_led_state(LED_STATE_RED_SLOW_BLINK); // 红灯慢闪
			return;
		}
        else
		{
			// 正常握手流程
			uint8_t result = data[2] ^ data[3] ^ data[4] ^ data[5];
			uint8_t reply[4] = {0xAA, 0x01, result, 0x55};
			bt_ven_notify(reply, 4);
			printk("APP and BLE handshake completed\r");
			return;
		}

    }
	// 2. 查询解锁状态指令
	if (app_cmd == APP_CMD_QUERY_UNLOCK_RSP)
	{
		 // 新增：未握手成功直接回复失败
		if (!g_controler_state.handshake_ok)
		{
			reply_app_status(APP_CMD_QUERY_UNLOCK_RSP, 0x02); // 0x02=蓝牙与控制器未握手
			printk("Handshake not completed, cannot query unlock status\n");
			return;
		}

		printk("APP send comand to query the Controler unlock status\r");
		BLE_send_protocol_cmd(BLE_CMD_UNLOCK_STATUS_RSP); // 发送查询解锁状态指令
		return;
	}
    // 3. 解锁指令
    if (app_cmd == APP_CMD_UNLOCK_RSP)
	{
        // 发送解锁指令到主控，等待主控回复
        // 解析主控回复，获得解锁结果 result_code（00/01/02）
        // 组包回复

		//先查询控制器的激活状态，已解锁就不再进行解锁
		if(g_controler_state.active_ok)
		{
			printk("Controler is already activated\r");

			uint8_t result_code = 0x02; //已被解锁过，无需再次解锁
			uint8_t reply[4] = {0xAA, 0x02, result_code, 0x55};
			bt_ven_notify(reply, 4);
		}
		else //未激活，则发送解锁指令给到主控
		{
			printk("APP sent activate command, activating controller...\r");
			BLE_send_protocol_cmd(BLE_CMD_UNLOCK_RSP); // 发送解锁指令包给主控进行解锁控制器
		}
        return;
    }
	// 4. 锁定指令
	if (app_cmd == APP_CMD_LOCK_RSP)
	{
		printk("APP sent lock command, locking controller...\r");
		BLE_send_protocol_cmd(BLE_CMD_LOCK_RSP); // 发送锁定指令包给主控进行锁定控制器
		return;
	}
	// 4. 控制器故障查询指令
	if (app_cmd == APP_CMD_FAULT_QUERY_RSP)
	{
		printk("APP sent fault query command, querying controller...\r");
		BLE_send_protocol_cmd(BLE_CMD_FAULT_QUERY_RSP); // 发送故障查询指令包给主控进行故障查询
		return;
	}
	// 5. APP查询MAC地址指令
	// if (app_cmd == APP_CMD_QUERY_MAC_RSP)
	// {
	// 	uint8_t reply[9] = {0xAA, APP_CMD_QUERY_MAC_RSP};
	// 	bt_addr_le_t addr;
	// 	size_t count = 1;
	// 	bt_id_get(&addr, &count);
	// 	// MAC地址按大端序发送
	// 	for (int i = 0; i < 6; i++)
	// 	{
	// 		reply[2 + i] = addr.a.val[5 - i];
	// 	}
	// 	reply[8] = 0x55;
	// 	bt_ven_notify(reply, sizeof(reply));

	// 	printk("APP query MAC, reply: ");
	// 	for (int i = 0; i < 9; i++) printk("%02X ", reply[i]);
	// 	printk("\n");
	// 	return;
	// }
    // // 6. 修改蓝牙名称
	// if (app_cmd == APP_CMD_SET_BLE_NAME)
	// {
	// 	// 提取蓝牙名数据（假设第3字节开始为蓝牙名，长度为 len-3-1）
	// 	char new_ble_name[30] = {0};
	// 	uint16_t name_len = len - 3; // 去掉头、命令字、尾
	// 	if (name_len > 0 && name_len < sizeof(new_ble_name))
	// 	{
	// 		memcpy(new_ble_name, data + 2, name_len);
	// 		bt_set_name(new_ble_name); // 修改蓝牙名
	// 		printk("BLE name changed to: %s\n", new_ble_name);
	// 		// 可回复APP结果
	// 		reply_app_status(APP_CMD_SET_BLE_NAME, 0x00); // 0x00=成功
	// 	}
	// 	else
	// 	{
	// 		reply_app_status(APP_CMD_SET_BLE_NAME, 0x01); // 0x01=失败
	// 	}
	// 	return;
	// }
	// 6. 其他指令可扩
}




 static ssize_t read_vnd(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 void *buf, uint16_t len, uint16_t offset)
 {
	 const char *value = attr->user_data;

	 return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
				  strlen(value));
 }

 /////手机透传给蓝牙BLE，随后通过串口发给MCU/////
 static ssize_t write_vnd(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			  const void *buf, uint16_t len, uint16_t offset,
			  uint8_t flags)
 {
	 /* uint8_t *value = attr->user_data; */
	 // show_reg3(buf, len);
	 #if DEBUG_PRINT_ENABLE
	 user_print_auto("APP send data to BLE", (const uint8_t *)buf, len);
	 #endif
	 //对输入的数据进行粘包分帧处理，并回复app。
	 V3_handle_app_data((const uint8_t *)buf, len);
	 return len;
 }


 static void vnd_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
 {
	 ARG_UNUSED(attr);

	 bool notif_enabled = (value == BT_GATT_CCC_NOTIFY);

	 printk(" notifications %s", notif_enabled ? "enabled" : "disabled");
 }

 void indicate_cb(struct bt_conn *conn,
		  struct bt_gatt_indicate_params *params, uint8_t err)
 {
	 printk("Indication %s\n", err != 0U ? "fail" : "success");
 }

 void indicate_destroy(struct bt_gatt_indicate_params *params)
 {
	 printk("Indication complete\n");
	 indicating = 0U;
 }


 ssize_t write_without_rsp_vnd(struct bt_conn *conn,
				   const struct bt_gatt_attr *attr,
				   const void *buf, uint16_t len, uint16_t offset,
				   uint8_t flags)
 {
	 uint8_t *value = attr->user_data;

	 if (!(flags & BT_GATT_WRITE_FLAG_CMD)) {
		 /* Write Request received. Reject it since this Characteristic
		  * only accepts Write Without Response.
		  */
		 return BT_GATT_ERR(BT_ATT_ERR_WRITE_REQ_REJECTED);
	 }

	 if (offset + len > VND_MAX_LEN) {
		 return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	 }

	 memcpy(value + offset, buf, len);
	 value[offset + len] = 0;
	 return len;
 }



 /* Custom Service Variables */
 #define BT_UUID_SYGIC_SERVICE_VAL \
	 BT_UUID_128_ENCODE(0xDD3F0AD1, 0x6239, 0x4E1F, 0x81F1, 0x91F6C9F01D86)

 #define BT_UUID_CUSTOM_SERVICE_VAL \
	 BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)

 // Nordic UART Service UUIDs
 static struct bt_uuid_128 nus_service_uuid = BT_UUID_INIT_128(
	 0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
	 0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e
 );

 static struct bt_uuid_128 nus_rx_uuid = BT_UUID_INIT_128(
	 0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
	 0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e
 );

 static struct bt_uuid_128 nus_tx_uuid = BT_UUID_INIT_128(
	 0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
	 0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e
 );

 /* HID Service Declaration */ //HID服务
 BT_GATT_SERVICE_DEFINE(hog_svc,
	 BT_GATT_PRIMARY_SERVICE(BT_UUID_HIDS),
	 // Report Map 特征（只读）
	 BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT_MAP, BT_GATT_CHRC_READ,
		 BT_GATT_PERM_READ, read_report_map, NULL, NULL),
	 // Report 特征（读/写/通知）
	 BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT,
		 BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
		 BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
		 read_input_report, write_ctrl_point, NULL),
	 BT_GATT_CCC(input_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	 // Protocol Mode 特征（读/写）
	 BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_PROTOCOL_MODE,
		 BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
		 BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
		 NULL, NULL, NULL),
 );

 //自定义服务// Nordic UART Service
 BT_GATT_SERVICE_DEFINE(nus_svc,
	 BT_GATT_PRIMARY_SERVICE(&nus_service_uuid),
	 // TX特征（通知，手机接收BLE发来的数据）
	 BT_GATT_CHARACTERISTIC(&nus_tx_uuid.uuid,
		 BT_GATT_CHRC_NOTIFY,
		 BT_GATT_PERM_NONE,
		 NULL, NULL, NULL),
	 BT_GATT_CCC(vnd_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	 // RX特征（写，手机发数据到BLE/MCU）
	 BT_GATT_CHARACTERISTIC(&nus_rx_uuid.uuid,
		 BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
		 BT_GATT_PERM_WRITE,
		 NULL, write_vnd, NULL),
 );

 ////// Timer的回调函数 ///////
 static void timer_expired_cb(struct k_timer *timer)
 {
	 ANCS_FLAG = 3;
	 printk("time_ANCS_FLAG=%d\n",ANCS_FLAG);
	 printk("timer_ok\r\n");
 }
 ////// Timer的回调函数 ///////

 //////////////////////////开关广播的API/////////////////////////////////
 #define LIST_CONNECTED_SIZE  1  //设置为设备允许的最大 BLE 连接数。

 static struct k_work adv_start;
 static struct k_work adv_stop;
 static struct k_work disconn_start;
 uint32_t disconn_index;
 struct bt_conn   *list_connected[LIST_CONNECTED_SIZE];
 uint8_t volatile count_connected;

 //---广播包内容填充-------------
 static void fill_adv_data(struct bt_data ad[5])
 {
	 //mac地址先不放入广播包，有需求再放入
	 /*
	 uint8_t mac_adv[6] = {0};
	 mac_adv[0] = otp.m.mac_addr[5];
	 mac_adv[1] = otp.m.mac_addr[4];
	 mac_adv[2] = otp.m.mac_addr[3];
	 mac_adv[3] = otp.m.mac_addr[2];
	 mac_adv[4] = otp.m.mac_addr[1];
	 mac_adv[5] = otp.m.mac_addr[0];
	 */
	 // 1. Flags
	 ad[0].data_len = 0x01;
	 ad[0].type = 0x01;
	 static uint8_t le_data = 0x06;
	 ad[0].data = &le_data;

	 // 2. Appearance
	 ad[1].data_len = 0x02;
	 ad[1].type = 0x19;
	 static uint8_t appearance[2] = {0xC2, 0x03};
	 ad[1].data = appearance;

	 // 3. HID服务UUID（16位）
	 ad[2].data_len = 0x02;
	 ad[2].type = 0x02;
	 static uint8_t uid[2] = {0x12, 0x18};
	 ad[2].data = uid;

	 // 4. Nordic UART Service UUID（128位）
	 ad[3].data_len = 16;
	 ad[3].type = 0x07;
	 static uint8_t uart_service_uuid[16] = {
		 0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
		 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E
	 };
	 ad[3].data = uart_service_uuid;

	 // 5. 厂商数据
	 // ad[4].data_len = 0x02;
	 // ad[4].type = 0xff;
	 // static uint8_t manufacturer[2] = {0x30, 0x32};
	 // ad[4].data = manufacturer;
 }


 void adv_start_work(struct k_work *work)
 {
	 struct bt_data ad[4];
	 fill_adv_data(ad);
	 int err = bt_le_adv_start(BT_LE_ADV_CONN_NAME_APP, ad, ARRAY_SIZE(ad), NULL, 0);
	 if (err) {
		 printk("bt_le_adv_start failed: %d\n", err);
		 adv_is_open = false;
	 } else {
		 printk("Advertising start success\n");
		 adv_is_open = true;
	 }
 }

 // void adv_stop_work(struct k_work *work)
 // {
 //    bt_le_adv_stop();
 //    printk("close\n");
 // }

 int ble_disconnect(uint32_t index)
 {
	 if (index >= count_connected) {
		 return 1;
	 }

	 return bt_conn_disconnect(list_connected[index], BT_HCI_ERR_REMOTE_USER_TERM_CONN);
 }

 extern struct bt_keys *store_key;
 void disconn_start_work(struct k_work *work)
 {

	 //int err = ble_disconnect(disconn_index);

	 bt_conn_disconnect(connect_handle, BT_HCI_ERR_REMOTE_USER_TERM_CONN);

	 bt_keys_clear(store_key);
	 // if (err) {
	 // 	printk("DF (err %d)", err);
	 // } else {
	 // 	printk("Disonnecting ...");
	 // }
 }
 //////////////////////////开关广播的API/////////////////////////////////

 int hog_send_key(uint16_t key_val)
 {
	 int err = -1;

	 if (simulate_input == 1) {
		 err = bt_gatt_notify(NULL, &hog_svc.attrs[5], &key_val, sizeof(key_val));
	 }

	 return err;
 }

 int hog_send_consumer_key(uint16_t key_val)
 {
	 int err = -1;

	 if (simulate_input_consumer == 1) {
		 err = bt_gatt_notify(NULL, &hog_svc.attrs[9], &key_val, sizeof(key_val));
	 }

	 return err;
 }

 void hog_init(void)
 {
	 /* empty */
 }

 ///////////////////配对绑定的API/////////////////

 static struct bt_conn_cb *callback_list;

 bool peri_connected;
 bool central_connected[LIST_CONNECTED_SIZE];

 struct bt_conn   *list_connected[LIST_CONNECTED_SIZE];
 uint8_t volatile count_connected;


 #if defined(CONFIG_BT_GATT_CLIENT)
 static void mtu_exchange_cb(struct bt_conn *conn, uint8_t err,
				 struct bt_gatt_exchange_params *params)
 {

	 printk("MTU exchange %u %s (%u)\n", bt_conn_index(conn),
			err == 0U ? "successful" : "failed", bt_gatt_get_mtu(conn));

 }

 static struct bt_gatt_exchange_params mtu_exchange_params[CONFIG_BT_MAX_CONN];

 static int mtu_exchange(struct bt_conn *conn)
 {
	 uint8_t conn_index;
	 int err;

	 conn_index = bt_conn_index(conn);

	 printk("MTU (%u): %u\n", conn_index, bt_gatt_get_mtu(conn));

	 mtu_exchange_params[conn_index].func = mtu_exchange_cb;

	 err = bt_gatt_exchange_mtu(conn, &mtu_exchange_params[conn_index]);
	 if (err) {
		 printk("MTU exchange failed (err %d)", err);
	 } else {
		 printk("Exchange pending...");
	 }

	 return err;
 }
 #endif /* CONFIG_BT_GATT_CLIENT */

 bool Connect_Status;

 void mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx)
 {
	 printk("Updated MTU: TX: %d RX: %d bytes\n", tx, rx);

	//  if (tx != 247) {
	// 	 mtu_exchange(conn);
	//  }
 }


 static void connected(struct bt_conn *conn, uint8_t err)
 {
	 //1、保存连接句柄
	 connect_handle =  conn;

	 char addr[BT_ADDR_LE_STR_LEN];
	 //2、保存连接到的设备地址
	 bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));//获取连接的远程地址并转成字符串

	 if (err) {
		 printk("Failed to connect to %s (%u)\n", addr, err);
		 return;
	 }
	 printk("Connected %s\n", addr);

	 // 3、主动上报连接状态给主控
	 //ble_to_mcu_report_link_state(0); // 0=连接   主动上报

	 //4、请求更新连接参数
	 const struct bt_le_conn_param *param;

	 param = BT_LE_CONN_PARAM(6, 6, 0, 100);

	 //向主机（如手机）请求更新当前连接的参数，通常用于降低延迟、提升响应速度或降低功耗
	 bt_conn_le_param_update(conn, param);
 }

 static void disconnected(struct bt_conn *conn, uint8_t reason)
 {
	 //1、保存断开的连接句柄
	 char addr[BT_ADDR_LE_STR_LEN];

	 ancs_c_conn = NULL;
	 bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	 //2、向MCU发送关屏幕的命令
	 ///////////向MCU发送关屏幕的命令///////////////
	 //char ble_disstatus[15]="BLE_DISCONNECT";
	   //TGT_SendMultiData((uint8_t*)ble_disstatus, 14);
	 //printk("close-screen1\n");
	 ///////////向MCU发送关屏幕的命令///////////////

	 // 3、主动上报断开状态给主控
	 //ble_to_mcu_report_link_state(1); // 1=断开   主动上报

	 //4、 断开连接
	 ANCS_FLAG = 2;
	 printk("Disconnected from %s (reason 0x%02x)\n", addr, reason);

	 //5.打印断开连接的原因
	switch (reason)
	{
		case BT_HCI_ERR_CONN_TIMEOUT://连接超时，通常是信号丢失或数据处理不过来
			printk("Disconnect reason: Connection timeout (possibly data overflow or signal loss)\n");
			break;
		case BT_HCI_ERR_REMOTE_USER_TERM_CONN://远端设备主动断开，比如主控或手机主动断开
			printk("Disconnect reason: Remote device terminated the connection\n");
			break;
		case BT_HCI_ERR_CONN_FAIL_TO_ESTAB://连接建立失败
			printk("Disconnect reason: Connection establishment failed\n");
			break;
		default:
			printk("Disconnect reason: Other (0x%02x)\n", reason);
			break;
	}
    //6.打印断开连接后，蓝牙是否仍在广播
	if (adv_is_open) {
		printk("BLE status: Disconnected, still advertising\n");
	} else {
		printk("BLE status: Disconnected, advertising stopped\n");
	}
 }

 static void security_changed(struct bt_conn *conn, bt_security_t level,
				  enum bt_security_err err)
 {
	 char addr[BT_ADDR_LE_STR_LEN];

	 bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	 if (!err) //err == 0，表示安全等级变更（如配对/加密）成功
	 {
		 ///////////向MCU发送开屏幕的命令///////////////
		 //char ble_constatus[8]="START50";
		 //TGT_SendMultiData((uint8_t*)ble_constatus, 7);
		 //printk("open-screen\n");
		 ///////////向MCU发送开屏幕的命令///////////////


		 ////////////////发现ANCS的命令//////////////
		 //ancs_c_conn = conn;
		 //ble_ancs_c_discovery_service(ancs_c_conn);
		 ////////////////发现ANCS的命令//////////////
	 }
	 else
	 {
		 ///////////向MCU发送关屏幕的命令///////////////
		 //char ble_disstatus[15]="BLE_DISCONNECT";
		 //TGT_SendMultiData((uint8_t*)ble_disstatus, 14);
		 //printk("close-screen2\n");
		 ///////////向MCU发送关屏幕的命令///////////////

		 /////2024.5.10新增断开连接/////
		 bt_conn_disconnect(connect_handle, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		 /////2024.5.10新增断开连接/////
		 //printk("Security failed: %s level %u err %d\n", addr, level, err);
	 }
 }

 // 回调函数
 static struct bt_gatt_cb gatt_callbacks = {
	 .att_mtu_updated = mtu_updated
 };

 // 连接回调 //注册连接相关事件回调函数的宏
 BT_CONN_CB_DEFINE(conn_callbacks) = {
	 .connected = connected,
	 .disconnected = disconnected,
	 .security_changed = security_changed
 };



 static void bt_ready(void)
 {
	 int err;

	 printk("Bluetooth initialized\n");

	 if (IS_ENABLED(CONFIG_SETTINGS)) {
		 settings_load();
	 }

	 if (err) {
		 printk("Advertising failed to start (err %d)\n", err);
		 return;
	 }

	 printk("Advertising successfully started\n");
 }


 int bt_ven_notify(uint8_t *data, uint32_t len)
 {
	 int rc;
	 uint8_t data_buf[VND_MAX_LEN];

	 memcpy(data_buf, data, len);

	 #if DEBUG_PRINT_ENABLE
	 // 打印发给APP的数据
	 //printk("--------------------------------------\r");
	 //user_print_auto("MCU send data to APP", data_buf, len);
	 #endif

	 rc = bt_gatt_notify(NULL, &nus_svc.attrs[1], &data_buf, len);

	 return rc == -ENOTCONN ? 0 : rc;
 }


 struct uart_config uart_cfg = {
	 .baudrate = BAUDRATE,
	 .parity = UART_CFG_PARITY_NONE,
	 .stop_bits = UART_CFG_STOP_BITS_1,
	 .data_bits = UART_CFG_DATA_BITS_8,
	 .flow_ctrl = UART_CFG_FLOW_CTRL_NONE
 };

 const struct device *uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart1));

 fmc_data g_fmc_config_data;

 ssize_t __weak z_impl_hwinfo_get_efuse_mac(uint8_t *buffer, size_t length)
 {
	 return -ENOSYS;
 }


 uint8_t  num_to_srting(uint32_t num, uint8_t *string_out)
 {
	 uint8_t i;
	 uint8_t length = 0;
	 uint32_t temp = num;

	 if (temp == 0) {
		 string_out[0] = '0';
		 return 1;
	 }
	 for (i = 0; temp > 0; i++) {
		 string_out[i] = temp % 10 + '0';
		 temp /= 10;
	 }
	 length = i;
	 for (i = 0; i < length / 2; i++) {
		 temp = string_out[i];
		 string_out[i] = string_out[length - 1 - i];
		 string_out[length - 1 - i] = temp;
	 }
	 return length;
 }

 uint8_t string_to_num(uint8_t *string_in, uint32_t *num, uint8_t length)
 {
	 uint8_t i;
	 uint32_t temp = 0;

	 for (i = 0; i < length; i++) {
		 temp *= 10;
		 if ((string_in[i] < '0') || (string_in[i] > '9')) {
			 return 0;
		 }
		 temp += (string_in[i] - '0');
	 }
	 *num = temp;
	 return 1;
 }

 void hex_to_string(uint8_t *buff_num, uint8_t length, uint8_t *buff_str_out)
 {
	 uint8_t temp;

	 for (uint8_t i = 0; i < length; i++) {
		 temp = buff_num[i] >> 4;
		 if (temp > 9) {
			 buff_str_out[(i << 1)] = temp + 'A' - 10;
		 } else {
			 buff_str_out[(i << 1)] = temp + '0' - 0;
		 }
		 temp = buff_num[i] & 0x0f;
		 if (temp > 9) {
			 buff_str_out[(i << 1) + 1] = temp + 'A' - 10;
		 } else {
			 buff_str_out[(i << 1) + 1] = temp + '0' - 0;
		 }
	 }
 }

 uint8_t mac_addr_str_to_hex(unsigned char *str_addr, unsigned char *hex_addr, unsigned int out_hex_length)
 {
	 unsigned char i, j;

	 for (i = 0, j = 0; i < out_hex_length; i++) {
		 if ((str_addr[j]  <= '9') && (str_addr[j]  >= '0')) {
			 hex_addr[i] = (str_addr[j] - '0') << 4;
		 } else if ((str_addr[j]  <= 'f') && (str_addr[j]  >= 'a')) {
			 hex_addr[i] = (str_addr[j] - 'a' + 10) << 4;
		 } else if ((str_addr[j]  <= 'F') && (str_addr[j]  >= 'A')) {
			 hex_addr[i] = (str_addr[j] - 'A' + 10) << 4;
		 } else {
			 /* hex_addr[i] = 0; */
			 return 0;
		 }
		 j++;
		 if ((str_addr[j]  <= '9') && (str_addr[j]  >= '0')) {
			 hex_addr[i] += ((str_addr[j] - '0') << 0);
		 } else if ((str_addr[j]  <= 'f') && (str_addr[j]  >= 'a')) {
			 hex_addr[i] += ((str_addr[j] - 'a' + 10) << 0);
		 } else if ((str_addr[j]  <= 'F') && (str_addr[j]  >= 'A')) {
			 hex_addr[i] += ((str_addr[j] - 'A' + 10) << 0);
		 } else {
			 /* hex_addr[i] += 0; */
			 return 0;
		 }
		 j++;
	 }
	 return 1;
 }

 void proj_uart_at_handle(uint8_t *data, uint16_t pack_size)
 {

	 unsigned char buff[128];
	 uint8_t num_buff[14];
	 uint8_t length;
	 uint32_t num = 0;

	 memcpy(buff, data, pack_size);

	 struct uart_config uart_cfg_check;

 }


 #define RX_BUFFER_SIZE             240
 #define TGT_UART                  UART1

 uint8_t RX_Sendata[RX_BUFFER_SIZE] = { 0 };
 uint16_t USART_RX_CNT;
 extern bool Connect_Status;
 extern UART_EventDef uart_panchip_evt_cache_get(UART_T *uart);

//notify接口转换函数
static void ble_notify_adapter(const char *data, uint16_t len)
{
	bt_ven_notify((uint8_t *)data, (uint32_t)len);
}

// 发送MAC地址到上位机
static void send_mac_to_uc(void)
{
    uint8_t reply[8] = {0xAA};
    bt_addr_le_t addr;
    size_t count = 1;
    bt_id_get(&addr, &count);
    // 反转MAC地址顺序
    for (int i = 0; i < 6; i++) {
        reply[1 + i] = addr.a.val[5 - i];
    }
    reply[7] = 0x55;
    user_print_auto("ble send Data to uc", (const uint8_t *)reply, sizeof(reply));
    TGT_SendMultiData(reply, sizeof(reply));
}

// 处理上位机串口指令
static void handle_UC_uart_cmd(const uint8_t *data, uint16_t len)
{
    // 简单判断：AA 10 55
    if (len == 3 && data[0] == 0xAA && data[1] == 0x10 && data[2] == 0x55)
	{
        send_mac_to_uc();
    }
}

 //----uart接收数据处理函数----
 void ble_hid_uart_irq_rx_ready(const struct device *dev)
 {
	 UART_T *uart_inst = (UART_T *)(*(uint32_t *)dev->config);
	 uint8_t rec_num;

	 //超时 说明一帧数据接收完成
	 if (uart_panchip_evt_cache_get(uart_inst) == UART_EVENT_TIMEOUT)
	 {
		 rec_num = TGT_UART->RFL;
		 for (uint8_t i = 0; i < rec_num; i++)
			 RX_Sendata[USART_RX_CNT++] = UART_ReceiveData(TGT_UART);

		// printk("Received data from MCU (hex, len=%d): ", USART_RX_CNT);
		// for (uint16_t i = 0; i < USART_RX_CNT; i++) {
		// 	printk("%02X ", RX_Sendata[i]);
		// }
		// printk("\r\n");
		 // 根据首字节区分协议版本: 0x53=V3协议, 其他=V2协议
		 if (USART_RX_CNT > 0 && RX_Sendata[0] == V3_FRAME_HEAD_0 && RX_Sendata[1] == V3_FRAME_HEAD_1)
		 {
			 V3_handle_mcu_data(RX_Sendata, USART_RX_CNT);
		 }
		 else
		 {
			 // 统一私有协议处理入口
			 Handle_protocol_data((const char *)RX_Sendata, USART_RX_CNT);

			 //处理上位机发送的查询mac地址的指令
			 handle_UC_uart_cmd(RX_Sendata, USART_RX_CNT);
		 }


		USART_RX_CNT = 0;
	 }

	 //数据到达事件处理，等待超时接收
	 if (uart_panchip_evt_cache_get(uart_inst) == UART_EVENT_DATA) {
		 rec_num = TGT_UART->RFL;
		 for (uint8_t i = 0; i < (rec_num - 1); i++) {
			 RX_Sendata[USART_RX_CNT++] = UART_ReceiveData(TGT_UART);
		 }
	 }
 }

 /*
  * fun:  uart_fifo_callback
  * param:   dev - device pointer
  *          user_data - user data pointer
  * return:  void
  * description:  UART FIFO callback function to handle data reception.
  */
 static void uart_fifo_callback(const struct device *dev, void *user_data)
 {
	 ARG_UNUSED(user_data);
	 uart_running = true;

	 /* Verify uart_irq_update() */
	 if (!uart_irq_update(dev)) {
		 printk("retval should always be 1\n");
		 return;
	 }

	 ble_hid_uart_irq_rx_ready(dev);

	 uart_running = false;
 }

 void __ramfunc clock_control_before_wfi(void)
 {
	 /* Enable RCH */
	 CLK->CLK_TOP_CTRL |= CLK_TOPCTL_RCH_EN_Msk;
	 /* Wait for stable */
	 while (!(CLK->RCH_CTRL & CLK_STABLE_STATUS_Msk)) {
		 /* Busy wait */
		 __NOP();
	 }

	 /* Change Sysclock to RCH */
	 CLK->CLK_TOP_CTRL = (CLK->CLK_TOP_CTRL & (~CLK_TOPCTL_SYS_CLK_SEL_Msk)) | CLK_SYS_SRCSEL_RCH;

	 /* Disable XTH */
	 CLK->CLK_TOP_CTRL &= ~CLK_TOPCTL_XTH_EN_Msk;
 }

 void __ramfunc clock_control_after_wfi(void)
 {
	 /* Enable XTH */
	 CLK->CLK_TOP_CTRL |= CLK_TOPCTL_XTH_EN_Msk;
	 /* Wait for stable */
	 while (!(CLK->XTH_CTRL & CLK_STABLE_STATUS_Msk)) {
		 /* Busy wait */
		 __NOP();
	 }

	 /* Change Sysclock to XTH */
	 CLK->CLK_TOP_CTRL = (CLK->CLK_TOP_CTRL & (~CLK_TOPCTL_SYS_CLK_SEL_Msk)) | CLK_SYS_SRCSEL_XTH;

	 /* Disable RCH */
	 CLK->CLK_TOP_CTRL &= ~CLK_TOPCTL_RCH_EN_Msk;
 }
 #define DBG_PIN 0
 __ramfunc bool k_idle_thread_hook(void)
 {
	 if (!uart_running) {
		 __disable_irq();
		 #if DBG_PIN
		 P30 = 1;
		 #endif
		 CLK->CLK_TOP_CTRL = (CLK->CLK_TOP_CTRL & (~(CLK_TOPCTL_AHB_DIV_Msk | CLK_TOPCTL_APB2_DIV_Msk)))
					 | (11 << CLK_TOPCTL_AHB_DIV_Pos) | (0 << CLK_TOPCTL_APB2_DIV_Pos);
		 #if DBG_PIN
		 P30 = 0;
		 P30 = 1;
		 #endif
		 __WFI();

		 #if DBG_PIN
		 P30 = 0;
		 P30 = 1;
		 #endif

		 CLK->CLK_TOP_CTRL = (CLK->CLK_TOP_CTRL & (~(CLK_TOPCTL_AHB_DIV_Msk |  CLK_TOPCTL_APB2_DIV_Msk)))
					 | (0 << CLK_TOPCTL_AHB_DIV_Pos) | (6 << CLK_TOPCTL_APB2_DIV_Pos);
		 #if DBG_PIN
		 P30 = 0;
		 #endif
		 __enable_irq();
		 return true;
	 } else {
		 return false;
	 }

 }

 //////////////////////////publicmac////////////////////////////
 // uint8_t user_defined_pub_addr[6] = {0x29, 0x00, 0x00, 0x00, 0x00, 0xc1};
 // void bt_set_user_public_mac(uint8_t *addr)
 // {
 // 	memcpy(user_defined_pub_addr, addr, 6);
 // }
 //////////////////////////publicmac////////////////////////////

 void load_config(void)
 {
	 flash_device =  device_get_binding(DT_CHOSEN_ZEPHYR_FLASH_CONTROLLER_LABEL);

	 if (flash_device) {
		 printk("Flash I/O commands can be run.\n");
	 }

	 memset(g_fmc_config_data.Blue_mac, 0, sizeof(g_fmc_config_data));
 }

 void uart_at_init(void)
 {
	 uart_configure(uart_dev, &uart_cfg);

	 uart_irq_rx_disable(uart_dev);

	 /* Verify uart_irq_callback_set() */
	 uart_irq_callback_set(uart_dev, uart_fifo_callback);

	 uart_irq_rx_enable(uart_dev);
 }

 // 设置自定义MAC地址（public address）
void set_custom_mac(void)
{
    bt_addr_le_t addr = {
        .type = BT_ADDR_LE_RANDOM,
        .a = { {0xC0, 0xDE, 0x12, 0x34, 0x56, 0x78} } // 你的自定义MAC
    };
    int err = bt_id_create(&addr, NULL);
	printk("bt_id_create return: %d\n", err);
    if (err < 0) {
        printk("Failed to set custom MAC address (err %d)\n", err);
    } else {
        printk("Custom MAC address set successfully\n");
    }
}
 // BLE初始化函数
 void ble_init(void)
 {
	 uint16_t uid;
	 int err;
	 char ble_name[30];
	 uint8_t mac_data[] ={0x00,0x00,0x00,0x00,0x00,0x00};
	 uint8_t num_buff[14];

	 // 1. 生成蓝牙设备名（带MAC地址后缀）
	 mac_data[0] =  otp.m.mac_addr[5];
	 mac_data[1] =  otp.m.mac_addr[4];
	 mac_data[2] =  otp.m.mac_addr[3];
	 mac_data[3] =  otp.m.mac_addr[2];
	 mac_data[4] =  otp.m.mac_addr[1];
	 mac_data[5] =  otp.m.mac_addr[0];
	 //sprintf(ble_name,"Smark_%02X%02X%02X",mac_data[0],mac_data[1],mac_data[2]);
	 //sprintf(ble_name,"Decoder_test_v001");
	 sprintf(ble_name,"Keya_Bt_Dev_%02X%02X%02X%02X%02X%02X",mac_data[0],mac_data[1],mac_data[2],mac_data[3],mac_data[4],mac_data[5]);

	 //设置用户自定义MAC地址
	 //set_custom_mac();

	 // 2. 启动蓝牙协议栈
	 err = bt_enable(NULL);

	 // 3. 设置蓝牙设备名
	 bt_set_name(ble_name);

	 //4. 初始化开关广播等参数
	 k_work_init(&adv_stop, adv_stop_work);
	 k_work_init(&adv_start, adv_start_work);
	 k_work_init(&rssi_work, rssi_work_handler);

	 // 5. 协议栈初始化完成后处理（如启动广播等）
	 bt_ready();

	 // 6. 注册GATT回调（如MTU变化等）
	 bt_gatt_cb_register(&gatt_callbacks);

	 // 7. 打开广播  //新增：默认打开广播
	 int adv_ret = app_ble_adv_open();

	 // 8. 通知主控已准备就绪，可以发送命令了，同步信息
	 ble_notify_ready_status(err, adv_ret);
	 printk("INIT is ready, you can send commands now.\n");
	 // 9. 初始化成功后设置标志
	 if (err == 0 && adv_ret == 1) {
		 ble_init_ok = true;
	 } else {
		 ble_init_ok = false;
	 }

	 printk("Bluetooth is ready, you can send commands now.\n");
	 k_msleep(10); // 增加短暂延时，保证BLE初始化后时序稳定
 }
 //---------------协议相关回调函数注册-----------------------

 // 广播关闭的实际处理函数
 void adv_stop_work(struct k_work *work)
 {
	 int err = bt_le_adv_stop();
	 if (err) {
		 printk("bt_le_adv_stop failed: %d\n", err);
	 } else {
		 printk("Advertising stopped successfully\n");
	 }
	 adv_is_open = false;
	 adv_busy = false;
 }

 // 打开广播，带状态判断
 static int app_ble_adv_open(void)
 {
	 if (adv_is_open) {
		 printk("Advertising already started\n");
		 return 2; // 已经打开
	 }
	 k_work_submit(&adv_start);
	 return 1;
 }
 // 关闭广播，带状态判断
 static int app_ble_adv_close(void)
 {
	 if (adv_busy) {
		 printk("Advertising stop is busy\n");
		 return 0;
	 }
	 adv_busy = true;

	 struct bt_le_ext_adv *adv = bt_le_adv_lookup_legacy();
	 if (!adv || !atomic_test_bit(adv->flags, BT_ADV_ENABLED)) {
		 printk("Advertising already stopped (stack)\n");
		 adv_is_open = false;
		 adv_busy = false;
		 return 2;
	 }

	 // 只提交异步任务，不直接调用 bt_le_adv_stop
	 k_work_submit(&adv_stop);
	 return 1;
 }

 // 封装BLE就绪状态通知函数  //蓝牙芯片协议栈和广播均初始化成功后，向主控发送通知，用于连个芯片间的同步
 static void ble_notify_ready_status(int ble_err, int adv_ret)
 {
	char ready_msg[32];
	if (ble_err == 0 && adv_ret == 1)
	{
		//snprintf(ready_msg, sizeof(ready_msg), "%s", CMD_BLE_READY); // 拷贝就绪字符串
		strcpy(ready_msg, CMD_BLE_READY);
	}
	else
	{
		//snprintf(ready_msg, sizeof(ready_msg), "%s", CMD_BLE_NOT_READY); // 拷贝未就绪字符串
		strcpy(ready_msg, CMD_BLE_NOT_READY);
	}
	//先注释掉  暂时不用发送给主控
	//TGT_SendMultiData((const uint8_t *)ready_msg, strlen(ready_msg)); // 发送未就绪状态
	printk("BLE ready status sent to MCU: %s\n", ready_msg);

 }

 uint16_t get_conn_handle(struct bt_conn *conn)
 {
	 if (!conn) return 0xFFFF;
	 return conn->handle;
 }
 // 获取当前连接的RSSI
 int get_conn_rssi(struct bt_conn *conn, int *rssi)
 {
	 if (!conn || !rssi) return -EINVAL;

	 // 可加互斥保护，防止重入
	 static bool rssi_busy = false;
	 if (rssi_busy) return -EBUSY;
	 rssi_busy = true;


	 uint16_t handle = conn->handle;
	 if (handle == 0xFFFF) return -ENOTCONN;

	 struct net_buf *buf, *rsp;
	 struct bt_hci_cp_read_rssi *cp;
	 struct bt_hci_rp_read_rssi *rp;
	 int err;

	 buf = bt_hci_cmd_create(BT_HCI_OP_READ_RSSI, sizeof(*cp));
	 if (!buf) return -ENOMEM;
	 cp = net_buf_add(buf, sizeof(*cp));
	 cp->handle = sys_cpu_to_le16(handle);

	 err = bt_hci_cmd_send_sync(BT_HCI_OP_READ_RSSI, buf, &rsp);

	 rssi_busy = false;

	 if (err) {
		 net_buf_unref(buf);
		 return err;
	 }

	 rp = (struct bt_hci_rp_read_rssi *)rsp->data;
	 *rssi = (int8_t)rp->rssi;
	 net_buf_unref(rsp);

	 return 0;
 }

//定义级别与RSSI阈值的映射表
static const int rssi_threshold_table[6] = { -50, -60, -70, -80, -85, -90 };
static int current_rssi_threshold = -70; // 默认级别

// 例如 app 发送 MCU|DIST|3，表示选择第3级
void handle_app_distance_cmd(int level)
{
    if (level >= 0 && level <= 5)
	{
        current_rssi_threshold = rssi_threshold_table[level];
    }
}

// rssi_work_handler 实现  //函数功能查询当前的RSSI值，是否达到阈值，超过则说明达到解锁条件了，可以解锁。
static void rssi_work_handler(struct k_work *work)
{
    int rssi = 0;
    const char *data_val = "FAR";
    if (get_conn_rssi(connect_handle, &rssi) == 0)
	{
        if (rssi >= current_rssi_threshold)
		{
            data_val = "NEAR";
        }
    }
    char reply_buf[32];
    snprintf(reply_buf, sizeof(reply_buf), "BLE|GTH|ASK|%s", data_val);
    ble_send_data(reply_buf, strlen(reply_buf));
}


 void ble_send_data(const char *data, uint16_t len)
 {
	 // 发送数据到UART
	 TGT_SendMultiData((const uint8_t *)data, len);
	 // 可加打印
	 //printf("==== BLE TX ====\n");
	 //printk("len is : %d\n", len);
	 #if DEBUG_PRINT_ENABLE
	 log_debug("BLE send data: %.*s\n\r", len,data);
	 log_debug("-------------------------\n");
	 #endif
 }

 /*
  * fun: protocol_fun_init
  * param:  void
  * return: void
  * description:  初始化协议相关回调函数
  */
 void protocol_callbackfun_init(void)
 {
	 // 注册发送函数
	 ble_to_mcu_register_send_func((ble_protocol_send_func_t)ble_send_data);
	 //ble_uart1_register_send_callback(ble_send_data);
 }

static struct k_timer handshake_timer;

static void handshake_timer_cb(struct k_timer *timer)
{
	//printk("handshake_timer_cb called at %d ms\n", k_uptime_get_32());
    if (!g_controler_state.handshake_ok) //// 定时10s发送握手包，直到握手成功
	{
        if (handshake_retry_count < HANDSHAKE_MAX_RETRY)
		{
            BLE_send_protocol_cmd(BLE_CMD_HANDSHAKE_RSP); // 定时发送握手包
            handshake_retry_count++;
        }
		else
		{
            // 超过最大重试次数，判定失败
            set_led_state(LED_STATE_RED_FAST_BLINK); // 红灯快闪 0.5s
            k_timer_stop(&handshake_timer);
            printk("Handshake failed after max retries\n");
        }
    }
	else
	{
		//BLE_send_protocol_cmd(BLE_CMD_UNLOCK_STATUS_RSP); // 握手成功后，查询一次车辆的解锁激活状态
        k_timer_stop(&handshake_timer); // 握手成功，停止定时器
    }
}


static struct k_timer check_timeout_timer; //定时检查主控是否按时回复

void check_timeout_timer_cb(struct k_timer *timer) //对app的指令的处理
{
    uint32_t now = k_uptime_get_32();

    V3_timeout_check(now);
    for (int i = 1; i < 5; i++)
    {
        if (cmd_wait[i].waiting && (now - cmd_wait[i].send_time_ms > 198))
        {
            cmd_wait[i].waiting = false;
            cmd_wait[i].timeout_state = true;//超时
			printk("MCU reply for cmd %d is timeout (no response)\n", i);

			#if (!BLE_CONTROLER_SIMU_ENABLED)
			//处理超时逻辑，比如解锁指令，2秒内未回复则认为解锁失败。
            switch (i)
			{
				case BLE_CMD_HANDSHAKE_RSP:
					// 处理握手超时
					// 蓝牙与主控握手超时，不需要特殊处理
					//BLE_send_protocol_cmd发送蓝牙与app的握手指令时，只在上电后，蓝牙主动发送的，
					//不是app发送握手指令时，蓝牙才发送与主控握手的指令，所以check_timeout_timer_cb下
					//的case BLE_CMD_HANDSHAKE_RSP:的处理就是个特殊的。
					break;
				case BLE_CMD_UNLOCK_STATUS_RSP: //解锁状态查询指令
					reply_app_status(APP_CMD_QUERY_UNLOCK_RSP, 0x01);//0x03--其他原因查询失败 0x00--已解锁 0x01--已锁定
					printk("UNLOCK_STATUS cmd is no response\n");
					break;
				case BLE_CMD_UNLOCK_RSP: //解锁指令
					reply_app_status(APP_CMD_UNLOCK_RSP, 0x01);//超时未回复，则返回失败
					printk("UNLOCK cmd is no response\n");
					break;
				case BLE_CMD_LOCK_RSP: //锁定指令
					reply_app_status(APP_CMD_LOCK_RSP, 0x01);//超时未回复，则返回失败
					printk("LOCK cmd is no response\n");
					break;
				case BLE_CMD_FAULT_QUERY_RSP: //故障查询指令
				{
					// 超时未回复，默认无故障，回复AA 05 00 00 55
					uint8_t reply[5] = {0xAA, APP_CMD_FAULT_QUERY_RSP, 0x00, 0x00, 0x55};
					bt_ven_notify(reply, 5);
					set_led_state(LED_STATE_GREEN_ON); //绿灯 无故障
					printk("FAULT_QUERY cmd is no response, default no fault\n");
				}
					break;
			}
			#endif

			// 蓝牙和控制器模拟通信使能
			#if BLE_CONTROLER_SIMU_ENABLED
            // 这里直接处理超时逻辑，比如通知APP
				switch (i)
				{
					case BLE_CMD_HANDSHAKE_RSP:
						// 处理握手超时
						//reply_app_status(APP_CMD_HANDSHAKE_RSP, 0x02); // 0x02=超时
						break;
					case BLE_CMD_UNLOCK_STATUS_RSP:                 //模拟时，默认查询为上锁或者解锁
						reply_app_status(APP_CMD_QUERY_UNLOCK_RSP, 0x01);//0x03--其他原因查询失败
						printk("UNLOCK_STATUS cmd is response\n");
						break;
					case BLE_CMD_UNLOCK_RSP:
						reply_app_status(APP_CMD_UNLOCK_RSP, 0x00);
						printk("UNLOCK cmd is response\n");
						break;
					case BLE_CMD_LOCK_RSP:
						reply_app_status(APP_CMD_LOCK_RSP, 0x00);
						printk("LOCK cmd is response\n");
						break;
					case BLE_CMD_FAULT_QUERY_RSP: //故障查询指令
					{
						// 测试回复，回复AA 05 00 00 55
						uint8_t reply[5] = {0xAA, APP_CMD_FAULT_QUERY_RSP, 0x01, 0x01, 0x55};
						bt_ven_notify(reply, 5);
						//printk("FAULT_QUERY cmd is no response, default no fault\n");
					}
					break;
				}
			#endif
        }
    }
}

//----------for test-------------
// void check_timeout_timer_cb(struct k_timer *timer)
// {
//     uint32_t now = k_uptime_get_32();
//     for (int i = 0; i < 4; i++)
//     {
//         if (cmd_wait[i].waiting)
//         {
//             cmd_wait[i].waiting = false;
//             cmd_wait[i].timeout_state = true;
//             // 这里直接处理超时逻辑，比如通知APP
//             switch (i)
// 			{
//                 case BLE_CMD_HANDSHAKE_RSP:
//                     // 处理握手超时
//                     //reply_app_status(APP_CMD_HANDSHAKE_RSP, 0x02); // 0x02=超时
//                     break;
//                 case BLE_CMD_UNLOCK_STATUS_RSP:
//                     reply_app_status(APP_CMD_QUERY_UNLOCK_RSP, 0x01);//0x00--已解锁 0x01--已锁定 0x03--其他原因查询失败
//                     break;
//                 case BLE_CMD_UNLOCK_RSP:
//                     reply_app_status(APP_CMD_UNLOCK_RSP, 0x00);
//                     break;
//                 case BLE_CMD_LOCK_RSP:
//                     reply_app_status(APP_CMD_LOCK_RSP, 0x00);
//                     break;
//             }
//             printk("MCU reply for cmd %d \n", (i==1 ? BLE_CMD_UNLOCK_STATUS_RSP : (i==2 ? BLE_CMD_UNLOCK_RSP : BLE_CMD_LOCK_RSP)));
//         }
//     }
// }



void main(void)
{
	load_config();//初始化 Flash 设备和配置数据

	uart_at_init();//初始化串口 UART1 波特率9600

	user_parameter_init();//初始化用户参数

	user_led_init();//初始化LED // 初始化双色灯，只让绿灯亮
	//protocol_callbackfun_init();//与私有协议相关回调函数注册

	ble_init();  //初始化BLE


	/******************  OTA SERVICE  *******************/
	#if OTA_ENABLED
	#ifdef CONFIG_MCUMGR_CMD_OS_MGMT
		os_mgmt_register_group();
	#endif
	#ifdef CONFIG_MCUMGR_CMD_IMG_MGMT
		img_mgmt_register_group();
	#endif
	#ifdef CONFIG_MCUMGR_CMD_STAT_MGMT
		stat_mgmt_register_group();
	#endif
	#ifdef CONFIG_MCUMGR_SMP_BT
		smp_bt_register();
	#endif
	#endif
	/*****************************************************/


	//Zephyr 默认 CONFIG_SYS_CLOCK_TICKS_PER_SEC=100，即10ms一个tick。
	printk("Timer is set enter \n");
	//初始化和启动定时器进行握手包指令的定时发送
	k_timer_init(&handshake_timer, handshake_timer_cb, NULL);
	k_timer_start(&handshake_timer, K_NO_WAIT, K_MSEC(1000)); //10秒定时发送一次

	// 初始化和启动定时器进行检查超时的定时发送 2秒定时
	k_timer_init(&check_timeout_timer, check_timeout_timer_cb, NULL);
	k_timer_start(&check_timeout_timer, K_NO_WAIT, K_MSEC(200)); // 2秒周期
	printk("Timer is set ready \n");


	#if BLE_CONTROLER_SIMU_ENABLED   // 等于1说明打开模拟，那么下面的要不使用
		g_controler_state.handshake_ok = true;
		k_timer_stop(&handshake_timer); // 如果有握手定时器
		printk("模拟握手成功，蓝牙已准备好\n");
	#endif
}

 //////////间隔在43s/////////左右/////////K_SECONDS(5)=K_MSEC(5000)
