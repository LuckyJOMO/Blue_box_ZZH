#ifndef MCU_TO_BLE_PROTOCOL_V3_H
#define MCU_TO_BLE_PROTOCOL_V3_H
#include <stdint.h>
#include <stdbool.h>

#define V3_FRAME_HEAD_0 0x53
#define V3_FRAME_HEAD_1 0x35

/* Frame sizes */
#define V3_FRAME_SIZE_HANDSHAKE 	9 /* 53 35 01 R0-R3(4) CRC(2) */
#define V3_FRAME_SIZE_NORMAL 		20 /* 53 35 CMD GRP DATA(14) CRC(2) */
#define V3_FRAME_SIZE_ERROR 		7 /* 53 35 ECMD GRP STA CRC(2) */

/* CRC payload byte counts (from byte index 2 = after 53 35 header) */
#define V3_CRC_COUNT_HANDSHAKE 		5 /* CMD(01) + 4 random bytes */
#define V3_CRC_COUNT_NORMAL 		16 /* CMD + GRP + 14 data bytes */
#define V3_CRC_COUNT_ERROR 		3 /* ECMD(1F/10) + GRP + STA */

/* CRC byte offsets within frame (CL stored first, then CH) */
#define V3_CRC_OFFSET_HANDSHAKE 	7 /* CL=byte[7], CH=byte[8] */
#define V3_CRC_OFFSET_NORMAL 		18 /* CL=byte[18], CH=byte[19] */
#define V3_CRC_OFFSET_ERROR 		5 /* CL=byte[5], CH=byte[6] */

/* MCU恢复等待时间200ms */
#define V3_MCU_REPLY_TIMEOUT_MS 200

typedef enum {
	V3_CMD_WRITE_CTRL = 0x00, /* Write controller */
	V3_CMD_HANDSHAKE = 0x01, /* Handshake */
	V3_CMD_READ_CTRL = 0x0F, /* Read controller */
	V3_CMD_WRITE_ERR = 0x10, /* Write error reply (BLE generated) */
	V3_CMD_READ_ERR = 0x1F, /* Read error reply (BLE generated) */
} v3_cmd_t;

typedef enum {
	V3_ERR_CRC = 0x01, /* CRC check failed */
	V3_ERR_MCU_NO_REPLY = 0x02, /* MCU did not reply within timeout */
} v3_error_code_t;

typedef struct {
	bool waiting_mcu_reply;
	uint8_t pending_cmd;
	uint8_t pending_grp;
	uint32_t send_time_ms;
} v3_bridge_state_t;

static v3_bridge_state_t v3_state;

void V3_handle_app_data(const uint8_t *data, uint16_t len);
void V3_handle_mcu_data(const uint8_t *data, uint16_t len);
void V3_timeout_check(uint32_t now_ms);


extern int bt_ven_notify(uint8_t *data, uint32_t len);
extern void ble_send_data(const char *data, uint16_t len);

#endif /* MCU_TO_BLE_PROTOCOL_V3_H */
