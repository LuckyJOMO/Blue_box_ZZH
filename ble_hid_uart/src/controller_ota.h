#ifndef CONTROLLER_OTA_H
#define CONTROLLER_OTA_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "i2c_eeprom.h"

/* ── EEPROM layout (last page, 256 bytes at ) ──────────── */
#define CTRL_OTA_FLAG_ADDR      (EEPROM_SIZE - EEPROM_PAGE_SIZE)
#define CTRL_OTA_META_FILE_LEN  (CTRL_OTA_FLAG_ADDR + 1)
#define CTRL_OTA_META_FILE_CRC  (CTRL_OTA_FLAG_ADDR + 5)

#define CTRL_OTA_FLAG_READY     0x01
#define CTRL_OTA_FLAG_IDLE      0x00

/* ── Protocol frame header ──────────────────────────────────── */
#define CTRL_OTA_HEAD_BYTE      0xEB
#define CTRL_OTA_HEAD_BYTE2     0x90
#define CTRL_OTA_HDR_LEN        6


/* ── Downlink commands (BLE → Controller) ──────────────────── */
#define CTRL_CMD_WRITE_REQ      0x06
#define CTRL_CMD_VERIFY_REQ     0x05
#define CTRL_CMD_DATA           0x2B

/* ── BLE → APP notification commands (V3 frame) ───────────── */
#define CTRL_OTA_V3_PROGRESS     0x09   /* 53 35 09 05 XX */
#define CTRL_OTA_V3_RESULT       0x0A   /* 53 35 0A 05 XX */
#define CTRL_OTA_RESULT_OK       0x01
#define CTRL_OTA_RESULT_ERROR    0x02
#define CTRL_OTA_RESULT_FAILED	 0x03

/* ── Uplink commands (Controller → BLE) ────────────────────── */
#define CTRL_CMD_REQ_CONFIRM    0x07
#define CTRL_CMD_FLASH_ERASE_OK 0x17
#define CTRL_CMD_DATA_ACK       0x0B
#define CTRL_CMD_RECV_COMPLETE  0x08
#define CTRL_CMD_CRC_SUCCESS    0x0C
#define CTRL_CMD_CRC_FAIL       0x0D
#define CTRL_CMD_CRC_ERROR      0x09
#define CTRL_CMD_LEN_ERROR      0x0A

/* ── Frame sizes ────────────────────────────────────────────── */
#define CTRL_REQ_FRAME_LEN      16   /* HDR(6)+LEN(1)+CMD(1)+fileLen(4)+fileCrc(4) */
#define CTRL_RSP_FRAME_LEN      8    /* HDR(6)+LEN(1)+CMD(1) */
#define CTRL_DATA_HDR_OVERHEAD  10   /* HDR(6)+LEN(1)+CMD(1)+hasMore(1)+CRC(1) */
#define CTRL_DATA_MAX_PAYLOAD   200

/* ── State machine ──────────────────────────────────────────── */
typedef enum {
	CTRL_OTA_IDLE = 0,
	CTRL_OTA_SEND_REQUEST,
	CTRL_OTA_WAIT_CONFIRM,
	CTRL_OTA_SEND_DATA,
	CTRL_OTA_WAIT_DATA_ACK,
	CTRL_OTA_WAIT_COMPLETE,
	CTRL_OTA_COMPLETE,
	CTRL_OTA_ERROR,
	CTRL_OTA_FILED,
} CtrlOtaState;

/* ── Public API ─────────────────────────────────────────────── */
void Ctrl_ota_init(void);
void Ctrl_ota_process(void);
bool Ctrl_ota_is_active(void);
void Ctrl_ota_handle_reply(uint8_t cmd);
void Ctrl_ota_setota_readyflag(uint32_t file_length, uint32_t file_crc);

extern void TGT_SendMultiData(const uint8_t *data, size_t size);
extern int bt_ven_notify(uint8_t *data, uint32_t len);

#endif /* CONTROLLER_OTA_H */
