#ifndef BLE_OTA_H
#define BLE_OTA_H

#include <stdint.h>
#include <stdbool.h>

/* OTA Command Codes */
#define OTA_CMD_START             0x06
#define OTA_CMD_FLASH_ERASE_DONE  0x07
#define OTA_CMD_DATA              0x08
#define OTA_CMD_DATA_RETRY	  0x0B
#define OTA_CMD_ERASE_RETRY       0x0C

/* OTA Frame Sizes */
#define OTA_FRAME_START_REQ_SIZE  12
#define OTA_FRAME_START_RSP_SIZE  5
#define OTA_FRAME_FLASH_ERASE_RSP_SIZE  5
#define OTA_FRAME_DATA_ACK_SIZE   7
#define OTA_FRAME_DATA_MIN_SIZE   8
#define OTA_FRAME_DATA_MAX_SIZE   208

/* OTA Data Constraints */
#define OTA_DATA_MAX_CONTENT      200

/* OTA Result Codes */
#define OTA_RESULT_SUCCESS        0x01
#define OTA_RESULT_FAIL           0x02
#define OTA_ACK_CONTINUE          0x01
#define OTA_ACK_RESEND            0x02
#define OTA_ACK_FAIL              0x03

/* OTA EEPROM Base Address (configurable) */
#define OTA_EEPROM_BASE_ADDR      0x0000

/* Max resend retries before abort */
#define OTA_MAX_RESEND_COUNT      10

/* OTA State */
typedef enum {
	OTA_STATE_IDLE = 0,
	OTA_STATE_START_RECEIVED,
	OTA_STATE_ERASING,
	OTA_STATE_RECEIVING,
	OTA_STATE_COMPLETE,
	OTA_STATE_ERROR,
} OtaState;

/* OTA Context */
typedef struct {
	OtaState state;
	uint32_t file_length;			//文件总长度
	uint32_t file_crc;
	uint32_t received_length;
	uint32_t running_crc;			//总crc校验
	uint16_t expected_seq;			//包序号
	uint32_t eeprom_addr;			//eeprom地址
	uint8_t  resend_count;			//重发次数
	uint32_t crc_partial_word;		//包crc字节
	uint8_t  crc_partial_shift;		//包crc偏移量
} OtaContext;

/* Public API */
void Ota_init(void);
void OtaHandleStartRequest(const uint8_t *frame);
void OtaHandleDataPacket(const uint8_t *frame, uint16_t frame_len);
void OtaReset(void);
void OtaPackageRetry(const uint8_t *frame);
void OtaEraseRetry(const uint8_t *frame);
bool OtaIsInProgress(void);



#endif /* BLE_OTA_H */
