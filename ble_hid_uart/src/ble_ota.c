#include "ble_ota.h"
#include "mcu_to_ble_protocol_v3.0.h"
#include "i2c_eeprom.h"
#include "controller_ota.h"
#include <string.h>
#include <sys/printk.h>

extern int bt_ven_notify(uint8_t *data, uint32_t len);

static OtaContext g_ota_ctx;

/* ── CRC functions ─────────────────────────────────────────────── */

static uint32_t OtaFileCrcCalculate(const uint8_t *content, uint32_t len)
{
	uint32_t crc = 0;
	uint8_t  buf[4];

	for (uint32_t i = 0; i < len; i += 4) {
		uint32_t data;
		if (i + 4 <= len)
		{
			data = *(const uint32_t *)(content + i);
		}
		else
		{
			buf[0] = buf[1] = buf[2] = buf[3] = 0xFF;
			for (uint32_t j = 0; i + j < len; j++) {
				buf[j] = content[i + j];
			}
			data = *(const uint32_t *)buf;
		}
		crc ^= data;
	}
	return crc;
}

static uint8_t Ota_Packet_CrcCalculate(const uint8_t *data, uint32_t len)
{
	uint8_t crc = 0;
	for (uint32_t i = 0; i < len; i++)
	{
		crc ^= data[i];
	}
	return crc;
}

/* ── Response builders ─────────────────────────────────────────── */

static void OtaSendStartResponse(uint8_t result)
{
	uint8_t frame[OTA_FRAME_START_RSP_SIZE];
	frame[0] = V3_FRAME_HEAD_0;
	frame[1] = V3_FRAME_HEAD_1;
	frame[2] = OTA_CMD_START;
	frame[3] = OTA_FRAME_START_RSP_SIZE;
	frame[4] = result;
	bt_ven_notify(frame, OTA_FRAME_START_RSP_SIZE);
	printk("[OTA] Start response sent, result=0x%02X\n", result);
}

static void OtaSendFlashEraseDone(uint8_t result)
{
	uint8_t frame[OTA_FRAME_FLASH_ERASE_RSP_SIZE];
	frame[0] = V3_FRAME_HEAD_0;
	frame[1] = V3_FRAME_HEAD_1;
	frame[2] = OTA_CMD_FLASH_ERASE_DONE;
	frame[3] = OTA_FRAME_FLASH_ERASE_RSP_SIZE;
	frame[4] = result;
	bt_ven_notify(frame, OTA_FRAME_FLASH_ERASE_RSP_SIZE);
	printk("[OTA] Flash erase done sent, result=0x%02X\n", result);
}

static void OtaSendDataAck(uint16_t seq, uint8_t result)
{
	uint8_t frame[OTA_FRAME_DATA_ACK_SIZE];
	frame[0] = V3_FRAME_HEAD_0;
	frame[1] = V3_FRAME_HEAD_1;
	frame[2] = OTA_CMD_DATA;
	frame[3] = OTA_FRAME_DATA_ACK_SIZE;
	frame[4] = (uint8_t)(seq & 0xFF);
	frame[5] = (uint8_t)((seq >> 8) & 0xFF);
	frame[6] = result;
	bt_ven_notify(frame, OTA_FRAME_DATA_ACK_SIZE);
	printk("[OTA] Data ACK seq=%u result=0x%d\n", seq, result);
}

static void OtaSendPackageRetryAck(uint8_t result)
{
	uint8_t frame[OTA_FRAME_FLASH_ERASE_RSP_SIZE];
	frame[0] = V3_FRAME_HEAD_0;
	frame[1] = V3_FRAME_HEAD_1;
	frame[2] = OTA_CMD_DATA_RETRY;
	frame[3] = OTA_FRAME_FLASH_ERASE_RSP_SIZE;
	frame[4] = result;
	bt_ven_notify(frame, OTA_FRAME_DATA_ACK_SIZE);
	printk("[OTA] Retry result=0x%02X\n", result);
}

static void OtaSendEraseRetryAck(uint8_t result)
{
	uint8_t frame[OTA_FRAME_FLASH_ERASE_RSP_SIZE];
	frame[0] = V3_FRAME_HEAD_0;
	frame[1] = V3_FRAME_HEAD_1;
	frame[2] = OTA_CMD_ERASE_RETRY;
	frame[3] = OTA_FRAME_FLASH_ERASE_RSP_SIZE;
	frame[4] = result;
	bt_ven_notify(frame, OTA_FRAME_DATA_ACK_SIZE);
	printk("[OTA] Erase Retry result=0x%02X\n", result);
}
/* ── Incremental file CRC helpers ───────────────────────────────── */

static void OtaFileCrcReset(void)
{
	g_ota_ctx.running_crc = 0;
	g_ota_ctx.crc_partial_word = 0;
	g_ota_ctx.crc_partial_shift = 0;
}

/*
data:文件包
len:文件包长度
OtaFileCrcUpdate:对整个文件包进行CRC校验计算，便于后续与整个文件的CRC作对比，验证准确性
*/
static void OtaFileCrcUpdate(const uint8_t *data, uint32_t len)
{
	uint32_t *p = &g_ota_ctx.crc_partial_word;
	uint8_t  *s = &g_ota_ctx.crc_partial_shift;

	for (uint32_t i = 0; i < len; i++)
	{
		*p |= (uint32_t)data[i] << *s;
		*s += 8;
		if (*s == 32) {
			g_ota_ctx.running_crc ^= *p;
			*p = 0;
			*s = 0;
		}
	}
}

static uint32_t OtaFileCrcFinalize(void)
{
	uint32_t *p = &g_ota_ctx.crc_partial_word;
	uint8_t  *s = &g_ota_ctx.crc_partial_shift;

	if (*s > 0)
	{
		while (*s < 32)
		{
			*p |= (uint32_t)0xFF << *s;
			*s += 8;
		}
		g_ota_ctx.running_crc ^= *p;
		*p = 0;
		*s = 0;
	}
	return g_ota_ctx.running_crc;
}

/* ── Public API ────────────────────────────────────────────────── */

void Ota_init(void)
{
	memset(&g_ota_ctx, 0, sizeof(g_ota_ctx));
	g_ota_ctx.state = OTA_STATE_IDLE;
	printk("[OTA] Module initialized\n");
}
//处理APP发送的OTA请求
void OtaHandleStartRequest(const uint8_t *frame)
{
	if (g_ota_ctx.state != OTA_STATE_IDLE)
	{

		 printk("igorned [OTA] is in progress\n ");
		 return;
	}
	uint8_t flag = 0;
	EepromWrite(CTRL_OTA_FLAG_ADDR, &flag, 1);//在下载数据的时候清除OTA标志位，防止控制器无回复，单CRC错误，长度不对情况下更新文件时控制器的OTA同时进行

	uint32_t file_len = (uint32_t)frame[4]
	                  | ((uint32_t)frame[5] << 8)
	                  | ((uint32_t)frame[6] << 16)
	                  | ((uint32_t)frame[7] << 24);

	uint32_t file_crc = (uint32_t)frame[8]
	                  | ((uint32_t)frame[9] << 8)
	                  | ((uint32_t)frame[10] << 16)
	                  | ((uint32_t)frame[11] << 24);

	printk("[OTA] Start request: file_len=%u, file_crc=0x%08X\n", file_len, file_crc);

	if (file_len == 0)
	{
		printk("[OTA] Invalid file length\n");
		OtaSendStartResponse(OTA_RESULT_FAIL);
		OtaReset();
		return;
	}

	if (file_len > CTRL_OTA_FLAG_ADDR)
	{
		printk("[OTA] File too large: %u > %u\n", file_len, (EEPROM_SIZE - EEPROM_PAGE_SIZE));
		OtaSendStartResponse(OTA_RESULT_FAIL);
		OtaReset();
		return;
	}

	g_ota_ctx.file_length = file_len;
	g_ota_ctx.file_crc = file_crc;
	g_ota_ctx.state = OTA_STATE_START_RECEIVED;

	OtaSendStartResponse(OTA_RESULT_SUCCESS);

	g_ota_ctx.state = OTA_STATE_ERASING;
	EepromErase(OTA_EEPROM_BASE_ADDR, file_len);

	g_ota_ctx.state = OTA_STATE_RECEIVING;
	g_ota_ctx.eeprom_addr = OTA_EEPROM_BASE_ADDR;
	g_ota_ctx.expected_seq = 0;
	g_ota_ctx.received_length = 0;
	g_ota_ctx.resend_count = 0;
	OtaFileCrcReset();
	printk("[OTA] start to ERASE\n");

	OtaSendFlashEraseDone(OTA_RESULT_SUCCESS);
	printk("[OTA] Ready to receive data\n");
}

// static uint8_t test_data = 0;
//APP发送的数据包处理逻辑
void OtaHandleDataPacket(const uint8_t *frame, uint16_t frame_len)
{
	if (g_ota_ctx.state != OTA_STATE_RECEIVING)
	{
		printk("[OTA] Data packet ignored, state=%d\n", g_ota_ctx.state);
		return;
	}

	if (frame_len < OTA_FRAME_DATA_MIN_SIZE || frame_len > OTA_FRAME_DATA_MAX_SIZE)
	{
		printk("[OTA] Invalid data frame length %u\n", frame_len);
		return;
	}

	/*小端，先验证包序号*/
	uint16_t seq = (uint16_t)frame[4] | ((uint16_t)frame[5] << 8);

	// if(seq == 0x63 && test_data == 0)
	// {
	// 	g_ota_ctx.expected_seq = g_ota_ctx.expected_seq + 1;
	// 	test_data ++;
	// }
	// else if (seq == 0x63 && test_data == 1 )
	// {
	// 	g_ota_ctx.expected_seq = g_ota_ctx.expected_seq -1;
	// }
	if (seq != g_ota_ctx.expected_seq)
	{
		printk("[OTA] Seq mismatch: expected=%u got=%u\n", g_ota_ctx.expected_seq, seq);
		g_ota_ctx.resend_count++;
		if (g_ota_ctx.resend_count >= OTA_MAX_RESEND_COUNT)
		{
			printk("[OTA] seq error %d\n",seq);
			g_ota_ctx.state = OTA_STATE_ERROR;
			OtaSendDataAck(seq, OTA_ACK_FAIL);
			OtaReset();
		}
		else
		{
			OtaSendDataAck(seq, OTA_ACK_RESEND);
		}
		return;
	}

	/*
	实际数据包长度-8字节
	实际包内容从第七字节开始
	*/
	uint16_t content_len = frame_len - 8;
	const uint8_t *content = frame + 7;

	/* 验证CRC */
	uint8_t computed_crc = Ota_Packet_CrcCalculate(content, content_len);
	uint8_t received_crc = frame[frame_len - 1];
	// if(test_data == 1)
	// {
	// 	computed_crc++;
	// 	test_data++;
	// }
	// else
	// {
	// 	computed_crc = Ota_Packet_CrcCalculate(content, content_len);
	// }
	if (computed_crc != received_crc)
	{
		printk("[OTA] Packet CRC mismatch: calc=0x%02X recv=0x%02X\n", computed_crc, received_crc);
		g_ota_ctx.resend_count++;
		if (g_ota_ctx.resend_count >= OTA_MAX_RESEND_COUNT)
		{
			printk("[OTA] Too many resends, aborting\n");
			g_ota_ctx.state = OTA_STATE_ERROR;
			OtaSendDataAck(g_ota_ctx.expected_seq, OTA_ACK_FAIL);//10次OTA都失败，传输03错误
			OtaReset();
		}
		else
		{
			OtaSendDataAck(g_ota_ctx.expected_seq, OTA_ACK_RESEND);
		}
		return;
	}

	/* 是否是尾包 00最后一包,01继续*/
	uint8_t has_more = frame[6];

	printk("[OTA] Data packet seq=%u hasMore=%u frameLen=%u\n", seq, has_more, frame_len);



	uint32_t remaining = g_ota_ctx.file_length - g_ota_ctx.received_length;
	uint16_t valid_len = (content_len <= remaining) ? content_len : (uint16_t)remaining;

	if (valid_len > 0)
	{
		/* 写数据包至EEPROM */
		EepromWrite(g_ota_ctx.eeprom_addr, content, valid_len);

		/* 更新总文件的四字节CRC*/
		OtaFileCrcUpdate(content, valid_len);

		g_ota_ctx.eeprom_addr += valid_len;
		g_ota_ctx.received_length += valid_len;
	}

	g_ota_ctx.expected_seq++;		//包序号自增
	g_ota_ctx.resend_count = 0;		//重置重发次数

	/*是否是最后一包*/
	if (has_more == 0x00)
	{
		uint32_t final_crc = OtaFileCrcFinalize();
		printk("[OTA] Last packet, file CRC calc=0x%08X expected=0x%08X\n", final_crc, g_ota_ctx.file_crc);

		if (final_crc == g_ota_ctx.file_crc)
		{
			g_ota_ctx.state = OTA_STATE_COMPLETE;
			printk("[OTA] Download complete, CRC OK\n");
			OtaSendDataAck(seq, OTA_ACK_CONTINUE);

			//OTA标质量置1，总存储文件长度及CRC
			Ctrl_ota_setota_readyflag(g_ota_ctx.file_length, g_ota_ctx.file_crc);
		}
		else
		{
			printk("[OTA] File CRC mismatch\n");
			printk("APP_CRC_RESULT %x\r\n", g_ota_ctx.file_crc);
			printk("BLE_CRC_RESULT %x\r\n", final_crc);
			OtaSendDataAck(seq, OTA_ACK_FAIL);
		}
		OtaReset();
		return;
	}

	/* ACK to continue */
	OtaSendDataAck(seq, OTA_ACK_CONTINUE);
}

//重置整个EEMPROM的数据包，从头0000开始存，但是总文件大小不变
void OtaPackageRetry(const uint8_t *frame)
{
	if (frame[4] == 0x0E)
	{
		g_ota_ctx.state = OTA_STATE_ERASING;
		EepromErase(OTA_EEPROM_BASE_ADDR, g_ota_ctx.file_length);

		g_ota_ctx.state = OTA_STATE_RECEIVING;
		g_ota_ctx.eeprom_addr = OTA_EEPROM_BASE_ADDR;
		g_ota_ctx.expected_seq = 0;
		g_ota_ctx.received_length = 0;
		g_ota_ctx.resend_count = 0;
		OtaFileCrcReset();
		OtaSendPackageRetryAck(OTA_RESULT_SUCCESS);
		printk("[OTA] Package Retry\n");
	}
	else
	{
		OtaSendPackageRetryAck(OTA_RESULT_FAIL);
	}
}

//重置OTA状态，恢复成默认值，可以重新擦除
void OtaEraseRetry(const uint8_t *frame)
{
	if (frame[4] == 0x09)
	{
		g_ota_ctx.state = OTA_STATE_IDLE;//重置OTA状态，
		printk("[OTA] Erase Retry\n");
		OtaSendEraseRetryAck(OTA_RESULT_SUCCESS);

	}
	else
	{
		OtaSendEraseRetryAck(OTA_RESULT_FAIL);
	}
}

void OtaReset(void)
{
	memset(&g_ota_ctx, 0, sizeof(g_ota_ctx));
	g_ota_ctx.state = OTA_STATE_IDLE;
	printk("[OTA] State reset to IDLE\n");
}

bool OtaIsInProgress(void)
{
	return (g_ota_ctx.state != OTA_STATE_IDLE);
}


