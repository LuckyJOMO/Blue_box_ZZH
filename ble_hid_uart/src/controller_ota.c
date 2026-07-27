#include "controller_ota.h"
#include "i2c_eeprom.h"
#include <string.h>
#include <sys/printk.h>
#include <kernel.h>

volatile uint8_t test_data_ota;
static const uint8_t ctrl_ota_header[CTRL_OTA_HDR_LEN] =
{
	0xEB, 0x90, 0xEB, 0x90, 0xEB, 0x90
};

/* ── OTA context ──────────────────────────────────────────────── */
typedef struct
{
	CtrlOtaState state;
	uint32_t file_length;
	uint32_t file_crc;
	uint32_t sent_length;
	uint32_t eeprom_read_addr;
	uint32_t last_request_time;
} CtrlOtaCtx;

static CtrlOtaCtx g_ctx;

//单字节校验数据包内容
static uint8_t CtrlOtaCrc8(const uint8_t *data, uint32_t len)
{
	uint8_t crc = 0;
	for (uint32_t i = 0; i < len; i++) {
		crc ^= data[i];
	}
	return crc;
}

/*串口数据发送给控制器*/
static void Ctrl_ota_send_raw(const uint8_t *data, uint16_t len)
{
	printk("[CTRL_OTA] TX %u bytes: ", len);
	// for (uint16_t i = 0; i < len; i++) {
	// 	printk("%02X ", data[i]);
	// }
	// printk("\n");
	TGT_SendMultiData((const uint8_t *)data, len);
}


/*给控制器发送OTA请求帧06*/
static void Ctrl_ota_send_request(void)
{
	uint8_t frame[CTRL_REQ_FRAME_LEN];

	memcpy(frame, ctrl_ota_header, CTRL_OTA_HDR_LEN);
	frame[6] = 0x10;                        /* length */
	frame[7] = CTRL_CMD_WRITE_REQ;          /* command */
	frame[8]  = (uint8_t)(g_ctx.file_length & 0xFF);
	frame[9]  = (uint8_t)((g_ctx.file_length >> 8) & 0xFF);
	frame[10] = (uint8_t)((g_ctx.file_length >> 16) & 0xFF);
	frame[11] = (uint8_t)((g_ctx.file_length >> 24) & 0xFF);
	frame[12] = (uint8_t)(g_ctx.file_crc & 0xFF);
	frame[13] = (uint8_t)((g_ctx.file_crc >> 8) & 0xFF);
	frame[14] = (uint8_t)((g_ctx.file_crc >> 16) & 0xFF);
	frame[15] = (uint8_t)((g_ctx.file_crc >> 24) & 0xFF);

	printk("[CTRL_OTA]  test_data_ota=\r\n%d ", test_data_ota);
	// uint8_t test_data_flag = 0 ;

	// EepromRead((CTRL_OTA_FLAG_ADDR+100),&test_data_flag,1);
	// if (test_data_ota == 0 && test_data_flag != 0x81)
	// {
	// 	frame[12] = frame[12] + 1;
	// }

	Ctrl_ota_send_raw(frame, CTRL_REQ_FRAME_LEN);
}

/*给控制器下发文件数据(0x2B)*/
static void Ctrl_ota_send_data_frame(void)
{
	uint32_t remaining = g_ctx.file_length - g_ctx.sent_length;
	uint16_t chunk = (remaining > CTRL_DATA_MAX_PAYLOAD) ? CTRL_DATA_MAX_PAYLOAD : (uint16_t)remaining;
	uint8_t has_more = (chunk < remaining) ? 0x01 : 0x00;

	static uint8_t data_buf[CTRL_DATA_MAX_PAYLOAD];
	EepromRead(g_ctx.eeprom_read_addr, data_buf, chunk);

	uint8_t data_crc = CtrlOtaCrc8(data_buf, chunk);

	uint16_t frame_len = CTRL_DATA_HDR_OVERHEAD + chunk; /*报文总长度*/
	static uint8_t frame[CTRL_DATA_HDR_OVERHEAD + CTRL_DATA_MAX_PAYLOAD];

	memcpy(frame, ctrl_ota_header, CTRL_OTA_HDR_LEN);
	frame[6] = (uint8_t)(frame_len); 	/*报文总长度*/
	frame[7] = CTRL_CMD_DATA;               /* command 0x2B */
	frame[8] = has_more;                    /* 01=continue, 00=last */
	memcpy(frame + 9, data_buf, chunk);
	frame[9 + chunk] = data_crc;            /* 单个数据包CRC校验 */

	// if((g_ctx.sent_length / CTRL_DATA_MAX_PAYLOAD) > 35)
	// {
	// 	frame[9 + chunk] = data_crc + 1;
	// }

	Ctrl_ota_send_raw(frame, frame_len);

	// printk("[CTRL_OTA] seq=%u\r\n",(unsigned int)(g_ctx.sent_length / CTRL_DATA_MAX_PAYLOAD));
}

/*更新写入地址及发送长度*/
static void Ctrl_ota_advance_data(void)
{
	uint32_t remaining = g_ctx.file_length - g_ctx.sent_length;
	uint16_t chunk = (remaining > CTRL_DATA_MAX_PAYLOAD) ? CTRL_DATA_MAX_PAYLOAD : (uint16_t)remaining;
	g_ctx.eeprom_read_addr += chunk;
	g_ctx.sent_length += chunk;
}

/*蓝牙上报给小程序OTA数据包更新进度*/
static void Ctrl_ota_notify_progress(uint32_t sent, uint32_t total)
{
	uint8_t pct;
	if (total == 0)
	{
		pct = 0;
	} else
	{
		pct = (uint8_t)((sent * 100U) / total);
	}
	uint8_t frame[5] = { 0x53, 0x35, CTRL_OTA_V3_PROGRESS, 0x05, pct };
	bt_ven_notify(frame, sizeof(frame));
	printk("[CTRL_OTA] APP notify: progress=%u%%\n", pct);
}

static void Ctrl_ota_notify_result(uint8_t result)
{
	uint8_t frame[5] = { 0x53, 0x35, CTRL_OTA_V3_RESULT, 0x05, result };
	bt_ven_notify(frame, sizeof(frame));
	printk("[CTRL_OTA] APP notify: result= %u \n", result );
}

/* ═══════════════════════════════════════════════════════════════
 *  Public API
 * ═══════════════════════════════════════════════════════════════ */

void Ctrl_ota_init(void)
{
	test_data_ota = 0;
	uint8_t flag = 0;
	EepromRead(CTRL_OTA_FLAG_ADDR, &flag, 1);

	printk("[CTRL_OTA] Boot check: flag=0x%02X\n", flag);

	if (flag != CTRL_OTA_FLAG_READY)
	{
		memset(&g_ctx, 0, sizeof(g_ctx));
		g_ctx.state = CTRL_OTA_IDLE;
		return;
	}

	uint8_t meta[8];
	EepromRead(CTRL_OTA_META_FILE_LEN, meta, sizeof(meta));

	g_ctx.file_length = (uint32_t)meta[0]
	                  | ((uint32_t)meta[1] << 8)
	                  | ((uint32_t)meta[2] << 16)
	                  | ((uint32_t)meta[3] << 24);

	g_ctx.file_crc    = (uint32_t)meta[4]
	                  | ((uint32_t)meta[5] << 8)
	                  | ((uint32_t)meta[6] << 16)
	                  | ((uint32_t)meta[7] << 24);

	g_ctx.sent_length = 0;
	g_ctx.eeprom_read_addr = 0;   /* OTA data stored from EEPROM addr 0 */
	g_ctx.last_request_time = 0;
	g_ctx.state = CTRL_OTA_SEND_REQUEST;
	printk("[CTRL_OTA] OTA pending: file_len=%u file_crc=0x%08X, entering SEND_REQUEST\n",
	       g_ctx.file_length, g_ctx.file_crc);
}


//蓝牙发送给控制器OTA状态机函数
void Ctrl_ota_process(void)
{
	uint32_t now = k_uptime_get_32();

	switch (g_ctx.state)
	{

	case CTRL_OTA_IDLE:
	case CTRL_OTA_COMPLETE:
		break;

	case CTRL_OTA_SEND_REQUEST:
		if(now <= g_ctx.last_request_time) break;	//ERROR状态下冷却50ms，冷却期跳过
		if (now >= (g_ctx.last_request_time + 30))
		{
			Ctrl_ota_send_request();
			g_ctx.last_request_time = now;
		}
		break;

	case CTRL_OTA_SEND_DATA:
	{
		uint32_t remaining = g_ctx.file_length - g_ctx.sent_length;
		if (remaining == 0)
		{
			g_ctx.state = CTRL_OTA_WAIT_COMPLETE;
			printk("[CTRL_OTA] All data sent, waiting for controller completion\n");
		}
		else
		{
			g_ctx.state = CTRL_OTA_WAIT_DATA_ACK;
			Ctrl_ota_send_data_frame();
			g_ctx.last_request_time = now;
		}
		break;
	}
	case CTRL_OTA_WAIT_DATA_ACK:
		if (now - g_ctx.last_request_time >= 500)
		{
			printk("[CTRL_OTA] Data ACK timeout, resending...\n");
			g_ctx.state = CTRL_OTA_SEND_DATA;
			// g_ctx.state = CTRL_OTA_FILED; // 无重发逻辑，改到FILED状态
			// uint8_t flag = 0; //OTA标质位置0
			// EepromWrite(CTRL_OTA_FLAG_ADDR, &flag, 1);
		}
		break;
	case CTRL_OTA_WAIT_CONFIRM:

	case CTRL_OTA_WAIT_COMPLETE:
		/* Waiting for controller reply — no action until Ctrl_ota_handle_reply() */
		break;

	case CTRL_OTA_ERROR:
		printk("[CTRL_OTA] Error state, restarting request phase after 500ms\n");
		g_ctx.state = CTRL_OTA_SEND_REQUEST;
		g_ctx.sent_length = 0;
		g_ctx.eeprom_read_addr = 0;
		g_ctx.last_request_time = now; /* 延迟50ms */
		break;

	case CTRL_OTA_FILED:

		Ctrl_ota_notify_result(CTRL_OTA_RESULT_FAILED);
		k_msleep(500);	//防止还没连接就报错，、导致小程序显示进度条卡死，500ms发一次触发小程序重新下载逻辑
		break;
	}
}
//控制器传给蓝牙的OTA命令处理函数
void Ctrl_ota_handle_reply(uint8_t cmd)
{
	// printk("[CTRL_OTA] RX reply cmd=0x%02X, current state=%d\n", cmd, g_ctx.state);

	switch (cmd) {

	case CTRL_CMD_REQ_CONFIRM:       /* 0x07 */
		if (g_ctx.state == CTRL_OTA_SEND_REQUEST)
		{
			g_ctx.state = CTRL_OTA_WAIT_CONFIRM;
			printk("[CTRL_OTA] Request confirmed\n");
		}
		break;

	case CTRL_CMD_FLASH_ERASE_OK:    /* 0x17 */
		if (g_ctx.state == CTRL_OTA_SEND_REQUEST || g_ctx.state == CTRL_OTA_WAIT_CONFIRM)
		{
			printk("[CTRL_OTA] Flash erase done\n");
			g_ctx.state = CTRL_OTA_SEND_DATA;
		}
		break;

	case CTRL_CMD_DATA_ACK:         /* 0x0B */
		if (g_ctx.state == CTRL_OTA_WAIT_DATA_ACK)
		{
			uint32_t now = k_uptime_get_32();
			Ctrl_ota_advance_data();

			/*上报给小程序OTA进度 */
			Ctrl_ota_notify_progress(g_ctx.sent_length, g_ctx.file_length);

			g_ctx.last_request_time = now;
			if (g_ctx.sent_length == g_ctx.file_length)
			{
				g_ctx.state = CTRL_OTA_WAIT_COMPLETE;
				printk("[CTRL_OTA] All data ACKed, waiting for completion\n");
			}
			else
			{
				g_ctx.state = CTRL_OTA_SEND_DATA;
			}
		}
		break;

	case CTRL_CMD_RECV_COMPLETE:    /* 0x08 */
		if (g_ctx.state == CTRL_OTA_WAIT_COMPLETE) {
			printk("[CTRL_OTA] Controller confirmed file receive complete, waiting CRC verify\n");
		}
		break;

	case CTRL_CMD_CRC_SUCCESS:      /* 0x0C */
	{
		printk("[CTRL_OTA] CRC verify SUCCESS — OTA complete!\n");

		/*上报给小程序OTA结果*/
		Ctrl_ota_notify_result(CTRL_OTA_RESULT_OK);

		/* 清除标志位 */
		uint8_t clear_flag = CTRL_OTA_FLAG_IDLE;
		EepromWrite(CTRL_OTA_FLAG_ADDR, &clear_flag, 1);
		printk("[CTRL_OTA] EEPROM flag cleared\n");

		g_ctx.state = CTRL_OTA_COMPLETE;
		break;
	}

	case CTRL_CMD_CRC_FAIL:         /* 0x0D */
	{
		printk("[CTRL_OTA] CRC verify FAIL — restarting OTA flow\n");

		/* 上报给小程序OTA结果*/
		Ctrl_ota_notify_result(CTRL_OTA_RESULT_ERROR);
		// test_data_ota++;
		// uint8_t data = 0x81;
		// EepromWrite((CTRL_OTA_FLAG_ADDR + 100), &data, 1);
		g_ctx.state = CTRL_OTA_ERROR;
		break;
	}

	case CTRL_CMD_CRC_ERROR:        /* 0x09 — data frame CRC error */

		if (g_ctx.state == CTRL_OTA_WAIT_DATA_ACK)
		{
			printk("[CTRL_OTA] Data CRC error, resending current frame\n");
			g_ctx.state = CTRL_OTA_FILED; //重新上电，无补发包逻辑，控制器Bootlader中会卡死

			Ctrl_ota_notify_result(CTRL_OTA_RESULT_FAILED);
		}
		break;

	case CTRL_CMD_LEN_ERROR:        /* 0x0A — file length not multiple of 4 */
		printk("[CTRL_OTA] File length error — restarting OTA flow\n");
		g_ctx.state = CTRL_OTA_FILED; //重新上电，无从头重发逻辑，控制器Bootlader中会卡死

		Ctrl_ota_notify_result(CTRL_OTA_RESULT_FAILED);
		break;

	default:
		printk("[CTRL_OTA] Unknown reply cmd=0x%02X, ignored\n", cmd);
		break;
	}
}
//判断是否进入OTA流程
bool Ctrl_ota_is_active(void)
{
	return (g_ctx.state != CTRL_OTA_IDLE && g_ctx.state != CTRL_OTA_COMPLETE);
}

//蓝牙完成最后一个包传输后，OTA标志位置1，存入文件长度及从文件CRC
void Ctrl_ota_setota_readyflag(uint32_t file_length, uint32_t file_crc)
{
	uint8_t meta[9];

	meta[0] = CTRL_OTA_FLAG_READY;                    /* OTA标志位，置1蓝牙小程序下载完成 */
	meta[1] = (uint8_t)(file_length & 0xFF);           /* 文件大小 */
	meta[2] = (uint8_t)((file_length >> 8) & 0xFF);
	meta[3] = (uint8_t)((file_length >> 16) & 0xFF);
	meta[4] = (uint8_t)((file_length >> 24) & 0xFF);
	meta[5] = (uint8_t)(file_crc & 0xFF);              /* 总包CRC */
	meta[6] = (uint8_t)((file_crc >> 8) & 0xFF);
	meta[7] = (uint8_t)((file_crc >> 16) & 0xFF);
	meta[8] = (uint8_t)((file_crc >> 24) & 0xFF);

	EepromWrite(CTRL_OTA_FLAG_ADDR, meta, sizeof(meta));

	printk("[CTRL_OTA] OTA ready flag written: flag=0x01 file_len=%u file_crc=0x%08X\n",
	       file_length, file_crc);
}
