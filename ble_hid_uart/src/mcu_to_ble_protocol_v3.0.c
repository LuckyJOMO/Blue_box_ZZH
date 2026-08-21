#include "mcu_to_ble_protocol_v2.0.h"
#include "mcu_to_ble_protocol_v3.0.h"
#include "ble_ota.h"
#include "i2c_eeprom.h"
#include "controller_ota.h"
#include <string.h>
#include <stdio.h>
#include "kernel.h"
#include "dev_led.h"
static uint16_t V3_crc_calculate(const uint8_t *frame, uint8_t count)
{
	uint16_t crc = 0;
	for (uint8_t i = 0; i < count; i++) {
		crc += (uint16_t)(i + 1) * (uint16_t)frame[2 + i];
	}
	return crc;
}
static bool V3_crc_verify(const uint8_t *frame, uint8_t count, uint8_t crc_offset)
{
	uint16_t calculated = V3_crc_calculate(frame, count);
	uint8_t cl = (uint8_t)(calculated & 0xFF);
	uint8_t ch = (uint8_t)((calculated >> 8) & 0xFF);
	return (frame[crc_offset] == cl) && (frame[crc_offset + 1] == ch);
}

static void V3_build_error_reply(uint8_t error_cmd, uint8_t grp, uint8_t sta, uint8_t *tx_frame)
{
	tx_frame[0] = V3_FRAME_HEAD_0;
	tx_frame[1] = V3_FRAME_HEAD_1;
	tx_frame[2] = error_cmd;
	tx_frame[3] = grp;
	tx_frame[4] = sta;

	uint16_t crc = V3_crc_calculate(tx_frame, V3_CRC_COUNT_ERROR);
	tx_frame[V3_CRC_OFFSET_ERROR] = (uint8_t)(crc & 0xFF);
	tx_frame[V3_CRC_OFFSET_ERROR + 1] = (uint8_t)((crc >> 8) & 0xFF);
}

static void V3_build_handshake_reply(uint8_t *handshake_buff, uint8_t *frame)
{
	handshake_buff[0] = V3_FRAME_HEAD_0;
	handshake_buff[1] = V3_FRAME_HEAD_1;
	handshake_buff[2] = V3_CMD_HANDSHAKE;
	uint8_t flag = 0;
	EepromRead(CTRL_OTA_FLAG_ADDR, &flag, 1);

	printk("[CTRL_OTA] Boot check: flag=0x%02X\n", flag);

	if (flag != CTRL_OTA_FLAG_READY)
	{
		handshake_buff[3] = V3_OTA_FLAG_FINISH;
	}
	else
	{
		handshake_buff[3] = V3_OTA_FLAG_NEED;
	}
	handshake_buff[4] = frame[3];
	handshake_buff[5] = frame[4];
	handshake_buff[6] = frame[5];
	handshake_buff[7] = frame[6];

	uint16_t crc = V3_crc_calculate(handshake_buff, V3_CRC_COUNT_HANDSHAKE_REPLY);
	handshake_buff[V3_CRC_OFFSET_HANDSHAKE_REPLY] = (uint8_t)(crc & 0xFF);
	handshake_buff[V3_CRC_OFFSET_HANDSHAKE_REPLY + 1] = (uint8_t)((crc >> 8) & 0xFF);
}
static void V3_handle_handshake(const uint8_t *frame)
{
	if (!V3_crc_verify(frame, V3_CRC_COUNT_HANDSHAKE, V3_CRC_OFFSET_HANDSHAKE))
	{
		printk("[V3] Handshake CRC error, dropping\n");

		return;
	}
	uint8_t handshake_buff[V3_FRAME_SIZE_HANDSHAKE_REPLY];
	V3_build_handshake_reply((uint8_t *)handshake_buff, (uint8_t *)frame);

	set_led_state(LED_STATE_GREEN_FAST_BLINK); // 红灯慢闪
	printk("[V3] Handshake OK, echoing back\n");
	bt_ven_notify(handshake_buff, V3_FRAME_SIZE_HANDSHAKE_REPLY);
}

static void V3_handle_read_write(const uint8_t *frame, uint8_t cmd)
{
	uint8_t grp = frame[3];//指定的地址
	if (!V3_crc_verify(frame, V3_CRC_COUNT_NORMAL, V3_CRC_OFFSET_NORMAL))
	{
		printk("[V3] Cmd 0x%02X GRP 0x%02X: CRC error\n", cmd, grp);
		uint8_t err_cmd = (cmd == V3_CMD_WRITE_CTRL) ? V3_CMD_WRITE_ERR : V3_CMD_READ_ERR;
		uint8_t err_frame[V3_FRAME_SIZE_ERROR];
		V3_build_error_reply(err_cmd, grp, V3_ERR_CRC, err_frame);
		bt_ven_notify(err_frame, V3_FRAME_SIZE_ERROR);
		return;
	}

	/* Forward to MCU */
	// printk("[V3] Forwarding cmd 0x%02X GRP 0x%02X to MCU\n", cmd, grp);
	ble_send_data(frame, V3_FRAME_SIZE_NORMAL);

	/* Set waiting state */
	v3_state.waiting_mcu_reply = true;
	v3_state.pending_cmd = cmd;
	v3_state.pending_grp = grp;
	v3_state.send_time_ms = k_uptime_get_32();
}

static void V3_handle_mcu_reply(const uint8_t *frame, uint8_t cmd)
{
	uint8_t grp = frame[3]; //指定的地址
	// printk("[V3] MCU reply: cmd 0x%02X\n", cmd);
	if (!V3_crc_verify(frame, V3_CRC_COUNT_NORMAL, V3_CRC_OFFSET_NORMAL))
	{
		printk("[V3] MCU reply CRC error,command: %02x \n", cmd);
		uint8_t err_cmd = (cmd == V3_CMD_WRITE_CTRL) ? V3_CMD_WRITE_ERR : V3_CMD_READ_ERR;
		uint8_t err_frame[V3_FRAME_SIZE_ERROR];
		V3_build_error_reply(err_cmd, grp, V3_ERR_CRC, err_frame);
		bt_ven_notify(err_frame, V3_FRAME_SIZE_ERROR);
		return;
	}



	bt_ven_notify(frame, V3_FRAME_SIZE_NORMAL);
	v3_state.waiting_mcu_reply = false;
	// printk("v3_state.send_time_ms %d", v3_state.send_time_ms);
	// printk("[V3] MCU reply forwarded to APP\n");

}
static void V3_parse_buffer(const uint8_t *data, uint16_t len, bool from_app)
{
	uint16_t offset = 0;
	uint16_t pkt_len = 0;
	uint8_t cmd = 0;
	while (offset < len)
	{
		// 查找包头
		if ((data[offset] != APP_FRAME_HEAD) && (data[offset] != V3_FRAME_HEAD_0)) {
			offset++;
			continue;
		}
		if (data[offset] == APP_FRAME_HEAD)
		{
			// 判断命令字
			if (offset + 2 >= len)
				break; // 不够命令字和包尾

			cmd = data[offset + 1]; //命令字
			if (cmd == 0x01)
				pkt_len = 7; // 55 01 xx xx xx xx AA  //握手指令包长
			else if (cmd == 0x02)
				pkt_len = 3; // 55 02 AA  //查询解锁状态包长
			else if (cmd == 0x03)
				pkt_len = 3; // 55 03 AA  //解锁指令包长
			else if (cmd == 0x04)
				pkt_len = 3; // 55 04 AA  //锁定指令包长
			else if (cmd == 0x05)
				pkt_len = 3; // 55 05 AA  //故障查询指令包长
			else {
				offset++;
				continue;
			} // 未知命令，跳过

			if (offset + pkt_len > len)
				break; // 数据不够一帧，等待下次补齐
			// 检查包尾
			if (data[offset + pkt_len - 1] == APP_FRAME_TRAIL)
			{
				//printk("APP send data to BLE,cmd = %02X\r\n", cmd);
				// 识别成功，处理该帧
				Handle_app_cmd_and_reply(cmd, data + offset, pkt_len);
			}

		}
		else
		{
			if ((data[offset + 1] != V3_FRAME_HEAD_1) || (offset + 3 >= len)) {
				break;
			}
			cmd = data[offset + 2]; //命令字
			switch(cmd)
			{
				case V3_CMD_HANDSHAKE:
				  pkt_len = V3_FRAME_SIZE_HANDSHAKE; // 53 35 01 xx xx xx xx CL CH  //握手指令包长
				  break;
				case V3_CMD_READ_CTRL: // 53 35 0F GRP 00 00 00 00 00 00 00 00 00 00 00 00 00 00 CL CH //查询控制器信息
				case V3_CMD_WRITE_CTRL: // 53 35 00 GRP [14字节有效数据] CL CH 写控制器
				  pkt_len = V3_FRAME_SIZE_NORMAL;
				  break;
				case OTA_CMD_START: // OTA 烧写请求, 仅 APP→BLE
				  pkt_len = data[offset + 3];
				  if (pkt_len != OTA_FRAME_START_REQ_SIZE) { offset++; continue; }
				  break;
				case OTA_CMD_DATA: // OTA 文件数据传输, 仅 APP→BLE
				  pkt_len = data[offset + 3];
				  if (pkt_len < OTA_FRAME_DATA_MIN_SIZE || pkt_len > OTA_FRAME_DATA_MAX_SIZE)
				   { offset++; continue; }
				  break;
				case OTA_CMD_DATA_RETRY:
				case OTA_CMD_ERASE_RETRY: // OTA 烧写请求, 仅 APP→BLE
					pkt_len = data[offset + 3];
					printk("2\r\n");
					if (pkt_len != OTA_FRAME_FLASH_ERASE_RSP_SIZE) {offset++; continue;}
					break;
				default:
				offset++;
				continue;
				}
			if (offset + pkt_len > len ) break; // 数据不够一帧，等待下次补齐
			const uint8_t *frame = &data[offset];

			if(cmd == V3_CMD_HANDSHAKE)
			{
			  if(from_app)
			  {
				V3_handle_handshake(frame);
			  }

			}
			else if (cmd == V3_CMD_WRITE_CTRL || cmd == V3_CMD_READ_CTRL)
			{
				if (from_app)
				{
					V3_handle_read_write(frame, cmd);
				}
				else
				{
					V3_handle_mcu_reply(frame, cmd);
					// show_reg3(data, len);
				}
			}
			else if (cmd == OTA_CMD_START && from_app)
			{
				OtaHandleStartRequest(frame);
			}
			else if (cmd == OTA_CMD_DATA && from_app)
			{
				OtaHandleDataPacket(frame, pkt_len);
			}
			else if (cmd == OTA_CMD_DATA_RETRY && from_app)
			{
				OtaPackageRetry(frame);
			}
			else if (cmd == OTA_CMD_ERASE_RETRY && from_app)
			{
				OtaEraseRetry(frame);
			}
		}
		// 移动到下一个包
		offset += pkt_len;
	}
}
// 根据V3帧命令字获取帧长度，0表示未知命令
uint16_t V3_frame_size_from_cmd(uint8_t cmd)
{
	switch (cmd) {
	case V3_CMD_HANDSHAKE:
		return V3_FRAME_SIZE_HANDSHAKE; // 9
	case V3_CMD_READ_CTRL:
	case V3_CMD_WRITE_CTRL:
		return V3_FRAME_SIZE_NORMAL; // 20
	case V3_CMD_WRITE_ERR:
	case V3_CMD_READ_ERR:
		return V3_FRAME_SIZE_ERROR; // 7
	default:
		return 0;
	}
}
void V3_handle_app_data(const uint8_t *data, uint16_t len)
{
    if (!data || len == 0) return;
    V3_parse_buffer(data, len, true);
}

void V3_handle_mcu_data(const uint8_t *data, uint16_t len)
{
	if (!data || len == 0) return;
	V3_parse_buffer(data, len, false);
}
bool V3_is_waiting_mcu_reply(void)
{
	return v3_state.waiting_mcu_reply;
}
void V3_timeout_check(uint32_t now_ms)
{
	if (!v3_state.waiting_mcu_reply)
		return;

	if ((now_ms - v3_state.send_time_ms) >= V3_MCU_REPLY_TIMEOUT_MS)
	{
		printk("[V3] MCU reply timeout for cmd 0x%02XGRP 0x%02X\n", v3_state.pending_cmd,v3_state.pending_grp);

		uint8_t err_cmd = (v3_state.pending_cmd == V3_CMD_WRITE_CTRL) ? V3_CMD_WRITE_ERR : V3_CMD_READ_ERR;

		uint8_t err_frame[V3_FRAME_SIZE_ERROR];
		V3_build_error_reply(err_cmd, v3_state.pending_grp, V3_ERR_MCU_NO_REPLY, err_frame);
		bt_ven_notify(err_frame, V3_FRAME_SIZE_ERROR);

		v3_state.waiting_mcu_reply = false;
	}
}
