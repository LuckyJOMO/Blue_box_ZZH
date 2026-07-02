/**
 * @file    i2c_eeprom.c
 * @brief   Hardware I2C EEPROM driver (1Mbit / 128KB) using Zephyr I2C API.
 *
 * Addressing: 17-bit memory address split as:
 *   - bit 16  → I2C device address bit A1 (block select)
 *   - bit 15:0 → 2-byte memory address (high, low)
 *
 * Page size 128 bytes, ~5ms write-cycle delay per page.
 */

#include "i2c_eeprom.h"
#include <zephyr.h>
#include <device.h>
#include <drivers/i2c.h>
#include <sys/printk.h>
#include <string.h>

static const struct device *i2c_dev;

/* ── Helper: resolve 7-bit I2C address from 17-bit memory address ── */
static inline uint8_t eeprom_i2c_addr(uint32_t mem_addr)
{
	return (uint8_t)(EEPROM_I2C_BASE_ADDR_7BIT | ((mem_addr >> 16) & 0x01));
}



static void I2c_eeprom_write(uint32_t mem_addr, const uint8_t *buf, uint32_t len)
{
	if (!buf || !len)
	{
		return;
	}

	while (len > 0)
	{
		uint8_t dev_addr = eeprom_i2c_addr(mem_addr);

		/* Bytes remaining in current 128-byte page */
		uint32_t page_off = mem_addr & (EEPROM_PAGE_SIZE - 1U); //页内偏移地址
		uint32_t space     = EEPROM_PAGE_SIZE - page_off;	//当前页剩余的空间
		uint32_t chunk     = (len < space) ? len : space;	//本次最多写入的多少字节

		/* Build tx buffer: [addr_high, addr_low, data...] */
		uint8_t tx[2 + chunk];
		tx[0] = (uint8_t)((mem_addr >> 8) & 0xFF);
		tx[1] = (uint8_t)(mem_addr & 0xFF);
		memcpy(tx + 2, buf, chunk);

		i2c_write(i2c_dev, tx, sizeof(tx), dev_addr);

		/* Wait for internal write cycle to complete */
		k_msleep(5);

		buf      += chunk;
		mem_addr += chunk;
		len      -= chunk;
	}
}

static void I2c_eeprom_read(uint32_t mem_addr, uint8_t *buf, uint32_t len)
{
	if (!buf || !len) {
		return;
	}

	while (len > 0)
	{
		uint8_t dev_addr = eeprom_i2c_addr(mem_addr);

		/* Respect 64KB block boundary (I2C device-address switch) */
		uint32_t block_end = (mem_addr & 0x10000U) ? 0x20000U : 0x10000U;
		uint32_t space     = block_end - mem_addr;
		uint32_t chunk     = (len < space) ? len : space;

		uint8_t addr_buf[2];
		addr_buf[0] = (uint8_t)((mem_addr >> 8) & 0xFF);
		addr_buf[1] = (uint8_t)(mem_addr & 0xFF);

		/* Dummy write to set internal address pointer */
		i2c_write(i2c_dev, addr_buf, sizeof(addr_buf), dev_addr);

		/* Sequential read */
		i2c_read(i2c_dev, buf, chunk, dev_addr);

		buf      += chunk;
		mem_addr += chunk;
		len      -= chunk;
	}
}

static void I2c_eeprom_erase(uint32_t addr, uint32_t size)
{
	uint8_t buf[EEPROM_PAGE_SIZE];

	memset(buf, 0xFF, sizeof(buf));

	printk("Retry  erase: addr=0x%05X size=%u (%u pages)\n",
	       addr, size, (size + EEPROM_PAGE_SIZE - 1U) / EEPROM_PAGE_SIZE);

	// while (size > 0) {
	// 	uint32_t chunk = (size < EEPROM_PAGE_SIZE) ? size : EEPROM_PAGE_SIZE;
	// 	I2c_eeprom_write(addr, buf, chunk);
	// 	addr += chunk;
	// 	size -= chunk;
	// }
}
/* ── EEPROM wrapper functions ──────────────────────────────────── */

void I2c_eeprom_init(void)
{
	uint32_t i2c_cfg = I2C_SPEED_SET(I2C_SPEED_FAST) | I2C_MODE_MASTER;

	i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));

	if (!device_is_ready(i2c_dev)) {
		printk("I2C EEPROM: device not ready\n");
		return;
	}

	if (i2c_configure(i2c_dev, i2c_cfg) == 0) {
		printk("I2C EEPROM init done (P12=SCL, P13=SDA, base addr=0x%02X, %uKB)\n", EEPROM_I2C_BASE_ADDR_7BIT,
		       EEPROM_SIZE / 512U);
	}
}

void EepromErase(uint32_t addr, uint32_t size)
{
	I2c_eeprom_erase(addr, size);
}

void EepromWrite(uint32_t addr, const uint8_t *data, uint32_t len)
{
	I2c_eeprom_write(addr, data, len);
}

void EepromRead(uint32_t addr, uint8_t *data, uint32_t len)
{
	I2c_eeprom_read(addr, data, len);
}
