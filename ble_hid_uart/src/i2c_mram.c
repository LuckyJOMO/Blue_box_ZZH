/**
 * @file    i2c_mram.c
 * @brief   Hardware I2C MRAM driver using Zephyr I2C API (PAN1080 I2C0).
 *
 * Pin mapping (hardware fixed):
 *   P12 = I2C0_SCL,  P13 = I2C0_SDA
 *
 * Requires overlay to enable &i2c0 with p1_2/p1_3 pinctrl.
 */

#include "i2c_mram.h"
#include <zephyr.h>
#include <device.h>
#include <drivers/i2c.h>
#include <sys/printk.h>

static const struct device *i2c_dev;

void i2c_mram_init(void)
{
    uint32_t i2c_cfg = I2C_SPEED_SET(I2C_SPEED_STANDARD) | I2C_MODE_MASTER;

    i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));

    if (!device_is_ready(i2c_dev)) {
        printk("I2C MRAM: device not ready\n");
        return;
    }

    if (i2c_configure(i2c_dev, i2c_cfg) == 0) {
        printk("I2C MRAM init done (P12=SCL, P13=SDA, addr=0x%02X)\n", MRAM_I2C_ADDR_7BIT);
    }
}

void i2c_mram_write_byte(uint16_t mem_addr, uint8_t data)
{
    uint8_t buf[3];

    buf[0] = (uint8_t)(mem_addr >> 8);   /* high byte */
    buf[1] = (uint8_t)(mem_addr & 0xFF); /* low byte */
    buf[2] = data;

    i2c_write(i2c_dev, buf, sizeof(buf), MRAM_I2C_ADDR_7BIT);
}

uint8_t i2c_mram_read_byte(uint16_t mem_addr)
{
    uint8_t addr_buf[2];
    uint8_t data = 0;

    addr_buf[0] = (uint8_t)(mem_addr >> 8);
    addr_buf[1] = (uint8_t)(mem_addr & 0xFF);

    /* Dummy write to set internal address pointer */
    i2c_write(i2c_dev, addr_buf, sizeof(addr_buf), MRAM_I2C_ADDR_7BIT);

    /* Read one byte */
    i2c_read(i2c_dev, &data, 1, MRAM_I2C_ADDR_7BIT);

    return data;
}

void i2c_mram_write_buf(uint16_t mem_addr, const uint8_t *buf, uint16_t len)
{
    if (!buf || !len) {
        return;
    }

    /* Build combined buffer: [addr_high, addr_low, data...] */
    uint8_t tx_buf[2 + len];
    tx_buf[0] = (uint8_t)(mem_addr >> 8);
    tx_buf[1] = (uint8_t)(mem_addr & 0xFF);
    for (uint16_t i = 0; i < len; i++) {
        tx_buf[2 + i] = buf[i];
    }

    i2c_write(i2c_dev, tx_buf, sizeof(tx_buf), MRAM_I2C_ADDR_7BIT);
}

void i2c_mram_read_buf(uint16_t mem_addr, uint8_t *buf, uint16_t len)
{
    if (!buf || !len) {
        return;
    }

    uint8_t addr_buf[2];

    addr_buf[0] = (uint8_t)(mem_addr >> 8);
    addr_buf[1] = (uint8_t)(mem_addr & 0xFF);

    /* Dummy write to set internal address pointer */
    i2c_write(i2c_dev, addr_buf, sizeof(addr_buf), MRAM_I2C_ADDR_7BIT);

    /* Sequential read */
    i2c_read(i2c_dev, buf, len, MRAM_I2C_ADDR_7BIT);
}
