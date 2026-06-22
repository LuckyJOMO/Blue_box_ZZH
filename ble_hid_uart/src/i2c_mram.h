/**
 * @file    i2c_mram.h
 * @brief   Hardware I2C MRAM driver for PAN1080 BLE SoC (Zephyr I2C API).
 *
 * Pin mapping:
 *   SCL - P12 (I2C0_SCL)
 *   SDA - P13 (I2C0_SDA)
 *
 * Device address: 0xA0 (7-bit: 0x50, write: 0xA0, read: 0xA1)
 * Memory address: 16-bit, high byte first
 * Data: 8-bit
 */

#ifndef _I2C_MRAM_H_
#define _I2C_MRAM_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 7-bit I2C address (Zephyr API shifts internally to 0xA0/0xA1) */
#define MRAM_I2C_ADDR_7BIT       0x50

void i2c_mram_init(void);
uint8_t i2c_mram_read_byte(uint16_t mem_addr);
void i2c_mram_write_byte(uint16_t mem_addr, uint8_t data);
void i2c_mram_read_buf(uint16_t mem_addr, uint8_t *buf, uint16_t len);
void i2c_mram_write_buf(uint16_t mem_addr, const uint8_t *buf, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* _I2C_MRAM_H_ */
