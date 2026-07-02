#ifndef _I2C_EEPROM_H_
#define _I2C_EEPROM_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define EEPROM_I2C_BASE_ADDR_7BIT  0x50
#define EEPROM_PAGE_SIZE           256
#define EEPROM_SIZE                (256U * 512U)

void I2c_eeprom_init(void);

/* EEPROM stub functions (user to implement) */
void EepromErase(uint32_t addr, uint32_t size);
void EepromWrite(uint32_t addr, const uint8_t *data, uint32_t len);
void EepromRead(uint32_t addr, uint8_t *data, uint32_t len);
#ifdef __cplusplus
}
#endif

#endif /* _I2C_EEPROM_H_ */
