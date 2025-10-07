/***
 *udp cmd list:
 * CMD 0X00: pause getting sensor data
 * | 0x55 | 0xBB | 0x00 |
 * CMD 0x01: continue getting sensor data
 * | 0x55 | 0xBB | 0x01 |
 * CMD 0X02: read sensor config
 * | 0x55 | 0xBB | 0x02 | modbus_slave_ID |
 * CMD 0x03: set sensor register, set all if modbus_slave_ID == 0x00
 * | 0x55 | 0xBB | 0x03 | modbus_slave_ID | Start_reg | Length(uint8_t) | [data]
 * 
 * success: return 55 AA FF + [raw frame] | CRC
 * fail:    return 55 AA FF 00 00 00 | CRC
 ***/
