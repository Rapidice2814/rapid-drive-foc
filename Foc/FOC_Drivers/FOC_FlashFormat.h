#ifndef FOC_FLASH_FORMAT_H
#define FOC_FLASH_FORMAT_H
#include <stdint.h>
#include <stddef.h>
#define FOC_FLASH_MAGIC 0x52444632UL
#define FOC_FLASH_FORMAT_VERSION 2U
/** @brief CRC-32/ISO-HDLC over the stored settings bytes preceding the CRC. */
static inline uint32_t FOC_FlashCRC(const void *data, size_t size){
    const uint8_t *p = data;
    uint32_t crc = 0xFFFFFFFFU;
    while(size--){
        crc ^= *p++;
        for(unsigned bit = 0; bit < 8; bit++) crc = (crc >> 1) ^ (0xEDB88320U & (0U - (crc & 1U)));
    }
    return ~crc;
}
#endif
