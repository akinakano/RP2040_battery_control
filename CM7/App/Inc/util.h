#ifndef _UTIL_H_
#define _UTIL_H_

#include <stdint.h>

#define     TRUE    (1)
#define     FALSE   (0)

#define     OK  (0)
#define     NG  (-1)

int32_t isDec(int32_t c);
int32_t isHex(int32_t c);
int32_t dec2Int(const char *s, int32_t *n);
int32_t hex2Int(const char *s, int32_t *n);
int32_t inputChar(void);
int32_t inputDec(const int8_t *msg, int32_t *n, int32_t min, int32_t max);
int32_t inputHex(const int8_t *msg, int32_t *n, int32_t min, int32_t max);
void uint2Dec(uint32_t n, char *buf);
void uint2Hex(uint32_t n, uint16_t upper, char *buf);
void dumpHex(const char *msg, const char *s, uint16_t len);

#endif
