#ifndef ISO_14443B_H
#define ISO_14443B_H

#include "stdint.h"

/////////////////////////////////////////////////////////////////////
// ISO14443B COMMAND
/////////////////////////////////////////////////////////////////////
#define ISO14443B_ANTICOLLISION 0x05
#define ISO14443B_ATTRIB 0x1D
#define ISO14443B_HLTB 0x50

#define FSDI 8 // Frame Size for proximity coupling Device, in EMV test. 身份证必须FSDI = 8

/////////////////////////////////////////////////////////////////////
// 函数原型
/////////////////////////////////////////////////////////////////////
uint8_t PcdRequestB(unsigned char req_code, uint8_t AFI, uint8_t N, uint8_t *ATQB);
uint8_t PcdSlotMarker(unsigned char N, uint8_t *ATQB);
uint8_t PcdAttriB(unsigned char *PUPI, uint8_t dsi_dri, uint8_t pro_type, uint8_t CID, uint8_t *answer);
uint8_t GetIdcardNum(unsigned char *ATQB);
void SetTimeOut(unsigned int microseconds);
uint8_t ProtectSr176(uint8_t lockblock);
uint8_t CompletionSr();
uint8_t WriteSr176(uint8_t addr, uint8_t *writedata);
uint8_t ReadSr176(uint8_t addr, uint8_t *readdata);
uint8_t SelectSr(uint8_t *chip_id);
uint8_t PcdHaltB(uint8_t *PUPI);
#endif // ISO_14443B_H