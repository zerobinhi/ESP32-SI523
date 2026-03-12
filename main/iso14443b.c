#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "si523.h"
#include "iso14443b.h"

#define TP_FWT_302us 2048
#define TP_dFWT 192

unsigned char g_fwi = 4; // frame waiting time integer

//////////////////////////////////////////////////////////////////////
// 函数原型:    char pcdrequestb(u8 req_code, u8 AFI, u8 N, u8 *ATQB)
// 函数功能:    B型卡请求
// 入口参数:    req_code				// 请求代码	ISO14443_3B_REQIDL 0x00 -- 空闲的卡
//										//			ISO14443_3B_REQALL 0x08 -- 所有的卡
//				AFI						// 应用标识符，0x00：全选
//				N						// 时隙总数,取值范围0--4。
// 出口参数:    *ATQB					// 请求应答，11字节
// 返 回 值:    STATUS_SUCCESS -- 成功；其它值 -- 失败。
// 说    明:	-
//////////////////////////////////////////////////////////////////////
uint8_t PcdRequestB(uint8_t req_code, uint8_t AFI, uint8_t N, uint8_t *ATQB)
{
    uint8_t status = MI_ERR;

    unsigned int unLen;
    uint8_t ucComMF522Buf[MAXRLEN];

    SetTimeOut(1);

    ucComMF522Buf[0] = ISO14443B_ANTICOLLISION;
    ucComMF522Buf[1] = AFI;
    ucComMF522Buf[2] = (req_code & 0x08) | (N & 0x07);
    status = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 3, ucComMF522Buf, &unLen);

    if (status != MI_OK && status != MI_NOTAGERR)
    {
        status = MI_COLLERR;
    }
    if (status == MI_OK && unLen != 96)
    {
        status = MI_COM_ERR;
    }
    if (status == MI_OK)
    {
        memcpy(ATQB, &ucComMF522Buf[0], 16);
        SetTimeOut(ATQB[11] >> 4); // set FWT
        g_fwi = (ATQB[11] >> 4);   // 帧等待时间整数（4位）
    }
    return status;
}

//////////////////////////////////////////////////////////////////////
// SLOT-MARKER
//////////////////////////////////////////////////////////////////////
uint8_t PcdSlotMarker(uint8_t N, uint8_t *ATQB)
{
    uint8_t status;
    unsigned int unLen;
    uint8_t ucComMF522Buf[MAXRLEN];

    SetTimeOut(5);

    if (!N || N > 15)
    {
        status = MI_WRONG_PARAMETER_VALUE;
    }
    else
    {
        ucComMF522Buf[0] = 0x05 | (N << 4);
        status = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 1, ucComMF522Buf, &unLen);

        if (status != MI_OK && status != MI_NOTAGERR)
        {
            status = MI_COLLERR;
        }
        if (status == MI_OK && unLen != 96)
        {
            status = MI_COM_ERR;
        }
        if (status == MI_OK)
        {
            memcpy(ATQB, &ucComMF522Buf[0], 16);
            SetTimeOut(ATQB[11] >> 4); // set FWT
            g_fwi = ATQB[11] >> 4;
        }
    }
    return status;
}

//////////////////////////////////////////////////////////////////////
// ATTRIB
//  函数原型:    INchar PcdAttriB(u8 *PUPI, u8 pro_type, u8 CID, u8 *answer)
//
//  函数功能:    选择PICC
//  入口参数:    u8 *PUPI					// 4字节PICC标识符
//				u8 dsi_dri					// PCD<-->PICC 速率选择
//				u8 pro_type					// 支持的协议，由请求回应中的ProtocolType指定
//  返 回 值:    MI_OK -- 成功；其它值 -- 失败。
//  说    明:	-
//////////////////////////////////////////////////////////////////////
uint8_t PcdAttriB(uint8_t *PUPI, uint8_t dsi_dri, uint8_t pro_type, uint8_t CID, uint8_t *answer)
{
    uint8_t status;
    unsigned int unLen;
    uint8_t ucComMF522Buf[MAXRLEN];

    SetTimeOut(g_fwi);
    pro_type = pro_type;
    ucComMF522Buf[0] = ISO14443B_ATTRIB;
    memcpy(&ucComMF522Buf[1], PUPI, 4);
    ucComMF522Buf[5] = 0x00;
    ucComMF522Buf[6] = ((dsi_dri << 4) | FSDI);
    ;
    ucComMF522Buf[7] = 0x01;
    ucComMF522Buf[8] = (CID & 0x0f);

    I_SI522A_SetBitMask(0X1E, BIT7 | BIT6); // EOF SOF required

    status = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 9, ucComMF522Buf, &unLen);
    if (status == MI_OK)
    {
        *answer = ucComMF522Buf[0];
    }
    return status;
}
//////////////////////////////////////////////////////////////////////
// 获取B型卡ID
//////////////////////////////////////////////////////////////////////
uint8_t GetIdcardNum(uint8_t *pid)
{
    uint8_t status;

    unsigned int unLen;
    uint8_t ucComMF522Buf[MAXRLEN];

    ucComMF522Buf[0] = 0x00; // ISO14443B_ANTICOLLISION;     	       // APf code
    ucComMF522Buf[1] = 0x36; // AFI;                //
    ucComMF522Buf[2] = 0x00; //((req_code<<3)&0x08) | (N&0x07);  // PARAM
    ucComMF522Buf[3] = 0x00;
    ucComMF522Buf[4] = 0x08;

    status = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 5, ucComMF522Buf, &unLen);
    if (status == MI_OK)
    {
        memcpy(pid, &ucComMF522Buf[0], 10);
    }
    return status;
}

//////////////////////////////////////////////////////////////////////
// 函数原型:    char pcd_halt_b(u8 *PUPI)
// 函数功能:    挂起卡
// 入口参数:    INT8U *pPUPI					// 4字节PICC标识符
// 出口参数:    -
// 返 回 值:    MI_OK -- 成功；其它值 -- 失败。//////////////////////////////////////////////////////////////////////
uint8_t PcdHaltB(uint8_t *PUPI)
{
    uint8_t status;
    unsigned int unLen;
    uint8_t ucComMF522Buf[MAXRLEN];

    SetTimeOut(g_fwi);

    ucComMF522Buf[0] = ISO14443B_ATTRIB;
    memcpy(&ucComMF522Buf[1], PUPI, 4);

    status = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 5, ucComMF522Buf, &unLen);

    return status;
}

/**
 ****************************************************************
 * @brief select_sr()
 *
 * 防冲撞函数
 * @param:
 * @param:
 * @return: status 值为MI_OK:成功
 * @retval: chip_id  得到的SR卡片的chip_id
 ****************************************************************
 */
uint8_t SelectSr(uint8_t *chip_id)
{
    uint8_t status;
    unsigned int unLen;
    uint8_t ucComMF522Buf[MAXRLEN];

    SetTimeOut(5);

    ucComMF522Buf[0] = 0x06; // initiate card
    ucComMF522Buf[1] = 0;

    status = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 2, ucComMF522Buf, &unLen);

    if (status != MI_OK && status != MI_NOTAGERR)
    {
        status = MI_COLLERR; // collision occurs
    }
    if (unLen != 8)
    {
        status = MI_COM_ERR;
    }
    if (status == MI_OK)
    {
        SetTimeOut(5);

        ucComMF522Buf[1] = ucComMF522Buf[0];
        ucComMF522Buf[0] = 0x0E; // Slect card

        status = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 2, ucComMF522Buf, &unLen);
        if (status != MI_OK && status != MI_NOTAGERR) // collision occurs
        {
            status = MI_COLLERR; // collision occurs
        }
        if (unLen != 8)
        {
            status = MI_COM_ERR;
        }
        if (status == MI_OK)
        {
            *chip_id = ucComMF522Buf[0];
        }
    }
    return status;
}

//////////////////////////////////////////////////////////////////////
// SR176卡读块
//////////////////////////////////////////////////////////////////////
uint8_t ReadSr176(uint8_t addr, uint8_t *readdata)
{
    uint8_t status;
    unsigned int unLen;
    uint8_t ucComMF522Buf[MAXRLEN];

    SetTimeOut(5);

    ucComMF522Buf[0] = 0x08;
    ucComMF522Buf[1] = addr;

    status = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 2, ucComMF522Buf, &unLen);

    if ((status == MI_OK) && (unLen != 16))
    {
        status = MI_BITCOUNTERR;
    }
    if (status == MI_OK)
    {
        *readdata = ucComMF522Buf[0];
        *(readdata + 1) = ucComMF522Buf[1];
    }
    return status;
}
//////////////////////////////////////////////////////////////////////
// SR176卡写块
//////////////////////////////////////////////////////////////////////
uint8_t WriteSr176(uint8_t addr, uint8_t *writedata)
{
    uint8_t status;
    unsigned int unLen;
    uint8_t ucComMF522Buf[MAXRLEN];

    SetTimeOut(5);

    ucComMF522Buf[0] = 9;
    ucComMF522Buf[1] = addr;
    ucComMF522Buf[2] = *writedata;
    ucComMF522Buf[3] = *(writedata + 1);

    status = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 4, ucComMF522Buf, &unLen);

    return status;
}

//////////////////////////////////////////////////////////////////////
// SR176卡块锁定
//////////////////////////////////////////////////////////////////////
uint8_t ProtectSr176(uint8_t lockblock)
{
    uint8_t status;
    unsigned int unLen;
    uint8_t ucComMF522Buf[MAXRLEN];

    SetTimeOut(5);

    ucComMF522Buf[0] = 0x09;
    ucComMF522Buf[1] = 0x0F;
    ucComMF522Buf[2] = 0;
    ucComMF522Buf[3] = lockblock;

    status = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 4, ucComMF522Buf, &unLen);

    return status;
}

//////////////////////////////////////////////////////////////////////
// COMPLETION ST
//////////////////////////////////////////////////////////////////////
uint8_t CompletionSr()
{
    uint8_t status;
    unsigned int unLen;
    uint8_t ucComMF522Buf[MAXRLEN];

    SetTimeOut(5);
    ucComMF522Buf[0] = 0x0F;

    status = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 1, ucComMF522Buf, &unLen);

    return status;
}

/////////////////////////////////////////////////////////////////////
// 设置PCD定时器
// input:microseconds=0~15
/////////////////////////////////////////////////////////////////////
void SetTimeOut(unsigned int microseconds)

{
    unsigned long timereload;
    unsigned int tprescaler;

    if (microseconds == 0)
        microseconds = 1; // 定时时间不能为0

    tprescaler = 0;
    timereload = 0;
    while (tprescaler < 0xfff)
    {
        timereload = ((microseconds * (long)13560) - 1) / ((tprescaler * 2) + 1);

        if (timereload < 0xffff)
            break;
        tprescaler++;
    }
    timereload = timereload & 0xFFFF;

    si523_write_reg(TPrescalerReg, (tprescaler) & 0xFF);
    si523_write_reg(TModeReg, BIT7 | (((tprescaler) >> 8) & 0xFF));

    si523_write_reg(TReloadRegL, timereload >> 8);
    si523_write_reg(TReloadRegH, timereload & 0xFF);
}