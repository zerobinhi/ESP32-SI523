#include "si523.h"

static const char *TAG = "si523";

extern uint8_t PCD_IRQ_flagA;
unsigned char ACDConfigRegK_Val;
unsigned char ACDConfigRegC_Val;

esp_err_t si523_write_reg(uint8_t reg, uint8_t data)
{
    i2c_master_transmit_multi_buffer_info_t buffers[2] = {
        {.write_buffer = &reg, .buffer_size = 1},
        {.write_buffer = &data, .buffer_size = 1},
    };
    esp_err_t err = i2c_master_multi_buffer_transmit(si523_handle, buffers, 2, portMAX_DELAY);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Write reg 0x%02X failed", reg);
    }
    return err;
    // uint8_t buf[2] = {reg, data};
    // esp_err_t err = i2c_master_transmit(si523_handle, buf, 2, portMAX_DELAY);
    // if (err != ESP_OK)
    //     ESP_LOGE(TAG, "Write reg 0x%02X failed", reg);
    // return err;
}

uint8_t si523_read_reg(uint8_t reg)
{
    uint8_t data = 0;
    i2c_master_transmit_receive(si523_handle, &reg, 1, &data, 1, portMAX_DELAY);
    return data;
}

void si523_gpio_init(void)
{
    // 配置RST引脚为输出，并设置初始状态为高电平
    gpio_config_t rst_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << SI523_RST_PIN),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&rst_conf);
}

/////////////////////////////////////////////////////////////////////
// 开启天线
// 每次启动或关闭天险发射之间应至少有1ms的间隔
/////////////////////////////////////////////////////////////////////
void PcdAntennaOn(void)
{
    unsigned char i;
    i = si523_read_reg(SI523_REG_TX_CONTROL);
    if (!(i & 0x03))
    {
        I_SI523_SetBitMask(SI523_REG_TX_CONTROL, 0x03);
    }
}

/////////////////////////////////////////////////////////////////////
// 关闭天线
/////////////////////////////////////////////////////////////////////
void PcdAntennaOff(void)
{
    I_SI523_ClearBitMask(SI523_REG_TX_CONTROL, 0x03);
}

/////////////////////////////////////////////////////////////////////
// 用MF522计算CRC16函数
/////////////////////////////////////////////////////////////////////
void CalulateCRC(unsigned char *pIndata, unsigned char len, unsigned char *pOutData)
{
    unsigned char i, n;
    I_SI523_ClearBitMask(SI523_REG_DIV_IRQ, 0x04);
    si523_write_reg(SI523_REG_COMMAND, SI523_CMD_IDLE);
    I_SI523_SetBitMask(SI523_REG_FIFO_LEVEL, 0x80);
    for (i = 0; i < len; i++)
    {
        si523_write_reg(SI523_REG_FIFO_DATA, *(pIndata + i));
    }
    si523_write_reg(SI523_REG_COMMAND, SI523_CMD_CALC_CRC);
    i = 0xFF;
    do
    {
        n = si523_read_reg(SI523_REG_DIV_IRQ);
        i--;
    } while ((i != 0) && !(n & 0x04));
    pOutData[0] = si523_read_reg(SI523_REG_CRC_RESULT_L);
    pOutData[1] = si523_read_reg(SI523_REG_CRC_RESULT_H);
}

unsigned char aaa = 0;

/////////////////////////////////////////////////////////////////////
// 功    能：通过RC522和ISO14443卡通讯
// 参数说明：Command[IN]:RC522命令字
//           pInData[IN]:通过RC522发送到卡片的数据
//           InLenByte[IN]:发送数据的字节长度
//           pOutData[OUT]:接收到的卡片返回数据
//           *pOutLenBit[OUT]:返回数据的位长度
/////////////////////////////////////////////////////////////////////
// status = PcdComMF522(SI523_CMD_TRANSCEIVE,ucComMF522Buf,1,ucComMF522Buf,&unLen);
char PcdComMF522(unsigned char Command,
                 unsigned char *pInData,
                 unsigned char InLenByte,
                 unsigned char *pOutData,
                 unsigned int *pOutLenBit)
{
    char status = SI523_ERR;
    unsigned char irqEn = 0x00;
    unsigned char waitFor = 0x00;
    unsigned char lastBits;
    unsigned char n;
    unsigned int i;
    switch (Command)
    {
    case SI523_CMD_AUTHENT:
        irqEn = 0x12;
        waitFor = 0x10;
        break;
    case SI523_CMD_TRANSCEIVE:
        irqEn = 0x77;
        waitFor = 0x30;
        break;
    default:
        break;
    }

    // si523_write_reg(SI523_REG_COM_IEN,irqEn|0x80);
    I_SI523_ClearBitMask(SI523_REG_COM_IRQ, 0x80);
    si523_write_reg(SI523_REG_COMMAND, SI523_CMD_IDLE);
    I_SI523_SetBitMask(SI523_REG_FIFO_LEVEL, 0x80);

    for (i = 0; i < InLenByte; i++)
    {
        si523_write_reg(SI523_REG_FIFO_DATA, pInData[i]);
    }
    si523_write_reg(SI523_REG_COMMAND, Command);

    if (Command == SI523_CMD_TRANSCEIVE)
    {
        I_SI523_SetBitMask(SI523_REG_BIT_FRAMING, 0x80);
    }

    // i = 600;//根据时钟频率调整，操作M1卡最大等待时间25ms
    i = 2000;
    do
    {
        n = si523_read_reg(SI523_REG_COM_IRQ);
        i--;
    } while ((i != 0) && !(n & 0x01) && !(n & waitFor));
    I_SI523_ClearBitMask(SI523_REG_BIT_FRAMING, 0x80);

    if (i != 0)
    {
        aaa = si523_read_reg(SI523_REG_ERROR);

        if (!(si523_read_reg(SI523_REG_ERROR) & 0x1B))
        {
            status = SI523_OK;
            if (n & irqEn & 0x01)
            {
                status = SI523_ERR_NO_TAG;
            }
            if (Command == SI523_CMD_TRANSCEIVE)
            {
                n = si523_read_reg(SI523_REG_FIFO_LEVEL);
                lastBits = si523_read_reg(SI523_REG_CONTROL) & 0x07;
                if (lastBits)
                {
                    *pOutLenBit = (n - 1) * 8 + lastBits;
                }
                else
                {
                    *pOutLenBit = n * 8;
                }
                if (n == 0)
                {
                    n = 1;
                }
                if (n > SI523_MAX_RLEN)
                {
                    n = SI523_MAX_RLEN;
                }
                for (i = 0; i < n; i++)
                {
                    pOutData[i] = si523_read_reg(SI523_REG_FIFO_DATA);
                }
            }
        }
        else
        {
            status = SI523_ERR;
        }
    }

    I_SI523_SetBitMask(SI523_REG_CONTROL, 0x80); // stop timer now
    si523_write_reg(SI523_REG_COMMAND, SI523_CMD_IDLE);
    return status;
}

/////////////////////////////////////////////////////////////////////
// 功    能：寻卡
// 参数说明: req_code[IN]:寻卡方式
//                 0x52 = 寻感应区内所有符合14443A标准的卡
//                 0x26 = 寻未进入休眠状态的卡
//           pTagType[OUT]：卡片类型代码
//                 0x4400 = Mifare_UltraLight
//                 0x0400 = Mifare_One(S50)
//                 0x0200 = Mifare_One(S70)
//                 0x0800 = Mifare_Pro(X)
//                 0x4403 = Mifare_DESFire
// 返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////
char PcdRequest(unsigned char req_code, unsigned char *pTagType)
{
    char status;
    unsigned int unLen;
    unsigned char ucComMF522Buf[SI523_MAX_RLEN];

    I_SI523_ClearBitMask(SI523_REG_STATUS2, 0x08);
    si523_write_reg(SI523_REG_BIT_FRAMING, 0x07);
    I_SI523_SetBitMask(SI523_REG_TX_CONTROL, 0x03);

    ucComMF522Buf[0] = req_code;

    status = PcdComMF522(SI523_CMD_TRANSCEIVE, ucComMF522Buf, 1, ucComMF522Buf, &unLen);
    if ((status == SI523_OK) && (unLen == 0x10))
    {
        *pTagType = ucComMF522Buf[0];
        *(pTagType + 1) = ucComMF522Buf[1];
    }
    else
    {
        status = SI523_ERR;
    }

    return status;
}

/////////////////////////////////////////////////////////////////////
// 功    能：防冲撞
// 参数说明: pSnr[OUT]:卡片序列号，4字节
// 返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////
char PcdAnticoll(unsigned char *pSnr, unsigned char anticollision_level)
{
    char status;
    unsigned char i, snr_check = 0;
    unsigned int unLen;
    unsigned char ucComMF522Buf[SI523_MAX_RLEN];

    I_SI523_ClearBitMask(SI523_REG_STATUS2, 0x08);
    si523_write_reg(SI523_REG_BIT_FRAMING, 0x00);
    I_SI523_ClearBitMask(SI523_REG_COLL, 0x80);

    ucComMF522Buf[0] = anticollision_level;
    ucComMF522Buf[1] = 0x20;

    status = PcdComMF522(SI523_CMD_TRANSCEIVE, ucComMF522Buf, 2, ucComMF522Buf, &unLen);

    if (status == SI523_OK)
    {
        for (i = 0; i < 4; i++)
        {
            *(pSnr + i) = ucComMF522Buf[i];
            snr_check ^= ucComMF522Buf[i];
        }
        if (snr_check != ucComMF522Buf[i])
        {
            status = SI523_ERR;
        }
    }

    I_SI523_SetBitMask(SI523_REG_COLL, 0x80);
    return status;
}

/////////////////////////////////////////////////////////////////////
// 功    能：选定卡片
// 参数说明: pSnr[IN]:卡片序列号，4字节
// 返    回: 成功返回MI_OK
////////////////////////////////////////////////////////////////////
char PcdSelect(unsigned char *pSnr, unsigned char *sak)
{
    char status;
    unsigned char i;
    unsigned int unLen;
    unsigned char ucComMF522Buf[SI523_MAX_RLEN];

    ucComMF522Buf[0] = SI523_PICC_ANTICOLL1;
    ucComMF522Buf[1] = 0x70;
    ucComMF522Buf[6] = 0;
    for (i = 0; i < 4; i++)
    {
        ucComMF522Buf[i + 2] = *(pSnr + i);
        ucComMF522Buf[6] ^= *(pSnr + i);
    }
    CalulateCRC(ucComMF522Buf, 7, &ucComMF522Buf[7]);

    I_SI523_ClearBitMask(SI523_REG_STATUS2, 0x08);

    status = PcdComMF522(SI523_CMD_TRANSCEIVE, ucComMF522Buf, 9, ucComMF522Buf, &unLen);

    if ((status == SI523_OK) && (unLen == 0x18))
    {
        *sak = ucComMF522Buf[0];
        status = SI523_OK;
    }
    else
    {
        status = SI523_ERR;
    }

    return status;
}

char PcdSelect1(unsigned char *pSnr, unsigned char *sak)
{
    char status;
    unsigned char i;
    unsigned int unLen;
    unsigned char ucComMF522Buf[SI523_MAX_RLEN];

    ucComMF522Buf[0] = SI523_PICC_ANTICOLL1;
    ucComMF522Buf[1] = 0x70;
    ucComMF522Buf[6] = 0;
    for (i = 0; i < 4; i++)
    {
        ucComMF522Buf[i + 2] = *(pSnr + i);
        ucComMF522Buf[6] ^= *(pSnr + i);
    }
    CalulateCRC(ucComMF522Buf, 7, &ucComMF522Buf[7]);

    I_SI523_ClearBitMask(SI523_REG_STATUS2, 0x08);

    status = PcdComMF522(SI523_CMD_TRANSCEIVE, ucComMF522Buf, 9, ucComMF522Buf, &unLen);

    if ((status == SI523_OK) && (unLen == 0x18))
    {
        *sak = ucComMF522Buf[0];
        status = SI523_OK;
    }
    else
    {
        status = SI523_ERR;
    }

    return status;
}

char PcdSelect2(unsigned char *pSnr, unsigned char *sak)
{
    char status;
    unsigned char i;
    unsigned int unLen;
    unsigned char ucComMF522Buf[SI523_MAX_RLEN];

    ucComMF522Buf[0] = SI523_PICC_ANTICOLL2;
    ucComMF522Buf[1] = 0x70;
    ucComMF522Buf[6] = 0;
    for (i = 0; i < 4; i++)
    {
        ucComMF522Buf[i + 2] = *(pSnr + i);
        ucComMF522Buf[6] ^= *(pSnr + i);
    }
    CalulateCRC(ucComMF522Buf, 7, &ucComMF522Buf[7]);

    I_SI523_ClearBitMask(SI523_REG_STATUS2, 0x08);

    status = PcdComMF522(SI523_CMD_TRANSCEIVE, ucComMF522Buf, 9, ucComMF522Buf, &unLen);

    if ((status == SI523_OK) && (unLen == 0x18))
    {
        *sak = ucComMF522Buf[0];
        status = SI523_OK;
    }
    else
    {
        status = SI523_ERR;
    }

    return status;
}

char PcdSelect3(unsigned char *pSnr, unsigned char *sak)
{
    char status;
    unsigned char i;
    unsigned int unLen;
    unsigned char ucComMF522Buf[SI523_MAX_RLEN];

    ucComMF522Buf[0] = SI523_PICC_ANTICOLL2;
    ucComMF522Buf[1] = 0x70;
    ucComMF522Buf[6] = 0;
    for (i = 0; i < 4; i++)
    {
        ucComMF522Buf[i + 2] = *(pSnr + i);
        ucComMF522Buf[6] ^= *(pSnr + i);
    }
    CalulateCRC(ucComMF522Buf, 7, &ucComMF522Buf[7]);

    I_SI523_ClearBitMask(SI523_REG_STATUS2, 0x08);

    status = PcdComMF522(SI523_CMD_TRANSCEIVE, ucComMF522Buf, 9, ucComMF522Buf, &unLen);

    if ((status == SI523_OK) && (unLen == 0x18))
    {
        *sak = ucComMF522Buf[0];
        status = SI523_OK;
    }
    else
    {
        status = SI523_ERR;
    }

    return status;
}

/////////////////////////////////////////////////////////////////////
// 功    能：命令卡片进入休眠状态
// 返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////
char PcdHalt(void)
{
    char status;
    unsigned int unLen;
    unsigned char ucComMF522Buf[SI523_MAX_RLEN];

    ucComMF522Buf[0] = SI523_PICC_HALT;
    ucComMF522Buf[1] = 0;
    CalulateCRC(ucComMF522Buf, 2, &ucComMF522Buf[2]);

    status = PcdComMF522(SI523_CMD_TRANSCEIVE, ucComMF522Buf, 4, ucComMF522Buf, &unLen);

    return status;
}

/////////////////////////////////////////////////////////////////////
// 功    能：验证卡片密码
// 参数说明: auth_mode[IN]: 密码验证模式
//                  0x60 = 验证A密钥
//                  0x61 = 验证B密钥
//           addr[IN]：块地址
//           pKey[IN]：密码
//           pSnr[IN]：卡片序列号，4字节
// 返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////
char PcdAuthState(unsigned char auth_mode, unsigned char addr, unsigned char *pKey, unsigned char *pSnr)
{
    char status;
    unsigned int unLen;
    unsigned char ucComMF522Buf[SI523_MAX_RLEN];

    ucComMF522Buf[0] = auth_mode;
    ucComMF522Buf[1] = addr;
    memcpy(&ucComMF522Buf[2], pKey, 6);
    memcpy(&ucComMF522Buf[8], pSnr, 6);

    status = PcdComMF522(SI523_CMD_AUTHENT, ucComMF522Buf, 12, ucComMF522Buf, &unLen);
    if ((status != SI523_OK) || (!(si523_read_reg(SI523_REG_STATUS2) & 0x08)))
    {
        status = SI523_ERR;
    }

    return status;
}

/////////////////////////////////////////////////////////////////////
// 功    能：读取M1卡一块数据
// 参数说明: addr[IN]：块地址
//           pData[OUT]：读出的数据，16字节
// 返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////
char PcdRead(unsigned char addr, unsigned char *pData)
{
    char status;
    unsigned int unLen;
    unsigned char ucComMF522Buf[SI523_MAX_RLEN];

    ucComMF522Buf[0] = SI523_PICC_READ;
    ucComMF522Buf[1] = addr;
    CalulateCRC(ucComMF522Buf, 2, &ucComMF522Buf[2]);

    status = PcdComMF522(SI523_CMD_TRANSCEIVE, ucComMF522Buf, 4, ucComMF522Buf, &unLen);
    if ((status == SI523_OK) && (unLen == 0x90))
    {
        memcpy(pData, ucComMF522Buf, 16);
    }
    else
    {
        status = SI523_ERR;
    }

    return status;
}

/////////////////////////////////////////////////////////////////////
// 功    能：写数据到M1卡一块
// 参数说明: addr[IN]：块地址
//           pData[IN]：写入的数据，16字节
// 返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////
char PcdWrite(unsigned char addr, unsigned char *pData)
{
    char status;
    unsigned int unLen;
    unsigned char ucComMF522Buf[SI523_MAX_RLEN];

    ucComMF522Buf[0] = SI523_PICC_WRITE;
    ucComMF522Buf[1] = addr;
    CalulateCRC(ucComMF522Buf, 2, &ucComMF522Buf[2]);

    status = PcdComMF522(SI523_CMD_TRANSCEIVE, ucComMF522Buf, 4, ucComMF522Buf, &unLen);

    if ((status != SI523_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
    {
        status = SI523_ERR;
    }

    if (status == SI523_OK)
    {
        memcpy(ucComMF522Buf, pData, 16);
        CalulateCRC(ucComMF522Buf, 16, &ucComMF522Buf[16]);

        status = PcdComMF522(SI523_CMD_TRANSCEIVE, ucComMF522Buf, 18, ucComMF522Buf, &unLen);
        if ((status != SI523_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
        {
            status = SI523_ERR;
        }
    }

    return status;
}

/*===============================
 函数功能：读A卡初始化配置

 ================================*/
void PCD_SI523_TypeA_Init(void)
{

    I_SI523_ClearBitMask(SI523_REG_STATUS2, 0x08);
    // Reset baud rates
    si523_write_reg(SI523_REG_TX_MODE, 0x00);
    si523_write_reg(SI523_REG_RX_MODE, 0x00);
    // Reset SI523_REG_MOD_WIDTH
    si523_write_reg(SI523_REG_MOD_WIDTH, 0x26);
    // RxGain:110,43dB by default;
    si523_write_reg(SI523_REG_RF_CFG, SI523_RF_CFG_DEFAULT);
    // When communicating with a PICC we need a timeout if something goes wrong.
    // f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
    // TPrescaler_Hi are the four low bits in SI523_REG_T_MODE. TPrescaler_Lo is SI523_REG_T_PRESCALER.
    si523_write_reg(SI523_REG_T_MODE, 0x80);      // TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
    si523_write_reg(SI523_REG_T_PRESCALER, 0xa9); // TPreScaler = SI523_REG_T_MODE[3..0]:SI523_REG_T_PRESCALER
    si523_write_reg(SI523_REG_T_RELOAD_H, 0x03);   // Reload timer
    si523_write_reg(SI523_REG_T_RELOAD_L, 0xe8);   // Reload timer
    si523_write_reg(SI523_REG_TX_AUTO, 0x40);      // Default 0x00. Force a 100 % ASK modulation independent of the SI523_REG_MOD_GS_P register setting
    si523_write_reg(SI523_REG_MODE, 0x3D);       // Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
    si523_write_reg(SI523_REG_COMMAND, 0x00);    // Turn on the analog part of receiver

    PcdAntennaOn();
}

/*===============================
 函数功能：读A卡

 ================================*/
char PCD_SI523_TypeA_GetUID(void)
{
    unsigned char ATQA[2];
    unsigned char UID[12];
    unsigned char SAK = 0;
    unsigned char UID_complate1 = 0;
    unsigned char UID_complate2 = 0;

    ESP_LOGI(TAG, "Test_SI523_GetUID");
    si523_write_reg(SI523_REG_RF_CFG, SI523_RF_CFG_DEFAULT); // 复位接收增益

    // 寻卡
    if (PcdRequest(SI523_PICC_REQ_IDL, ATQA) != SI523_OK) // 寻天线区内未进入休眠状态的卡，返回卡片类型 2字节
    {
        si523_write_reg(SI523_REG_RF_CFG, 0x48);
        if (PcdRequest(SI523_PICC_REQ_IDL, ATQA) != SI523_OK)
        {
            si523_write_reg(SI523_REG_RF_CFG, 0x58);
            if (PcdRequest(SI523_PICC_REQ_IDL, ATQA) != SI523_OK)
            {
                ESP_LOGI(TAG, "Request:fail");
                return 1;
            }
            else
            {
                ESP_LOGI(TAG, "Request1:ok  ATQA:%02x %02x", ATQA[0], ATQA[1]);
            }
        }
        else
        {
            ESP_LOGI(TAG, "Request2:ok  ATQA:%02x %02x", ATQA[0], ATQA[1]);
        }
    }
    else
    {
        ESP_LOGI(TAG, "Request3:ok  ATQA:%02x %02x", ATQA[0], ATQA[1]);
    }

    // UID长度=4
    // Anticoll 冲突检测 level1
    if (PcdAnticoll(UID, SI523_PICC_ANTICOLL1) != SI523_OK)
    {
        ESP_LOGI(TAG, "Anticoll1:fail");
        return 1;
    }
    else
    {
        if (PcdSelect1(UID, &SAK) != SI523_OK)
        {
            ESP_LOGI(TAG, "Select1:fail");
            return 1;
        }
        else
        {
            ESP_LOGI(TAG, "Select1:ok  SAK1:%02x", SAK);
            if (SAK & 0x04)
            {
                UID_complate1 = 0;

                // UID长度=7
                if (UID_complate1 == 0)
                {
                    // Anticoll 冲突检测 level2
                    if (PcdAnticoll(UID + 4, SI523_PICC_ANTICOLL2) != SI523_OK)
                    {
                        ESP_LOGI(TAG, "Anticoll2:fail");
                        return 1;
                    }
                    else
                    {
                        if (PcdSelect2(UID + 4, &SAK) != SI523_OK)
                        {
                            ESP_LOGI(TAG, "Select2:fail");
                            return 1;
                        }
                        else
                        {
                            ESP_LOGI(TAG, "Select2:ok  SAK2:%02x", SAK);
                            if (SAK & 0x04)
                            {
                                UID_complate2 = 0;

                                // UID长度=10
                                if (UID_complate2 == 0)
                                {
                                    // Anticoll 冲突检测 level3
                                    if (PcdAnticoll(UID + 8, SI523_PICC_ANTICOLL3) != SI523_OK)
                                    {
                                        ESP_LOGI(TAG, "Anticoll3:fail");
                                        return 1;
                                    }
                                    else
                                    {
                                        if (PcdSelect3(UID + 8, &SAK) != SI523_OK)
                                        {
                                            ESP_LOGI(TAG, "Select3:fail");
                                            return 1;
                                        }
                                        else
                                        {
                                            ESP_LOGI(TAG, "Select3:ok  SAK3:%02x", SAK);
                                            if (SAK & 0x04)
                                            {
                                                //												UID_complate3 = 0;
                                            }
                                            else
                                            {
                                                //											UID_complate3 = 1;
                                                ESP_LOGI(TAG, "Anticoll3:ok  UID:%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
                                                         UID[1], UID[2], UID[3], UID[5], UID[6], UID[7], UID[8], UID[9], UID[10], UID[11]);
                                            }
                                        }
                                    }
                                }
                            }
                            else
                            {
                                UID_complate2 = 1;
                                ESP_LOGI(TAG, "Anticoll2:ok  UID:%02x %02x %02x %02x %02x %02x %02x",
                                         UID[1], UID[2], UID[3], UID[4], UID[5], UID[6], UID[7]);
                            }
                        }
                    }
                }
            }
            else
            {
                UID_complate1 = 1;
                ESP_LOGI(TAG, "Anticoll1:ok  UID:%02x %02x %02x %02x", UID[0], UID[1], UID[2], UID[3]);
            }
        }
    }
    // Halt
    //	if(PcdHalt() != SI523_OK)
    //	{
    //		ESP_LOGI(TAG, "Halt:fail");
    //		return 1;
    //	}
    //	else
    //	{
    //		ESP_LOGI(TAG, "Halt:ok");
    //	}

    // delay_us(100);
    vTaskDelay(pdMS_TO_TICKS(1));
    return 0;
}

/*===============================
 函数功能：读A卡扇区

 ================================*/
char PCD_SI523_TypeA_rw_block(void)
{
    unsigned char ATQA[2];
    unsigned char UID[12];
    unsigned char SAK = 0;
    unsigned char CardReadBuf[16] = {0};
    unsigned char CardWriteBuf[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
    unsigned char DefaultKeyABuf[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    ESP_LOGI(TAG, "Test_Si522_GetCard");

    // request 寻卡
    if (PcdRequest(SI523_PICC_REQ_IDL, ATQA) != SI523_OK) // 寻天线区内未进入休眠状态的卡，返回卡片类型 2字节
    {
        ESP_LOGI(TAG, "Request:fail");
        return 1;
    }
    else
    {
        ESP_LOGI(TAG, "Request:ok  ATQA:%02x %02x", ATQA[0], ATQA[1]);
    }

    // Anticoll 冲突检测
    if (PcdAnticoll(UID, SI523_PICC_ANTICOLL1) != SI523_OK)
    {
        ESP_LOGI(TAG, "Anticoll:fail");
        return 1;
    }
    else
    {
        ESP_LOGI(TAG, "Anticoll:ok  UID:%02x %02x %02x %02x", UID[0], UID[1], UID[2], UID[3]);
    }

    // Select 选卡
    if (PcdSelect1(UID, &SAK) != SI523_OK)
    {
        ESP_LOGI(TAG, "Select:fail");
        return 1;
    }
    else
    {
        ESP_LOGI(TAG, "Select:ok  SAK:%02x", SAK);
    }

    // Authenticate 验证密码
    if (PcdAuthState(SI523_PICC_AUTH_A, 4, DefaultKeyABuf, UID) != SI523_OK)
    {
        ESP_LOGI(TAG, "Authenticate:fail");
        return 1;
    }
    else
    {
        ESP_LOGI(TAG, "Authenticate:ok");
    }

    // 读BLOCK原始数据
    if (PcdRead(4, CardReadBuf) != SI523_OK)
    {
        ESP_LOGI(TAG, "PcdRead:fail");
        return 1;
    }
    else
    {
        ESP_LOGI(TAG, "PcdRead:ok  ");
        for (unsigned char i = 0; i < 16; i++)
        {
            ESP_LOGI(TAG, "%02x", CardReadBuf[i]);
        }
    }

    // 产生随机数
    for (unsigned char i = 0; i < 16; i++)
        CardWriteBuf[i] = rand();

    // 写BLOCK 写入新的数据
    if (PcdWrite(4, CardWriteBuf) != SI523_OK)
    {
        ESP_LOGI(TAG, "PcdWrite:fail");
        return 1;
    }
    else
    {
        ESP_LOGI(TAG, "PcdWrite:ok  ");
        for (unsigned char i = 0; i < 16; i++)
        {
            ESP_LOGI(TAG, "%02x", CardWriteBuf[i]);
        }
    }

    // 读BLOCK 读出新写入的数据
    if (PcdRead(4, CardReadBuf) != SI523_OK)
    {
        ESP_LOGI(TAG, "PcdRead:fail");
        return 1;
    }
    else
    {
        ESP_LOGI(TAG, "PcdRead:ok  ");
        for (unsigned char i = 0; i < 16; i++)
        {
            ESP_LOGI(TAG, "%02x", CardReadBuf[i]);
        }
    }

    //	//Halt
    //	if(PcdHalt() != SI523_OK)
    //	{
    //		ESP_LOGI(TAG, "Halt:fail");
    //		return 1;
    //	}
    //	else
    //	{
    //		ESP_LOGI(TAG, "Halt:ok");
    //	}

    return 0;
}

/*===============================
 函数功能：读B卡初始化配置

 ================================*/
void PCD_SI523_TypeB_Init(void)
{

    I_SI523_ClearBitMask(SI523_REG_STATUS2, 0x08);
    si523_write_reg(SI523_REG_MODE, 0x3F); // For 0xFFFF crc
    si523_write_reg(SI523_REG_T_RELOAD_L, 30);
    si523_write_reg(SI523_REG_T_RELOAD_H, 0);
    si523_write_reg(SI523_REG_T_MODE, 0x8D);
    si523_write_reg(SI523_REG_T_PRESCALER, 0x3E);
    si523_write_reg(SI523_REG_TX_AUTO, 0);         // Force 100ASK = 0//		delay_ms(100);
    si523_write_reg(SI523_REG_GS_N_ON, 0xff);        // TX输出电导设置f8 fa N
    si523_write_reg(SI523_REG_CW_GS_P, 0x3f);      // P_改变1的幅度
    si523_write_reg(SI523_REG_MOD_GS_P, 0x07);     // 调制指数设置RegModGsp,, TYPEB ModConductance 0x1A P_改变0的幅度
    si523_write_reg(SI523_REG_TX_MODE, 0x83);     // 编码器设置,106kbps,14443B 03
    si523_write_reg(SI523_REG_BIT_FRAMING, 0x00); // 调制脉宽,0x13->2.95us RegTypeBFraming ,,TYPEB
    si523_write_reg(SI523_REG_AUTO_TEST, 0x00);
    // 低二位为接收增益，
    // 00,10,20,30,40,50,60,70
    // 18,23,18,23,33,38,43,48dB
    si523_write_reg(SI523_REG_RF_CFG, SI523_RF_CFG_DEFAULT);
    si523_write_reg(SI523_REG_RX_MODE, 0x83);
    si523_write_reg(SI523_REG_RX_THRESHOLD, 0x65);
    I_SI523_ClearBitMask(SI523_REG_RX_SEL, 0x3F);
    I_SI523_SetBitMask(SI523_REG_RX_SEL, 0x08);
    I_SI523_ClearBitMask(SI523_REG_TX_MODE, 0x80); // 无CRC,无奇偶校验
    I_SI523_ClearBitMask(SI523_REG_RX_MODE, 0x80);
    I_SI523_ClearBitMask(SI523_REG_STATUS2, 0x08); // MFCrypto1On =0

    PcdAntennaOn();
}

/*=================================
 函数功能：循环读取A卡UID

=================================*/
void PCD_SI523_TypeA(void)
{
    while (1)
    {
        PCD_SI523_TypeA_GetUID(); // 读A卡

        // PCD_SI523_TypeA_rw_block();		//读A卡扇区

        // delay_ms(500);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/*================================
 函数功能：循环读取B卡UID

=================================*/
void PCD_SI523_TypeB(void)
{
    while (1)
    {
        PCD_SI523_TypeB_GetUID(); // 读B卡

        // PCD_SI523_IdentityCard_GetUID();		//读身份证

        // delay_ms(500);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

char PCD_SI523_TypeB_GetUID(void)
{

    ESP_LOGI(TAG, "Test_B_GetUID");

    // si523_write_reg(0x02, 0xa0); //打开接收中断,则读卡会产生中断
    //  Enable external interrupt
    // EXTI->IMR |= 0x00000008;

    // request 寻B卡;返回卡号
    unsigned int len1;
    unsigned char buf1[18] = {0x05, 0x00, 0x00, 0x71, 0xFF};

    if (PcdComMF522(SI523_CMD_TRANSCEIVE, buf1, 5, buf1, &len1) != SI523_OK)
    {
        ESP_LOGI(TAG, "Request:fail");
        return 1;
    }
    else
    {
        if (buf1[0] == 0x50) // 判断是不是ATQB
            ESP_LOGI(TAG, "Request:ok  UID:%02x %02x %02x %02x",
                     buf1[1], buf1[2], buf1[3], buf1[4]);
    }

    return 0;
}

char PCD_SI523_IdentityCard_GetUID(void)
{
    // ESP_LOGI(TAG, "Test_identitycard");

    // request 寻B卡
    unsigned int len1;
    unsigned char buf1[18] = {0x05, 0x00, 0x00, 0x71, 0xFF};

    if (PcdComMF522(SI523_CMD_TRANSCEIVE, buf1, 5, buf1, &len1) != SI523_OK)
    {
        ESP_LOGI(TAG, "Request:fail");
        return 1;
    }

    // si523_write_reg(0x02, 0xa0); //打开接收中断,则读卡会产生中断
    //  Enable external interrupt
    // EXTI->IMR |= 0x00000008;

    // 发送二代证非标ATTRIB指令
    unsigned int len2;
    unsigned char buf2[18] = {0x1D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x01, 0x08, 0xF3, 0x10};
    if (PcdComMF522(SI523_CMD_TRANSCEIVE, buf2, 11, buf2, &len2) != SI523_OK)
    {
        ESP_LOGI(TAG, "ATTRIB:fail");
        return 1;
    }

    // 获取UID
    unsigned int len3;
    unsigned char buf3[18] = {0x00, 0x36, 0x00, 0x00, 0x08, 0x57, 0x44};

    if (PcdComMF522(SI523_CMD_TRANSCEIVE, buf3, 7, buf3, &len3) != SI523_OK)
    {
        ESP_LOGI(TAG, "UID:fail");
        return 1;
    }
    else
    {
        if (buf3[8] == 0x90 || buf3[9] == 0x00) // 判断是不是identitycard
            ESP_LOGI(TAG, "UID:ok  UID:%02x %02x %02x %02x %02x %02x %02x %02x ",
                     buf3[0], buf3[1], buf3[2], buf3[3], buf3[4], buf3[5], buf3[6], buf3[7]);
    }

    return 0;
}

//***********************************//修改新增内容

/*
 * 函数名：PcdReset
 * 描述  ：复位RC522
 * 输入  ：无
 * 返回  : 无
 * 调用  ：外部调用
 */
void PcdReset(void)
{
    // hard reset
    //	HAL_GPIO_WritePin(S52_NRSTPD_GPIO_Port,S52_NRSTPD_Pin,GPIO_PIN_RESET);
    //	// delay_us(100);
    //	HAL_GPIO_WritePin(S52_NRSTPD_GPIO_Port,S52_NRSTPD_Pin,GPIO_PIN_SET);
    //	// delay_us(100);

    si523_write_reg(SI523_REG_COMMAND, 0x0f); // 向CommandReg 写入 0x0f	作用是使RC522复位
    while (si523_read_reg(SI523_REG_COMMAND) & 0x10)
        ; // Powerdown位为0时，表示RC522已准备好
    // delay_us(100);
    vTaskDelay(pdMS_TO_TICKS(1));
}

void Pcd_Hard_Reset(void)
{
    gpio_set_level(SI523_RST_PIN, 0);
    // delay_ms(2);
    vTaskDelay(pdMS_TO_TICKS(2));
    gpio_set_level(SI523_RST_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(2));
    // delay_ms(2);
}

// SI523_interfaces
void I_SI523_ClearBitMask(unsigned char reg, unsigned char mask)
{
    char tmp = 0x00;
    tmp = si523_read_reg(reg);
    si523_write_reg(reg, tmp & ~mask); // clear bit mask
}

void I_SI523_SetBitMask(unsigned char reg, unsigned char mask)
{
    char tmp = 0x00;
    tmp = si523_read_reg(reg);
    si523_write_reg(reg, tmp | mask); // set bit mask
}

void I_SI523_SiModifyReg(unsigned char RegAddr, unsigned char ModifyVal, unsigned char MaskByte)
{
    unsigned char RegVal;
    RegVal = si523_read_reg(RegAddr);
    if (ModifyVal)
    {
        RegVal |= MaskByte;
    }
    else
    {
        RegVal &= (~MaskByte);
    }
    si523_write_reg(RegAddr, RegVal);
}

/*===============================
 函数功能：ACD模式初始化配置

 ================================*/
void ACD_init_Fun(void)
{
    PCD_SI523_TypeA_Init(); // 读A卡初始化配置

    PCD_ACD_AutoCalc(); // 自动获取阈值

    PCD_ACD_Init(); // ACD初始化配置
}

/*===============================
 函数功能：ACD寻卡

 ================================*/
void ACD_Fun(void)
{
    // EXTI->IMR |= 0x00000008; // Enable external interrupt
    gpio_intr_enable(SI523_INT_PIN); // Enable GPIO interrupt
    PCD_IRQ_flagA = 0;               // clear IRQ flag
}

/*===============================
 函数功能：自动获取阈值

 ================================*/
void PCD_ACD_AutoCalc(void)
{
    unsigned char temp;
    unsigned char temp_Compare = 0;
    unsigned char VCON_TR[8] = {0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f}; // acd灵敏度调节
    unsigned char TR_Compare[4] = {0x00, 0x00, 0x00, 0x00};
    ACDConfigRegC_Val = 0x7f;
    unsigned char ACDConfigRegK_RealVal = 0;

    si523_write_reg(SI523_REG_TX_CONTROL, 0x83);  // 打开天线
    I_SI523_SetBitMask(SI523_REG_COMMAND, 0x06); // 开启ADC_EXCUTE
    // delay_us(200);
    vTaskDelay(pdMS_TO_TICKS(1));

    for (int i = 7; i > 0; i--)
    {
        si523_write_reg(SI523_REG_PAGE2, (SI523_ACD_REG_LPD_CFG1 << 2) | 0x40);
        si523_write_reg(SI523_REG_ACD_CFG, VCON_TR[i]);

        si523_write_reg(SI523_REG_PAGE2, (SI523_ACD_REG_ADC_VAL << 2) | 0x40);
        temp_Compare = si523_read_reg(SI523_REG_ACD_CFG);
        for (int m = 0; m < 100; m++)
        {
            si523_write_reg(SI523_REG_PAGE2, (SI523_ACD_REG_ADC_VAL << 2) | 0x40);
            temp = si523_read_reg(SI523_REG_ACD_CFG);

            if (temp == 0)
                break; // 处在接近的VCON值附近值，如果偶合出现0值，均有概率误触发，应舍弃该值。

            temp_Compare = (temp_Compare + temp) / 2;
            // delay_us(100);
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        if (temp_Compare == 0 || temp_Compare == 0x7f) // 比较当前值和所存值
        {
        }
        else
        {
            if (temp_Compare < ACDConfigRegC_Val)
            {
                ACDConfigRegC_Val = temp_Compare;
                ACDConfigRegK_Val = VCON_TR[i];
            }
        }
    } // 取得最接近的参考电压VCON

    ACDConfigRegK_RealVal = ACDConfigRegK_Val; // 取得最接近的参考电压VCON

    for (int j = 0; j < 4; j++)
    {
        si523_write_reg(SI523_REG_PAGE2, (SI523_ACD_REG_LPD_CFG1 << 2) | 0x40);
        si523_write_reg(SI523_REG_ACD_CFG, j * 32 + ACDConfigRegK_Val);

        si523_write_reg(SI523_REG_PAGE2, (SI523_ACD_REG_ADC_VAL << 2) | 0x40);
        temp_Compare = si523_read_reg(SI523_REG_ACD_CFG);
        for (int n = 0; n < 100; n++)
        {
            si523_write_reg(SI523_REG_PAGE2, (SI523_ACD_REG_ADC_VAL << 2) | 0x40);
            temp = si523_read_reg(SI523_REG_ACD_CFG);
            temp_Compare = (temp_Compare + temp) / 2;
            // delay_us(100);
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        TR_Compare[j] = temp_Compare;
    } // 再调TR的档位，将采集值填入TR_Compare[]

    for (int z = 0; z < 3; z++)
    {
        if (TR_Compare[z] == 0x7f)
        {
        }
        else
        {
            ACDConfigRegC_Val = TR_Compare[z]; // 最终选择的配置
            ACDConfigRegK_Val = ACDConfigRegK_RealVal + z * 32;
        }
    } // 再选出一个非7f大值

    si523_write_reg(SI523_REG_PAGE2, (SI523_ACD_REG_LPD_CFG1 << 2) | 0x40);
    ESP_LOGI(TAG, "ACDConfigRegK_Val:%02x ", ACDConfigRegK_Val);

    I_SI523_SetBitMask(SI523_REG_COMMAND, 0x06); // 关闭ADC_EXCUTE
}

/*===============================
 函数功能：ACD初始化配置

 ================================*/
void PCD_ACD_Init(void)
{
    si523_write_reg(SI523_REG_DIV_IRQ, 0x60); ////清中断，该处不清中断，进入ACD模式后会异常产生有卡中断。
    si523_write_reg(SI523_REG_PAGE2, (SI523_ACD_REG_ACC_CFG << 2) | 0x40);
    si523_write_reg(SI523_REG_ACD_CFG, 0x55); // Clear ACC_IRQ

    si523_write_reg(SI523_REG_PAGE2, (SI523_ACD_REG_RCCFG1 << 2) | 0x40); // 设置轮询时间
    si523_write_reg(SI523_REG_ACD_CFG, SI523_ACD_RCCFG1_DEFAULT);
    si523_write_reg(SI523_REG_PAGE2, (SI523_ACD_REG_ACRDCFG << 2) | 0x40); // 设置相对模式或者绝对模式
    si523_write_reg(SI523_REG_ACD_CFG, SI523_ACD_ACRDCFG_DEFAULT);
    si523_write_reg(SI523_REG_PAGE2, (SI523_ACD_REG_MAN_REF << 2) | 0x40); // 设置无卡场强值
    si523_write_reg(SI523_REG_ACD_CFG, ACDConfigRegC_Val);
    si523_write_reg(SI523_REG_PAGE2, (SI523_ACD_REG_VAL_DELTA << 2) | 0x40); // 设置灵敏度，一般建议为4，在调试时，可以适当降低验证该值，验证ACD功能
    si523_write_reg(SI523_REG_ACD_CFG, SI523_ACD_VAL_DELTA_DEFAULT);
    si523_write_reg(SI523_REG_PAGE2, (SI523_ACD_REG_WDT_CNT << 2) | 0x40); // 设置看门狗定时器时间
    si523_write_reg(SI523_REG_ACD_CFG, SI523_ACD_WDT_CNT_DEFAULT);
    si523_write_reg(SI523_REG_PAGE2, (SI523_ACD_REG_ARI_CFG << 2) | 0x40); // 设置ARI功能，在天线场强打开前1us产生ARI电平控制触摸芯片Si12T的硬件屏蔽引脚SCT
    si523_write_reg(SI523_REG_ACD_CFG, SI523_ACD_ARI_CFG_DEFAULT);
    si523_write_reg(SI523_REG_PAGE2, (SI523_ACD_REG_LPD_CFG1 << 2) | 0x40); // 设置ADC的基准电压和放大增益
    si523_write_reg(SI523_REG_ACD_CFG, ACDConfigRegK_Val);
    si523_write_reg(SI523_REG_PAGE2, (SI523_ACD_REG_RF_LOW_DET << 2) | 0x40); // 设置监测ACD功能是否产生场强，意外产生可能导致读卡芯片复位或者寄存器丢失
    si523_write_reg(SI523_REG_ACD_CFG, SI523_ACD_RF_LOW_DET_DEFAULT);
    si523_write_reg(SI523_REG_PAGE2, (SI523_ACD_REG_IRQ_EN << 2) | 0x40); // 设置ACD模式下相关功能的标志位传导到IRQ引脚
    si523_write_reg(SI523_REG_ACD_CFG, SI523_ACD_IRQ_EN_DEFAULT);

    si523_write_reg(SI523_REG_COM_IEN, SI523_COM_IEN_DEFAULT); // ComIEnReg，DivIEnReg   设置IRQ选择上升沿或者下降沿
    si523_write_reg(SI523_REG_DIV_IEN, SI523_DIV_IEN_DEFAULT);

    si523_write_reg(SI523_REG_PAGE2, (SI523_ACD_REG_ACC_CFG << 2) | 0x40); // 设置监测ACD功能下的重要寄存器的配置值，寄存器丢失后会立即产生中断
    si523_write_reg(SI523_REG_ACD_CFG, SI523_ACD_ACC_CFG_DEFAULT);           // 写非0x55的值即开启功能，写0x55清除使能停止功能。

    si523_write_reg(SI523_REG_COMMAND, 0xb0); // 进入ACD
}

char PCD_IRQ(void)
{
    unsigned char status_SI523CD_IRQ;
    unsigned char temp_SI523CD_IRQ;

    temp_SI523CD_IRQ = si523_read_reg(SI523_REG_DIV_IRQ);
    if (temp_SI523CD_IRQ & 0x40) // ACD中断
    {
        si523_write_reg(SI523_REG_DIV_IRQ, 0x40); // Clear ACDIRq

        status_SI523CD_IRQ = 1;
        return status_SI523CD_IRQ;
    }

    if (temp_SI523CD_IRQ & 0x20) // ACD看门狗中断
    {
        si523_write_reg(SI523_REG_DIV_IRQ, 0x20); // Clear ACDTIMER_IRQ

        status_SI523CD_IRQ = 2;
        return status_SI523CD_IRQ;
    }

    si523_write_reg(SI523_REG_DIV_IRQ, 0x40); // Clear ACDIRq
    si523_write_reg(SI523_REG_DIV_IRQ, 0x20); // Clear ACDTIMER_IRQ
    si523_write_reg(0x20, (0x0f << 2) | 0x40);
    si523_write_reg(0x0f, 0x0a); // Clear OSCMon_IRQ,RFLowDetect_IRQ
    si523_write_reg(0x20, (0x09 << 2) | 0x40);
    si523_write_reg(0x0f, 0x55); // Clear ACC_IRQ

    return status_SI523CD_IRQ = 0;
}
