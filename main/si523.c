#include "si523.h"

static const char *TAG = "si523";

extern uint8_t PCD_IRQ_flagA;
unsigned char ACDConfigRegK_Val;
unsigned char ACDConfigRegC_Val;

static uint8_t pcd_irq_flag_a = 0;
static uint8_t acd_cfg_k_val = 0;
static uint8_t acd_cfg_c_val = 0;

esp_err_t si523_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = {reg, data};
    esp_err_t err = i2c_master_transmit(si523_handle, buf, 2, portMAX_DELAY);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Write reg 0x%02X failed", reg);
    return err;
}

uint8_t si523_read_reg(uint8_t reg)
{
    uint8_t data = 0;
    i2c_master_transmit_receive(si523_handle, &reg, 1, &data, 1, portMAX_DELAY);
    return data;
}

void si523_set_bit_mask(unsigned char reg, unsigned char mask)
{
    uint8_t tmp = si523_read_reg(reg);
    si523_write_reg(reg, tmp | mask);
}

void si523_clear_bit_mask(unsigned char reg, unsigned char mask)
{
    uint8_t tmp = si523_read_reg(reg);
    si523_write_reg(reg, tmp & (~mask));
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

void si523_hard_reset(void)
{
    gpio_set_level(SI523_RST_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(2));

    gpio_set_level(SI523_RST_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(2));
}

void si523_soft_reset(void)
{
    uint32_t timeout = 100;

    si523_write_reg(SI523_REG_COMMAND, SI523_CMD_SOFT_RESET);

    while ((si523_read_reg(SI523_REG_COMMAND) & 0x10) && timeout--)
    {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    vTaskDelay(pdMS_TO_TICKS(1));
}

esp_err_t si523_init(void)
{
    uint8_t chip_version = si523_read_reg(SI523_REG_VERSION);
    ESP_LOGI(TAG, "Si523 Chip Version: 0x%02x", chip_version);
    return ESP_OK;
}

bool si523_check_chip(void)
{
    uint8_t chip_version = si523_read_reg(SI523_REG_VERSION);
    return (chip_version != 0x00 && chip_version != 0xFF);
}

void si523_antenna_on(void)
{
    uint8_t val = si523_read_reg(SI523_REG_TX_CONTROL);

    if (!(val & 0x03))
    {
        si523_set_bit_mask(SI523_REG_TX_CONTROL, 0x03);
    }
}

void si523_antenna_off(void)
{
    si523_clear_bit_mask(SI523_REG_TX_CONTROL, 0x03);
}

void si523_set_rx_gain(uint8_t gain)
{
    uint8_t reg_val = si523_read_reg(SI523_REG_RF_CFG);

    reg_val &= 0x8F;
    reg_val |= (gain & 0x70);
    si523_write_reg(SI523_REG_RF_CFG, reg_val);
}

void si523_calculate_crc(uint8_t *in_buf, uint8_t data_len, uint8_t *out_buf)
{
    uint8_t loop_cnt;
    uint8_t irq_reg;
    uint32_t timeout = 0xFF;

    si523_clear_bit_mask(SI523_REG_DIV_IRQ, 0x04);
    si523_write_reg(SI523_REG_COMMAND, SI523_CMD_IDLE);
    si523_set_bit_mask(SI523_REG_FIFO_LEVEL, 0x80);

    for (loop_cnt = 0; loop_cnt < data_len; loop_cnt++)
    {
        si523_write_reg(SI523_REG_FIFO_DATA, in_buf[loop_cnt]);
    }

    si523_write_reg(SI523_REG_COMMAND, SI523_CMD_CALC_CRC);

    do
    {
        irq_reg = si523_read_reg(SI523_REG_DIV_IRQ);
        timeout--;
    } while ((timeout != 0) && !(irq_reg & 0x04));

    out_buf[0] = si523_read_reg(SI523_REG_CRC_RESULT_L);
    out_buf[1] = si523_read_reg(SI523_REG_CRC_RESULT_H);
}

static uint8_t si523_raw_cmd(uint8_t cmd, uint8_t *in_buf, uint8_t in_len, uint8_t *out_buf, uint32_t *out_bits)
{
    uint8_t ret_status = SI523_ERR;
    uint8_t irq_en = 0x00;
    uint8_t wait_for = 0x00;
    uint8_t last_bits;
    uint8_t reg_val;
    uint32_t loop_cnt;
    uint32_t timeout = 2000;

    switch (cmd)
    {
    case SI523_CMD_AUTHENT:
        irq_en = 0x12;
        wait_for = 0x10;
        break;
    case SI523_CMD_TRANSCEIVE:
        irq_en = 0x77;
        wait_for = 0x30;
        break;
    default:
        break;
    }

    si523_clear_bit_mask(SI523_REG_COM_IRQ, 0x80);
    si523_write_reg(SI523_REG_COMMAND, SI523_CMD_IDLE);
    si523_set_bit_mask(SI523_REG_FIFO_LEVEL, 0x80); /* Flush FIFO */

    for (loop_cnt = 0; loop_cnt < in_len; loop_cnt++)
    {
        si523_write_reg(SI523_REG_FIFO_DATA, in_buf[loop_cnt]);
    }

    si523_write_reg(SI523_REG_COMMAND, cmd);

    if (cmd == SI523_CMD_TRANSCEIVE)
    {
        si523_set_bit_mask(SI523_REG_BIT_FRAMING, 0x80); /* StartSend */
    }

    do
    {
        reg_val = si523_read_reg(SI523_REG_COM_IRQ);
        timeout--;
    } while ((timeout != 0) && !(reg_val & 0x01) && !(reg_val & wait_for));

    si523_clear_bit_mask(SI523_REG_BIT_FRAMING, 0x80);

    if (timeout != 0)
    {
        if (!(si523_read_reg(SI523_REG_ERROR) & 0x1B))
        {
            ret_status = SI523_OK;

            if (reg_val & irq_en & 0x01)
            {
                ret_status = SI523_ERR_NO_TAG;
            }

            if (cmd == SI523_CMD_TRANSCEIVE)
            {
                reg_val = si523_read_reg(SI523_REG_FIFO_LEVEL);
                last_bits = si523_read_reg(SI523_REG_CONTROL) & 0x07;

                if (last_bits)
                {
                    *out_bits = (reg_val - 1) * 8 + last_bits;
                }
                else
                {
                    *out_bits = reg_val * 8;
                }

                if (reg_val == 0)
                {
                    reg_val = 1;
                }
                if (reg_val > SI523_MAX_RLEN)
                {
                    reg_val = SI523_MAX_RLEN;
                }

                for (loop_cnt = 0; loop_cnt < reg_val; loop_cnt++)
                {
                    out_buf[loop_cnt] = si523_read_reg(SI523_REG_FIFO_DATA);
                }
            }
        }
        else
        {
            ret_status = SI523_ERR;
        }
    }

    si523_set_bit_mask(SI523_REG_CONTROL, 0x80); /* StopTimerNow */
    si523_write_reg(SI523_REG_COMMAND, SI523_CMD_IDLE);

    return ret_status;
}

uint8_t si523_request(uint8_t req_code, uint8_t *tag_type)
{
    uint8_t ret_status;
    uint32_t recv_bits;
    uint8_t fifo_buf[SI523_MAX_RLEN];

    si523_clear_bit_mask(SI523_REG_STATUS2, 0x08);
    si523_write_reg(SI523_REG_BIT_FRAMING, 0x07);
    si523_set_bit_mask(SI523_REG_TX_CONTROL, 0x03);

    fifo_buf[0] = req_code;
    ret_status = si523_raw_cmd(SI523_CMD_TRANSCEIVE, fifo_buf, 1, fifo_buf, &recv_bits);

    if ((ret_status == SI523_OK) && (recv_bits == 0x10))
    {
        tag_type[0] = fifo_buf[0];
        tag_type[1] = fifo_buf[1];
    }
    else
    {
        ret_status = SI523_ERR;
    }

    return ret_status;
}

uint8_t si523_anticollision(uint8_t *uid, uint8_t anticoll_level)
{
    uint8_t ret_status;
    uint8_t loop_cnt;
    uint8_t uid_check = 0;
    uint32_t recv_bits;
    uint8_t fifo_buf[SI523_MAX_RLEN];

    si523_clear_bit_mask(SI523_REG_STATUS2, 0x08);
    si523_write_reg(SI523_REG_BIT_FRAMING, 0x00);
    si523_clear_bit_mask(SI523_REG_COLL, 0x80);

    fifo_buf[0] = anticoll_level;
    fifo_buf[1] = 0x20;

    ret_status = si523_raw_cmd(SI523_CMD_TRANSCEIVE, fifo_buf, 2, fifo_buf, &recv_bits);

    if (ret_status == SI523_OK)
    {
        for (loop_cnt = 0; loop_cnt < 4; loop_cnt++)
        {
            uid[loop_cnt] = fifo_buf[loop_cnt];
            uid_check ^= fifo_buf[loop_cnt];
        }
        if (uid_check != fifo_buf[loop_cnt])
        {
            ret_status = SI523_ERR;
        }
    }

    si523_set_bit_mask(SI523_REG_COLL, 0x80);
    return ret_status;
}

uint8_t si523_select_card(uint8_t *uid, uint8_t anticoll_level, uint8_t *sak)
{
    uint8_t ret_status;
    uint8_t loop_cnt;
    uint32_t recv_bits;
    uint8_t fifo_buf[SI523_MAX_RLEN];

    fifo_buf[0] = anticoll_level;
    fifo_buf[1] = 0x70;
    fifo_buf[6] = 0;

    for (loop_cnt = 0; loop_cnt < 4; loop_cnt++)
    {
        fifo_buf[loop_cnt + 2] = uid[loop_cnt];
        fifo_buf[6] ^= uid[loop_cnt];
    }

    si523_calculate_crc(fifo_buf, 7, &fifo_buf[7]);
    si523_clear_bit_mask(SI523_REG_STATUS2, 0x08);

    ret_status = si523_raw_cmd(SI523_CMD_TRANSCEIVE, fifo_buf, 9, fifo_buf, &recv_bits);

    if ((ret_status == SI523_OK) && (recv_bits == 0x18))
    {
        *sak = fifo_buf[0];
        ret_status = SI523_OK;
    }
    else
    {
        ret_status = SI523_ERR;
    }

    return ret_status;
}

uint8_t si523_authenticate(uint8_t auth_mode, uint8_t block_addr, uint8_t *key, uint8_t *uid)
{
    uint8_t ret_status;
    uint32_t recv_bits;
    uint8_t fifo_buf[SI523_MAX_RLEN];

    fifo_buf[0] = auth_mode;
    fifo_buf[1] = block_addr;
    memcpy(&fifo_buf[2], key, 6);
    memcpy(&fifo_buf[8], uid, 4);

    ret_status = si523_raw_cmd(SI523_CMD_AUTHENT, fifo_buf, 12, fifo_buf, &recv_bits);

    if ((ret_status != SI523_OK) || !(si523_read_reg(SI523_REG_STATUS2) & 0x08))
    {
        ret_status = SI523_ERR_AUTH;
    }

    return ret_status;
}

uint8_t si523_read_block(uint8_t block_addr, uint8_t *data)
{
    uint8_t ret_status;
    uint32_t recv_bits;
    uint8_t fifo_buf[SI523_MAX_RLEN];

    fifo_buf[0] = SI523_PICC_READ;
    fifo_buf[1] = block_addr;
    si523_calculate_crc(fifo_buf, 2, &fifo_buf[2]);

    ret_status = si523_raw_cmd(SI523_CMD_TRANSCEIVE, fifo_buf, 4, fifo_buf, &recv_bits);

    if ((ret_status == SI523_OK) && (recv_bits == 0x90))
    {
        memcpy(data, fifo_buf, 16);
    }
    else
    {
        ret_status = SI523_ERR;
    }

    return ret_status;
}

uint8_t si523_write_block(uint8_t block_addr, uint8_t *data)
{
    uint8_t ret_status;
    uint32_t recv_bits;
    uint8_t fifo_buf[SI523_MAX_RLEN];

    /* Step 1: Send write command */
    fifo_buf[0] = SI523_PICC_WRITE;
    fifo_buf[1] = block_addr;
    si523_calculate_crc(fifo_buf, 2, &fifo_buf[2]);

    ret_status = si523_raw_cmd(SI523_CMD_TRANSCEIVE, fifo_buf, 4, fifo_buf, &recv_bits);
    if ((ret_status != SI523_OK) || (recv_bits != 4) || ((fifo_buf[0] & 0x0F) != 0x0A))
    {
        return SI523_ERR;
    }

    /* Step 2: Send actual data */
    memcpy(fifo_buf, data, 16);
    si523_calculate_crc(fifo_buf, 16, &fifo_buf[16]);

    ret_status = si523_raw_cmd(SI523_CMD_TRANSCEIVE, fifo_buf, 18, fifo_buf, &recv_bits);
    if ((ret_status != SI523_OK) || (recv_bits != 4) || ((fifo_buf[0] & 0x0F) != 0x0A))
    {
        ret_status = SI523_ERR;
    }

    return ret_status;
}

uint8_t si523_halt(void)
{
    uint8_t ret_status;
    uint32_t recv_bits;
    uint8_t fifo_buf[SI523_MAX_RLEN];

    fifo_buf[0] = SI523_PICC_HALT;
    fifo_buf[1] = 0;
    si523_calculate_crc(fifo_buf, 2, &fifo_buf[2]);

    ret_status = si523_raw_cmd(SI523_CMD_TRANSCEIVE, fifo_buf, 4, fifo_buf, &recv_bits);
    return ret_status;
}

void si523_type_a_init(void)
{
    si523_clear_bit_mask(SI523_REG_STATUS2, 0x08);

    si523_write_reg(SI523_REG_TX_MODE, 0x00);
    si523_write_reg(SI523_REG_RX_MODE, 0x00);
    si523_write_reg(SI523_REG_MOD_WIDTH, 0x26);
    si523_write_reg(SI523_REG_RF_CFG, SI523_RF_CFG_DEFAULT);

    /* Timer configuration */
    si523_write_reg(SI523_REG_T_MODE, 0x80);
    si523_write_reg(SI523_REG_T_PRESCALER, 0xA9);
    si523_write_reg(SI523_REG_T_RELOAD_H, 0x03);
    si523_write_reg(SI523_REG_T_RELOAD_L, 0xE8);

    si523_write_reg(SI523_REG_TX_AUTO, 0x40); /* Force 100% ASK */
    si523_write_reg(SI523_REG_MODE, 0x3D);    /* CRC preset 0x6363 */
    si523_write_reg(SI523_REG_COMMAND, 0x00); /* Idle */

    si523_antenna_on();
}

void si523_type_b_init(void)
{
    si523_clear_bit_mask(SI523_REG_STATUS2, 0x08);

    si523_write_reg(SI523_REG_MODE, 0x3F); /* CRC preset 0xFFFF */
    si523_write_reg(SI523_REG_T_RELOAD_L, 30);
    si523_write_reg(SI523_REG_T_RELOAD_H, 0);
    si523_write_reg(SI523_REG_T_MODE, 0x8D);
    si523_write_reg(SI523_REG_T_PRESCALER, 0x3E);
    si523_write_reg(SI523_REG_TX_AUTO, 0);
    si523_write_reg(SI523_REG_GS_N_ON, 0xFF);
    si523_write_reg(SI523_REG_CW_GS_P, 0x3F);
    si523_write_reg(SI523_REG_MOD_GS_P, 0x07);
    si523_write_reg(SI523_REG_TX_MODE, 0x83); /* 106kbps, TypeB */
    si523_write_reg(SI523_REG_BIT_FRAMING, 0x00);
    si523_write_reg(SI523_REG_AUTO_TEST, 0x00);
    si523_write_reg(SI523_REG_RF_CFG, SI523_RF_CFG_DEFAULT);
    si523_write_reg(SI523_REG_RX_MODE, 0x83);
    si523_write_reg(SI523_REG_RX_THRESHOLD, 0x65);

    si523_clear_bit_mask(SI523_REG_RX_SEL, 0x3F);
    si523_set_bit_mask(SI523_REG_RX_SEL, 0x08);
    si523_clear_bit_mask(SI523_REG_TX_MODE, 0x80);
    si523_clear_bit_mask(SI523_REG_RX_MODE, 0x80);
    si523_clear_bit_mask(SI523_REG_STATUS2, 0x08);

    si523_antenna_on();
}

#if 0
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

char PCD_SI523_TypeB_GetUID(void)
{

    ESP_LOGI(TAG, "Test_B_GetUID");

    // si523_write_reg(0x02, 0xa0); //打开接收中断,则读卡会产生中断
    //  Enable external interrupt
    // EXTI->IMR |= 0x00000008;

    // request 寻B卡;返回卡号
    uint32_t len1;
    unsigned char buf1[18] = {0x05, 0x00, 0x00, 0x71, 0xFF};

    if (si523_transceive(SI523_CMD_TRANSCEIVE, buf1, 5, buf1, &len1) != SI523_OK)
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

    si523_write_reg(SI523_REG_TX_CONTROL, 0x83); // 打开天线
    si523_set_bit_mask(SI523_REG_COMMAND, 0x06); // 开启ADC_EXCUTE
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

    si523_set_bit_mask(SI523_REG_COMMAND, 0x06); // 关闭ADC_EXCUTE
}

#endif
uint8_t si523_type_a_get_uid(uint8_t *uid, uint8_t *uid_len)
{
    uint8_t atqa[2];
    uint8_t uid_buf[12] = {0};
    uint8_t sak = 0;
    uint8_t uid_offset = 0;

    ESP_LOGI(TAG, "Type A: Get UID");
    si523_write_reg(SI523_REG_RF_CFG, SI523_RF_CFG_DEFAULT);

    /* 1. Request card with auto gain fallback */
    if (si523_request(SI523_PICC_REQ_IDL, atqa) != SI523_OK)
    {
        si523_set_rx_gain(SI523_RX_GAIN_33DB);
        if (si523_request(SI523_PICC_REQ_IDL, atqa) != SI523_OK)
        {
            si523_set_rx_gain(SI523_RX_GAIN_38DB);
            if (si523_request(SI523_PICC_REQ_IDL, atqa) != SI523_OK)
            {
                ESP_LOGW(TAG, "Request failed (no card)");
                return SI523_ERR_NO_TAG;
            }
        }
    }
    ESP_LOGI(TAG, "ATQA: %02x %02x", atqa[0], atqa[1]);

    /* 2. Anticollision Level 1 */
    if (si523_anticollision(uid_buf, SI523_PICC_ANTICOLL1) != SI523_OK)
    {
        ESP_LOGE(TAG, "Anticoll L1 failed");
        return SI523_ERR;
    }
    if (si523_select_card(uid_buf, SI523_PICC_ANTICOLL1, &sak) != SI523_OK)
    {
        ESP_LOGE(TAG, "Select L1 failed");
        return SI523_ERR;
    }
    ESP_LOGI(TAG, "SAK L1: 0x%02x", sak);

    memcpy(uid, uid_buf, 4);
    uid_offset = 4;

    /* 3. Anticollision Level 2 (Cascade) */
    if (sak & 0x04)
    {
        uid[0] = 0x88; /* Cascade tag */

        if (si523_anticollision(uid_buf + 4, SI523_PICC_ANTICOLL2) != SI523_OK)
        {
            return SI523_ERR;
        }
        if (si523_select_card(uid_buf + 4, SI523_PICC_ANTICOLL2, &sak) != SI523_OK)
        {
            return SI523_ERR;
        }
        ESP_LOGI(TAG, "SAK L2: 0x%02x", sak);

        memcpy(uid + 3, uid_buf + 4, 4);
        uid_offset = 7;

        /* 4. Anticollision Level 3 (Double Cascade) */
        if (sak & 0x04)
        {
            if (si523_anticollision(uid_buf + 8, SI523_PICC_ANTICOLL3) != SI523_OK)
            {
                return SI523_ERR;
            }
            if (si523_select_card(uid_buf + 8, SI523_PICC_ANTICOLL3, &sak) != SI523_OK)
            {
                return SI523_ERR;
            }
            ESP_LOGI(TAG, "SAK L3: 0x%02x", sak);

            memcpy(uid + 6, uid_buf + 8, 4);
            uid_offset = 10;
        }
    }

    *uid_len = uid_offset;

    ESP_LOGI(TAG, "UID (Len:%d):", *uid_len);
    for (int i = 0; i < *uid_len; i++)
    {
        ESP_LOGI(TAG, " %02x", uid[i]);
    }

    return SI523_OK;
}

uint8_t si523_type_b_get_uid(uint8_t *uid, uint8_t *uid_len)
{
    ESP_LOGI(TAG, "Type B: Get UID");

    uint32_t recv_bits;
    uint8_t fifo_buf[18] = {0x05, 0x00, 0x00, 0x71, 0xFF}; /* REQB */

    if (si523_raw_cmd(SI523_CMD_TRANSCEIVE, fifo_buf, 5, fifo_buf, &recv_bits) != SI523_OK)
    {
        return SI523_ERR_NO_TAG;
    }

    if (fifo_buf[0] == 0x50)
    {                                 /* ATQB response */
        memcpy(uid, &fifo_buf[1], 4); /* PUPI is usually bytes 1-4 */
        *uid_len = 4;
        ESP_LOGI(TAG, "PUPI: %02x %02x %02x %02x", uid[0], uid[1], uid[2], uid[3]);
        return SI523_OK;
    }

    return SI523_ERR;
}

uint8_t si523_type_a_rw_block_test(void)
{
    uint8_t atqa[2];
    uint8_t uid[12] = {0};
    uint8_t sak = 0;
    uint8_t card_read_buf[16] = {0};
    uint8_t card_write_buf[16] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                                  0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};
    /* M1卡默认A密钥 (仅前6字节有效，后面冗余字节移除) */
    uint8_t default_key_a[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t loop_cnt;

    ESP_LOGI(TAG, "Type A card block read/write test");

    /* 寻卡 (REQ_IDL) */
    if (si523_request(SI523_PICC_REQ_IDL, atqa) != SI523_OK)
    {
        ESP_LOGE(TAG, "Step1: Request card failed");
        return 1;
    }
    ESP_LOGI(TAG, "Step1: Request ok, ATQA: 0x%02x 0x%02x", atqa[0], atqa[1]);

    /* 防冲撞 (Level1) */
    if (si523_anticollision(uid, SI523_PICC_ANTICOLL1) != SI523_OK)
    {
        ESP_LOGE(TAG, "Step2: Anticollision failed");
        return 2;
    }
    ESP_LOGI(TAG, "Step2: Anticoll ok, UID: 0x%02x 0x%02x 0x%02x 0x%02x",
             uid[0], uid[1], uid[2], uid[3]);

    /* 选卡 */
    if (si523_select_card(uid, SI523_PICC_ANTICOLL1, &sak) != SI523_OK)
    {
        ESP_LOGE(TAG, "Step3: Select card failed");
        return 3;
    }
    ESP_LOGI(TAG, "Step3: Select ok, SAK: 0x%02x", sak);

    /* 验证A密钥 (块4) */
    if (si523_authenticate(SI523_PICC_AUTH_A, 4, default_key_a, uid) != SI523_OK)
    {
        ESP_LOGE(TAG, "Step4: Authenticate key A failed");
        return 4;
    }
    ESP_LOGI(TAG, "Step4: Authenticate ok");

    /* 读块4原始数据 */
    if (si523_read_block(4, card_read_buf) != SI523_OK)
    {
        ESP_LOGE(TAG, "Step5: Read block 4 failed");
        return 5;
    }
    ESP_LOGI(TAG, "Step5: Read block 4 ok, data:");
    for (loop_cnt = 0; loop_cnt < 16; loop_cnt++)
    {
        ESP_LOGI(TAG, "%02x ", card_read_buf[loop_cnt]);
    }

    /* 生成随机写入数据 */
    for (loop_cnt = 0; loop_cnt < 16; loop_cnt++)
    {
        card_write_buf[loop_cnt] = rand() & 0xFF; /* 限制为8位 */
    }

    /* 写块4 */
    if (si523_write_block(4, card_write_buf) != SI523_OK)
    {
        ESP_LOGE(TAG, "Step7: Write block 4 failed");
        return 6;
    }
    ESP_LOGI(TAG, "Step7: Write block 4 ok, data:");
    for (loop_cnt = 0; loop_cnt < 16; loop_cnt++)
    {
        ESP_LOGI(TAG, "%02x ", card_write_buf[loop_cnt]);
    }

    /* 重读块4验证写入结果 */
    memset(card_read_buf, 0, sizeof(card_read_buf)); /* 清空缓冲区 */
    if (si523_read_block(4, card_read_buf) != SI523_OK)
    {
        ESP_LOGE(TAG, "Step8: Re-read block 4 failed");
        return 5;
    }
    ESP_LOGI(TAG, "Step8: Re-read block 4 ok, data:");
    for (loop_cnt = 0; loop_cnt < 16; loop_cnt++)
    {
        ESP_LOGI(TAG, "%02x ", card_read_buf[loop_cnt]);
    }

    /* 可选：休眠卡片 (根据需求启用) */
    // if (si523_halt() != SI523_OK) {
    //     ESP_LOGW(TAG, "Halt card failed");
    // } else {
    //     ESP_LOGI(TAG, "Halt ok");
    // }

    ESP_LOGI(TAG, "Type A RW test completed successfully");
    return 0;
}

uint8_t si523_identity_card_get_uid(uint8_t *uid, uint8_t *uid_len)
{
    ESP_LOGI(TAG, "Identity Card: Get UID");

    uint32_t recv_bits;
    uint8_t fifo_buf[18];

    /* Send REQB */
    uint8_t reqb_buf[5] = {0x05, 0x00, 0x00, 0x71, 0xFF};
    if (si523_raw_cmd(SI523_CMD_TRANSCEIVE, reqb_buf, 5, fifo_buf, &recv_bits) != SI523_OK)
    {
        return SI523_ERR_NO_TAG;
    }

    /* Send proprietary ATTRIB */
    uint8_t attrib_buf[11] = {0x1D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x01, 0x08, 0xF3, 0x10};
    if (si523_raw_cmd(SI523_CMD_TRANSCEIVE, attrib_buf, 11, fifo_buf, &recv_bits) != SI523_OK)
    {
        return SI523_ERR;
    }

    /* Send Get UID command */
    uint8_t get_uid_cmd[7] = {0x00, 0x36, 0x00, 0x00, 0x08, 0x57, 0x44};
    if (si523_raw_cmd(SI523_CMD_TRANSCEIVE, get_uid_cmd, 7, fifo_buf, &recv_bits) != SI523_OK)
    {
        return SI523_ERR;
    }

    if (fifo_buf[8] == 0x90 || fifo_buf[9] == 0x00)
    { /* Check SW1SW2 */
        memcpy(uid, fifo_buf, 8);
        *uid_len = 8;

        ESP_LOGI(TAG, "ID Card UID:");
        for (int i = 0; i < 8; i++)
        {
            ESP_LOGI(TAG, " %02x", uid[i]);
        }
        return SI523_OK;
    }

    return SI523_ERR;
}

static void si523_write_acd_reg(uint8_t sub_reg, uint8_t val)
{
    si523_write_reg(SI523_REG_PAGE2, (sub_reg << 2) | 0x40);
    si523_write_reg(SI523_REG_ACD_CFG, val);
}

static uint8_t si523_read_acd_reg(uint8_t sub_reg)
{
    si523_write_reg(SI523_REG_PAGE2, (sub_reg << 2) | 0x40);
    return si523_read_reg(SI523_REG_ACD_CFG);
}

void si523_acd_auto_calc(void)
{
    uint8_t adc_val;
    uint8_t avg_adc_val;
    uint8_t vcon_tr_table[8] = {0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};
    uint8_t tr_compare_table[4] = {0x00, 0x00, 0x00, 0x00};
    uint8_t cfg_k_real_val = 0;
    int vcon_idx;
    int avg_loop;
    int tr_idx;
    int final_tr_idx;

    acd_cfg_c_val = 0x7F;

    si523_write_reg(SI523_REG_TX_CONTROL, 0x83);
    si523_set_bit_mask(SI523_REG_COMMAND, 0x06); /* ADC_EXECUTE */
    vTaskDelay(pdMS_TO_TICKS(1));

    /* Find optimal VCON */
    for (vcon_idx = 7; vcon_idx > 0; vcon_idx--)
    {
        si523_write_acd_reg(SI523_ACD_REG_LPD_CFG1, vcon_tr_table[vcon_idx]);
        (void)si523_read_acd_reg(SI523_ACD_REG_ADC_VAL); /* Discard first read */
        avg_adc_val = si523_read_acd_reg(SI523_ACD_REG_ADC_VAL);

        for (avg_loop = 0; avg_loop < 100; avg_loop++)
        {
            adc_val = si523_read_acd_reg(SI523_ACD_REG_ADC_VAL);
            if (adc_val == 0)
            {
                break;
            }
            avg_adc_val = (avg_adc_val + adc_val) / 2;
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        if (avg_adc_val != 0 && avg_adc_val != 0x7F)
        {
            if (avg_adc_val < acd_cfg_c_val)
            {
                acd_cfg_c_val = avg_adc_val;
                acd_cfg_k_val = vcon_tr_table[vcon_idx];
            }
        }
    }
    cfg_k_real_val = acd_cfg_k_val;

    /* Find optimal TR */
    for (tr_idx = 0; tr_idx < 4; tr_idx++)
    {
        si523_write_acd_reg(SI523_ACD_REG_LPD_CFG1, tr_idx * 32 + cfg_k_real_val);
        avg_adc_val = si523_read_acd_reg(SI523_ACD_REG_ADC_VAL);

        for (avg_loop = 0; avg_loop < 100; avg_loop++)
        {
            adc_val = si523_read_acd_reg(SI523_ACD_REG_ADC_VAL);
            avg_adc_val = (avg_adc_val + adc_val) / 2;
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        tr_compare_table[tr_idx] = avg_adc_val;
    }

    for (final_tr_idx = 0; final_tr_idx < 3; final_tr_idx++)
    {
        if (tr_compare_table[final_tr_idx] != 0x7F)
        {
            acd_cfg_c_val = tr_compare_table[final_tr_idx];
            acd_cfg_k_val = cfg_k_real_val + final_tr_idx * 32;
        }
    }

    ESP_LOGI(TAG, "ACD AutoCalc: CfgK=0x%02x, CfgC=0x%02x", acd_cfg_k_val, acd_cfg_c_val);
    si523_clear_bit_mask(SI523_REG_COMMAND, 0x06);
}

void si523_acd_init(void)
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
    si523_write_reg(SI523_REG_PAGE2, (SI523_ACD_REG_ARI_CFG << 2) | 0x40); // 设置ARI功能，在天线场强打开前1us产生ARI电平控制触摸芯片Si14TP的硬件屏蔽引脚SCT
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
    si523_write_reg(SI523_REG_ACD_CFG, SI523_ACD_ACC_CFG_DEFAULT);         // 写非0x55的值即开启功能，写0x55清除使能停止功能。

    si523_write_reg(SI523_REG_COMMAND, 0xB0); // 进入ACD
}

void si523_modify_reg(uint8_t reg_addr, bool set_bit, uint8_t mask_byte)
{
    uint8_t reg_val = si523_read_reg(reg_addr);

    if (set_bit)
    {
        reg_val |= mask_byte;
    }
    else
    {
        reg_val &= ~mask_byte;
    }

    si523_write_reg(reg_addr, reg_val);
}

void si523_acd_start(void)
{
    si523_type_a_init(); // 读A卡初始化配置

    si523_acd_auto_calc(); // 自动获取阈值

    si523_acd_init(); // ACD初始化配置
}

uint8_t si523_acd_irq_process(void)
{
    uint8_t div_irq_reg = si523_read_reg(SI523_REG_DIV_IRQ);

    if (div_irq_reg & 0x40)
    {
        si523_write_reg(SI523_REG_DIV_IRQ, 0x40); /* Clear ACDIRq */
        return 1;
    }

    if (div_irq_reg & 0x20)
    {
        si523_write_reg(SI523_REG_DIV_IRQ, 0x20); /* Clear ACDTIMER_IRQ */
        return 2;
    }

    si523_write_reg(SI523_REG_DIV_IRQ, 0x60); /* Clear ACDIRq + WdtIRq */

    si523_write_reg(SI523_REG_PAGE2, (SI523_ACD_REG_IRQ_FLAG << 2) | 0x40);
    si523_write_reg(SI523_REG_ACD_CFG, 0x0A);
    si523_write_reg(SI523_REG_PAGE2, (SI523_ACD_REG_ACC_CFG << 2) | 0x40);
    si523_write_reg(SI523_REG_ACD_CFG, 0x55);

    return 0;
}