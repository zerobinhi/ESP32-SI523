#ifndef SI523_H
#define SI523_H

#include <esp_log.h>
#include <esp_err.h>
#include <stdint.h>
#include <stdbool.h>
#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>




#define RFCfgReg_Val 0x68
#define DivIEnReg_Val 0xC0
#define ComIEnReg_Val 0x80
#define IRQMODE 0x01
#define EXTI_Trigger_Mode 0x0C
#define ACDConfigRegA_Val 0x02
#define ACDConfigRegB_Val 0xA8
#define ACDConfigRegD_Val 0x02
#define ACDConfigRegH_Val 0x26
#define ACDConfigRegI_Val 0x00
#define ACDConfigRegJ_Val 0x55
#define ACDConfigRegM_Val 0x01
#define ACDConfigRegO_Val 0x00

//********************************************//
// MF522寄存器定义
//********************************************//
// Page 0: Command and status
#define RFU00 0x00
#define CommandReg 0x01
#define ComIEnReg 0x02
#define DivIEnReg 0x03
#define ComIrqReg 0x04
#define DivIrqReg 0x05
#define ErrorReg 0x06
#define Status1Reg 0x07
#define Status2Reg 0x08
#define FIFODataReg 0x09
#define FIFOLevelReg 0x0A
#define WaterLevelReg 0x0B
#define ControlReg 0x0C
#define BitFramingReg 0x0D
#define CollReg 0x0E
#define ACDConfigReg 0x0F

// Page 1: Command
#define RFU10 0x10
#define ModeReg 0x11
#define TxModeReg 0x12
#define RxModeReg 0x13
#define TxControlReg 0x14
#define TxASKReg 0x15
#define TxSelReg 0x16
#define RxSelReg 0x17
#define RxThresholdReg 0x18
#define DemodReg 0x19
#define RFU1A 0x1A
#define RFU1B 0x1B
#define MfTxReg 0x1C
#define MfRxReg 0x1D
#define TypeBReg 0x1E
#define SerialSpeedReg 0x1F

// Page 2: Configuration
#define ACDConfigSelReg 0x20
#define CRCResultRegH 0x21
#define CRCResultRegL 0x22
#define RFU23 0x23
#define ModWidthReg 0x24
#define RFU25 0x25
#define RFCfgReg 0x26
#define GsNReg 0x27
#define CWGsPReg 0x28
#define ModGsPReg 0x29
#define TModeReg 0x2A
#define TPrescalerReg 0x2B
#define TReloadRegH 0x2C
#define TReloadRegL 0x2D
#define TCounterValueRegH 0x2E
#define TCounterValueRegL 0x2F

// Page 3: Test Register
#define RFU30 0x30
#define TestSel1Reg 0x31
#define TestSel2Reg 0x32
#define TestPinEnReg 0x33
#define TestPinValueReg 0x34
#define TestBusReg 0x35
#define AutoTestReg 0x36
#define VersionReg 0x37
#define AnalogTestReg 0x38
#define TestDAC1Reg 0x39
#define TestDAC2Reg 0x3A
#define TestADCReg 0x3B
#define RFU3C 0x3C
#define RFU3D 0x3D
#define RFU3E 0x3E
#define RFU3F 0x3F

/////////////////////////////////////////////////////////////////////
// MF522命令字
/////////////////////////////////////////////////////////////////////
#define PCD_IDLE 0x00       // 取消当前命令
#define PCD_AUTHENT 0x0E    // 验证密钥
#define PCD_RECEIVE 0x08    // 接收数据
#define PCD_TRANSMIT 0x04   // 发送数据
#define PCD_TRANSCEIVE 0x0C // 发送并接收数据
#define PCD_RESETPHASE 0x0F // 复位
#define PCD_CALCCRC 0x03    // CRC计算

#define ACDConfigA 0x00
#define ACDConfigB 0x01
#define ACDConfigC 0x02
#define ACDConfigD 0x03
#define ACDConfigE 0x04
#define ACDConfigF 0x05
#define ACDConfigG 0x06
#define ACDConfigH 0x07
#define ACDConfigI 0x08
#define ACDConfigJ 0x09
#define ACDConfigK 0x0a
#define ACDConfigL 0x0b
#define ACDConfigM 0x0c
#define ACDConfigN 0x0d
#define ACDConfigO 0x0e
#define ACDConfigP 0x0f








// -------------------------- 引脚与 I2C 配置 --------------------------
#define SI523_RST_PIN 21
#define SI523_INT_PIN 15

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SCL_IO 18
#define I2C_MASTER_SDA_IO 19
#define I2C_MASTER_FREQ_HZ 100000
#define SI523_I2C_ADDR 0x28

// -------------------------- 核心寄存器定义 --------------------------
// PAGE0: 命令与状态寄存器
#define SI523_REG_PAGE0 0x00       // Page0 页寄存器
#define SI523_REG_COMMAND 0x01     // 命令执行控制
#define SI523_REG_COM_IEN 0x02     // 主中断使能
#define SI523_REG_DIV_IEN 0x03     // 次中断使能
#define SI523_REG_COM_IRQ 0x04     // 主中断标志
#define SI523_REG_DIV_IRQ 0x05     // 次中断标志
#define SI523_REG_ERROR 0x06       // 错误状态
#define SI523_REG_STATUS1 0x07     // 通信状态1
#define SI523_REG_STATUS2 0x08     // 通信状态2
#define SI523_REG_FIFO_DATA 0x09   // FIFO数据读写
#define SI523_REG_FIFO_LEVEL 0x0A  // FIFO数据长度
#define SI523_REG_WATER_LEVEL 0x0B // FIFO水位告警
#define SI523_REG_CONTROL 0x0C     // 通用控制
#define SI523_REG_BIT_FRAMING 0x0D // 位帧调整
#define SI523_REG_COLL 0x0E        // 位冲突检测
#define SI523_REG_ACD_CFG 0x0F     // ACD配置组访问

// PAGE1: 通信配置寄存器
#define SI523_REG_PAGE1 0x10        // Page1 页寄存器
#define SI523_REG_MODE 0x11         // 收发通用模式
#define SI523_REG_TX_MODE 0x12      // 发射模式配置
#define SI523_REG_RX_MODE 0x13      // 接收模式配置
#define SI523_REG_TX_CONTROL 0x14   // 天线驱动控制
#define SI523_REG_TX_AUTO 0x15      // 发射自动配置
#define SI523_REG_TX_SEL 0x16       // 发射源选择
#define SI523_REG_RX_SEL 0x17       // 接收源选择
#define SI523_REG_RX_THRESHOLD 0x18 // 接收阈值
#define SI523_REG_DEMOD 0x19        // 解调电路配置
#define SI523_REG_MIF_NFC 0x1C      // ISO14443A 控制
#define SI523_REG_MANUAL_RCV 0x1D   // 手动接收参数细调
#define SI523_REG_TYPE_B 0x1E       // ISO14443B配置
#define SI523_REG_SERIAL_SPEED 0x1F // UART速率配置

// PAGE2: 射频与定时器配置寄存器
#define SI523_REG_PAGE2 0x20        // Page2 页寄存器
#define SI523_REG_CRC_RESULT_H 0x21 // CRC结果高字节
#define SI523_REG_CRC_RESULT_L 0x22 // CRC结果低字节
#define SI523_REG_MOD_WIDTH 0x24    // 调制宽度配置
#define SI523_REG_RF_CFG 0x26       // 射频增益配置
#define SI523_REG_GS_N_ON 0x27      // 射频开启时驱动电导
#define SI523_REG_CW_GS_P 0x28      // 载波驱动电导
#define SI523_REG_MOD_GS_P 0x29     // 调制驱动电导
#define SI523_REG_T_MODE 0x2A       // 定时器模式
#define SI523_REG_T_PRESCALER 0x2B  // 定时器预分频
#define SI523_REG_T_RELOAD_H 0x2C   // 定时器重载值高字节
#define SI523_REG_T_RELOAD_L 0x2D   // 定时器重载值低字节
#define SI523_REG_T_COUNTER_H 0x2E  // 定时器当前值高字节
#define SI523_REG_T_COUNTER_L 0x2F  // 定时器当前值低字节

// PAGE3: 测试与版本寄存器
#define SI523_REG_PAGE3 0x30          // Page3 页寄存器
#define SI523_REG_TEST_SEL1 0x31      // 通用测试信号配置
#define SI523_REG_TEST_SEL2 0x32      // PRBS控制与测试总线
#define SI523_REG_TEST_PIN_EN 0x33    // 测试管脚使能
#define SI523_REG_TEST_PIN_VALUE 0x34 // 测试管脚状态
#define SI523_REG_TEST_BUS 0x35       // 内部测试总线状态
#define SI523_REG_AUTO_TEST 0x36      // 数字自测控制
#define SI523_REG_VERSION 0x37        // 芯片版本
#define SI523_REG_ANALOG_TEST 0x38    // 模拟管脚 AUX1/AUX2 控制
#define SI523_REG_TEST_DAC1 0x39      // DAC1 测试值
#define SI523_REG_TEST_DAC2 0x3A      // DAC2 测试值
#define SI523_REG_TEST_ADC 0x3B       // ADC I/Q 通道实际值

// ACD配置子寄存器
#define SI523_ACD_REG_RCCFG1 0x00     // 3K RC配置1
#define SI523_ACD_REG_ACRDCFG 0x01    // 射频卡/场检测配置
#define SI523_ACD_REG_MAN_REF 0x02    // 手动模式参考值
#define SI523_ACD_REG_VAL_DELTA 0x03  // 场强变化阈值
#define SI523_ACD_REG_ADC_CFG 0x04    // 轮询ADC配置
#define SI523_ACD_REG_RCCFG2 0x05     // 3K RC配置2
#define SI523_ACD_REG_ADC_VAL 0x06    // 轮询ADC采样值
#define SI523_ACD_REG_WDT_CNT 0x07    // 看门狗间隔配置
#define SI523_ACD_REG_ARI_CFG 0x08    // ARI功能配置
#define SI523_ACD_REG_ACC_CFG 0x09    // 配置监测功能
#define SI523_ACD_REG_LPD_CFG1 0x0A   // 检波器配置1
#define SI523_ACD_REG_LPD_CFG2 0x0B   // 检波器配置2
#define SI523_ACD_REG_RF_LOW_DET 0x0C // 低RF监测配置
#define SI523_ACD_REG_EX_RF_DET 0x0D  // 外部RF监测配置
#define SI523_ACD_REG_IRQ_EN 0x0E     // ACD中断使能
#define SI523_ACD_REG_IRQ_FLAG 0x0F   // ACD中断标志

// -------------------------- 核心命令码定义 --------------------------
#define SI523_CMD_IDLE 0x00       // 空闲/取消当前命令
#define SI523_CMD_CALC_CRC 0x03   // 启动CRC计算
#define SI523_CMD_TRANSMIT 0x04   // 发射FIFO数据
#define SI523_CMD_MSTART 0x05     // 3K RC自动校正
#define SI523_CMD_ADC_EXCUTE 0x06 // RF参考值自动获取
#define SI523_CMD_RECEIVE 0x08    // 激活接收
#define SI523_CMD_TRANSCEIVE 0x0C // 发射并接收数据
#define SI523_CMD_AUTHENT 0x0E    // 密钥验证
#define SI523_CMD_SOFT_RESET 0x0F // 软复位

// -------------------------- 寄存器默认配置值 --------------------------
// 射频与通用中断配置
#define SI523_RF_CFG_DEFAULT 0x68  // 接收机增益默认值 (43dB)
#define SI523_COM_IEN_DEFAULT 0x80 // 主中断使能默认值
#define SI523_DIV_IEN_DEFAULT 0xC0 // 次中断使能默认值

// ACD模式配置
#define SI523_ACD_RCCFG1_DEFAULT 0x02     // 唤醒间隔 300ms
#define SI523_ACD_ACRDCFG_DEFAULT 0xA8    // 相对值模式+下降检测+卡/场同时使能
#define SI523_ACD_VAL_DELTA_DEFAULT 0x02  // 场强变化阈值
#define SI523_ACD_WDT_CNT_DEFAULT 0x26    // 看门狗中断间隔
#define SI523_ACD_ARI_CFG_DEFAULT 0x00    // ARI功能关闭
#define SI523_ACD_ACC_CFG_DEFAULT 0x55    // 配置监测关闭
#define SI523_ACD_RF_LOW_DET_DEFAULT 0x01 // 低RF监测使能
#define SI523_ACD_IRQ_EN_DEFAULT 0x00     // ACD中断全部关闭

// -------------------------- ISO14443 协议命令定义 --------------------------
#define SI523_PICC_REQ_IDL 0x26   // 寻未休眠的卡
#define SI523_PICC_REQ_ALL 0x52   // 寻所有卡
#define SI523_PICC_ANTICOLL1 0x93 // 1级防冲撞
#define SI523_PICC_ANTICOLL2 0x95 // 2级防冲撞
#define SI523_PICC_ANTICOLL3 0x97 // 3级防冲撞
#define SI523_PICC_AUTH_A 0x60    // 验证A密钥
#define SI523_PICC_AUTH_B 0x61    // 验证B密钥
#define SI523_PICC_READ 0x30      // 读数据块
#define SI523_PICC_WRITE 0xA0     // 写数据块
#define SI523_PICC_HALT 0x50      // 卡片休眠

// -------------------------- 状态与错误码定义 --------------------------
#define SI523_OK 0         // 操作成功
#define SI523_ERR_NO_TAG 1 // 未检测到卡片
#define SI523_ERR 2        // 通用错误
#define SI523_ERR_COMM 3   // 通信失败
#define SI523_ERR_CRC 4    // CRC校验错误
#define SI523_ERR_AUTH 5   // 密钥验证失败

// -------------------------- 通用配置常量 --------------------------
#define SI523_FIFO_MAX_LEN 64     // FIFO最大长度
#define SI523_MAX_RLEN 18         // 最大接收数据长度
#define SI523_MIFARE_BLOCK_LEN 16 // M1卡块数据长度

// 射频增益配置
#define SI523_RX_GAIN_18DB           0x08
#define SI523_RX_GAIN_23DB           0x18
#define SI523_RX_GAIN_33DB           0x48
#define SI523_RX_GAIN_38DB           0x58
#define SI523_RX_GAIN_43DB           0x68
#define SI523_RX_GAIN_48DB           0x78

// -------------------------- 外部变量声明 --------------------------
extern i2c_master_dev_handle_t si523_handle;
extern i2c_master_bus_handle_t i2c_bus_handle;

// -------------------------- 驱动接口函数声明 --------------------------
// 基础硬件操作
void si523_gpio_init(void);
void si523_hard_reset(void);
void si523_soft_reset(void);
esp_err_t si523_init(void);
bool si523_check_chip(void);

// 寄存器基础操作
esp_err_t si523_write_reg(uint8_t reg, uint8_t data);
uint8_t si523_read_reg(uint8_t reg);
void si523_set_bit_mask(uint8_t reg, uint8_t mask);
void si523_clear_bit_mask(uint8_t reg, uint8_t mask);

// 射频与天线控制
void si523_antenna_on(void);
void si523_antenna_off(void);
void si523_set_rx_gain(uint8_t gain);

// CRC硬件计算
void si523_calculate_crc(uint8_t *in_buf, uint8_t data_len, uint8_t *out_buf);

// ISO14443A 核心操作
uint8_t si523_request(uint8_t req_code, uint8_t *tag_type);
uint8_t si523_anticollision(uint8_t *uid, uint8_t anticoll_level);
uint8_t si523_select_card(uint8_t *uid, uint8_t anticoll_level, uint8_t *sak);
uint8_t si523_authenticate(uint8_t auth_mode, uint8_t block_addr, uint8_t *key, uint8_t *uid);
uint8_t si523_read_block(uint8_t block_addr, uint8_t *data);
uint8_t si523_write_block(uint8_t block_addr, uint8_t *data);
uint8_t si523_halt(void);

// ISO14443A/B 初始化
void si523_type_a_init(void);
void si523_type_b_init(void);

// 卡操作高级接口
uint8_t si523_type_a_get_uid(uint8_t *uid, uint8_t *uid_len);
uint8_t si523_type_b_get_uid(uint8_t *uid, uint8_t *uid_len);
uint8_t si523_identity_card_get_uid(uint8_t *uid, uint8_t *uid_len);

// ACD低功耗功能
void si523_acd_auto_calc(void);
void si523_acd_config(void);
void si523_acd_init(void);
void si523_acd_start(void);
uint8_t si523_acd_irq_process(void);
void si523_modify_reg(uint8_t reg_addr, bool set_bit, uint8_t mask_byte);

#endif // SI523_H