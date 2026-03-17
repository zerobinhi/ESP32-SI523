#include "si523.h"

static const char *TAG = "main";

uint8_t g_uid[4];
uint8_t g_uid_len = 4;

uint8_t g_pcd_irq_flag_a = 0;

// 全局句柄
SemaphoreHandle_t si523_semaphore = NULL;
i2c_master_bus_handle_t i2c_bus_handle = NULL;
i2c_master_dev_handle_t si523_handle = NULL;

// -------------------------- 资源清理函数 --------------------------
static void i2c_master_deinit(void)
{
    if (si523_handle != NULL)
    {
        i2c_master_bus_rm_device(si523_handle);
        si523_handle = NULL;
    }
    if (i2c_bus_handle != NULL)
    {
        i2c_del_master_bus(i2c_bus_handle);
        i2c_bus_handle = NULL;
    }
}

// -------------------------- IRQ中断处理函数 --------------------------
static void IRAM_ATTR si523_irq_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    ESP_EARLY_LOGI(TAG, "GPIO %d interrupt triggered", gpio_num);
    if (gpio_num == SI523_INT_PIN)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (si523_semaphore != NULL)
        {
            xSemaphoreGiveFromISR(si523_semaphore, &xHigherPriorityTaskWoken);
            g_pcd_irq_flag_a = 1;
        }
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// -------------------------- I2C初始化函数 --------------------------
static esp_err_t i2c_master_init(void)
{
    i2c_master_bus_config_t i2c_bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_NUM,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_handle);

    i2c_device_config_t si523_dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SI523_I2C_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    i2c_master_bus_add_device(i2c_bus_handle, &si523_dev_cfg, &si523_handle);

    return ESP_OK;
}

// -------------------------- GPIO中断配置函数 --------------------------
static esp_err_t si523_irq_init(void)
{
    gpio_config_t irq_gpio_cfg = {
        .pin_bit_mask = (1ULL << SI523_INT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&irq_gpio_cfg);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(SI523_INT_PIN, si523_irq_handler, (void *)SI523_INT_PIN);

    si523_semaphore = xSemaphoreCreateBinary();
    return (si523_semaphore == NULL) ? ESP_ERR_NO_MEM : ESP_OK;
}

// -------------------------- 卡片操作任务 --------------------------
void si523_task(void *arg)
{
    while (1)
    {
        if (xSemaphoreTake(si523_semaphore, portMAX_DELAY) == pdTRUE)
        {

            if (g_pcd_irq_flag_a)
            {
                ESP_LOGI(TAG, "g_pcd_irq_flag_a");
                gpio_intr_disable(SI523_INT_PIN); // Disable GPIO interrupt

                switch (si523_acd_irq_process())
                {
                case 0: // Other_IRQ
                    ESP_LOGI(TAG, "Other_IRQ:Read UID and reconfigure the register");
                    // ESP_LOGI(TAG, "Other IRQ Occur");
                    si523_type_a_init(); // 读A卡初始化配置
                    si523_soft_reset(); // 软复位
                    // si523_hard_reset(); // 硬复位
                    si523_type_a_init();
                    si523_acd_init();
                    break;

                case 1: // ACD_IRQ
                    ESP_LOGI(TAG, "ACD_IRQ:Read UID and reconfigure the register");
                    si523_clear_bit_mask(0x01, 0x20); // Turn on the analog part of receiver
                    // si523_type_a_rw_block_test();
                    si523_type_a_get_uid(g_uid, &g_uid_len);
                    si523_write_reg(SI523_REG_COMMAND, 0xb0); // 进入软掉电,重新进入ACD（ALPPL）
                    break;

                case 2: // ACDTIMER_IRQ
                    ESP_LOGI(TAG, "ACDTIMER_IRQ:Reconfigure the register");
                    si523_soft_reset(); // 软复位
                    // si523_hard_reset(); // 硬复位
                    si523_type_a_init();
                    si523_acd_init();
                    break;
                }

                gpio_intr_enable(SI523_INT_PIN); // Enable GPIO interrupt
                g_pcd_irq_flag_a = 0;
            }
            else
            {
                // delay_ms(500);
                vTaskDelay(pdMS_TO_TICKS(500));
            }
        }
    }
}

// -------------------------- 主程序入口 --------------------------
void app_main(void)
{
    ESP_LOGI(TAG, "Starting SI523 demo...");
    esp_err_t err;

    // 初始化外设和GPIO
    if ((err = i2c_master_init()) != ESP_OK)
        goto app_exit;

    si523_gpio_init(); // 初始化GPIO

    si523_hard_reset(); // 硬复位

    vTaskDelay(pdMS_TO_TICKS(500)); // 等待GPIO稳定

    // 配置中断
    if ((err = si523_irq_init()) != ESP_OK)
        goto app_exit;

    gpio_intr_disable(SI523_INT_PIN); // 先禁用中断，等任务准备好后再启用

    si523_acd_start();

    gpio_intr_enable(SI523_INT_PIN); // Enable GPIO interrupt
    g_pcd_irq_flag_a = 0;               // clear IRQ flag

    xTaskCreate(si523_task, "si523_task", 2048, NULL, 10, NULL);

    return;

app_exit:
    i2c_master_deinit();
    if (si523_semaphore)
    {
        vSemaphoreDelete(si523_semaphore);
        si523_semaphore = NULL;
    }
    ESP_LOGE(TAG, "Demo exited with error: %s", esp_err_to_name(err));
}