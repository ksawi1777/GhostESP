#include "jammer.h"
#include "sdkconfig.h"

#if CONFIG_JAMMER_ENABLE

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include <stdint.h>

static const char *TAG = "jammer";

/* PODMIEN NAZWY na te z twojego forka (sprawdz w CZESCI G) */
extern void     nrf24_write_reg(uint8_t reg, uint8_t val);
extern void     nrf24_ce_set(int level);
extern void     cc1101_write_reg(uint8_t reg, uint8_t val);
extern void     cc1101_strobe(uint8_t cmd);
extern void     cc1101_set_freq(uint32_t freq_khz);
extern void     subghz_ant_switch(int sw0, int sw1);

#define NRF24_REG_CONFIG    0x00
#define NRF24_REG_RF_CH     0x05
#define NRF24_REG_RF_SETUP  0x06

#define CC1101_STX          0x35
#define CC1101_SIDLE        0x36

static SemaphoreHandle_t s_spi_mutex = NULL;
static TaskHandle_t      s_task      = NULL;
static volatile bool     s_run       = false;

static inline void lock(void)   { xSemaphoreTake(s_spi_mutex, portMAX_DELAY); }
static inline void unlock(void) { xSemaphoreGive(s_spi_mutex); }

static void nrf24_carrier_on(uint8_t ch)
{
    nrf24_write_reg(NRF24_REG_CONFIG,   0x0E);
    nrf24_write_reg(NRF24_REG_RF_CH,    ch);
    nrf24_write_reg(NRF24_REG_RF_SETUP, 0x0E);
    nrf24_ce_set(1);
}

static void nrf24_carrier_off(void)
{
    nrf24_ce_set(0);
    nrf24_write_reg(NRF24_REG_CONFIG, 0x0C);
}

static void cc1101_carrier_on(uint32_t freq_khz)
{
    cc1101_set_freq(freq_khz);
    cc1101_strobe(CC1101_STX);
}

static void cc1101_carrier_off(void)
{
    cc1101_strobe(CC1101_SIDLE);
}

static void jammer_task(void *arg)
{
    uint8_t  nrf_ch   = 0;
    uint32_t cc_freq  = CONFIG_SUBGHZ_BASE_FREQ_MHZ * 1000U;

    ESP_LOGI(TAG, "task up, hop=%d ms", CONFIG_JAMMER_HOP_MS);

    while (s_run) {
        lock();
        nrf24_carrier_on(nrf_ch);
        cc1101_carrier_on(cc_freq);
        unlock();

        nrf_ch  = (uint8_t)((nrf_ch + CONFIG_JAMMER_NRF24_DRIFT) % 84);
        cc_freq += CONFIG_JAMMER_CC1101_SWEEP_KHZ;
        if (cc_freq > (CONFIG_SUBGHZ_BASE_FREQ_MHZ * 1000U) + 1000U) {
            cc_freq = CONFIG_SUBGHZ_BASE_FREQ_MHZ * 1000U;
        }

        vTaskDelay(pdMS_TO_TICKS(CONFIG_JAMMER_HOP_MS));
    }

    lock();
    nrf24_carrier_off();
    cc1101_carrier_off();
    unlock();

    ESP_LOGI(TAG, "task down");
    vTaskDelete(NULL);
}

esp_err_t jammer_start(void)
{
    if (s_run) return ESP_OK;

    if (!s_spi_mutex) {
        s_spi_mutex = xSemaphoreCreateMutex();
        if (!s_spi_mutex) return ESP_ERR_NO_MEM;
    }

    subghz_ant_switch(1, 0);

    s_run = true;
    if (xTaskCreatePinnedToCore(jammer_task, "jammer", 4096,
                                NULL, 5, &s_task, 1) != pdPASS) {
        s_run = false;
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t jammer_stop(void)
{
    if (!s_run) return ESP_OK;
    s_run = false;
    vTaskDelay(pdMS_TO_TICKS((CONFIG_JAMMER_HOP_MS * 2) + 50));
    s_task = NULL;
    return ESP_OK;
}

bool jammer_is_running(void)
{
    return s_run;
}

#endif /* CONFIG_JAMMER_ENABLE */
