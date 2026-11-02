#include "jammer.h"
#include "sdkconfig.h"

#if CONFIG_JAMMER_ENABLE

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

/* Publiczne API menedżerów - sprawdzamy tylko czy skanery nie działają */
extern bool nrf24_remote_manager_is_running(void);
extern bool subghz_remote_manager_is_running(void);

static const char *TAG = "jammer";

/* ==================== nRF24 ==================== */
#define NRF_CMD_W_REGISTER 0x20
#define NRF_REG_CONFIG     0x00
#define NRF_REG_EN_AA      0x01
#define NRF_REG_EN_RXADDR  0x02
#define NRF_REG_SETUP_AW   0x03
#define NRF_REG_RF_CH      0x05
#define NRF_REG_RF_SETUP   0x06
#define NRF_REG_DYNPD      0x1C
#define NRF_REG_FEATURE    0x1D

/* ==================== CC1101 ==================== */
#define CC1101_REG_IOCFG0    0x02
#define CC1101_REG_FIFOTHR   0x03
#define CC1101_REG_PKTCTRL0  0x08
#define CC1101_REG_FSCTRL1   0x0B
#define CC1101_REG_CHANNR    0x0A
#define CC1101_REG_FREQ2     0x0D
#define CC1101_REG_FREQ1     0x0E
#define CC1101_REG_FREQ0     0x0F
#define CC1101_REG_MDMCFG4   0x10
#define CC1101_REG_MDMCFG3   0x11
#define CC1101_REG_MDMCFG2   0x12
#define CC1101_REG_MDMCFG1   0x13
#define CC1101_REG_MDMCFG0   0x14
#define CC1101_REG_MCSM0     0x18
#define CC1101_REG_FOCCFG    0x19
#define CC1101_REG_AGCCTRL2  0x1B
#define CC1101_REG_AGCCTRL1  0x1C
#define CC1101_REG_AGCCTRL0  0x1D
#define CC1101_REG_FREND1    0x21
#define CC1101_REG_FREND0    0x22
#define CC1101_REG_WORCTRL   0x20

#define CC1101_STROBE_SRES  0x30
#define CC1101_STROBE_SCAL  0x33
#define CC1101_STROBE_STX   0x35
#define CC1101_STROBE_SIDLE 0x36
#define CC1101_STROBE_SFRX  0x3A
#define CC1101_STROBE_SFTX  0x3B

typedef struct { uint8_t reg; uint8_t val; } cc1101_reg_entry_t;

/* kopia z subghz_remote_manager.c - preset OOK650 */
static const cc1101_reg_entry_t s_preset_ook650[] = {
    {CC1101_REG_IOCFG0,   0x0D},
    {CC1101_REG_FIFOTHR,  0x07},
    {CC1101_REG_PKTCTRL0, 0x32},
    {CC1101_REG_FSCTRL1,  0x06},
    {CC1101_REG_MDMCFG0,  0x00},
    {CC1101_REG_MDMCFG1,  0x00},
    {CC1101_REG_MDMCFG2,  0x30},
    {CC1101_REG_MDMCFG3,  0x32},
    {CC1101_REG_MDMCFG4,  0x17},
    {CC1101_REG_MCSM0,    0x18},
    {CC1101_REG_FOCCFG,   0x18},
    {CC1101_REG_AGCCTRL0, 0x91},
    {CC1101_REG_AGCCTRL1, 0x00},
    {CC1101_REG_AGCCTRL2, 0x07},
    {CC1101_REG_WORCTRL,  0xFB},
    {CC1101_REG_FREND0,   0x11},
    {CC1101_REG_FREND1,   0xB6},
    {0, 0},
};

/* kopia z subghz_remote_manager.c */
static const uint8_t s_ook_patable[8] = {0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/* ==================== stan ==================== */
static SemaphoreHandle_t s_spi_mutex = NULL;
static TaskHandle_t      s_task      = NULL;
static volatile bool     s_run       = false;

static spi_device_handle_t s_nrf_dev = NULL;
static spi_device_handle_t s_cc_dev  = NULL;
static spi_host_device_t   s_spi_host = SPI3_HOST;
static bool                s_bus_initialized_by_us = false;

static inline void lock(void)   { if (s_spi_mutex) xSemaphoreTake(s_spi_mutex, portMAX_DELAY); }
static inline void unlock(void) { if (s_spi_mutex) xSemaphoreGive(s_spi_mutex); }

static void bus_deinit(void); /* forward */

/* ==================== SPI helper ==================== */
static esp_err_t spi_xfer(spi_device_handle_t dev, const uint8_t *tx, uint8_t *rx, size_t len)
{
    if (!dev || !tx || len == 0) return ESP_ERR_INVALID_STATE;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = (uint32_t)(len * 8);
    t.tx_buffer = tx;
    t.rx_buffer = rx;
    return spi_device_polling_transmit(dev, &t);
}

/* ==================== nRF24 ==================== */
static esp_err_t nrf_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { (uint8_t)(NRF_CMD_W_REGISTER | (reg & 0x1F)), val };
    uint8_t rx[2] = {0};
    return spi_xfer(s_nrf_dev, tx, rx, 2);
}

static esp_err_t nrf_carrier_init(void)
{
    gpio_config_t io_cfg = {
        .pin_bit_mask = (1ULL << CONFIG_NRF24_CE_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&io_cfg);
    if (err != ESP_OK) return err;
    gpio_set_level((gpio_num_t)CONFIG_NRF24_CE_PIN, 0);

    err = nrf_write_reg(NRF_REG_EN_AA, 0x00);
    if (err == ESP_OK) err = nrf_write_reg(NRF_REG_EN_RXADDR, 0x01);
    if (err == ESP_OK) err = nrf_write_reg(NRF_REG_SETUP_AW, 0x03);
    if (err == ESP_OK) err = nrf_write_reg(NRF_REG_RF_SETUP, 0x0E); /* 0dBm, 2Mbps */
    if (err == ESP_OK) err = nrf_write_reg(NRF_REG_DYNPD, 0x00);
    if (err == ESP_OK) err = nrf_write_reg(NRF_REG_FEATURE, 0x00);
    if (err == ESP_OK) err = nrf_write_reg(NRF_REG_CONFIG, 0x0E);   /* PWR_UP, PTX */
    return err;
}

static esp_err_t nrf_set_channel(uint8_t ch)
{
    return nrf_write_reg(NRF_REG_RF_CH, (uint8_t)(ch & 0x7F));
}

static inline void nrf_ce(int level)
{
    gpio_set_level((gpio_num_t)CONFIG_NRF24_CE_PIN, level ? 1 : 0);
}

/* ==================== CC1101 ==================== */
static esp_err_t cc_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { reg, val };
    uint8_t rx[2] = {0};
    return spi_xfer(s_cc_dev, tx, rx, 2);
}

static esp_err_t cc_strobe(uint8_t cmd)
{
    uint8_t tx[1] = { cmd };
    uint8_t rx[1] = {0};
    return spi_xfer(s_cc_dev, tx, rx, 1);
}

static esp_err_t cc_write_patable(const uint8_t *data, size_t len)
{
    if (!data || len == 0 || !s_cc_dev) return ESP_ERR_INVALID_ARG;
    size_t total = 1 + len;
    uint8_t *buf = (uint8_t *)malloc(total);
    if (!buf) return ESP_ERR_NO_MEM;
    buf[0] = 0x7E;
    memcpy(&buf[1], data, len);
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = (uint32_t)(total * 8);
    t.tx_buffer = buf;
    esp_err_t err = spi_device_polling_transmit(s_cc_dev, &t);
    free(buf);
    return err;
}

static esp_err_t cc_set_freq_hz(uint32_t freq_hz)
{
    uint32_t freq_word = (uint32_t)(((uint64_t)freq_hz * 65536ULL) / 26000000ULL);
    esp_err_t err = cc_write_reg(CC1101_REG_FREQ2, (uint8_t)((freq_word >> 16) & 0xFF));
    if (err == ESP_OK) err = cc_write_reg(CC1101_REG_FREQ1, (uint8_t)((freq_word >> 8) & 0xFF));
    if (err == ESP_OK) err = cc_write_reg(CC1101_REG_FREQ0, (uint8_t)(freq_word & 0xFF));
    return err;
}

static esp_err_t cc_apply_preset_ook(void)
{
    esp_err_t err = ESP_OK;
    for (int i = 0; s_preset_ook650[i].reg != 0 && err == ESP_OK; i++) {
        err = cc_write_reg(s_preset_ook650[i].reg, s_preset_ook650[i].val);
    }
    if (err == ESP_OK) err = cc_write_patable(s_ook_patable, 8);
    return err;
}

static esp_err_t cc_carrier_init(uint32_t freq_hz)
{
    /* GDO0 jako OUTPUT - carrier OOK ciagly gdy GDO0=1 */
    gpio_config_t gdo_cfg = {
        .pin_bit_mask = (1ULL << CONFIG_SUBGHZ_GDO0_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&gdo_cfg);
    if (err != ESP_OK) return err;
    gpio_set_level((gpio_num_t)CONFIG_SUBGHZ_GDO0_PIN, 1);

    err = cc_strobe(CC1101_STROBE_SRES);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(2));
    cc_strobe(CC1101_STROBE_SIDLE);
    cc_strobe(CC1101_STROBE_SFRX);
    cc_strobe(CC1101_STROBE_SFTX);

    err = cc_set_freq_hz(freq_hz);
    if (err == ESP_OK) err = cc_apply_preset_ook();
    if (err == ESP_OK) err = cc_write_reg(CC1101_REG_CHANNR, 0x00);
    if (err == ESP_OK) err = cc_write_reg(CC1101_REG_IOCFG0, 0x2E); /* GDO0 = TX data in */
    if (err == ESP_OK) err = cc_strobe(CC1101_STROBE_SCAL);
    return err;
}

static esp_err_t cc_set_chan(uint8_t ch)
{
    return cc_write_reg(CC1101_REG_CHANNR, ch);
}

/* ==================== SPI bus ==================== */
static esp_err_t bus_init(void)
{
    s_spi_host = SPI3_HOST;
#if defined(SPI3_HOST)
    if (CONFIG_SUBGHZ_SPI_HOST == 2) s_spi_host = SPI2_HOST;
#endif

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = CONFIG_SUBGHZ_SPI_MOSI_PIN,
        .miso_io_num = CONFIG_SUBGHZ_SPI_MISO_PIN,
        .sclk_io_num = CONFIG_SUBGHZ_SPI_SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
    esp_err_t err = spi_bus_initialize(s_spi_host, &bus_cfg, SPI_DMA_CH_AUTO);
    if (err == ESP_OK) {
        s_bus_initialized_by_us = true;
    } else if (err == ESP_ERR_INVALID_STATE) {
        s_bus_initialized_by_us = false;
    } else {
        return err;
    }

    spi_device_interface_config_t nrf_cfg = {
        .clock_speed_hz = CONFIG_NRF24_SPI_CLOCK_HZ,
        .mode = 0,
        .spics_io_num = CONFIG_NRF24_CSN_PIN,
        .queue_size = 1,
    };
    err = spi_bus_add_device(s_spi_host, &nrf_cfg, &s_nrf_dev);
    if (err != ESP_OK) { bus_deinit(); return err; }

    spi_device_interface_config_t cc_cfg = {
        .clock_speed_hz = CONFIG_SUBGHZ_SPI_CLOCK_HZ,
        .mode = 0,
        .spics_io_num = CONFIG_SUBGHZ_CSN_PIN,
        .queue_size = 1,
    };
    err = spi_bus_add_device(s_spi_host, &cc_cfg, &s_cc_dev);
    if (err != ESP_OK) { bus_deinit(); return err; }

    return ESP_OK;
}

static void bus_deinit(void)
{
    if (s_nrf_dev) { spi_bus_remove_device(s_nrf_dev); s_nrf_dev = NULL; }
    if (s_cc_dev)  { spi_bus_remove_device(s_cc_dev);  s_cc_dev  = NULL; }
    if (s_bus_initialized_by_us) {
        spi_bus_free(s_spi_host);
        s_bus_initialized_by_us = false;
    }
}

/* ==================== task ==================== */
static void jammer_task(void *arg)
{
    (void)arg;
    uint8_t nrf_ch = 0;
    uint8_t cc_ch  = 0;

    ESP_LOGI(TAG, "up, hop=%d ms", CONFIG_JAMMER_HOP_MS);

    lock();
    nrf_ce(1);
    cc_strobe(CC1101_STROBE_STX);
    unlock();

    while (s_run) {
        lock();
        nrf_set_channel(nrf_ch);
        cc_set_chan(cc_ch);
        unlock();

        nrf_ch = (uint8_t)((nrf_ch + CONFIG_JAMMER_NRF24_DRIFT) % 84);
        cc_ch  = (uint8_t)(cc_ch + 1);

        vTaskDelay(pdMS_TO_TICKS(CONFIG_JAMMER_HOP_MS));
    }

    lock();
    nrf_ce(0);
    cc_strobe(CC1101_STROBE_SIDLE);
    gpio_set_level((gpio_num_t)CONFIG_SUBGHZ_GDO0_PIN, 0);
    unlock();

    ESP_LOGI(TAG, "down");
    vTaskDelete(NULL);
}

/* ==================== API ==================== */
esp_err_t jammer_start(void)
{
    if (s_run) return ESP_OK;

    if (nrf24_remote_manager_is_running() || subghz_remote_manager_is_running()) {
        ESP_LOGE(TAG, "stop scanners first (nrf24/subghz)");
        return ESP_ERR_INVALID_STATE;
    }

    if (!s_spi_mutex) {
        s_spi_mutex = xSemaphoreCreateMutex();
        if (!s_spi_mutex) return ESP_ERR_NO_MEM;
    }

    lock();
    esp_err_t err = bus_init();
    if (err == ESP_OK) err = nrf_carrier_init();
    if (err == ESP_OK) {
        uint32_t base = (uint32_t)(CONFIG_SUBGHZ_BASE_FREQ_MHZ * 1000000ULL);
        err = cc_carrier_init(base);
    }
    unlock();

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "init failed: %s", esp_err_to_name(err));
        lock(); bus_deinit(); unlock();
        return err;
    }

    s_run = true;
    if (xTaskCreatePinnedToCore(jammer_task, "jammer", 4096, NULL, 5, &s_task, 1) != pdPASS) {
        s_run = false;
        lock(); bus_deinit(); unlock();
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t jammer_stop(void)
{
    if (!s_run) return ESP_OK;
    s_run = false;
    vTaskDelay(pdMS_TO_TICKS(CONFIG_JAMMER_HOP_MS * 2 + 100));
    s_task = NULL;
    lock(); bus_deinit(); unlock();
    return ESP_OK;
}

bool jammer_is_running(void)
{
    return s_run;
}

#endif /* CONFIG_JAMMER_ENABLE */
