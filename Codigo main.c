// Laboratorio 4: María José Rojas Mosquera y Orlando Velásquez Granda
// main.c

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "nvs_flash.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "LAB4";

//  PINES
#define PIN_LED_ROJO    25
#define PIN_LED_VERDE   26
#define PIN_LED_AZUL    27
#define PIN_BUZZER      32

#define PIN_SPI_MISO    19
#define PIN_SPI_MOSI    23
#define PIN_SPI_SCK     18
#define PIN_RC522_CS    5
#define PIN_RC522_RST   22

#define I2C_PORT        I2C_NUM_0
#define PIN_I2C_SDA     16
#define PIN_I2C_SCL     17

#define LCD_ADDR        0x27
#define DS1307_ADDR     0x68

//  UIDS AUTORIZADOS
static const uint8_t authorized_uid[4] = {
    0xA9, 0xE5, 0x28, 0x07
};

//  RC522 REGISTROS
#define CommandReg      0x01
#define ComIrqReg       0x04
#define ErrorReg        0x06
#define FIFODataReg     0x09
#define FIFOLevelReg    0x0A
#define ControlReg      0x0C
#define BitFramingReg   0x0D
#define ModeReg         0x11
#define TxControlReg    0x14
#define TxASKReg        0x15
#define TModeReg        0x2A
#define TPrescalerReg   0x2B
#define TReloadRegH     0x2C
#define TReloadRegL     0x2D
#define VersionReg      0x37
#define PCD_IDLE        0x00
#define PCD_TRANSCEIVE  0x0C
#define PCD_SOFTRESET   0x0F
#define PICC_REQALL     0x52
#define PICC_ANTICOLL   0x93

//  ESTADO GLOBAL
volatile int sistema_activo    = 0;
char ultimo_mensaje[17]        = "Sin mensajes";
char ble_received_msg[100]     = {0};
bool ble_message_received      = false;

//  BUZZER
static void buzzer_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz         = 2000,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t channel = {
        .gpio_num   = PIN_BUZZER,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&channel);
}

static void buzzer_on(void)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 512);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static void buzzer_off(void)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static void buzzer_beep(int ms)
{
    buzzer_on();
    vTaskDelay(pdMS_TO_TICKS(ms));
    buzzer_off();
}

//  LEDS
static void led_init(void)
{
    gpio_reset_pin(PIN_LED_ROJO);
    gpio_reset_pin(PIN_LED_VERDE);
    gpio_reset_pin(PIN_LED_AZUL);
    gpio_set_direction(PIN_LED_ROJO,  GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_LED_VERDE, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_LED_AZUL,  GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_LED_ROJO,  0);
    gpio_set_level(PIN_LED_VERDE, 0);
    gpio_set_level(PIN_LED_AZUL,  0);
}

//  I2C
static void i2c_init(void)
{
    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = PIN_I2C_SDA,
        .scl_io_num       = PIN_I2C_SCL,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };
    i2c_param_config(I2C_PORT, &conf);
    i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
}

//  LCD I2C

#define LCD_BACKLIGHT 0x08
#define LCD_ENABLE    0x04
#define LCD_RS        0x01

static void lcd_write_byte(uint8_t data)
{
    i2c_master_write_to_device(
        I2C_PORT, LCD_ADDR, &data, 1, pdMS_TO_TICKS(100));
}

static void lcd_pulse_enable(uint8_t data)
{
    lcd_write_byte(data | LCD_ENABLE | LCD_BACKLIGHT);
    esp_rom_delay_us(1000);
    lcd_write_byte((data & ~LCD_ENABLE) | LCD_BACKLIGHT);
    esp_rom_delay_us(1000);
}

static void lcd_send_nibble(uint8_t nibble, uint8_t mode)
{
    uint8_t data = (nibble & 0xF0) | mode | LCD_BACKLIGHT;
    lcd_pulse_enable(data);
}

static void lcd_send_byte(uint8_t value, uint8_t mode)
{
    lcd_send_nibble(value & 0xF0, mode);
    lcd_send_nibble((value << 4) & 0xF0, mode);
}

static void lcd_cmd(uint8_t cmd) { lcd_send_byte(cmd, 0); }
static void lcd_char(uint8_t c)  { lcd_send_byte(c, LCD_RS); }

static void lcd_init(void)
{
    vTaskDelay(pdMS_TO_TICKS(100));
    lcd_send_nibble(0x30, 0); vTaskDelay(pdMS_TO_TICKS(10));
    lcd_send_nibble(0x30, 0); vTaskDelay(pdMS_TO_TICKS(10));
    lcd_send_nibble(0x30, 0); vTaskDelay(pdMS_TO_TICKS(10));
    lcd_send_nibble(0x20, 0); vTaskDelay(pdMS_TO_TICKS(10));
    lcd_cmd(0x28); vTaskDelay(pdMS_TO_TICKS(5));
    lcd_cmd(0x08); vTaskDelay(pdMS_TO_TICKS(5));
    lcd_cmd(0x01); vTaskDelay(pdMS_TO_TICKS(5));
    lcd_cmd(0x06); vTaskDelay(pdMS_TO_TICKS(5));
    lcd_cmd(0x0C); vTaskDelay(pdMS_TO_TICKS(5));
}

static void lcd_clear(void)
{
    lcd_cmd(0x01);
    vTaskDelay(pdMS_TO_TICKS(5));
}

static void lcd_set_cursor(uint8_t row, uint8_t col)
{
    if (row == 0) lcd_cmd(0x80 + col);
    else          lcd_cmd(0xC0 + col);
}

static void lcd_print(const char *str)
{
    while (*str) lcd_char((uint8_t)*str++);
}

//  RTC 
static uint8_t dec_to_bcd(uint8_t val)
{
    return ((val / 10) << 4) | (val % 10);
}

static uint8_t bcd_to_dec(uint8_t val)
{
    return ((val >> 4) * 10) + (val & 0x0F);
}

static void rtc_init(void)
{
    uint8_t reg = 0x00;
    uint8_t sec;
    i2c_master_write_read_device(
        I2C_PORT, DS1307_ADDR, &reg, 1, &sec, 1,
        pdMS_TO_TICKS(100));

    if (sec & 0x80) {
        uint8_t data[8] = {
            0x00,
            dec_to_bcd(0),
            dec_to_bcd(0),
            dec_to_bcd(0),
            dec_to_bcd(1),
            dec_to_bcd(1),
            dec_to_bcd(1),
            dec_to_bcd(25)
        };
        i2c_master_write_to_device(
            I2C_PORT, DS1307_ADDR, data, 8,
            pdMS_TO_TICKS(100));
    }
}

static void rtc_get_time(char *buffer)
{
    uint8_t reg = 0x00;
    uint8_t data[3];
    i2c_master_write_read_device(
        I2C_PORT, DS1307_ADDR, &reg, 1, data, 3,
        pdMS_TO_TICKS(100));

    uint8_t seconds = bcd_to_dec(data[0] & 0x7F);
    uint8_t minutes = bcd_to_dec(data[1]);
    uint8_t hours   = bcd_to_dec(data[2] & 0x3F);

    sprintf(buffer, "%02d:%02d:%02d", hours, minutes, seconds);
}

//  RC522 
static spi_device_handle_t rc522_spi;

static void rc522_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { (reg << 1) & 0x7E, value };
    spi_transaction_t t = { .length = 16, .tx_buffer = tx };
    spi_device_transmit(rc522_spi, &t);
}

static uint8_t rc522_read_reg(uint8_t reg)
{
    uint8_t tx[2] = { ((reg << 1) & 0x7E) | 0x80, 0x00 };
    uint8_t rx[2] = {0};
    spi_transaction_t t = {
        .length = 16, .tx_buffer = tx, .rx_buffer = rx };
    spi_device_transmit(rc522_spi, &t);
    return rx[1];
}

static void rc522_set_bit_mask(uint8_t reg, uint8_t mask)
{
    rc522_write_reg(reg, rc522_read_reg(reg) | mask);
}

static void rc522_clear_bit_mask(uint8_t reg, uint8_t mask)
{
    rc522_write_reg(reg, rc522_read_reg(reg) & (~mask));
}

static void rc522_reset(void)
{
    gpio_set_direction(PIN_RC522_RST, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_RC522_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(PIN_RC522_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
    rc522_write_reg(CommandReg, PCD_SOFTRESET);
    vTaskDelay(pdMS_TO_TICKS(50));
}

static void rc522_antenna_on(void)
{
    uint8_t temp = rc522_read_reg(TxControlReg);
    if (!(temp & 0x03))
        rc522_set_bit_mask(TxControlReg, 0x03);
}

static void rc522_init_chip(void)
{
    rc522_reset();
    rc522_write_reg(TModeReg,      0x8D);
    rc522_write_reg(TPrescalerReg, 0x3E);
    rc522_write_reg(TReloadRegL,   30);
    rc522_write_reg(TReloadRegH,   0);
    rc522_write_reg(TxASKReg,      0x40);
    rc522_write_reg(ModeReg,       0x3D);
    rc522_antenna_on();
}

static int rc522_to_card(uint8_t command,
                          uint8_t *sendData, uint8_t sendLen,
                          uint8_t *backData, uint16_t *backLen)
{
    uint8_t waitIRq = 0x00;
    uint8_t n;
    uint16_t i;

    if (command == PCD_TRANSCEIVE) waitIRq = 0x30;

    rc522_write_reg(ComIrqReg, 0x7F);
    rc522_set_bit_mask(FIFOLevelReg, 0x80);
    rc522_write_reg(CommandReg, PCD_IDLE);

    for (i = 0; i < sendLen; i++)
        rc522_write_reg(FIFODataReg, sendData[i]);

    rc522_write_reg(CommandReg, command);

    if (command == PCD_TRANSCEIVE)
        rc522_set_bit_mask(BitFramingReg, 0x80);

    i = 5000;
    do {
        n = rc522_read_reg(ComIrqReg);
        i--;
    } while ((i != 0) && !(n & 0x01) && !(n & waitIRq));

    rc522_clear_bit_mask(BitFramingReg, 0x80);

    if (i == 0) return 0;
    if (rc522_read_reg(ErrorReg) & 0x1B) return 0;

    if (command == PCD_TRANSCEIVE) {
        n = rc522_read_reg(FIFOLevelReg);
        uint8_t lastBits = rc522_read_reg(ControlReg) & 0x07;
        if (lastBits) *backLen = (n - 1) * 8 + lastBits;
        else          *backLen = n * 8;
        if (n > 16) n = 16;
        for (i = 0; i < n; i++)
            backData[i] = rc522_read_reg(FIFODataReg);
    }
    return 1;
}

static int rc522_request(uint8_t reqMode, uint8_t *TagType)
{
    uint16_t backBits;
    uint8_t buffer[1];
    buffer[0] = reqMode;
    rc522_write_reg(BitFramingReg, 0x07);
    int status = rc522_to_card(
        PCD_TRANSCEIVE, buffer, 1, TagType, &backBits);
    if ((status != 1) || (backBits != 0x10)) return 0;
    return 1;
}

static int rc522_anticoll(uint8_t *serNum)
{
    uint16_t unLen;
    uint8_t buffer[2];
    buffer[0] = PICC_ANTICOLL;
    buffer[1] = 0x20;
    rc522_write_reg(BitFramingReg, 0x00);
    int status = rc522_to_card(
        PCD_TRANSCEIVE, buffer, 2, serNum, &unLen);
    if (status) {
        uint8_t check = 0;
        for (int i = 0; i < 4; i++) check ^= serNum[i];
        if (check != serNum[4]) return 0;
        return 1;
    }
    return 0;
}

static void rfid_init(void)
{
    spi_bus_config_t buscfg = {
        .miso_io_num   = PIN_SPI_MISO,
        .mosi_io_num   = PIN_SPI_MOSI,
        .sclk_io_num   = PIN_SPI_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000,
        .mode           = 0,
        .spics_io_num   = PIN_RC522_CS,
        .queue_size     = 1
    };
    spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    spi_bus_add_device(SPI2_HOST, &devcfg, &rc522_spi);
    rc522_init_chip();
    printf("RC522 Version: 0x%02X\n",
           rc522_read_reg(VersionReg));
}

static void rfid_task(void)
{
    uint8_t tagType[2];
    uint8_t uid[5];

    if (rc522_request(PICC_REQALL, tagType)) {
        if (rc522_anticoll(uid)) {
            printf("UID: %02X %02X %02X %02X\n",
                   uid[0], uid[1], uid[2], uid[3]);

            while (rc522_request(PICC_REQALL, tagType))
                vTaskDelay(pdMS_TO_TICKS(50));

            bool auth =
                uid[0] == authorized_uid[0] &&
                uid[1] == authorized_uid[1] &&
                uid[2] == authorized_uid[2] &&
                uid[3] == authorized_uid[3];

            // Cierre de sesion
            if (auth && sistema_activo == 1) {
                printf("Cierre de sesion\n");
                buzzer_beep(500);
                gpio_set_level(PIN_LED_ROJO,  0);
                gpio_set_level(PIN_LED_VERDE, 0);
                gpio_set_level(PIN_LED_AZUL,  0);
                gpio_set_level(PIN_LED_ROJO,  1);
                sistema_activo = 0;
                strncpy(ultimo_mensaje, "Sin mensajes", 16);
                lcd_clear();
                vTaskDelay(pdMS_TO_TICKS(20));
                lcd_set_cursor(0, 0); lcd_print("Panel bloqueado");
                lcd_set_cursor(1, 0); lcd_print("Acerque cred.");
                vTaskDelay(pdMS_TO_TICKS(1000));
                return;
            }

            // Acceso concedido 
            if (auth && sistema_activo == 0) {
                printf("Acceso concedido\n");
                gpio_set_level(PIN_LED_ROJO,  0);
                gpio_set_level(PIN_LED_VERDE, 0);
                gpio_set_level(PIN_LED_AZUL,  0);
                gpio_set_level(PIN_LED_VERDE, 1);
                buzzer_beep(500);
                char hora[20];
                rtc_get_time(hora);
                lcd_clear();
                vTaskDelay(pdMS_TO_TICKS(20));
                lcd_set_cursor(0, 0); lcd_print("Acceso concedido");
                lcd_set_cursor(1, 0); lcd_print(hora);
                vTaskDelay(pdMS_TO_TICKS(1000));
                gpio_set_level(PIN_LED_VERDE, 0);
                gpio_set_level(PIN_LED_AZUL,  1);
                sistema_activo = 1;
                strncpy(ultimo_mensaje, "Sin mensajes", 16);
                return;
            }

            // Acceso denegado 
            if (!auth) {
                printf("Acceso denegado\n");
                lcd_clear();
                vTaskDelay(pdMS_TO_TICKS(20));
                lcd_set_cursor(0, 0); lcd_print("Acceso denegado");
                lcd_set_cursor(1, 0); lcd_print("UID no registrado");
                buzzer_on();
                for (int i = 0; i < 3; i++) {
                    gpio_set_level(PIN_LED_ROJO, 1);
                    vTaskDelay(pdMS_TO_TICKS(300));
                    gpio_set_level(PIN_LED_ROJO, 0);
                    vTaskDelay(pdMS_TO_TICKS(300));
                }
                vTaskDelay(pdMS_TO_TICKS(200));
                buzzer_off();

                // Apagar rojo siempre al terminar
                gpio_set_level(PIN_LED_ROJO, 0);

                // Restaurar estado anterior
                if (sistema_activo == 0) {
                    // Estaba bloqueado: rojo fijo
                    gpio_set_level(PIN_LED_ROJO, 1);
                    lcd_clear();
                    vTaskDelay(pdMS_TO_TICKS(20));
                    lcd_set_cursor(0, 0); lcd_print("Panel bloqueado");
                    lcd_set_cursor(1, 0); lcd_print("Acerque cred.");
                } else {
                    // Estaba activo: azul sigue encendido
                    gpio_set_level(PIN_LED_AZUL, 1);
                    lcd_clear();
                    vTaskDelay(pdMS_TO_TICKS(20));
                    lcd_set_cursor(0, 0); lcd_print(ultimo_mensaje);
                    char hora_tmp[20];
                    rtc_get_time(hora_tmp);
                    lcd_set_cursor(1, 0); lcd_print(hora_tmp);
                }
            }

            vTaskDelay(pdMS_TO_TICKS(1500));
        }
    }
}

#define DEVICE_NAME "PanelHMI"

static uint8_t ble_addr_type;

static const ble_uuid128_t nus_service_uuid =
    BLE_UUID128_INIT(
        0x9E,0xCA,0xDC,0x24,0x0E,0xE5,0xA9,0xE0,
        0x93,0xF3,0xA3,0xB5,0x01,0x00,0x40,0x6E);

static const ble_uuid128_t nus_rx_uuid =
    BLE_UUID128_INIT(
        0x9E,0xCA,0xDC,0x24,0x0E,0xE5,0xA9,0xE0,
        0x93,0xF3,0xA3,0xB5,0x02,0x00,0x40,0x6E);

static void ble_app_advertise(void);

static int ble_rx_callback(uint16_t conn_handle,
                            uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt,
                            void *arg)
{
    if (sistema_activo == 1) {
        int len = OS_MBUF_PKTLEN(ctxt->om);
        if (len >= (int)sizeof(ble_received_msg))
            len = sizeof(ble_received_msg) - 1;
        os_mbuf_copydata(ctxt->om, 0, len, ble_received_msg);
        ble_received_msg[len] = '\0';
        ble_message_received = true;
        printf("Mensaje BLE: %s\n", ble_received_msg);
    }
    return 0;
}

static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            ESP_LOGI("BLE", "Conexion BLE %s",
                event->connect.status == 0 ?
                "exitosa" : "fallida");
            if (event->connect.status != 0)
                ble_app_advertise();
            break;
        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI("BLE", "BLE desconectado");
            ble_app_advertise();
            break;
        default:
            break;
    }
    return 0;
}

static void ble_app_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN |
                   BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name             = (uint8_t *)DEVICE_NAME;
    fields.name_len         = strlen(DEVICE_NAME);
    fields.name_is_complete = 1;
    ble_gap_adv_set_fields(&fields);
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER,
                      &adv_params, ble_gap_event, NULL);
}

static void ble_app_on_sync(void)
{
    ble_hs_id_infer_auto(0, &ble_addr_type);
    ble_app_advertise();
    ESP_LOGI("BLE", "BLE listo - PanelHMI");
}

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &nus_service_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid      = &nus_rx_uuid.u,
                .access_cb = ble_rx_callback,
                .flags     = BLE_GATT_CHR_F_WRITE,
            },
            { 0 }
        }
    },
    { 0 }
};

static void ble_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void ble_init(void)
{
    nimble_port_init();
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svcs);
    ble_gatts_add_svcs(gatt_svcs);
    ble_svc_gap_device_name_set(DEVICE_NAME);
    ble_hs_cfg.sync_cb = ble_app_on_sync;
    nimble_port_freertos_init(ble_host_task);
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    ESP_LOGI(TAG, "=== Laboratorio 4 - Panel HMI ===");

    led_init();
    i2c_init();
    lcd_init();
    rtc_init();
    rfid_init();
    buzzer_init();
    ble_init();

    // Estado inicial bloqueado
    gpio_set_level(PIN_LED_ROJO,  0);
    gpio_set_level(PIN_LED_VERDE, 0);
    gpio_set_level(PIN_LED_AZUL,  0);
    gpio_set_level(PIN_LED_ROJO,  1);
    lcd_clear();
    vTaskDelay(pdMS_TO_TICKS(20));
    lcd_set_cursor(0, 0); lcd_print("Panel bloqueado");
    lcd_set_cursor(1, 0); lcd_print("Acerque cred.");

    char hora[20];

    while (1) {
        rfid_task();

        if (sistema_activo == 1) {
            if (ble_message_received) {
                strncpy(ultimo_mensaje, ble_received_msg, 16);
                ultimo_mensaje[16] = '\0';
                ble_message_received = false;
            }
            rtc_get_time(hora);
            lcd_clear();
            vTaskDelay(pdMS_TO_TICKS(20));
            lcd_set_cursor(0, 0); lcd_print(ultimo_mensaje);
            lcd_set_cursor(1, 0); lcd_print(hora);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
