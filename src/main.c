/********************************************************************************************* */
// Eduboard2 ESP32-S3 - QAM Paketempfaenger, erster Transporttest ueber I2C
// Sender: TMP112 -> START / DATA / CHECK (Gray Code) -> I2C Master
// Empfaenger: I2C Slave 0x42 -> Gray decodieren -> pruefen -> Temperatur anzeigen
// Verdrahtung: GPIO2 (SDA) <-> GPIO2, GPIO1 (SCL) <-> GPIO1, GND <-> GND.
// Externe Pullups nach 3.3 V verwenden, sofern nicht bereits auf dem Board vorhanden.
// Ein Paket = 6 Bytes: START_H, START_L, DATA_H, DATA_L, CHECK_H, CHECK_L.
// Jedes 12-Bit-Feld wird als 16-Bit-Wert (MSB zuerst, oberes Nibble 0) gesendet.
/********************************************************************************************* */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/i2c_slave.h"
#include "driver/spi_master.h"
#include "esp_timer.h"

// ---------------- Pins ----------------
#define PIN_I2C_SDA     2
#define PIN_I2C_SCL     1
#define PIN_LCD_MOSI    13
#define PIN_LCD_SCK     14
#define PIN_LCD_CS      11
#define PIN_LCD_DC      16
#define PIN_LCD_RESET   8
#define LCD_WIDTH       320
#define LCD_HEIGHT      480
#define FONT_SCALE      2
#define RECEIVER_ADDR   0x42
#define PACKET_START    0xA5A
#define PACKET_BYTES    6

static spi_device_handle_t lcd;
static i2c_slave_dev_handle_t receiver;
static QueueHandle_t receive_queue;
static volatile uint32_t dropped_packets;

typedef struct {
    uint8_t bytes[PACKET_BYTES];
    uint32_t length;
} received_packet_t;

// ---------------- I2C Empfang ----------------
// Callback laeuft im Interrupt: Daten sofort kopieren, Display nur im Task bedienen.
static bool receive_done(i2c_slave_dev_handle_t handle,
                         const i2c_slave_rx_done_event_data_t *event, void *arg)
{
    (void)handle;
    (void)arg;
    received_packet_t packet = { .length = event->length };
    if (event->length == PACKET_BYTES) {
        memcpy(packet.bytes, event->buffer, PACKET_BYTES);
    }
    BaseType_t wake = pdFALSE;
    if (xQueueSendFromISR(receive_queue, &packet, &wake) != pdTRUE) {
        dropped_packets++;
    }
    return wake == pdTRUE;
}

static void receiver_init(void)
{
    receive_queue = xQueueCreate(8, sizeof(received_packet_t));
    ESP_ERROR_CHECK(receive_queue ? ESP_OK : ESP_ERR_NO_MEM);
    i2c_slave_config_t cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = PIN_I2C_SDA,
        .scl_io_num = PIN_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .send_buf_depth = 32,
        .receive_buf_depth = 64,
        .slave_addr = RECEIVER_ADDR,
        .addr_bit_len = I2C_ADDR_BIT_LEN_7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_slave_device(&cfg, &receiver));
    i2c_slave_event_callbacks_t callbacks = { .on_receive = receive_done };
    ESP_ERROR_CHECK(i2c_slave_register_event_callbacks(receiver, &callbacks, NULL));
}

// ---------------- Display (ILI9488 ueber SPI) ----------------
static void lcd_cmd(uint8_t cmd)
{
    spi_transaction_t t = { .length = 8, .tx_buffer = &cmd };
    gpio_set_level(PIN_LCD_DC, 0);
    spi_device_polling_transmit(lcd, &t);
}

static void lcd_data(const uint8_t *data, int len)
{
    spi_transaction_t t = { .length = len * 8, .tx_buffer = data };
    gpio_set_level(PIN_LCD_DC, 1);
    spi_device_polling_transmit(lcd, &t);
}

static void lcd_init(void)
{
    gpio_set_direction(PIN_LCD_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_LCD_RESET, GPIO_MODE_OUTPUT);

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_LCD_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = PIN_LCD_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = 20 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = PIN_LCD_CS,
        .queue_size = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev_cfg, &lcd));

    // Hardware Reset
    gpio_set_level(PIN_LCD_RESET, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(PIN_LCD_RESET, 1);
    vTaskDelay(pdMS_TO_TICKS(150));

    lcd_cmd(0x3A); lcd_data((uint8_t[]){0x66}, 1);  // 18 Bit pro Pixel (SPI kann kein 16 Bit beim ILI9488)
    lcd_cmd(0x36); lcd_data((uint8_t[]){0x48}, 1);  // Ausrichtung / BGR
    lcd_cmd(0x11);                                  // Sleep Out
    vTaskDelay(pdMS_TO_TICKS(120));
    lcd_cmd(0x29);                                  // Display On
}

static void lcd_set_window(int x, int y, int w, int h)
{
    int x2 = x + w - 1, y2 = y + h - 1;
    lcd_cmd(0x2A); lcd_data((uint8_t[]){x >> 8, x & 0xFF, x2 >> 8, x2 & 0xFF}, 4);
    lcd_cmd(0x2B); lcd_data((uint8_t[]){y >> 8, y & 0xFF, y2 >> 8, y2 & 0xFF}, 4);
    lcd_cmd(0x2C);  // ab hier folgen Pixeldaten
}

static void lcd_clear(void)
{
    static uint8_t line[LCD_WIDTH * 3];     // eine Zeile schwarz (3 Byte pro Pixel)
    memset(line, 0, sizeof(line));
    lcd_set_window(0, 0, LCD_WIDTH, LCD_HEIGHT);
    for (int y = 0; y < LCD_HEIGHT; y++) {
        lcd_data(line, sizeof(line));
    }
}

// 5x7 Font fuer ASCII ' ' bis 'Z' (spaltenweise, Bit 0 = oberste Zeile)
static const uint8_t font5x7[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, {0x00,0x00,0x5F,0x00,0x00}, {0x00,0x07,0x00,0x07,0x00}, {0x14,0x7F,0x14,0x7F,0x14}, //  !"#
    {0x24,0x2A,0x7F,0x2A,0x12}, {0x23,0x13,0x08,0x64,0x62}, {0x36,0x49,0x55,0x22,0x50}, {0x00,0x05,0x03,0x00,0x00}, // $%&'
    {0x00,0x1C,0x22,0x41,0x00}, {0x00,0x41,0x22,0x1C,0x00}, {0x08,0x2A,0x1C,0x2A,0x08}, {0x08,0x08,0x3E,0x08,0x08}, // ()*+
    {0x00,0x50,0x30,0x00,0x00}, {0x08,0x08,0x08,0x08,0x08}, {0x00,0x60,0x60,0x00,0x00}, {0x20,0x10,0x08,0x04,0x02}, // ,-./
    {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00}, {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4B,0x31}, // 0123
    {0x18,0x14,0x12,0x7F,0x10}, {0x27,0x45,0x45,0x45,0x39}, {0x3C,0x4A,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03}, // 4567
    {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1E}, {0x00,0x36,0x36,0x00,0x00}, {0x00,0x56,0x36,0x00,0x00}, // 89:;
    {0x00,0x08,0x14,0x22,0x41}, {0x14,0x14,0x14,0x14,0x14}, {0x41,0x22,0x14,0x08,0x00}, {0x02,0x01,0x51,0x09,0x06}, // <=>?
    {0x32,0x49,0x79,0x41,0x3E}, {0x7E,0x11,0x11,0x11,0x7E}, {0x7F,0x49,0x49,0x49,0x36}, {0x3E,0x41,0x41,0x41,0x22}, // @ABC
    {0x7F,0x41,0x41,0x22,0x1C}, {0x7F,0x49,0x49,0x49,0x41}, {0x7F,0x09,0x09,0x01,0x01}, {0x3E,0x41,0x41,0x51,0x32}, // DEFG
    {0x7F,0x08,0x08,0x08,0x7F}, {0x00,0x41,0x7F,0x41,0x00}, {0x20,0x40,0x41,0x3F,0x01}, {0x7F,0x08,0x14,0x22,0x41}, // HIJK
    {0x7F,0x40,0x40,0x40,0x40}, {0x7F,0x02,0x04,0x02,0x7F}, {0x7F,0x04,0x08,0x10,0x7F}, {0x3E,0x41,0x41,0x41,0x3E}, // LMNO
    {0x7F,0x09,0x09,0x09,0x06}, {0x3E,0x41,0x51,0x21,0x5E}, {0x7F,0x09,0x19,0x29,0x46}, {0x46,0x49,0x49,0x49,0x31}, // PQRS
    {0x01,0x01,0x7F,0x01,0x01}, {0x3F,0x40,0x40,0x40,0x3F}, {0x1F,0x20,0x40,0x20,0x1F}, {0x7F,0x20,0x18,0x20,0x7F}, // TUVW
    {0x63,0x14,0x08,0x14,0x63}, {0x03,0x04,0x78,0x04,0x03}, {0x61,0x51,0x49,0x45,0x43},                              // XYZ
};

#define CHAR_W  (6 * FONT_SCALE)    // 5 Pixel + 1 Pixel Abstand
#define CHAR_H  (8 * FONT_SCALE)

// Zeichnet ein Zeichen weiss auf schwarz (Kleinbuchstaben werden als Grossbuchstaben dargestellt)
static void lcd_char(int x, int y, char c)
{
    static uint8_t buf[CHAR_W * CHAR_H * 3];
    c = toupper((unsigned char)c);
    if (c < ' ' || c > 'Z') c = ' ';
    const uint8_t *glyph = font5x7[c - ' '];

    for (int py = 0; py < CHAR_H; py++) {
        for (int px = 0; px < CHAR_W; px++) {
            int col = px / FONT_SCALE, row = py / FONT_SCALE;
            uint8_t on = (col < 5) && (glyph[col] & (1 << row));
            memset(&buf[(py * CHAR_W + px) * 3], on ? 0xFC : 0x00, 3);
        }
    }
    lcd_set_window(x, y, CHAR_W, CHAR_H);
    lcd_data(buf, sizeof(buf));
}

static void lcd_text(int x, int y, const char *s)
{
    while (*s) {
        lcd_char(x, y, *s++);
        x += CHAR_W;
    }
}

// Wandelt die untersten 'bits' Bits von value in einen String aus '0' und '1' um
static void to_binary(uint16_t value, int bits, char *out)
{
    for (int i = 0; i < bits; i++) {
        out[i] = (value & (1 << (bits - 1 - i))) ? '1' : '0';
    }
    out[bits] = '\0';
}

// ---------------- Paket decodieren ----------------
static uint16_t from_gray(uint16_t gray)
{
    uint16_t value = gray;
    while ((gray >>= 1) != 0) value ^= gray;
    return value;
}

static uint16_t checksum(uint16_t value12)
{
    return (((value12 >> 8) & 0xF) + ((value12 >> 4) & 0xF) + (value12 & 0xF)) & 0xF;
}

// Jede Zeile vollstaendig ueberschreiben, damit alte Zeichen verschwinden.
static void lcd_line(int y, const char *text)
{
    char padded[26];
    snprintf(padded, sizeof(padded), "%-25.25s", text);
    lcd_text(10, y, padded);
}

static void receiver_task(void *arg)
{
    (void)arg;
    received_packet_t packet;
    uint32_t good = 0, bad = 0;
    int64_t last_good = 0;
    char line[40], bits[13];
    lcd_line(10, "I2C -> QAM EMPFAENGER");
    lcd_line(35, "SLAVE 0X42 / 100 KHZ");
    lcd_line(70, "WARTE AUF SENDER");
    lcd_line(120, "T = -- C");
    lcd_line(190, "PAKET IN GRAY CODE:");
    lcd_line(335, "LETZTER GUELTIGER WERT");

    while (1) {
        if (xQueueReceive(receive_queue, &packet, pdMS_TO_TICKS(250)) == pdTRUE) {
            const char *error = NULL;
            uint16_t fields[3] = {0};
            uint16_t value = 0;
            int16_t signed_value = 0;
            if (packet.length != PACKET_BYTES) {
                error = "FEHLER: PAKETLAENGE";
            } else {
                for (int i = 0; i < 3; i++) {
                    fields[i] = ((uint16_t)packet.bytes[2*i] << 8) | packet.bytes[2*i+1];
                    to_binary(fields[i], 12, bits);
                    snprintf(line, sizeof(line), "%s %s", i == 0 ? "START" : i == 1 ? "DATA " : "CHECK", bits);
                    lcd_line(215 + 25*i, line);
                }
                snprintf(line, sizeof(line), "HEX %03X/%03X/%03X", fields[0], fields[1], fields[2]);
                lcd_line(300, line);
                value = from_gray(fields[1]);
                signed_value = (value & 0x800) ? (int16_t)((int)value - 4096) : (int16_t)value;
                if ((fields[0] | fields[1] | fields[2]) & 0xF000) error = "FEHLER: MEHR ALS 12 BIT";
                else if (from_gray(fields[0]) != PACKET_START) error = "FEHLER: STARTSEQUENZ";
                else if (from_gray(fields[2]) != checksum(value)) error = "FEHLER: CHECKSUMME";
                // 12-Bit-Normalmodus wie im Sender: -55 bis +127.9375 C.
                else if (signed_value < -880 || signed_value > 2047) error = "FEHLER: TEMPERATUR";
            }
            if (error) {
                bad++;
                lcd_line(70, error);
                printf("%s (Laenge: %lu)\n", error, (unsigned long)packet.length);
            } else {
                good++;
                last_good = esp_timer_get_time();
                float temperature = signed_value * 0.0625f;
                lcd_line(70, "EMPFANG OK");
                snprintf(line, sizeof(line), "T = %.4f C", temperature);
                lcd_line(120, line);
                to_binary(value, 12, bits);
                snprintf(line, sizeof(line), "BIN %s", bits);
                lcd_line(145, line);
                printf("Temp: %.4f C  Paket (Gray): %03X/%03X/%03X  OK\n",
                       temperature, fields[0], fields[1], fields[2]);
            }
            snprintf(line, sizeof(line), "OK:%lu FEHLER:%lu", (unsigned long)good, (unsigned long)bad);
            lcd_line(365, line);
            snprintf(line, sizeof(line), "VERWORFEN:%lu", (unsigned long)dropped_packets);
            lcd_line(390, line);
        } else if (last_good && esp_timer_get_time() - last_good > 3500000) {
            lcd_line(70, "KEIN GUELTIGES PAKET");
        }
    }
}

void app_main(void)
{
    lcd_init();
    lcd_clear();
    receiver_init();
    printf("I2C Empfaenger bereit: SDA=%d SCL=%d Adresse=0x%02X\n",
           PIN_I2C_SDA, PIN_I2C_SCL, RECEIVER_ADDR);
    if (xTaskCreate(receiver_task, "receiver", 4096, NULL, 5, NULL) != pdPASS) {
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }
}