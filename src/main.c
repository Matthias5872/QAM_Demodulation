/**********************************************************************************************
 * EduBoard2 ESP32-S3 - QAM-4 Demodulator, erster ADC/QAM Versuch
 *
 * Erwarteter Sender:
 *   TMP112 -> START/DATA/CHECK (je 12 Bit, Gray Code)
 *   -> 36 Bit -> QAM-4 (2 Bit/Symbol)
 *   -> MCP4802 DAC -> Leitung -> ADC dieses Boards
 *
 * WICHTIG: Sender und Empfaenger muessen dieselben QAM-Parameter verwenden:
 *   Sample Rate:       ca. 16.67 kS/s
 *   Traeger:           ca. 260.4 Hz
 *   Samples/Periode:   64
 *   Perioden/Symbol:   4
 *   Samples/Symbol:    256
 *   Symbolrate:        ca. 65.1 Sym/s
 *   Bitrate QAM-4:     ca. 130.2 bit/s
 *
 * Modulations-Mapping:
 *   00 -> I=-1, Q=-1
 *   01 -> I=-1, Q=+1
 *   10 -> I=+1, Q=-1
 *   11 -> I=+1, Q=+1
 *
 * Preamble vor jedem Paket (8 Symbole):
 *   00, 11, 00, 11, 00, 11, 00, 11
 *
 * Danach folgen 18 QAM-4-Symbole = 36 Bit:
 *   START[11:0] | DATA[11:0] | CHECK[11:0]
 *
 * Dieser Zwischenstand ersetzt den bisherigen I2C-Transport erstmals durch den ADC.
 * Die QAM-Demodulation ist noch absichtlich einfach:
 *   ADC -> DC-Offset abziehen -> Samples puffern -> I/Q-Korrelation
 *       -> Symbolentscheidung -> Gray decode -> Paketpruefung
 *
 * Noch NICHT enthalten:
 *   - genaue Symbol-Timing-Suche
 *   - Phasenkorrektur mit der Preamble
 *
 * Dieser Stand ist als erster Integrationsversuch gedacht.
 *
 * ADC-EINGANG:
 *   Standard hier: ADC1 Channel 3 = GPIO4 beim ESP32-S3.
 *   Vor dem Verdrahten pruefen, ob GPIO4 auf eurem EduBoard2 frei/zugaenglich ist.
 *   Falls ihr einen anderen ADC-Pin benutzt, nur ADC_CHANNEL unten anpassen.
 *
 * Verdrahtung fuer ersten Test:
 *   Sender DAC OUT  -> Empfaenger ADC IN
 *   Sender GND      -> Empfaenger GND
 **********************************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "esp_err.h"
#include "esp_timer.h"
#include "esp_adc/adc_continuous.h"
#include "hal/adc_types.h"
#include "soc/soc_caps.h"

// ==============================================================================================
// QAM / ADC Parameter
// ==============================================================================================

#define SAMPLE_RATE_HZ          16667
#define SAMPLES_PER_CARRIER     64
#define CARRIER_HZ              (SAMPLE_RATE_HZ / SAMPLES_PER_CARRIER) // ca. 260 Hz
#define CARRIER_CYCLES_SYMBOL   4
#define SAMPLES_PER_SYMBOL      (SAMPLES_PER_CARRIER * CARRIER_CYCLES_SYMBOL) // 256

#define PREAMBLE_SYMBOLS        8
#define PAYLOAD_BITS            36
#define PAYLOAD_SYMBOLS         (PAYLOAD_BITS / 2)                 // 18

// Genuegend Reserve, weil der Trigger irgendwo im ersten Preamble-Symbol ausloesen kann.
#define CAPTURE_SYMBOLS         (PREAMBLE_SYMBOLS + PAYLOAD_SYMBOLS + 2)
#define CAPTURE_SAMPLES         (CAPTURE_SYMBOLS * SAMPLES_PER_SYMBOL)

// Preamble: 00,11,00,11,...
// Wir synchronisieren ab dem zweiten Preamble-Symbol (=11).
#define TIMING_SCORE_SYMBOLS    6

#define PACKET_START            0xA5A

// ADC1 Channel 3 entspricht beim ESP32-S3 GPIO4.
// Falls ein anderer analoger Eingang benutzt wird: hier anpassen.
#define QAM_ADC_UNIT            ADC_UNIT_1
#define QAM_ADC_CHANNEL         ADC_CHANNEL_3
#define QAM_ADC_ATTEN           ADC_ATTEN_DB_12
#define QAM_ADC_BITWIDTH        ADC_BITWIDTH_12

// Startdetektion. Fuer direkte DAC->ADC Leitung ist 35 ADC Counts ein guter Startwert.
// Falls kein Trigger kommt: kleiner machen. Falls Fehltrigger: groesser machen.
#define CARRIER_TRIGGER_LEVEL   35.0f

#define ADC_DMA_FRAME_BYTES     1024

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ==============================================================================================
// Display Pins
// ==============================================================================================

#define PIN_LCD_MOSI    13
#define PIN_LCD_SCK     14
#define PIN_LCD_CS      11
#define PIN_LCD_DC      16
#define PIN_LCD_RESET   8

#define LCD_WIDTH       320
#define LCD_HEIGHT      480
#define FONT_SCALE      2

static spi_device_handle_t lcd;

// ==============================================================================================
// QAM globale Daten
// ==============================================================================================

static adc_continuous_handle_t adc_handle;

static float sin_table[SAMPLES_PER_CARRIER];
static float cos_table[SAMPLES_PER_CARRIER];

// Gespeichert werden bereits um den DC-Arbeitspunkt zentrierte ADC-Werte.
static int16_t capture_buffer[CAPTURE_SAMPLES];

static float dc_estimate = 0.0f;
static float envelope_estimate = 0.0f;

static uint32_t good_packets = 0;
static uint32_t bad_packets = 0;

// ==============================================================================================
// Display (ILI9488 ueber SPI) - aus dem bisherigen Empfaenger uebernommen
// ==============================================================================================

static void lcd_cmd(uint8_t cmd)
{
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd
    };
    gpio_set_level(PIN_LCD_DC, 0);
    ESP_ERROR_CHECK(spi_device_polling_transmit(lcd, &t));
}

static void lcd_data(const uint8_t *data, int len)
{
    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = data
    };
    gpio_set_level(PIN_LCD_DC, 1);
    ESP_ERROR_CHECK(spi_device_polling_transmit(lcd, &t));
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

    gpio_set_level(PIN_LCD_RESET, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(PIN_LCD_RESET, 1);
    vTaskDelay(pdMS_TO_TICKS(150));

    lcd_cmd(0x3A);
    lcd_data((uint8_t[]){0x66}, 1);

    lcd_cmd(0x36);
    lcd_data((uint8_t[]){0x48}, 1);

    lcd_cmd(0x11);
    vTaskDelay(pdMS_TO_TICKS(120));

    lcd_cmd(0x29);
}

static void lcd_set_window(int x, int y, int w, int h)
{
    int x2 = x + w - 1;
    int y2 = y + h - 1;

    lcd_cmd(0x2A);
    lcd_data((uint8_t[]){
        (uint8_t)(x >> 8), (uint8_t)(x & 0xFF),
        (uint8_t)(x2 >> 8), (uint8_t)(x2 & 0xFF)
    }, 4);

    lcd_cmd(0x2B);
    lcd_data((uint8_t[]){
        (uint8_t)(y >> 8), (uint8_t)(y & 0xFF),
        (uint8_t)(y2 >> 8), (uint8_t)(y2 & 0xFF)
    }, 4);

    lcd_cmd(0x2C);
}

static void lcd_clear(void)
{
    static uint8_t line[LCD_WIDTH * 3];
    memset(line, 0, sizeof(line));

    lcd_set_window(0, 0, LCD_WIDTH, LCD_HEIGHT);

    for (int y = 0; y < LCD_HEIGHT; y++) {
        lcd_data(line, sizeof(line));
    }
}

// 5x7 Font fuer ASCII ' ' bis 'Z'
static const uint8_t font5x7[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, {0x00,0x00,0x5F,0x00,0x00}, {0x00,0x07,0x00,0x07,0x00}, {0x14,0x7F,0x14,0x7F,0x14},
    {0x24,0x2A,0x7F,0x2A,0x12}, {0x23,0x13,0x08,0x64,0x62}, {0x36,0x49,0x55,0x22,0x50}, {0x00,0x05,0x03,0x00,0x00},
    {0x00,0x1C,0x22,0x41,0x00}, {0x00,0x41,0x22,0x1C,0x00}, {0x08,0x2A,0x1C,0x2A,0x08}, {0x08,0x08,0x3E,0x08,0x08},
    {0x00,0x50,0x30,0x00,0x00}, {0x08,0x08,0x08,0x08,0x08}, {0x00,0x60,0x60,0x00,0x00}, {0x20,0x10,0x08,0x04,0x02},
    {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00}, {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4B,0x31},
    {0x18,0x14,0x12,0x7F,0x10}, {0x27,0x45,0x45,0x45,0x39}, {0x3C,0x4A,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1E}, {0x00,0x36,0x36,0x00,0x00}, {0x00,0x56,0x36,0x00,0x00},
    {0x00,0x08,0x14,0x22,0x41}, {0x14,0x14,0x14,0x14,0x14}, {0x41,0x22,0x14,0x08,0x00}, {0x02,0x01,0x51,0x09,0x06},
    {0x32,0x49,0x79,0x41,0x3E}, {0x7E,0x11,0x11,0x11,0x7E}, {0x7F,0x49,0x49,0x49,0x36}, {0x3E,0x41,0x41,0x41,0x22},
    {0x7F,0x41,0x41,0x22,0x1C}, {0x7F,0x49,0x49,0x49,0x41}, {0x7F,0x09,0x09,0x01,0x01}, {0x3E,0x41,0x41,0x51,0x32},
    {0x7F,0x08,0x08,0x08,0x7F}, {0x00,0x41,0x7F,0x41,0x00}, {0x20,0x40,0x41,0x3F,0x01}, {0x7F,0x08,0x14,0x22,0x41},
    {0x7F,0x40,0x40,0x40,0x40}, {0x7F,0x02,0x04,0x02,0x7F}, {0x7F,0x04,0x08,0x10,0x7F}, {0x3E,0x41,0x41,0x41,0x3E},
    {0x7F,0x09,0x09,0x09,0x06}, {0x3E,0x41,0x51,0x21,0x5E}, {0x7F,0x09,0x19,0x29,0x46}, {0x46,0x49,0x49,0x49,0x31},
    {0x01,0x01,0x7F,0x01,0x01}, {0x3F,0x40,0x40,0x40,0x3F}, {0x1F,0x20,0x40,0x20,0x1F}, {0x7F,0x20,0x18,0x20,0x7F},
    {0x63,0x14,0x08,0x14,0x63}, {0x03,0x04,0x78,0x04,0x03}, {0x61,0x51,0x49,0x45,0x43},
};

#define CHAR_W  (6 * FONT_SCALE)
#define CHAR_H  (8 * FONT_SCALE)

static void lcd_char(int x, int y, char c)
{
    static uint8_t buf[CHAR_W * CHAR_H * 3];

    c = toupper((unsigned char)c);
    if (c < ' ' || c > 'Z') {
        c = ' ';
    }

    const uint8_t *glyph = font5x7[c - ' '];

    for (int py = 0; py < CHAR_H; py++) {
        for (int px = 0; px < CHAR_W; px++) {
            int col = px / FONT_SCALE;
            int row = py / FONT_SCALE;
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

static void lcd_line(int y, const char *text)
{
    char padded[26];
    snprintf(padded, sizeof(padded), "%-25.25s", text);
    lcd_text(10, y, padded);
}

static void to_binary(uint16_t value, int bits, char *out)
{
    for (int i = 0; i < bits; i++) {
        out[i] = (value & (1U << (bits - 1 - i))) ? '1' : '0';
    }
    out[bits] = '\0';
}

// ==============================================================================================
// Paketfunktionen - passend zum bisherigen Sender
// ==============================================================================================

static uint16_t from_gray(uint16_t gray)
{
    uint16_t value = gray;

    while ((gray >>= 1) != 0) {
        value ^= gray;
    }

    return value;
}

static uint16_t checksum(uint16_t value12)
{
    return (((value12 >> 8) & 0xF)
          + ((value12 >> 4) & 0xF)
          + (value12 & 0xF)) & 0xF;
}

// ==============================================================================================
// QAM Mathematik
// ==============================================================================================

typedef struct {
    float i;
    float q;
} iq_t;


// Sinus/Cosinus Tabellen fuer eine Traegerperiode.
// Erstmal bewusst einfach gehalten.
static void qam_tables_init(void)
{
    for (int n = 0; n < SAMPLES_PER_CARRIER; n++) {
        float phase =
            2.0f * (float)M_PI *
            (float)n /
            (float)SAMPLES_PER_CARRIER;

        sin_table[n] = sinf(phase);
        cos_table[n] = cosf(phase);
    }
}


// Ein Symbol wird mit lokalem Sinus und Cosinus korreliert.
// In diesem Zwischenstand nehmen wir noch an, dass das Symbolfenster
// schon einigermassen richtig liegt und keine zusaetzliche Phase
// korrigiert werden muss.
static iq_t correlate_symbol_basic(const int16_t *samples, int start)
{
    float i_sum = 0.0f;
    float q_sum = 0.0f;

    for (int n = 0; n < SAMPLES_PER_SYMBOL; n++) {
        float x = (float)samples[start + n];

        int p = n % SAMPLES_PER_CARRIER;

        i_sum += x * cos_table[p];
        q_sum += x * sin_table[p];
    }

    iq_t result;
    result.i = i_sum / (float)SAMPLES_PER_SYMBOL;
    result.q = q_sum / (float)SAMPLES_PER_SYMBOL;

    return result;
}


// QAM-4 Entscheidung nur anhand der Vorzeichen.
// 00 -> -I -Q
// 01 -> -I +Q
// 10 -> +I -Q
// 11 -> +I +Q
static uint8_t qam4_decide_basic(iq_t z)
{
    uint8_t symbol = 0;

    if (z.i >= 0.0f) {
        symbol |= 0x2;
    }

    if (z.q >= 0.0f) {
        symbol |= 0x1;
    }

    return symbol;
}


// ==============================================================================================
// Paket aus 18 QAM-4-Symbolen rekonstruieren
// ==============================================================================================

static bool process_capture(const int16_t *samples)
{
    char line[40];
    char bits[13];

    // Erster Versuch:
    // Wir nehmen an, dass der Trigger ungefaehr am Beginn der Preamble liegt.
    // Darum wird einfach nach der kompletten Preamble mit den Nutzdaten begonnen.
    //
    // Spaeter wird hier eine echte Timing-/Phasensynchronisation ergaenzt.
    int payload_start = PREAMBLE_SYMBOLS * SAMPLES_PER_SYMBOL;

    if (payload_start + PAYLOAD_SYMBOLS * SAMPLES_PER_SYMBOL > CAPTURE_SAMPLES) {
        bad_packets++;
        lcd_line(70, "FEHLER: CAPTURE ZU KURZ");
        return false;
    }

    uint64_t frame_bits = 0;

    for (int s = 0; s < PAYLOAD_SYMBOLS; s++) {

        int start =
            payload_start +
            s * SAMPLES_PER_SYMBOL;

        iq_t iq =
            correlate_symbol_basic(samples, start);

        uint8_t symbol =
            qam4_decide_basic(iq);

        frame_bits =
            (frame_bits << 2) |
            symbol;

        printf(
            "SYM %02d: I=%8.2f Q=%8.2f -> %u%u\n",
            s,
            iq.i,
            iq.q,
            (symbol >> 1) & 1,
            symbol & 1
        );
    }

    uint16_t start_gray =
        (uint16_t)((frame_bits >> 24) & 0x0FFF);

    uint16_t data_gray =
        (uint16_t)((frame_bits >> 12) & 0x0FFF);

    uint16_t check_gray =
        (uint16_t)(frame_bits & 0x0FFF);

    uint16_t start_value =
        from_gray(start_gray);

    uint16_t data_value =
        from_gray(data_gray);

    uint16_t check_value =
        from_gray(check_gray);

    int16_t signed_value =
        (data_value & 0x800)
        ? (int16_t)((int)data_value - 4096)
        : (int16_t)data_value;

    lcd_line(70, "QAM BLOCK AUSGEWERTET");
    lcd_line(95, "NOCH OHNE SYNC");

    lcd_line(140, "QAM PAKET IN GRAY:");

    to_binary(start_gray, 12, bits);
    snprintf(line, sizeof(line), "START %s", bits);
    lcd_line(165, line);

    to_binary(data_gray, 12, bits);
    snprintf(line, sizeof(line), "DATA  %s", bits);
    lcd_line(190, line);

    to_binary(check_gray, 12, bits);
    snprintf(line, sizeof(line), "CHECK %s", bits);
    lcd_line(215, line);

    snprintf(
        line,
        sizeof(line),
        "HEX %03X/%03X/%03X",
        start_gray,
        data_gray,
        check_gray
    );

    lcd_line(240, line);

    const char *error = NULL;

    if (start_value != PACKET_START) {
        error = "FEHLER: STARTSEQUENZ";
    }
    else if (check_value != checksum(data_value)) {
        error = "FEHLER: CHECKSUMME";
    }
    else if (signed_value < -880 || signed_value > 2047) {
        error = "FEHLER: TEMPERATUR";
    }

    if (error) {
        bad_packets++;
        lcd_line(285, error);

        printf(
            "%s START=%03X DATA=%03X CHECK=%03X\n",
            error,
            start_value,
            data_value,
            check_value
        );

        return false;
    }

    good_packets++;

    float temperature =
        signed_value * 0.0625f;

    lcd_line(285, "EMPFANG OK");

    snprintf(
        line,
        sizeof(line),
        "T = %.4F C",
        temperature
    );

    lcd_line(310, line);

    snprintf(
        line,
        sizeof(line),
        "OK:%lu FEHLER:%lu",
        (unsigned long)good_packets,
        (unsigned long)bad_packets
    );

    lcd_line(350, line);

    printf(
        "QAM Paket OK: Gray %03X/%03X/%03X Temp %.4f C\n",
        start_gray,
        data_gray,
        check_gray,
        temperature
    );

    return true;
}


// ==============================================================================================
// ADC Continuous / DMA
// ==============================================================================================

static void adc_init(void)
{
    adc_continuous_handle_cfg_t handle_cfg = {
        .max_store_buf_size = 4096,
        .conv_frame_size = ADC_DMA_FRAME_BYTES,
    };

    ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_cfg, &adc_handle));

    adc_digi_pattern_config_t pattern = {
        .atten = QAM_ADC_ATTEN,
        .channel = QAM_ADC_CHANNEL,
        .unit = QAM_ADC_UNIT,
        .bit_width = QAM_ADC_BITWIDTH,
    };

    adc_continuous_config_t adc_cfg = {
        .sample_freq_hz = SAMPLE_RATE_HZ,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
        .pattern_num = 1,
        .adc_pattern = &pattern,
    };

    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &adc_cfg));
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));
}

// ==============================================================================================
// Empfangstask
// ==============================================================================================

typedef enum {
    RX_WAIT_FOR_SIGNAL = 0,
    RX_CAPTURE_FRAME
} rx_state_t;

static void qam_receiver_task(void *arg)
{
    (void)arg;

    uint8_t adc_dma_buffer[ADC_DMA_FRAME_BYTES];
    uint32_t bytes_read = 0;

    rx_state_t state = RX_WAIT_FOR_SIGNAL;

    int capture_index = 0;
    int trigger_counter = 0;

    // Startwert ungefaehr Mitte des 12-Bit ADC.
    // In WAIT_FOR_SIGNAL lernt der Empfaenger den tatsaechlichen Arbeitspunkt.
    dc_estimate = 2048.0f;

    lcd_line(10, "QAM-4 ADC TEST");
    lcd_line(35, "16.7KS/S FC~260HZ");
    lcd_line(70, "WARTE AUF QAM SIGNAL");
    lcd_line(95, "ADC1 CH3 / GPIO4");
    lcd_line(140, "BASIC I/Q - NO SYNC");

    printf("QAM-4 ADC Zwischenstand gestartet\n");
    printf("fs=%d S/s, fc=%d Hz, %d Samples/Symbol, %d Sym/s, %d bit/s\n",
           SAMPLE_RATE_HZ,
           CARRIER_HZ,
           SAMPLES_PER_SYMBOL,
           SAMPLE_RATE_HZ / SAMPLES_PER_SYMBOL,
           2 * SAMPLE_RATE_HZ / SAMPLES_PER_SYMBOL);

    while (1) {
        esp_err_t ret = adc_continuous_read(
            adc_handle,
            adc_dma_buffer,
            sizeof(adc_dma_buffer),
            &bytes_read,
            1000
        );

        if (ret == ESP_ERR_TIMEOUT) {
            continue;
        }

        ESP_ERROR_CHECK(ret);

        for (uint32_t pos = 0;
             pos + SOC_ADC_DIGI_RESULT_BYTES <= bytes_read;
             pos += SOC_ADC_DIGI_RESULT_BYTES) {

            adc_digi_output_data_t *sample =
                (adc_digi_output_data_t *)&adc_dma_buffer[pos];

            if (sample->type2.unit != QAM_ADC_UNIT ||
                sample->type2.channel != QAM_ADC_CHANNEL) {
                continue;
            }

            int raw = sample->type2.data;

            if (state == RX_WAIT_FOR_SIGNAL) {

                // Langsame DC-Nachfuehrung nur im Idle.
                dc_estimate += 0.001f * ((float)raw - dc_estimate);

                float centered = (float)raw - dc_estimate;
                float abs_value = fabsf(centered);

                // Huelle fuer robuste Startdetektion.
                envelope_estimate =
                    0.98f * envelope_estimate +
                    0.02f * abs_value;

                if (envelope_estimate > CARRIER_TRIGGER_LEVEL) {
                    trigger_counter++;
                } else {
                    trigger_counter = 0;
                }

                // Kurze Bestaetigung gegen Einzelspikes.
                if (trigger_counter >= 8) {
                    state = RX_CAPTURE_FRAME;
                    capture_index = 0;
                    trigger_counter = 0;

                    capture_buffer[capture_index++] =
                        (int16_t)centered;

                    lcd_line(70, "QAM SIGNAL ERKANNT");
                }
            }
            else {
                float centered = (float)raw - dc_estimate;

                if (capture_index < CAPTURE_SAMPLES) {
                    capture_buffer[capture_index++] =
                        (int16_t)centered;
                }

                if (capture_index >= CAPTURE_SAMPLES) {
                    bool ok = process_capture(capture_buffer);

                    if (!ok) {
                        char line[32];
                        snprintf(line, sizeof(line), "OK:%lu FEHLER:%lu",
                                 (unsigned long)good_packets,
                                 (unsigned long)bad_packets);
                        lcd_line(350, line);
                    }

                    // Zurueck in Idle. Sender sollte zwischen Frames
                    // eine kurze Pause ohne Traeger senden.
                    state = RX_WAIT_FOR_SIGNAL;
                    capture_index = 0;
                    envelope_estimate = 0.0f;
                    trigger_counter = 0;

                    lcd_line(410, "WARTE AUF NAECHSTES PAKET");
                }
            }
        }
    }
}

// ==============================================================================================
// Main
// ==============================================================================================

void app_main(void)
{
    lcd_init();
    lcd_clear();

    qam_tables_init();
    adc_init();

    if (xTaskCreate(
            qam_receiver_task,
            "qam_receiver",
            8192,
            NULL,
            6,
            NULL) != pdPASS) {
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }
}
