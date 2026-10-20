/********************************************************************************************* */
// Eduboard2 ESP32-S3 - QAM Paketempfaenger, I2C Transporttest
//
// Stand 2:
// - I2C Empfang funktioniert
// - START / DATA / CHECK werden aus dem Paket gelesen
// - Gray Code wird decodiert
// - Startsequenz und Checksumme werden kontrolliert
// - Temperatur wird bereits berechnet
//
// Anzeige kommt spaeter noch dazu.
/********************************************************************************************* */

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/i2c_slave.h"

#define PIN_I2C_SDA     2
#define PIN_I2C_SCL     1
#define RECEIVER_ADDR   0x42

#define PACKET_START    0xA5A
#define PACKET_BYTES    6

static i2c_slave_dev_handle_t receiver;
static QueueHandle_t receive_queue;

static volatile uint32_t dropped_packets = 0;


typedef struct
{
    uint8_t bytes[PACKET_BYTES];
    uint32_t length;
} received_packet_t;


static bool receive_done(i2c_slave_dev_handle_t handle,
                         const i2c_slave_rx_done_event_data_t *event,
                         void *arg)
{
    (void)handle;
    (void)arg;

    received_packet_t packet = {0};
    packet.length = event->length;

    if(event->length == PACKET_BYTES)
    {
        memcpy(packet.bytes, event->buffer, PACKET_BYTES);
    }

    BaseType_t wake = pdFALSE;

    if(xQueueSendFromISR(receive_queue, &packet, &wake) != pdTRUE)
    {
        dropped_packets++;
    }

    return wake == pdTRUE;
}


static void receiver_init(void)
{
    receive_queue = xQueueCreate(8, sizeof(received_packet_t));

    i2c_slave_config_t cfg =
    {
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

    i2c_slave_event_callbacks_t callbacks =
    {
        .on_receive = receive_done
    };

    ESP_ERROR_CHECK(
        i2c_slave_register_event_callbacks(receiver, &callbacks, NULL)
    );
}


// Gray -> binaer
static uint16_t from_gray(uint16_t gray)
{
    uint16_t value = gray;

    while((gray >>= 1) != 0)
    {
        value ^= gray;
    }

    return value;
}


// erstmal einfache 4 Bit Checksumme
static uint16_t checksum(uint16_t value12)
{
    uint16_t a = (value12 >> 8) & 0xF;
    uint16_t b = (value12 >> 4) & 0xF;
    uint16_t c = value12 & 0xF;

    return (a + b + c) & 0xF;
}


static void receiver_task(void *arg)
{
    (void)arg;

    received_packet_t packet;

    uint32_t good = 0;
    uint32_t bad = 0;

    while(1)
    {
        if(xQueueReceive(receive_queue, &packet, portMAX_DELAY) == pdTRUE)
        {
            if(packet.length != PACKET_BYTES)
            {
                bad++;
                printf("FEHLER Paketlaenge: %lu\n",
                       (unsigned long)packet.length);
                continue;
            }

            uint16_t fields[3];

            for(int i = 0; i < 3; i++)
            {
                fields[i] =
                    ((uint16_t)packet.bytes[2*i] << 8) |
                    packet.bytes[2*i + 1];
            }

            printf("Gray RX: %03X / %03X / %03X\n",
                   fields[0],
                   fields[1],
                   fields[2]);

            // sicherstellen, dass die oberen 4 Bit nicht gesetzt sind
            if((fields[0] | fields[1] | fields[2]) & 0xF000)
            {
                bad++;
                printf("FEHLER: mehr als 12 Bit\n");
                continue;
            }

            uint16_t start = from_gray(fields[0]);
            uint16_t value = from_gray(fields[1]);
            uint16_t check = from_gray(fields[2]);

            if(start != PACKET_START)
            {
                bad++;
                printf("FEHLER: Startsequenz %03X\n", start);
                continue;
            }

            if(check != checksum(value))
            {
                bad++;
                printf("FEHLER: Checksumme %X erwartet %X\n",
                       check,
                       checksum(value));
                continue;
            }

            int16_t signed_value;

            if(value & 0x800)
            {
                signed_value = (int16_t)((int)value - 4096);
            }
            else
            {
                signed_value = (int16_t)value;
            }

            // grobe Plausibilitaetspruefung vom TMP112
            if(signed_value < -880 || signed_value > 2047)
            {
                bad++;
                printf("FEHLER: Temperatur ausserhalb Bereich\n");
                continue;
            }

            float temperature = signed_value * 0.0625f;

            good++;

            printf("OK: T = %.4f C   OK=%lu ERR=%lu DROP=%lu\n",
                   temperature,
                   (unsigned long)good,
                   (unsigned long)bad,
                   (unsigned long)dropped_packets);
        }
    }
}


void app_main(void)
{
    receiver_init();

    printf("QAM Paketempfaenger - Transport noch ueber I2C\n");
    printf("Slave 0x%02X / 100 kHz\n", RECEIVER_ADDR);

    if(xTaskCreate(receiver_task,
                   "receiver",
                   4096,
                   NULL,
                   5,
                   NULL) != pdPASS)
    {
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }
}
