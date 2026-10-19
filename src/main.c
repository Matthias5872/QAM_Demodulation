/********************************************************************************************* */
// Eduboard2 ESP32-S3 - erster Empfaengertest ueber I2C
//
// Ziel fuer diesen Stand:
// - 6 Byte vom Sender empfangen
// - Paket in drei 12-Bit Felder zerlegen
// - Werte erstmal nur ueber die serielle Konsole ausgeben
//
// TODO:
// - Gray Code wieder zurueckwandeln
// - Startsequenz / Checksumme pruefen
// - Temperatur anzeigen
/********************************************************************************************* */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/i2c_slave.h"

#define PIN_I2C_SDA     2
#define PIN_I2C_SCL     1
#define RECEIVER_ADDR   0x42
#define PACKET_BYTES    6

static i2c_slave_dev_handle_t receiver;
static QueueHandle_t receive_queue;

typedef struct
{
    uint8_t bytes[PACKET_BYTES];
    uint32_t length;
} received_packet_t;


// Callback vom I2C Slave
static bool receive_done(i2c_slave_dev_handle_t handle,
                         const i2c_slave_rx_done_event_data_t *event,
                         void *arg)
{
    (void)handle;
    (void)arg;

    received_packet_t packet = {0};
    packet.length = event->length;

    // fuer den Moment interessieren nur komplette Pakete
    if(event->length == PACKET_BYTES)
    {
        memcpy(packet.bytes, event->buffer, PACKET_BYTES);
    }

    BaseType_t wake = pdFALSE;
    xQueueSendFromISR(receive_queue, &packet, &wake);

    return wake == pdTRUE;
}


static void receiver_init(void)
{
    receive_queue = xQueueCreate(5, sizeof(received_packet_t));
    ESP_ERROR_CHECK(receive_queue != NULL ? ESP_OK : ESP_ERR_NO_MEM);

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


static void receiver_task(void *arg)
{
    (void)arg;

    received_packet_t packet;

    while(1)
    {
        if(xQueueReceive(receive_queue, &packet, portMAX_DELAY) == pdTRUE)
        {
            if(packet.length != PACKET_BYTES)
            {
                printf("Falsche Paketlaenge: %lu\n",
                       (unsigned long)packet.length);
                continue;
            }

            uint16_t start =
                ((uint16_t)packet.bytes[0] << 8) |
                packet.bytes[1];

            uint16_t data =
                ((uint16_t)packet.bytes[2] << 8) |
                packet.bytes[3];

            uint16_t check =
                ((uint16_t)packet.bytes[4] << 8) |
                packet.bytes[5];

            printf("Paket empfangen: %03X / %03X / %03X\n",
                   start, data, check);

            // hier kommt als naechstes die eigentliche Decodierung rein
        }
    }
}


void app_main(void)
{
    receiver_init();

    printf("I2C Empfaenger gestartet\n");
    printf("Adresse 0x%02X, SDA=%d, SCL=%d\n",
           RECEIVER_ADDR,
           PIN_I2C_SDA,
           PIN_I2C_SCL);

    BaseType_t task_created = xTaskCreate(
        receiver_task,
        "receiver",
        3072,
        NULL,
        5,
        NULL
    );
    ESP_ERROR_CHECK(task_created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
}
