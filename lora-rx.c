#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "rm95.h"

#define LED_RED 13
#define LED_BLUE 12
#define LED_GREEN 11

static void led_setup(uint pin)
{
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, 0);
}

int main()
{
    stdio_init_all();

    // LEDs
    led_setup(LED_RED);
    led_setup(LED_BLUE);
    led_setup(LED_GREEN);

    // --- LoRa: init de HW + modem ---
    if (!lora_hw_init())
    {
        gpio_put(LED_RED, 1);
        printf("Falha HW LoRa\n");
        while (1)
            sleep_ms(1000);
    }
    // (915 MHz, BW125k, SF7, CR4/5, CRC on, preâmbulo 8, sync 0x34)
    if (!lora_init_915(BANDWIDTH_125K, SPREADING_7, ERROR_CODING_4_5,
                       true, PA_MED_BOOST, 8, 0x34))
    {
        gpio_put(LED_RED, 1);
        gpio_put(LED_BLUE, 1);
        printf("Falha init LoRa\n");
        while (1)
            sleep_ms(1000);
    }

    printf("LoRa RX pronto. Aguardando pacotes...\n");

    uint8_t buf[PAYLOAD_LENGTH + 1]; // +1 para terminador '\0'
    lora_rx_info_t info;

    while (true)
    {
        // pisca “ouvindo”
        gpio_put(LED_GREEN, 1);

        // RX “single shot” com timeout (ms). Repete no loop.
        bool ok = lora_receive_single(buf, PAYLOAD_LENGTH, &info, 5000);

        gpio_put(LED_GREEN, 0);

        if (ok)
        {
            buf[info.len < PAYLOAD_LENGTH ? info.len : PAYLOAD_LENGTH] = '\0'; // segurança para print

            // Indica pacote recebido
            gpio_put(LED_BLUE, 1);

            printf("[LoRa RX] len=%u  RSSI=%d dBm  SNR=%.2f dB  |  payload=\"%s\"\n",
                   info.len, info.rssi_dbm, info.snr_db, (char *)buf);

            // apaga depois de um tempinho
            sleep_ms(100);
            gpio_put(LED_BLUE, 0);
        }
        else
        {
            // Timeout ou erro de CRC
            // (a função já checa RxDone e descarta pacotes com CRC error) :contentReference[oaicite:2]{index=2}
            printf("[LoRa RX] timeout/sem pacote válido\n");
        }

        // pequeno intervalo para não saturar o loop
        tight_loop_contents();
    }
}
