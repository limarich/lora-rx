#ifndef RM95_DRV_H
#define RM95_DRV_H

#include <stdbool.h>
#include <stdint.h>
#include "hardware/spi.h"
#include "lora_registers.h"

#define LORA_SPI_PORT spi0
#define LORA_PIN_SCK 18
#define LORA_PIN_MOSI 19
#define LORA_PIN_MISO 16
#define LORA_PIN_NSS 17 // CS
#define LORA_PIN_RST 20 // RESET
#define LORA_PIN_DIO0 8 // IRQ TxDone/RxDone

typedef struct
{
    int16_t rssi_dbm; // RSSI estimado (dBm)
    float snr_db;     // SNR (dB)
    uint8_t len;      // tamanho do payload
} lora_rx_info_t;

bool lora_hw_init(void);
bool lora_init_915(uint8_t bw_bits, uint8_t sf_bits, uint8_t cr_bits,
                   bool crc_on, uint8_t power_cfg, uint16_t preamble, uint8_t sync_word);
bool lora_send_blocking(const uint8_t *data, uint8_t len, uint32_t timeout_ms);
bool lora_receive_single(uint8_t *buf, uint8_t max_len, lora_rx_info_t *info, uint32_t timeout_ms);
bool lora_set_frequency_hz(uint32_t freq_hz);
void lora_sleep(void);
void lora_standby(void);

#endif
