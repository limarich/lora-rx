#include "rm95.h"

#include "pico/stdlib.h"
#include "hardware/gpio.h"

// --- controle de CS ---
static inline void cs_select(void) { gpio_put(LORA_PIN_NSS, 0); }   // inicia comunicação
static inline void cs_deselect(void) { gpio_put(LORA_PIN_NSS, 1); } // finaliza comunicação

// Monta um pacote SPI com o bit mais alto 0 (indica leitura)
// Envia endereço e lê um byte de resposta do registrador
static uint8_t reg_read(uint8_t addr)
{
    uint8_t tx[2] = {(uint8_t)(addr & 0x7F), 0x00}, rx[2] = {0};
    cs_select();
    spi_write_read_blocking(LORA_SPI_PORT, tx, rx, 2);
    cs_deselect();
    return rx[1];
}
// Monta pacote SPI com o bit mais alto 1 (indica escrita)
// Envia endereço + valor para gravar
static void reg_write(uint8_t addr, uint8_t val)
{
    uint8_t tx[2] = {(uint8_t)(addr | 0x80), val};
    cs_select();
    spi_write_blocking(LORA_SPI_PORT, tx, 2);
    cs_deselect();
}
// Envia vários bytes seguidos
// Carrega FIFO de TX ou blocos de configuração
static void burst_write(uint8_t addr, const uint8_t *buf, uint8_t len)
{
    uint8_t a = addr | 0x80;
    cs_select();
    spi_write_blocking(LORA_SPI_PORT, &a, 1);
    spi_write_blocking(LORA_SPI_PORT, buf, len);
    cs_deselect();
}
// Lê vários bytes seguidos
// Lê FIFO de RX ou blocos de registradores
static void burst_read(uint8_t addr, uint8_t *buf, uint8_t len)
{
    uint8_t a = addr & 0x7F;
    cs_select();
    spi_write_blocking(LORA_SPI_PORT, &a, 1);
    spi_read_blocking(LORA_SPI_PORT, 0x00, buf, len);
    cs_deselect();
}

// --- modos ---
static void set_mode(uint8_t opmode) { reg_write(REG_OPMODE, opmode); }
void lora_sleep(void) { set_mode(RF95_MODE_SLEEP); }
void lora_standby(void) { set_mode(RF95_MODE_STANDBY); }

// programa a frequência de operação do RFM95W a partir de um valor em hertz
bool lora_set_frequency_hz(uint32_t freq_hz)
{
    uint32_t frf = (uint32_t)((((uint64_t)freq_hz) << 19) / 32000000ULL);
    reg_write(REG_FRF_MSB, (uint8_t)(frf >> 16));
    reg_write(REG_FRF_MID, (uint8_t)(frf >> 8));
    reg_write(REG_FRF_LSB, (uint8_t)(frf));
    return true;
}

//  Registradores de evento
static void map_dio0_txdone(void)
{
    uint8_t m = reg_read(REG_DIO_MAPPING_1);
    m &= ~(0xC0);
    m |= 0x40; // 01 -> TxDone
    reg_write(REG_DIO_MAPPING_1, m);
}
static void map_dio0_rxdone(void)
{
    uint8_t m = reg_read(REG_DIO_MAPPING_1);
    m &= ~(0xC0); // 00 -> RxDone
    reg_write(REG_DIO_MAPPING_1, m);
}

// IRQs
static void clear_irqs(void) { reg_write(REG_IRQ_FLAGS, 0xFF); }

// zera a FIFO
static void fifo_reset_ptrs(void)
{
    reg_write(REG_FIFO_TX_BASE_AD, 0x00);
    reg_write(REG_FIFO_RX_BASE_AD, 0x00);
    reg_write(REG_FIFO_ADDR_PTR, 0x00);
}

// inicialização LoRa
bool lora_hw_init(void)
{
    // GPIO
    gpio_init(LORA_PIN_NSS);
    gpio_set_dir(LORA_PIN_NSS, GPIO_OUT);
    gpio_put(LORA_PIN_NSS, 1);
    gpio_init(LORA_PIN_RST);
    gpio_set_dir(LORA_PIN_RST, GPIO_OUT);
    gpio_put(LORA_PIN_RST, 1);
    gpio_init(LORA_PIN_DIO0);
    gpio_set_dir(LORA_PIN_DIO0, GPIO_IN);
    gpio_pull_up(LORA_PIN_DIO0);

    // SPI
    spi_init(LORA_SPI_PORT, 8 * 1000 * 1000); // 8 MHz.
    gpio_set_function(LORA_PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(LORA_PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(LORA_PIN_MISO, GPIO_FUNC_SPI);

    // Reset do rádio
    gpio_put(LORA_PIN_RST, 0);
    sleep_us(200);
    gpio_put(LORA_PIN_RST, 1);
    sleep_ms(5);

    // Força LoRa (bit7 do OPMODE=1) e Standby
    set_mode(RF95_MODE_SLEEP); // 0x80: LoRa+Sleep
    sleep_ms(5);
    set_mode(RF95_MODE_STANDBY); // 0x81: LoRa+Standby
    sleep_ms(5);

    // checagem leve: ler REG_OPMODE e ver se bit7 (LoRa) está setado
    uint8_t om = reg_read(REG_OPMODE);
    if ((om & 0x80) == 0)
        return false;

    return true;
}

// --- configuração principal 915 MHz ---
// bw_bits: BANDWIDTH
// sf_bits: SPREADING
// cr_bits: ERROR_CODING (bits 3:1 de REG_MODEM_CONFIG)
// crc_on : true/false (bit 2 de REG_MODEM_CONFIG2)
// power_cfg: um dos PA_* do header (REG_PA_CONFIG)
// preamble: tamanho do preâmbulo
// sync_word: 0x34 (público) ou valor privado (p.ex. 0x12)
bool lora_init_915(uint8_t bw_bits, uint8_t sf_bits, uint8_t cr_bits,
                   bool crc_on, uint8_t power_cfg, uint16_t preamble, uint8_t sync_word)
{
    lora_standby();
    fifo_reset_ptrs();

    // LNA máximo ganho
    // reg_write(REG_LNA, LNA_MAX_GAIN);

    // Frequência de operação
    lora_set_frequency_hz(915000000);

    // MODEM_CONFIG (BW + CR + modo explícito)
    uint8_t mc1 = (bw_bits & 0xF0) | (cr_bits & 0x0E) | EXPLICIT_MODE;
    reg_write(REG_MODEM_CONFIG, mc1);

    // MODEM_CONFIG2 (SF + CRC + symb timeout LSB)
    uint8_t mc2 = (sf_bits & 0xF0) | (crc_on ? CRC_ON : CRC_OFF) | 0x00;
    reg_write(REG_MODEM_CONFIG2, mc2);
    reg_write(REG_PREAMBLE_MSB, (uint8_t)(preamble >> 8));
    reg_write(REG_PREAMBLE_LSB, (uint8_t)(preamble));

    // MODEM_CONFIG3: AGC on + LowDataRateOptimize para SF11/SF12 e BW 125k
    uint8_t mc3 = 0x04;
    if ((sf_bits >= SPREADING_11) && (bw_bits == BANDWIDTH_125K))
        mc3 |= 0x08;
    reg_write(REG_MODEM_CONFIG3, mc3);

    reg_write(REG_DETECT_OPT, 0x03); // DetectionOptimize
    reg_write(REG_DETECTION_THRESHOLD, 0x0A);

    // Potência no PA_BOOST
    reg_write(REG_PA_CONFIG, power_cfg);

    // Payload length default
    reg_write(REG_PAYLOAD_LENGTH, 0x40);

    // IRQs: limpando máscara
    reg_write(REG_IRQ_FLAGS_MASK, ~(0x40 | 0x08 | 0x20));

    lora_standby();
    return true;
}

// --- TX bloqueante ---
bool lora_send_blocking(const uint8_t *data, uint8_t len, uint32_t timeout_ms)
{
    if (!data || len == 0 || len > PAYLOAD_LENGTH)
        return false;

    // Aponta FIFO para base de TX e grava payload
    reg_write(REG_FIFO_ADDR_PTR, reg_read(REG_FIFO_TX_BASE_AD));
    reg_write(REG_PAYLOAD_LENGTH, len);
    burst_write(REG_FIFO, data, len);

    clear_irqs();
    map_dio0_txdone();

    set_mode(RF95_MODE_TX);

    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
    bool done = false;
    while (!time_reached(deadline))
    {
        if (gpio_get(LORA_PIN_DIO0))
        {
            done = true;
            break;
        }
        tight_loop_contents();
    }
    lora_standby();

    uint8_t flags = reg_read(REG_IRQ_FLAGS);
    clear_irqs();
    return done && (flags & 0x08); // TxDone
}

// --- RX single com timeout ---
bool lora_receive_single(uint8_t *buf, uint8_t max_len, lora_rx_info_t *info, uint32_t timeout_ms)
{
    if (!buf || max_len == 0)
        return false;

    clear_irqs();
    map_dio0_rxdone();

    // ponteiro de FIFO RX
    reg_write(REG_FIFO_ADDR_PTR, reg_read(REG_FIFO_RX_BASE_AD));

    // modo RX single
    set_mode(0x86); // LoRa + RxSingle (equivalente ao OPMODE_RXSINGLE)

    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
    bool got = false;
    while (!time_reached(deadline))
    {
        if (gpio_get(LORA_PIN_DIO0))
        {
            got = true;
            break;
        }
        tight_loop_contents();
    }
    lora_standby();

    uint8_t flags = reg_read(REG_IRQ_FLAGS);
    clear_irqs();

    if (!got || !(flags & 0x40) || (flags & 0x20)) // RxDone e sem CRC error
        return false;

    uint8_t len = reg_read(REG_RX_NB_BYTES);
    if (len > max_len)
        len = max_len;

    uint8_t cur = reg_read(REG_FIFO_RX_CURRENT_ADDR);
    reg_write(REG_FIFO_ADDR_PTR, cur);
    burst_read(REG_FIFO, buf, len);

    if (info)
    {
        int8_t snr_raw = (int8_t)reg_read(0x19); // REG_PKT_SNR_VALUE
        info->snr_db = ((float)snr_raw) / 4.0f;

        int16_t rssi_raw = reg_read(0x1A); // REG_PKT_RSSI_VALUE
        info->rssi_dbm = rssi_raw - 157;   // ajuste HF (868/915 MHz)
        info->len = len;
    }
    return true;
}
