#if defined(ESP_PLATFORM)

#include "../nts1_impl.h"

#include "assert.h"
#include "esp_types.h"
#include "stdio.h"

#include "driver/spi_slave.h"
#include "driver/gpio.h"

// -----------------------------
// pins and ports
#define SPI_MISO_PIN 19 // TX_PANEL
#define SPI_MOSI_PIN 23 // RX_PANEL
#define SPI_SCK_PIN 18  // SCK
#define ACK_PIN 21      // ACK

#define SPI_CS_PIN 5 // - None, controlled by software, allways on

#define SPI_MODE 3
#define SPI_BITORDER SPI_SLAVE_BIT_LSBFIRST

#define S_SPI_HOST VSPI_HOST // Use the SPI3 device
#define DMA_CHANNEL 0        // disable dma, use direct spi buffer

#define SPI_TRANSACTION_BYTES 4 // 4 bytes is the minimum the master supports
#define SPI_TRANSACTION_BITS 32 // 4 bytes is the minimum the master supports

#define SPI_QUEUE_TTW 10
// #define SPI_QUEUE_TTW portMAX_DELAY

// ----------------------------------------

static inline void s_port_startup_ack(void)
{
    WRITE_PERI_REG(GPIO_OUT_W1TS_REG, (1 << ACK_PIN));
}

static inline void s_port_wait_ack(void)
{
    WRITE_PERI_REG(GPIO_OUT_W1TC_REG, (1 << ACK_PIN));
}

// ----------------------------------------------------

uint32_t ready_transactions = 0;

// Called after a transaction is queued and ready for pickup by master. We use this to set the ACK line high.
void s_spi_irq_handler_post_setup(spi_slave_transaction_t *trans)
{
    s_port_startup_ack();
    ready_transactions += 1;
}

// Called after transaction is sent/received. We use this to set the ACK line low.
void s_spi_irq_handler_post_transaction(spi_slave_transaction_t *trans)
{
    s_port_wait_ack();
}

// ----------------------------------------------------

nts1_status_t s_spi_init()
{
    // DMA interrupts:
    // SPI_IN_DONE_INT
    // SPI_OUT_DONE_INT

    //Configuration for the SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = SPI_MOSI_PIN,
        .miso_io_num = SPI_MISO_PIN,
        .sclk_io_num = SPI_SCK_PIN,
    };

    //Configuration for the SPI slave interface
    spi_slave_interface_config_t slvcfg = {
        .mode = SPI_MODE,
        .spics_io_num = SPI_CS_PIN,
        .queue_size = 0xFFF,
        .flags = SPI_BITORDER,
        .post_setup_cb = s_spi_irq_handler_post_setup,
        .post_trans_cb = s_spi_irq_handler_post_transaction,
    };

    // Pull down on the Chip Select pin to make it always on
    gpio_set_pull_mode(SPI_CS_PIN, GPIO_PULLDOWN_ONLY);

    // Pull the SPI lines up (as per the schematics)
    gpio_set_pull_mode(SPI_SCK_PIN, GPIO_PULLUP_ONLY); // (CPOL = 1, normally high)
    gpio_set_pull_mode(SPI_MISO_PIN, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(SPI_MOSI_PIN, GPIO_PULLUP_ONLY);

    //Initialize SPI slave interface
    if (!spi_slave_initialize(S_SPI_HOST, &buscfg, &slvcfg, DMA_CHANNEL))
    {
        return k_nts1_status_error;
    }

    s_spi_tx_buf_write(s_dummy_tx_cmd);
    s_spi_tx_buf_write(s_dummy_tx_cmd);
    s_spi_tx_buf_write(s_dummy_tx_cmd);
    s_spi_tx_buf_write(s_dummy_tx_cmd);

    return k_nts1_status_ok;
}

nts1_status_t s_spi_teardown()
{
    printf("tearing down spi\n");
    spi_slave_free(S_SPI_HOST);
    return k_nts1_status_ok;
}

void s_ack_init()
{
    // Configuration for the handshake line
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = (1 << ACK_PIN)};

    //Configure handshake line as output
    gpio_config(&io_conf);
}

nts1_status_t nts1_init()
{
  // Init the ACK GPIO pin
  s_ack_init();

  // Empties the buffers for transmission and reception
  // and reset counters
  s_panel_rx_status = 0;
  s_panel_rx_data_cnt = 0;
  SPI_RX_BUF_RESET();
  SPI_TX_BUF_RESET();

  // More on this below
  nts1_status_t res = s_spi_init();

  if (res != k_nts1_status_ok)
  {
    return res;
  }

  // Sets the ACK pin to 1
  // More below
  s_port_startup_ack();
  s_started = true;

  return k_nts1_status_ok;
}

// ----------------------------------------------------

nts1_status_t nts1_idle()
{
    uint8_t txdata;
    spi_slave_transaction_t transaction;
    transaction.length = SPI_TRANSACTION_BITS;
    uint8_t *tx_buf_ptr_first_byte = s_spi_tx_buf + s_spi_tx_ridx;

    // HOST <- PANEL
    if (!SPI_TX_BUF_EMPTY() && !s_spi_chk_tx_buf_space(SPI_TRANSACTION_BYTES))
    {
        // If there's no space for the full transaction, reset the buffer
        SPI_TX_BUF_RESET();
    }

    uint8_t tx_added_bytes_ctr = 0;
    for (; tx_added_bytes_ctr < SPI_TRANSACTION_BYTES; tx_added_bytes_ctr++)
    {
        // If there's no data to be sent in the Tx buffer, exit the loop
        if (SPI_TX_BUF_EMPTY())
            break;

        // Save a pointer to the current read location on the Tx buffer
        uint8_t *tx_buf_ptr = s_spi_tx_buf + s_spi_tx_ridx;

        // read it and advance the read idx pointer
        txdata = s_spi_tx_buf_read();

        // Check if the data is a Status message
        if (txdata & PANEL_START_BIT)
        {
            // Check if there is more data (after the status) to be sent next in the send buffer
            if (!SPI_TX_BUF_EMPTY())
            {
                // Set the END_MARK on the status
                txdata |= PANEL_CMD_EMARK;
                // Note: this will set endmark on almost any status, especially those who have pending data,
                //       which seems to contradict the endmark common usage of marking only the last command of a group
            }
        }

        // Save changes to the buffer
        *tx_buf_ptr = txdata;
    }

    // If we processed any bytes for the transaction
    if (tx_added_bytes_ctr)
    {
        // Point the transaction's Tx buffer to the first byte we want to send
        // from the software Tx buffer
        transaction.tx_buffer = (void *)tx_buf_ptr_first_byte;

        if (!s_spi_chk_rx_buf_space(SPI_TRANSACTION_BITS))
        {
            // printf("resetting rx\n");
            // If there's no space for the full transaction, reset the buffer
            SPI_RX_BUF_RESET();
        }
        // Point the the transaction's Rx buffer to the current Rx write idx
        uint32_t *rx_buf_ptr = s_spi_rx_buf + s_spi_rx_widx;
        // zero out the values; note that rx_buf_ptr points to 32 bits, not 8
        *rx_buf_ptr = 0;
        transaction.rx_buffer = (void *)rx_buf_ptr;

        // We always need to increase both counters by the length of the
        // transaction because the transaction will always read and write
        // transaction.length bits from the respective buffers
        for (uint8_t i = 0; i < SPI_TRANSACTION_BYTES; i++)
        {
            // Always increase Rx write counter
            s_spi_rx_widx = SPI_BUF_INC(s_spi_rx_widx, SPI_RX_BUF_SIZE);
            // Increase Tx read counter if we haven't already in the first loop
            if (i >= tx_added_bytes_ctr)
            {
                s_spi_tx_ridx = SPI_BUF_INC(s_spi_tx_ridx, SPI_TX_BUF_SIZE);
            }
        }

        esp_err_t err = spi_slave_queue_trans(S_SPI_HOST, &transaction, SPI_QUEUE_TTW);
        if(err){
            // spi  0x3ffb2470
            // buff 0x3ffb270c
        }
    }

    if (ready_transactions)
    {
        // It's mandatory to call this function if using spi_slave_queue_trans
        spi_slave_transaction_t *out;
        spi_slave_get_trans_result(S_SPI_HOST, &out, SPI_QUEUE_TTW);
        ready_transactions -= 1;
    }

    // HOST I/F Give priority to Idle processing of received data
    // As long as the reception buffer is not empty
    while (!SPI_RX_BUF_EMPTY())
    {
        // Reads from the buffer and executes the handler
        s_rx_msg_handler(s_spi_rx_buf_read());
    }

    return k_nts1_status_ok;
}

#endif //ESP_PLATFORM