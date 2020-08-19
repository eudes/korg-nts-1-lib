#ifndef __nts1_impl_h
#define __nts1_impl_h

#include "nts1_iface.h"

#ifndef true
#define true 1
#endif

#ifndef false
#define false 0
#endif

// ----------------------------

#define PANEL_ID_MASK 0x38   // Bits 3-5 // 00111000
#define PANEL_CMD_EMARK 0x40 // Bit  6   // 01000000
#define PANEL_START_BIT 0x80 // Bit  7   // 10000000

// ----------------------------------------------------

extern uint8_t s_panel_id;     // Bits 3-5 "ppp"="111"
extern uint8_t s_dummy_tx_cmd; // B'11ppp111;

extern uint8_t s_started;

extern uint8_t s_panel_rx_status;
extern uint8_t s_panel_rx_data_cnt;
extern uint8_t s_panel_rx_data[127];

// --------------------------------------

#define SPI_TX_BUF_SIZE (0x200)
#define SPI_TX_BUF_MASK (SPI_TX_BUF_SIZE - 1)

#define SPI_RX_BUF_SIZE (0x200)
#define SPI_RX_BUF_MASK (SPI_RX_BUF_SIZE - 1)

extern uint8_t s_spi_tx_buf[SPI_TX_BUF_SIZE];
extern uint16_t s_spi_tx_ridx; // Read  Index (from s_spi_tx_buf)
extern uint16_t s_spi_tx_widx; // Write Index (to s_spi_tx_buf)

extern uint8_t s_spi_rx_buf[SPI_RX_BUF_SIZE];
extern uint16_t s_spi_rx_ridx; // Read  Index (from s_spi_rx_buf)
extern uint16_t s_spi_rx_widx; // Write Index (to s_spi_rx_buf)

// --------------------------------------

#define SPI_TX_BUF_RESET() (s_spi_tx_ridx = s_spi_tx_widx = 0)
// Tx read an write indexes point to the same location
#define SPI_TX_BUF_EMPTY() ((SPI_TX_BUF_MASK & s_spi_tx_ridx) == (SPI_TX_BUF_MASK & s_spi_tx_widx))
#define SPI_RX_BUF_RESET() (s_spi_rx_ridx = s_spi_rx_widx = 0)
// Rx read an write idexes point to the same location
#define SPI_RX_BUF_EMPTY() ((SPI_RX_BUF_MASK & s_spi_rx_ridx) == (SPI_RX_BUF_MASK & s_spi_rx_widx)) 
#define SPI_BUF_INC(idx, bufSize) (((idx + 1) == bufSize) ? 0 : idx + 1)

// -------------------------------------

nts1_status_t nts1_init();
nts1_status_t nts1_idle();

// ----------------------------------------------------

void s_rx_msg_handler(uint8_t data);

// -------------------------------------

uint8_t s_spi_chk_rx_buf_space(uint16_t size);
uint8_t s_spi_rx_buf_write(uint8_t data);
uint8_t s_spi_rx_buf_read(void);
uint8_t s_spi_chk_tx_buf_space(uint16_t size);
void s_spi_tx_buf_write(uint8_t data);
uint8_t s_spi_tx_buf_read(void);

#endif // __nts1_impl_h