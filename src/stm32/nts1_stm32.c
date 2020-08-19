/** 
 * @file nts1_iface.c
 * @brief C API handling communication with the NTS-1 digital kit's main board.
 *
 * Provides initialization of necessary peripherals, handling of low level 
 * protocol details and convenience functions for bidrectional communication
 * with the NTS-1 digital kit's main board via SPI.
 *   
 * BSD 3-Clause License
 *  Copyright (c) 2020, KORG INC.
 *  All rights reserved.
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 * * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 //*/

#if defined(STM32F0xx)

#include "../nts1_impl.h"

#include <assert.h>

#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_def.h"
#include "stm32f0xx_hal_spi.h"

#include "stm32_def.h"
#include "PeripheralPins.h"
#include "PinAF_STM32F1.h"
#include "pinconfig.h"
#include "spi_com.h"

// -----------------------------
// pins and ports
#define SPI_PERIPH SPI2
#define SPI_MISO_PORT GPIOB
#define SPI_MISO_PIN GPIO_PIN_14
#define SPI_MOSI_PORT GPIOB
#define SPI_MOSI_PIN GPIO_PIN_15
#define SPI_SCK_PORT GPIOB
#define SPI_SCK_PIN GPIO_PIN_13
#define SPI_GPIO_AF GPIO_AF0_SPI2

#define ACK_PORT GPIOB
#define ACK_PIN GPIO_PIN_12

// ----------------------------------------

#define SPI_IRQn SPI2_IRQn
#define SPI_IRQ_PRIORITY 1
#define SPI_IRQ_HANDLER SPI2_IRQHandler

#define SPI_GPIO_CLK_ENA() __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPI_FORCE_RESET() __HAL_RCC_SPI2_FORCE_RESET()
#define SPI_RELEASE_RESET() __HAL_RCC_SPI2_RELEASE_RESET()
#define SPI_CLK_ENABLE() __HAL_RCC_SPI2_CLK_ENABLE()
#define SPI_CLK_DISABLE() __HAL_RCC_SPI2_CLK_DISABLE()

#define ACK_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()

// ----------------------------------------------------

static SPI_HandleTypeDef s_spi;

// ----------------------------------------------------

static inline void s_port_startup_ack(void)
{
  // This sets the ACK_PIN GPIO pin to 1
  // ACK_PORT is the GPIO device (of which the STM32 has 2)
  // BSRR is the Bit Set Reset Register.
  // Setting one or more bits in this register will write to the
  // corresponding GPIO pin.
  // This is an atomic operation (takes 1 system clock tick),
  // as oppossed to setting 1 bit in general GPIO register,
  // which would force the CPU to first read the GPIO register,
  // perform the bitwise operation, and then set the GPIO register
  // again.
  ACK_PORT->BSRR = ACK_PIN;
  // NOTE: C doesn't guarantee that operations will be atomic.
}

static inline void s_port_wait_ack(void)
{
  // This sets the ACK_PIN GPIO pin to 0
  // Same as above, but the register is the Bit Reset Register
  ACK_PORT->BRR = ACK_PIN;
}

// ----------------------------------------------------

// TODO leave here or generalize?
static inline void s_spi_raw_fifo_push8(uint8_t data)
{
  SPI_TypeDef *SPIx = SPI_PERIPH;

  // SPIx is a pointer, so will a contain a memory address of the SPI device
  // 0x0C is the offset of the Data Register with respect to the SPI device.
  // The casting "(uint32_t)" is necessary here because SPIx is a pointer address,
  // but it's trying to save it into a non-pointer variable. The compiler won't allow
  // this unless you force it by doing the casting.
  const uint32_t spix_dr = (uint32_t)SPIx + 0x0C;
  // spix_dr is the address of the SPIX_DR data register
  // Reading from this address will return the oldest frame of data from the Rx FIFO
  // Writing to this address will write to the end of the Tx FIFO

  // In this case, we are writing to it
  *(__IO uint8_t *)spix_dr = data;
  // *(__IO uint8_t *) spix_dr = data means:
  // - (__IO uint8_t *) spix_dr: cast spidx_dr to a pointer (*) of type __IO uint8_t
  // - first "*": derreference the pointer we just casted and
  // - "= data": assign the value of data to the derefferenced location
  // In other words, it's equivalent to:
  // __IO uint8_t* ptr_spix_dr = (__IO uint8_t *) spix_dr; // again, casting to change from uint to *uint
  // *ptr_spix_dr = data;
}

static inline uint8_t s_spi_raw_fifo_pop8()
{
  const uint32_t spix_dr = (uint32_t)SPI_PERIPH + 0x0C;

  // In this case, we are reading it
  return *(__IO uint8_t *)spix_dr;
}

// ----------------------------------------------------

static inline void s_spi_struct_init(SPI_InitTypeDef *SPI_InitStruct)
{
  SPI_InitStruct->Mode = SPI_MODE_SLAVE;
  SPI_InitStruct->Direction = SPI_DIRECTION_2LINES;
  SPI_InitStruct->DataSize = SPI_DATASIZE_8BIT;
  SPI_InitStruct->CLKPolarity = SPI_POLARITY_HIGH;
  SPI_InitStruct->CLKPhase = SPI_PHASE_2EDGE;
  SPI_InitStruct->NSS = SPI_NSS_SOFT;
  SPI_InitStruct->BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  SPI_InitStruct->FirstBit = SPI_FIRSTBIT_LSB;
  SPI_InitStruct->TIMode = SPI_TIMODE_DISABLE;
  SPI_InitStruct->CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct->CRCPolynomial = 7;
  SPI_InitStruct->CRCLength = SPI_CRC_LENGTH_DATASIZE;
  SPI_InitStruct->NSSPMode = SPI_NSS_PULSE_DISABLE;
}

static inline void s_spi_enable_pins()
{
  // Same as ACK_GPIO_CLK_ENABLE,
  // mapped to __HAL_RCC_GPIOB_CLK_ENABLE
  SPI_GPIO_CLK_ENA();

  GPIO_InitTypeDef gpio;

  /* Enable SCK, MOSI, MISO. No NSS. */
  /* Peripherals alternate function */
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Pull = GPIO_NOPULL;
  // GPIO pins set to alternate function, which allows alternating the pins
  // between GPIO functions and internal peripheral functions
  // This is a requirement of the SPI device in the STM32
  // See Reference Manual, Figure 274
  gpio.Alternate = SPI_GPIO_AF;

  gpio.Pin = SPI_MISO_PIN;
  HAL_GPIO_Init(SPI_MISO_PORT, &gpio);

  gpio.Pin = SPI_MOSI_PIN;
  HAL_GPIO_Init(SPI_MOSI_PORT, &gpio);

  gpio.Pin = SPI_SCK_PIN;
  gpio.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SPI_SCK_PORT, &gpio);
}

nts1_status_t s_spi_init()
{
  // Enables the clock for the SYSCFG register
  // which is used for specific configurations of memory and
  // DMA requests remap and to control special I/O features.
  __HAL_RCC_SYSCFG_CLK_ENABLE();

  // Configures the GPIO pins for SPI
  // See below
  s_spi_enable_pins();

  // Resets and enables clock signals for SPI
  SPI_FORCE_RESET();
  SPI_RELEASE_RESET();
  SPI_CLK_ENABLE();

  // Configures SPI as seen above
  // Note that SPI_PERIPH is mapped to SPI2, which is one
  // of the 2 available SPI devices in the MCU
  s_spi.Instance = SPI_PERIPH;
  s_spi_struct_init(&(s_spi.Init));

  // Initializes the SPI driver
  const HAL_StatusTypeDef res = HAL_SPI_Init(&s_spi);
  if (res != HAL_OK)
  {
    return res;
  }

  // Sets the interrupt handler in the NVIC (Nested Vector Interrupt Controller)
  HAL_NVIC_SetPriority(SPI_IRQn, SPI_IRQ_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(SPI_IRQn);

  // Adds (bitwise OR) RXNE to the SPI control register 2
  // This enables the "RX buffer not empty" interrupt
  // See Reference Manual 28.5.10
  SPI_PERIPH->CR2 |= SPI_IT_RXNE;

  // Finally enables HAL SPI
  __HAL_SPI_ENABLE(&s_spi);

  // Fills the hardware transmission FIFO register with dummy data
  // Fill TX FIFO
  s_spi_raw_fifo_push8(s_dummy_tx_cmd);
  s_spi_raw_fifo_push8(s_dummy_tx_cmd);
  s_spi_raw_fifo_push8(s_dummy_tx_cmd);
  s_spi_raw_fifo_push8(s_dummy_tx_cmd);

  return (nts1_status_t)HAL_OK;
}

void s_ack_init()
{
  // Enables sampling the GPIO pin at clock rate
  // This is mapped to __HAL_RCC_GPIOB_CLK_ENABLE
  ACK_GPIO_CLK_ENABLE();

  GPIO_InitTypeDef gpio;

  // Configures a GPIO pin for ACK (acknowledge message)
  /* PANEL ACK */
  gpio.Mode = GPIO_MODE_OUTPUT_PP; // normal output, not Open Collector or Open Drain
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Pull = GPIO_NOPULL;
  gpio.Alternate = 0;
  gpio.Pin = ACK_PIN;
  HAL_GPIO_Init(ACK_PORT, &gpio);
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

nts1_status_t s_spi_teardown()
{
  __HAL_SPI_DISABLE(&s_spi);
  SPI_CLK_DISABLE();
  return (nts1_status_t)HAL_OK;
}

nts1_status_t nts1_teardown()
{
  nts1_status_t res = s_spi_teardown();

  return res;
}
// ----------------------------------------------------

extern void SPI_IRQ_HANDLER()
{
  volatile uint16_t sr;
  uint8_t txdata, rxdata;

  // HOST -> PANEL
  // While the RX FIFO is not empty
  // This is signified by the RXNE flag in the SPI Status Registry being 1
  // and calculated from the SPI Status registry
  // by doing a bitwise AND between the SPI_SR_RXNE flag and the SR itself
  // see Reference Manual 28.9.3
  while ((sr = SPI_PERIPH->SR) & SPI_SR_RXNE)
  {

    // Take out 8 bits from the FIFO
    rxdata = s_spi_raw_fifo_pop8(); // DR read clears RXNE flag
    // And write it into the software RX buffer
    // This function only writes if the buffer has enough space to accomodate the byte
    if (!s_spi_rx_buf_write(rxdata))
    { // write to the RX buffer
      // If there's not enough space in the buffer
      // this resets the index read and write indexes for the buffer
      // which will cause it to start writing into the buffer from the beginning

      // If RxBuf is full, reset it.
      SPI_RX_BUF_RESET();
    }
    else
    {
      // If there is not 32 bits of space
      if (!s_spi_chk_rx_buf_space(32))
      {
        // the ACK pin is set to 0
        s_port_wait_ack();
        // I assume this means the NTS1 will stop sending data
      }
      else
      { //Remaining buffer
        // otherwise the ACK pin is set to 1
        s_port_startup_ack();
        // which will allow the NTS1 to send data
      }
    }
    // so, basically, as long as there is data in the RX FIFO
    // read it and put it in the RX buffer
    // if there's no space, the RX write and read position pointers
    // are reset, so that we can start writing from the beginning of the
    // buffer again.
    // Do this until there's no more data in the RC FIFO (the RXNE flag clears then)
  }

  // HOST <- PANEL
  if (!SPI_TX_BUF_EMPTY())
  { // Send buffer has data
    // If there's data to be sent
    txdata = s_spi_tx_buf_read(); // read it and advance the read idx pointer
    if (txdata & PANEL_START_BIT)
    { // Check if the data contains 0x80 (is a Status message)
      if (!SPI_TX_BUF_EMPTY())
      { // There is (more) data (after the status) to be sent next in the send buffer
        // an EMARK is set on the status
        txdata |= PANEL_CMD_EMARK;
        // Note: this will set endmark on almost any status, especially those who have pending data,
        //       which seems to contradict the endmark common usage of marking only the last command of a group
      }
    }
    // Data is sent to the Tx FIFO register
    s_spi_raw_fifo_push8(txdata);
  }
  else
  { // Set the dummy because the send buffer is empty.
    // Dummy data is sent to the TX FIFO register
    s_spi_raw_fifo_push8(s_dummy_tx_cmd);
  }

  // write one byte of data from the Tx buffer to the Tx FIFO.
  // if the byte is a Status message, and it has more data after it,
  // add the ENDMARK (byte 7 set to 1)
  // If there's no data to be sent, write the DUMMY message
}

// ----------------------------------------------------

nts1_status_t nts1_idle()
{
  // Return of HOST communication Check
  // This should be true right after executing setup()
  if (s_started)
  {
    // Checks if the reception buffer is not full
    if (s_spi_chk_rx_buf_space(32))
    {
      // Sets the ACK pin to 1
      s_port_startup_ack();
    }
  }

  // HOST I/F Give priority to Idle processing of received data
  // As long as the reception buffer is not empty
  while (!SPI_RX_BUF_EMPTY())
  {
    // Reads from the buffer and executes the handler
    // Data in receive buffer
    s_rx_msg_handler(s_spi_rx_buf_read());
  }

  return k_nts1_status_ok;
}

#endif // STM32F030x8