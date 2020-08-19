
#include "nts1_impl.h"

#include "assert.h"
#include "stdio.h"

uint8_t s_panel_id = PANEL_ID_MASK;              // Bits 3-5 "ppp"="111"
uint8_t s_dummy_tx_cmd = (PANEL_ID_MASK + 0xC7); // B'11ppp111;

uint8_t s_started;

uint8_t s_panel_rx_status;
uint8_t s_panel_rx_data_cnt;
uint8_t s_panel_rx_data[127];

uint8_t s_spi_tx_buf[SPI_TX_BUF_SIZE];
uint16_t s_spi_tx_ridx; // Read  Index (from s_spi_tx_buf)
uint16_t s_spi_tx_widx; // Write Index (to s_spi_tx_buf)

uint8_t s_spi_rx_buf[SPI_RX_BUF_SIZE];
uint16_t s_spi_rx_ridx; // Read  Index (from s_spi_rx_buf)
uint16_t s_spi_rx_widx; // Write Index (to s_spi_rx_buf)

enum
{
  k_tx_cmd_event = 0x84U,
  k_tx_cmd_param = 0x85U,
  k_tx_cmd_other = 0x86U,
  k_tx_cmd_dummy = 0x87U
};

enum
{
  k_tx_subcmd_other_ack = 0x3U,
  k_tx_subcmd_other_version = 0x10U,
  k_tx_subcmd_other_bootmode = 0x11U,
};

enum
{
  k_rx_cmd_event = 0x84U,
  k_rx_cmd_param = 0x85U,
  k_rx_cmd_other = 0x86U,
  k_rx_cmd_dummy = 0x87U
};

enum
{
  k_rx_subcmd_other_panelid = 0x0U,
  k_rx_subcmd_other_stsreq = 0x1U,
  k_rx_subcmd_other_ackreq = 0x3U,
};

typedef struct __nts1_cmd_header
{
  uint8_t cmd : 3;
  uint8_t panel_id : 3;
  uint8_t end_mark : 1;
  uint8_t start_bit : 1;
} __nts1_cmd_header_t;

// ----------------------------------------------------

uint8_t s_spi_chk_rx_buf_space(uint16_t size)
{
  uint16_t count;

  if (s_spi_rx_ridx <= s_spi_rx_widx)
  {
    count = (SPI_RX_BUF_SIZE + s_spi_rx_ridx) - s_spi_rx_widx;
  }
  else
  {
    count = s_spi_rx_ridx - s_spi_rx_widx;
  }
  return (count > size);
}

uint8_t s_spi_rx_buf_write(uint8_t data)
{
  uint16_t bufdatacount;
  if (s_spi_rx_ridx <= s_spi_rx_widx)
  {
    bufdatacount = s_spi_rx_widx - s_spi_rx_ridx;
  }
  else
  {
    bufdatacount = SPI_RX_BUF_SIZE + s_spi_rx_widx - s_spi_rx_ridx;
  }
  if (bufdatacount < (SPI_RX_BUF_SIZE - 2))
  {
    s_spi_rx_buf[SPI_RX_BUF_MASK & s_spi_rx_widx] = data;
    s_spi_rx_widx = SPI_BUF_INC(s_spi_rx_widx, SPI_RX_BUF_SIZE);
    return true;
  }
  return false;
}

uint8_t s_spi_rx_buf_read(void)
{
  const uint8_t data = s_spi_rx_buf[SPI_RX_BUF_MASK & s_spi_rx_ridx];
  s_spi_rx_ridx = SPI_BUF_INC(s_spi_rx_ridx, SPI_RX_BUF_SIZE);
  return data;
}

// Check if there is space in the buffer to accomodate `size` in number of bits
uint8_t s_spi_chk_tx_buf_space(uint16_t size)
{
  uint16_t count;
  if (s_spi_tx_ridx <= s_spi_tx_widx)
  {
    count = SPI_TX_BUF_SIZE + s_spi_tx_ridx - s_spi_tx_widx;
  }
  else
  {
    count = s_spi_tx_ridx - s_spi_tx_widx;
  }
  return (count > size);
}

// writes the byte to the buffer at the appropiate position
// and moves the buffer pointers
void s_spi_tx_buf_write(uint8_t data)
{
  s_spi_tx_buf[SPI_TX_BUF_MASK & s_spi_tx_widx] = data;
  s_spi_tx_widx = SPI_BUF_INC(s_spi_tx_widx, SPI_TX_BUF_SIZE);
}

uint8_t s_spi_tx_buf_read(void)
{
  const uint8_t data = s_spi_tx_buf[SPI_TX_BUF_MASK & s_spi_tx_ridx];
  s_spi_tx_ridx = SPI_BUF_INC(s_spi_tx_ridx, SPI_TX_BUF_SIZE);
  return data;
}

// ----------------------------------------------------

uint8_t s_tx_cmd_event(const nts1_tx_event_t *event, uint8_t endmark)
{
  assert(event != NULL);
  if (!s_spi_chk_tx_buf_space(4))
    return false;
  // if (s_panel_id & PANEL_ID_MASK) + (endmark) > 0, add PANEL_CMD_EMARK bit to the
  // command byte
  const uint8_t cmd = (s_panel_id & PANEL_ID_MASK) + (endmark) ? (k_tx_cmd_event | PANEL_CMD_EMARK) : k_tx_cmd_event;
  s_spi_tx_buf_write(cmd);
  s_spi_tx_buf_write(event->event_id & 0x7F);
  s_spi_tx_buf_write(event->msb & 0x7F);
  s_spi_tx_buf_write(event->lsb & 0x7F);
  return true;
}

uint8_t s_tx_cmd_param_change(const nts1_tx_param_change_t *param_change, uint8_t endmark)
{
  assert(param_change != NULL);
  if (!s_spi_chk_tx_buf_space(4))
    return false;
  const uint8_t cmd = (s_panel_id & PANEL_ID_MASK) + (endmark) ? (k_tx_cmd_param | PANEL_CMD_EMARK) : k_tx_cmd_param;
  s_spi_tx_buf_write(cmd);
  s_spi_tx_buf_write(param_change->param_id & 0x7F);
  s_spi_tx_buf_write(param_change->param_subid & 0x7F);
  s_spi_tx_buf_write(param_change->msb & 0x7F);
  s_spi_tx_buf_write(param_change->lsb & 0x7F);
  return true;
}

uint8_t s_tx_cmd_other_ack(uint8_t endmark)
{
  if (!s_spi_chk_tx_buf_space(3))
    return false;
  const uint8_t cmd = (s_panel_id & PANEL_ID_MASK) + (endmark) ? (k_tx_cmd_other | PANEL_CMD_EMARK) : k_tx_cmd_other;
  s_spi_tx_buf_write(cmd);
  s_spi_tx_buf_write(3);
  s_spi_tx_buf_write(k_tx_subcmd_other_ack);
  return true;
}

uint8_t s_tx_cmd_other_version(uint8_t endmark)
{
  if (!s_spi_chk_tx_buf_space(5))
    return false;
  const uint8_t cmd = (s_panel_id & PANEL_ID_MASK) + (endmark) ? (k_tx_cmd_other | PANEL_CMD_EMARK) : k_tx_cmd_other;
  s_spi_tx_buf_write(cmd);
  s_spi_tx_buf_write(5);
  s_spi_tx_buf_write(k_tx_subcmd_other_version);
  s_spi_tx_buf_write(1);
  s_spi_tx_buf_write(0);
  return true;
}

uint8_t s_tx_cmd_other_bootmode(uint8_t endmark)
{
  if (!s_spi_chk_tx_buf_space(4))
    return false;
  const uint8_t cmd = (s_panel_id & PANEL_ID_MASK) + (endmark) ? (k_tx_cmd_other | PANEL_CMD_EMARK) : k_tx_cmd_other;
  s_spi_tx_buf_write(cmd);
  s_spi_tx_buf_write(4);
  s_spi_tx_buf_write(k_tx_subcmd_other_bootmode);
  s_spi_tx_buf_write(0);
  return true;
}

// ----------------------------------------------------

#define RX_EVENT_MAX_DECODE_SIZE 64
static uint8_t s_rx_event_decode_buf[RX_EVENT_MAX_DECODE_SIZE] = {0};

// This is the handler itself
void s_rx_msg_handler(uint8_t data)
{
  // data is 1 byte of the input buffer
  // If data byte starts with 10000000
  if (data >= PANEL_START_BIT)
  {
    // Status byte
    // resets the counter
    s_panel_rx_data_cnt = 0;
    // data = data AND not(PANEL_CMD_EMARK)
    // PANEL_CMD_EMARK is 01000000
    // not(PANEL_CMD_EMARK) is 10111111
    // so, the following discards the bit at position 6 (starting from pos 0)
    data &= ~PANEL_CMD_EMARK;

    if (data == 0xBEU)
    { // 10111110:Panel ID allocation
      // if data == 1x111110
      s_panel_rx_status = data & ~PANEL_ID_MASK; // discards bits 3,4 and 5
      //s_panel_rx_status  = 10111110 * 11000111 => 10000110
    }
    else if (
        (data & PANEL_ID_MASK) // bits 3 4 5 of data (00xxx000)
        ==
        (s_panel_id & PANEL_ID_MASK) // 00111000
    )
    {                                            // if data contains (xx111xxx)
      s_panel_rx_status = data & ~PANEL_ID_MASK; // produces 10000xxx
    }
    else
    {
      if (s_panel_rx_status)
      s_panel_rx_status = 0; // cancel any previous command reception
    }
    // exits the method to parse next byte
    return;
  }
  // the previous section stores in s_panel_rx_status the first byte
  // received, identified by having its first bit set to 1 (1xxxxxxx)
  // and it processes it to discard irrelevant data

  // Relevant statuses:
  // if data == 1x111110 => active_cmd = 10000110
  // if data == 1x111xxx => active_cmd = 10000xxx
  // otherwise, status byte is discarded

  // Stored status byte
  const uint8_t active_cmd = s_panel_rx_status;

  // Now we process bytes that don't start with 1
  switch (active_cmd)
  {
    // 1x111 100
  case k_rx_cmd_event:
  {
    // printf("received cmd_event\n");

    // this will push stuff into the array
    // for future use
    s_panel_rx_data[s_panel_rx_data_cnt++] = data;

    // transactions are at least 3 bytes long
    if (s_panel_rx_data_cnt < 2)
      break; // need more data

    // The second byte always contains the length of the message
    // so this will wait until all the bytes have been received to process the message
    // It also subtracts 1 from the indicated size, because the first byte
    // (the status byte), is not stored in this array
    if (s_panel_rx_data_cnt < (s_panel_rx_data[0] - 1))
      break; // need more data

    /*++++++++++++++++++++++++++++++++++++++++++++++
        CMD4 : Event
        1st    :[1][0][ppp][100]
        2nd    :[0][sssssss] Size
        3rd    :[0][eeeeeee] Event ID
        4th    :[0][ddddddd] Data word
        ...
        +++++++++++++++++++++++++++++++++++++++++++++*/

    const nts1_rx_event_header_t *rx_event = (const nts1_rx_event_header_t *)s_panel_rx_data;
    // payload is pointer to the position of s_panel_rx_data + nts1_rx_event_header_t
    // which I guess is the memory position of the actual data
    const uint8_t *payload = s_panel_rx_data + sizeof(nts1_rx_event_header_t);

    const uint32_t payload_size7 = (rx_event->size - sizeof(nts1_rx_event_header_t) - 1);
    const uint32_t payload_size8 = nts1_size_7to8(payload_size7);
    if (payload_size8 > RX_EVENT_MAX_DECODE_SIZE)
    {
      // Reset rx status
      s_panel_rx_status = 0;
      s_panel_rx_data_cnt = 0;
      break;
    }
    nts1_convert_7to8(s_rx_event_decode_buf, payload, payload_size7);

    switch (rx_event->event_id)
    {
    case k_nts1_rx_event_id_note_off:
      if (payload_size8 == sizeof(nts1_rx_note_off_t))
        nts1_handle_note_off_event((const nts1_rx_note_off_t *)s_rx_event_decode_buf);
      break;
    case k_nts1_rx_event_id_note_on:
      if (payload_size8 == sizeof(nts1_rx_note_on_t))
        nts1_handle_note_on_event((const nts1_rx_note_on_t *)s_rx_event_decode_buf);
      break;
    case k_nts1_rx_event_id_step_tick:
      nts1_handle_step_tick_event();
      break;
    case k_nts1_rx_event_id_unit_desc:
      //if (payload_size8 == sizeof(nts1_rx_unit_desc_t))
      nts1_handle_unit_desc_event((const nts1_rx_unit_desc_t *)s_rx_event_decode_buf);
      break;
    case k_nts1_rx_event_id_edit_param_desc:
      if (payload_size8 == sizeof(nts1_rx_edit_param_desc_t))
      {
        nts1_handle_edit_param_desc_event((const nts1_rx_edit_param_desc_t *)s_rx_event_decode_buf);
      }
      break;
    case k_nts1_rx_event_id_value:
      if (payload_size8 == sizeof(nts1_rx_value_t))
        nts1_handle_value_event((const nts1_rx_value_t *)s_rx_event_decode_buf);
      break;
    default:
      break;
    }

    // Reset rx status
    s_panel_rx_status = 0;
    s_panel_rx_data_cnt = 0;
  }
  break;

  // 1x111 101
  case k_rx_cmd_param:
  {
    // printf("received cmd_param\n");

    s_panel_rx_data[s_panel_rx_data_cnt++] = data;

    if (s_panel_rx_data_cnt < 4)
      break; // need more data

    /*++++++++++++++++++++++++++++++++++++++++++++++
        CMD5 : Param Change
        1st    :[1][0][ppp][101]
        2nd    :[0][eeeeeee] Param ID
        3rd    :[0][sssssss] Param Sub ID
        4th    :[0][hhhhhhh] MSB
        5th    :[0][lllllll] LSB
        +++++++++++++++++++++++++++++++++++++++++++++*/

    const nts1_rx_param_change_t *rx_param = (const nts1_rx_param_change_t *)s_panel_rx_data;
    nts1_handle_param_change(rx_param);

    // Reset rx status
    s_panel_rx_status = 0;
    s_panel_rx_data_cnt = 0;
  }
  break;

  // 1x111 110
  case k_rx_cmd_other:
  {
    // printf("received cmd_other\n");

    // continue reading until we have the full command
    s_panel_rx_data[s_panel_rx_data_cnt++] = data;
    // it's important to note that here, if s_panel_rx_data[0] (size)
    // is 0, the break will never happen
    if (s_panel_rx_data_cnt < (s_panel_rx_data[0] - 1))
    {
      // printf("received cmd_other need more data\n");
      break; // need more data
    }

    if (s_panel_rx_data_cnt < 2)
    {
      // printf("received cmd_other reset\n");
      // Command too short - ignore and reset
      s_panel_rx_status = 0;
      s_panel_rx_data_cnt = 0;
      break;
    }

    // printf("received cmd_other parsing\n");
    // switches on the 3rd byte (MessageID)
    switch (s_panel_rx_data[1])
    {
    case k_rx_subcmd_other_panelid: // Panel ID specified ("ppp" is left here but not here)
      /*++++++++++++++++++++++++++++++++++++++++++++++
          CMD6-0 :PanelID designation.
          Only in this case, specify ppp=7.
          1st    :[1][0][111][110] ppp=7 use
          2nd    :[0][0000100] Size=4
          3rd    :[0][0000000] MessageID = 0
          4th    :[0][0000PPP] The PanelID number you specify.
          +++++++++++++++++++++++++++++++++++++++++++++*/
      if (s_panel_rx_data_cnt >= 3 && s_panel_rx_data[0] == 4)
      {
        s_panel_id = ((s_panel_rx_data[2] & 0x07) << 3) & PANEL_ID_MASK;
        s_dummy_tx_cmd = s_panel_id | 0xC7; // B'11ppp111;
        // Send Version to HOST
        s_tx_cmd_other_version(false);
        // Send All SW Pattern to HOST
        s_tx_cmd_other_bootmode(true);
      }
      // Reset rx status
      s_panel_rx_status = 0;
      s_panel_rx_data_cnt = 0;
      break;

    case k_rx_subcmd_other_stsreq:
      /*++++++++++++++++++++++++++++++++++++++++++++++
          CMD6-1 :Status Request
          Request to send the current SwitchPattern (CMD6-17) and all knob commands.
          1st    :[1][0][ppp][110]
          2nd    :[0][0000011] Size=3
          3rd    :[0][0000001] MessageID = 1
          +++++++++++++++++++++++++++++++++++++++++++++*/
      s_tx_cmd_other_bootmode(true);
      // Reset rx status
      s_panel_rx_status = 0;
      s_panel_rx_data_cnt = 0;
      break;

    case k_rx_subcmd_other_ackreq: // Panel ACK req
      /*++++++++++++++++++++++++++++++++++++++++++++++
          CMD6-3 :ACK request
          For checking whether the Panel is operating normally.
          When Panel receives this, it returns an ACK command.
          1st    :[1][0][ppp][110]
          2nd    :[0][0000011] Size=3
          3rd    :[0][0000011] MessageID = 3
          +++++++++++++++++++++++++++++++++++++++++++++*/
      s_tx_cmd_other_ack(true);
      // Reset rx status
      s_panel_rx_status = 0;
      s_panel_rx_data_cnt = 0;
      break;

    default:
      // Undefined command - ignore and reset
      s_panel_rx_status = 0;
      s_panel_rx_data_cnt = 0;
      break;
    }
  } // end case k_rx_cmd_other:
  // 1x111 111
  case k_rx_cmd_dummy:
    // printf("received dummy\n");
    // continues to default
  default:
    // resets
    s_panel_rx_status = 0;   // Clear save status
    s_panel_rx_data_cnt = 0; // Initialize data count
    break;
  }
}

// ----------------------------------------------------

nts1_status_t nts1_send_events(nts1_tx_event_t *events, uint8_t count)
{
  assert(events != NULL);
  for (uint8_t i = 0; i < count; ++i)
  {
    if (!s_tx_cmd_event(&events[i], (i == count - 1)))
    {
      return k_nts1_status_busy;
    }
  }
  return k_nts1_status_ok;
}

nts1_status_t nts1_send_param_changes(nts1_tx_param_change_t *param_changes, uint8_t count)
{
  assert(param_changes != NULL);
  for (uint8_t i = 0; i < count; ++i)
  {
    if (!s_tx_cmd_param_change(&param_changes[i], (i == count - 1)))
    {
      return k_nts1_status_busy;
    }
  }
  return k_nts1_status_ok;
}

uint32_t nts1_convert_7to8(uint8_t *dest8, const uint8_t *src7, uint32_t size7)
{
  const uint32_t size8 = nts1_size_7to8(size7);
  // const uint8_t *src7_e = src7 + size7;
  for (uint32_t i7 = 0, i8 = 0; i7 < size7; ++i7)
  {
    const uint8_t i7mod8 = i7 % 8;
    switch (i7mod8)
    {
    case 0:
      dest8[i8++] = src7[i7] & 0x7F;
      break;
    case 7:
      dest8[i8 - 1] |= (src7[i7] & 0x7F) << 1;
      break;
    default:
    {
      const uint8_t offset = 8 - i7mod8;
      const uint8_t src = src7[i7];
      dest8[i8 - 1] |= (src & ((1U << i7mod8) - 1)) << offset;
      dest8[i8++] = (src & 0x7F) >> i7mod8;
    }
    break;
    }
  }
  return size8;
}

uint32_t nts1_convert_8to7(uint8_t *dest7, const uint8_t *src8, uint32_t size8)
{
  const uint32_t size7 = nts1_size_8to7(size8);
  // const uint8_t *dest7_e = dest7 + size7;
  for (uint32_t i7 = 0, i8 = 0; i7 < size7; ++i7)
  {
    const uint8_t i7mod8 = i7 % 8;
    switch (i7mod8)
    {
    case 0:
      dest7[i7] = src8[i8++] & 0x7F;
      break;
    case 7:
      dest7[i7] = (src8[i8 - 1] & (0x7F << 1)) >> 1;
      break;
    default:
    {
      const uint8_t offset = 8 - i7mod8;
      uint8_t dest = (src8[i8 - 1] & (0xFFU << offset)) >> offset;
      dest |= (src8[i8++] & (0x7F >> i7mod8)) << i7mod8;
      dest7[i7] = dest;
    }
    break;
    }
  }
  return size7;
}

nts1_status_t nts1_param_change(uint8_t id, uint8_t subid, uint16_t value)
{
  nts1_tx_param_change_t param;
  param.param_id = id;
  param.param_subid = subid;
  param.msb = (value >> 7) & 0x7F;
  param.lsb = value & 0x7F;
  return nts1_send_param_change(&param);
}

nts1_status_t nts1_note_on(uint8_t note, uint8_t velo)
{
  nts1_tx_event_t event;
  event.event_id = k_nts1_tx_event_id_note_on;
  event.msb = note & 0x7F;
  event.lsb = velo & 0x7F;
  return nts1_send_event(&event);
}

nts1_status_t nts1_note_off(uint8_t note)
{
  nts1_tx_event_t event;
  event.event_id = k_nts1_tx_event_id_note_off;
  event.msb = note & 0x7F;
  event.lsb = 0x00;
  return nts1_send_event(&event);
}

nts1_status_t nts1_req_sys_version(void)
{
  nts1_tx_event_t event;
  event.event_id = k_nts1_tx_event_id_req_value;
  event.msb = k_param_id_sys_version;
  event.lsb = 0x0;
  return nts1_send_event(&event);
}

nts1_status_t nts1_req_param_value(uint8_t id, uint8_t subid)
{
  nts1_tx_event_t event;
  event.event_id = k_nts1_tx_event_id_req_value;
  event.msb = id;
  event.lsb = subid;
  return nts1_send_event(&event);
}

nts1_status_t nts1_req_osc_count(void)
{
  nts1_tx_event_t event;
  event.event_id = k_nts1_tx_event_id_req_unit_count;
  event.msb = k_param_id_osc_type;
  event.lsb = 0x0;
  return nts1_send_event(&event);
}

nts1_status_t nts1_req_osc_desc(uint8_t idx)
{
  nts1_tx_event_t event;
  event.event_id = k_nts1_tx_event_id_req_unit_desc;
  event.msb = k_param_id_osc_type;
  event.lsb = idx & 0x7F;
  return nts1_send_event(&event);
}

nts1_status_t nts1_req_osc_edit_param_desc(uint8_t idx)
{
  nts1_tx_event_t event;
  event.event_id = k_nts1_tx_event_id_req_edit_param_desc;
  event.msb = k_param_id_osc_type;
  event.lsb = idx;
  return nts1_send_event(&event);
}

nts1_status_t nts1_req_filt_count(void)
{
  nts1_tx_event_t event;
  event.event_id = k_nts1_tx_event_id_req_unit_count;
  event.msb = k_param_id_filt_type;
  event.lsb = 0x0;
  return nts1_send_event(&event);
}

nts1_status_t nts1_req_filt_desc(uint8_t idx)
{
  nts1_tx_event_t event;
  event.event_id = k_nts1_tx_event_id_req_unit_desc;
  event.msb = k_param_id_filt_type;
  event.lsb = idx & 0x7F;
  return nts1_send_event(&event);
}

nts1_status_t nts1_req_ampeg_count(void)
{
  nts1_tx_event_t event;
  event.event_id = k_nts1_tx_event_id_req_unit_count;
  event.msb = k_param_id_ampeg_type;
  event.lsb = 0x0;
  return nts1_send_event(&event);
}

nts1_status_t nts1_req_ampeg_desc(uint8_t idx)
{
  nts1_tx_event_t event;
  event.event_id = k_nts1_tx_event_id_req_unit_desc;
  event.msb = k_param_id_ampeg_type;
  event.lsb = idx & 0x7F;
  return nts1_send_event(&event);
}

nts1_status_t nts1_req_mod_count(void)
{
  nts1_tx_event_t event;
  event.event_id = k_nts1_tx_event_id_req_unit_count;
  event.msb = k_param_id_mod_type;
  event.lsb = 0x0;
  return nts1_send_event(&event);
}

nts1_status_t nts1_req_mod_desc(uint8_t idx)
{
  nts1_tx_event_t event;
  event.event_id = k_nts1_tx_event_id_req_unit_desc;
  event.msb = k_param_id_mod_type;
  event.lsb = idx & 0x7F;
  return nts1_send_event(&event);
}

nts1_status_t nts1_req_del_count(void)
{
  nts1_tx_event_t event;
  event.event_id = k_nts1_tx_event_id_req_unit_count;
  event.msb = k_param_id_del_type;
  event.lsb = 0x0;
  return nts1_send_event(&event);
}

nts1_status_t nts1_req_del_desc(uint8_t idx)
{
  nts1_tx_event_t event;
  event.event_id = k_nts1_tx_event_id_req_unit_desc;
  event.msb = k_param_id_del_type;
  event.lsb = idx & 0x7F;
  return nts1_send_event(&event);
}

nts1_status_t nts1_req_rev_count(void)
{
  nts1_tx_event_t event;
  event.event_id = k_nts1_tx_event_id_req_unit_count;
  event.msb = k_param_id_rev_type;
  event.lsb = 0x0;
  return nts1_send_event(&event);
}

nts1_status_t nts1_req_rev_desc(uint8_t idx)
{
  nts1_tx_event_t event;
  event.event_id = k_nts1_tx_event_id_req_unit_desc;
  event.msb = k_param_id_rev_type;
  event.lsb = idx & 0x7F;
  return nts1_send_event(&event);
}

nts1_status_t nts1_req_arp_pattern_count(void)
{
  nts1_tx_event_t event;
  event.event_id = k_nts1_tx_event_id_req_unit_count;
  event.msb = k_param_id_arp_pattern;
  event.lsb = 0x0;
  return nts1_send_event(&event);
}

nts1_status_t nts1_req_arp_pattern_desc(uint8_t idx)
{
  nts1_tx_event_t event;
  event.event_id = k_nts1_tx_event_id_req_unit_desc;
  event.msb = k_param_id_arp_pattern;
  event.lsb = idx & 0x7F;
  return nts1_send_event(&event);
}

nts1_status_t nts1_req_arp_intervals_count(void)
{
  nts1_tx_event_t event;
  event.event_id = k_nts1_tx_event_id_req_unit_count;
  event.msb = k_param_id_arp_intervals;
  event.lsb = 0x0;
  return nts1_send_event(&event);
}

nts1_status_t nts1_req_arp_intervals_desc(uint8_t idx)
{
  nts1_tx_event_t event;
  event.event_id = k_nts1_tx_event_id_req_unit_desc;
  event.msb = k_param_id_arp_intervals;
  event.lsb = idx & 0x7F;
  return nts1_send_event(&event);
}
