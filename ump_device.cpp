/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 * Copyright (c) 2022 Michael Loh (AmeNote.com)
 *
 * NOTE: Code adjustments made to support USB MIDI 2.0 UMP Packet format as
 * Alternate Interface 1. See USB Device Class Definition for MIDI Devices,
 * Version 2.0 - May 2, 2020.
 *
 * UMP Driver Version 0.1 - June 28, 2022
 * UMP Driver Version 0.2 - Dec. 13, 2022
 *  - Splitting UMP Driver base from tud_midi
 * UMP Driver Version 0.3 - June 10, 2023
 *  - fixes issue with virtual cable ID and group IDs translation between
 *    USB MIDI 1.0 and USB MIDI 2.0
 * UMP Driver Version 0.4 - Sept. 4, 2023
 * - further fixes for multiple virtual cables when translating between
 *   USB MIDI 1.0 and USB MIDI 2.0. Remove dependance on external libraries.
 * UMP Driver Version 0.5 - Sept. 18, 2023
 * - bug fixes, USB MIDI 1.0 SYSEX on USB IN and USB OUT.
 * UMP Driver Version 1.0 - Sept. 26, 2024
 * - handling of USB MIDI 1.0 SYSEX translation
 * - Update of driver to latest tinyUSB implementation requirements
 *
 * The driver is backwards compatible with USB MIDI 1.0 if connected to an
 * operating system or other USB Hosting that does not support USB MIDI 2.0.
 * The driver does not currently support CIN 0xF, Single Byte as no known
 * operating system hosting is expected to send to device as CIN 0xf. If
 * CIN of 0xf is required, the implementer is welcome to contribute this
 * added processing.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.

 */

#include "tusb_option.h"

#if (TUSB_OPT_DEVICE_ENABLED && CFG_TUD_UMP)

//--------------------------------------------------------------------+
// INCLUDE
//--------------------------------------------------------------------+
#include "device/usbd.h"
#include "device/usbd_pvt.h"

#include "ump_device.h"

//--------------------------------------------------------------------+
// APP SPECIFIC DRIVERS
//--------------------------------------------------------------------+
#define TUSB_NUM_APP_DRIVERS 1  // defines number of app drivers
usbd_class_driver_t const tusb_app_drivers[TUSB_NUM_APP_DRIVERS] = {
  {
#if TUSB_VERSION_MAJOR == 0 && TUSB_VERSION_MINOR > 16
    "UMP",
#endif
    umpd_init,            // Driver init function
#if TUSB_VERSION_MAJOR == 0 && TUSB_VERSION_MINOR > 16
    umpd_deinit,          // Driver deinit function
#endif
    umpd_reset,           // Driver reset function
    umpd_open,            // Driver open function
    umpd_control_xfer_cb, // Driver control transfer callback function
    umpd_xfer_cb,         // Driver transfer callback function
    NULL                  // Driver sof function
  }
};

/**
 * @brief Routine to load app specific drivers
 * This routine is used for tinyUSB to associate to external application specific
 * drivers. This will be used until ump_device is included in the standard set of
 * tinyUSB drivers.
 * 
 * @param driver_count  Set with number of app specific drivers 
 * @return usbd_class_driver_t const* return pointer to structure
 */
usbd_class_driver_t const* usbd_app_driver_get_cb(uint8_t* driver_count)
{
  *driver_count = TUSB_NUM_APP_DRIVERS;

  return tusb_app_drivers;
}

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF
//--------------------------------------------------------------------+
typedef struct
{
  uint8_t buffer[4];
  uint8_t index;
}midid_stream_t;

typedef struct
{
  uint8_t   wordCount;
  union ump_device
  {
    uint32_t  umpWords[4];
    uint8_t   umpBytes[sizeof(uint32_t)*4];
  } umpData;
} UMP_PACKET, *PUMP_PACKET;

//
// Structure to aid in UMP SYSEX to USB MIDI 1.0
//
#define SYSEX_BS_RB_SIZE 16
typedef struct UMP_TO_MIDI1_SYSEX_t
{
    bool    inSysex;
    uint8_t sysexBS[SYSEX_BS_RB_SIZE];
    uint8_t usbMIDI1Tail;
    uint8_t usbMIDI1Head;
} UMP_TO_MIDI1_SYSEX;

#define MAX_NUM_GROUPS_CABLES 16

typedef struct
{
  uint8_t itf_num;
  uint8_t ep_in;
  uint8_t ep_out;

  bool midi1IsInSysex[MAX_NUM_GROUPS_CABLES];
  UMP_TO_MIDI1_SYSEX midi1OutSysex[MAX_NUM_GROUPS_CABLES];

  /*------------- From this point, data is not cleared by bus reset -------------*/
  // FIFO
  tu_fifo_t rx_ff;                            // reference to rx fifo
  tu_fifo_t tx_ff;                            // reference to tx fifo
  uint8_t rx_ff_buf[CFG_TUD_UMP_RX_BUFSIZE];  // storage buffer for rx fifo
  uint8_t tx_ff_buf[CFG_TUD_UMP_TX_BUFSIZE];  // storage buffer for tx fifo

  #if CFG_FIFO_MUTEX
  osal_mutex_def_t rx_ff_mutex;               // mutex for rx fifo if needed
  osal_mutex_def_t tx_ff_mutex;               // mutex for tx fifo if needed
  #endif

  // Endpoint Transfer buffer
  CFG_TUSB_MEM_ALIGN uint8_t epout_buf[CFG_TUD_UMP_EP_BUFSIZE]; // temp endpoint storage buffer
  CFG_TUSB_MEM_ALIGN uint8_t epin_buf[CFG_TUD_UMP_EP_BUFSIZE];  // temp endpoint storage buffer

  // Selected Interface
  uint8_t ump_interface_selected;             // for interface seclection - needed for USB MIDI 2.0 (UMP)

} umpd_interface_t;

// Default Group Terminal Block Descriptor
static uint8_t default_ump_group_terminal_blk_desc[] =
{
  // header
  5, // bLength
  MIDI_CS_INTERFACE_GR_TRM_BLOCK,
  MIDI_GR_TRM_BLOCK_HEADER,
  U16_TO_U8S_LE(sizeof(midi2_cs_interface_desc_group_terminal_blocks_t)), // wTotalLength

  // block
  13, // bLength
  MIDI_CS_INTERFACE_GR_TRM_BLOCK,
  MIDI_GR_TRM_BLOCK,
  1,          // bGrpTrmBlkID
  0x00,       // bGrpTrmBlkType: bi-directional
  0x00,       // nGroupTrm
  1,          // nNumGroupTrm
  0,          // iBlockItem: no string
  0x00,       // bMIDIProtocol: Unknown (Use MIDI-CI)
  0x00, 0x00, // wMaxInputBandwidth: Unknown or Not Fixed
  0x00, 0x00  // wMaxOutputBandwidth: Unknown or Not Fixed
};

#define ITF_MEM_RESET_SIZE   offsetof(umpd_interface_t, rx_ff)

//--------------------------------------------------------------------+
// INTERNAL OBJECT & FUNCTION DECLARATION
//--------------------------------------------------------------------+
CFG_TUSB_MEM_SECTION umpd_interface_t _umpd_itf[CFG_TUD_UMP];

extern "C"
{
bool     tud_USBMIDI1ToUMP    (uint32_t usbMidi1Pkt, bool* pbIsInSysex, PUMP_PACKET umpPkt);

bool tud_ump_n_mounted (uint8_t itf)
{
  umpd_interface_t* ump = &_umpd_itf[itf];
  return ump->ep_in && ump->ep_out;
}

static void _prep_out_transaction (umpd_interface_t* p_ump)
{
  uint8_t const rhport = TUD_OPT_RHPORT;
  uint16_t available = tu_fifo_remaining(&p_ump->rx_ff);

  // Prepare for incoming data but only allow what we can store in the ring buffer.
  // TODO Actually we can still carry out the transfer, keeping count of received bytes
  // and slowly move it to the FIFO when read().
  // This pre-check reduces endpoint claiming
  TU_VERIFY(available >= sizeof(p_ump->epout_buf), );

  // claim endpoint
  TU_VERIFY(usbd_edpt_claim(rhport, p_ump->ep_out), );

  // fifo can be changed before endpoint is claimed
  available = tu_fifo_remaining(&p_ump->rx_ff);

  if ( available >= sizeof(p_ump->epout_buf) )  {
    usbd_edpt_xfer(rhport, p_ump->ep_out, p_ump->epout_buf, sizeof(p_ump->epout_buf));
  }else
  {
    // Release endpoint since we don't make any transfer
    usbd_edpt_release(rhport, p_ump->ep_out);
  }
}

//--------------------------------------------------------------------+
// READ API
//--------------------------------------------------------------------+
uint32_t tud_ump_n_available(uint8_t itf)
{
  umpd_interface_t* ump = &_umpd_itf[itf];

  // is amount in fifo / 4 for 32 bit word count
  return tu_fifo_count(&ump->rx_ff) / 4;
}

/**
 * @brief return if MIDI UMP is enabled.
*
 * @param itf       interface number
 * @return bool true if enabled
 */
uint8_t tud_alt_setting( uint8_t itf) { 
    umpd_interface_t* ump = &_umpd_itf[itf];
    return ump->ump_interface_selected;
}

/**
 * @brief Process data read from USB OUT Stream and process into UMP data.
 * Read 32bit data words from USB stream, convert if necessary from USB MIDI 1.0
 * to UMP or pass UMP packets.
 *
 * @param itf       interface number
 * @param pkts      Array of 32 bit formatted UMP packet data (BE Formatted)
 * @param numAvail  Number of 32 bit UMP words that are available in handle
 *                  to populate. Note to accomodate possible SYSEX, needs to be at least 2
 *                  for 64bit size UMP Packet.
 * @return uint16_t Number of 32 bit UMP words populated in handle
 */
uint16_t tud_ump_read( uint8_t itf, uint32_t *pkts, uint16_t numAvail )
{
  umpd_interface_t* ump = &_umpd_itf[itf];
  uint16_t numRead = 0;
  uint16_t numProcessed = 0;

  static UMP_PACKET umpPacket;

  TU_VERIFY(ump->ep_out);

  // Determine if MIDI 1
  if (!ump->ump_interface_selected)
  {
    // Always look for enough space to process in a SYSEX message
    while ((numAvail - numProcessed) >= 2)
    {
      // Get next word from USB
      uint32_t readWord;
      if (tu_fifo_read_n(&ump->rx_ff, (void *)&readWord, sizeof(uint32_t)) != sizeof(uint32_t) )
      {
        goto END_READ;
      }
      numProcessed++;
      //readWord = RtlUlongByteSwap(readWord);

      if (readWord)
      {
        uint8_t *pBuffer = (uint8_t *)&readWord;
        uint8_t cbl_num = (pBuffer[0] & 0xf0) >> 4;
        UMP_PACKET pkt;
        if (tud_USBMIDI1ToUMP(readWord, &ump->midi1IsInSysex[cbl_num], &pkt))
        {
          for (uint8_t count = 0; count < pkt.wordCount; count++)
          {
            pkts[numRead++] = pkt.umpData.umpWords[count];
          }
        }
      }
    }
  }
  else
  {
    uint8_t umpBuffer[4];
    // Read in as much data as possible
    while (numRead < numAvail &&
      tu_fifo_read_n(&ump->rx_ff, umpBuffer, 4) == 4)
    {
      pkts[numRead++] = *(uint32_t *)umpBuffer;
    }
  }

END_READ:
  _prep_out_transaction(ump);

  return numRead;
}

//--------------------------------------------------------------------+
// WRITE API
//--------------------------------------------------------------------+

uint32_t tud_ump_n_writeable(uint8_t itf)
{
  umpd_interface_t* ump = &_umpd_itf[itf];

  // is amount in fifo / 4 for 32 bit word count
  return tu_fifo_remaining(&ump->tx_ff) / 4;
}

static uint32_t write_flush(umpd_interface_t* ump)
{
  // No data to send
  if ( !tu_fifo_count(&ump->tx_ff) ) return 0;

  uint8_t const rhport = TUD_OPT_RHPORT;

  // skip if previous transfer not complete
  TU_VERIFY( usbd_edpt_claim(rhport, ump->ep_in), 0 );

  uint16_t count = tu_fifo_read_n(&ump->tx_ff, ump->epin_buf, CFG_TUD_UMP_EP_BUFSIZE);

  if (count)
  {
    TU_ASSERT( usbd_edpt_xfer(rhport, ump->ep_in, ump->epin_buf, count), 0 );
    return count;
  }else
  {
    // Release endpoint since we don't make any transfer
    usbd_edpt_release(rhport, ump->ep_in);
    return 0;
  }
}

/**
 * @brief Process data write for USB IN Stream to host device.
 * Will write up to the number of UMP packets provided to the USB data stream.
 * If required, will convert to USB MIDI 1.0 stream format.
 * NOTE: This routine will translate to a single USB endpoint data message. Therefore
 * for optimization, it is suggested to group 32 bit UMP Words as much as possible.
 * Depending on if Full Speed or High Speed, the single transfer will be 64 bytes
 * or 512 bytes respectively - meaning 16 or 128 UMP words per transfer.
 *
 * @param itf       interface number
 * @param words     pointer to 32 bit formatted UMP data arrray
 * @param numWords   number of 32 bit UMP packets to try to write
 * @return uint16_t number of packets written
 */
uint16_t tud_ump_write( uint8_t itf, uint32_t *words, uint16_t numWords )
{
  umpd_interface_t* ump = &_umpd_itf[itf];
  TU_VERIFY(ump->ep_out);

  uint16_t    numProcessed = 0;
  uint8_t     *pBuffer = (uint8_t *)words;
  bool        bEnterSysex;
  bool        bEndSysex;
  uint8_t     numberBytes;
  uint8_t     sysexStatus;
  static UMP_PACKET  umpPacket;
  static UMP_PACKET  umpWritePacket; // used as storage to translate to USB MIDI 1.0

  // As long as there is data to process and room to write into fifo
  while (numProcessed < numWords)
  {
        // Process UMP Packet
    // Convert to USB MIDI 1.0?
    if ( ump->ump_interface_selected != 1 )
    {
      umpPacket.wordCount = 0;

      // Determine size of UMP packet based on message type
      //umpPacket.umpData.umpWords[0] = RtlUlongByteSwap(words[numProcessed]);
      umpPacket.umpData.umpWords[0] = words[numProcessed];

      switch (umpPacket.umpData.umpBytes[0] & UMP_MT_MASK)
      {
        case UMP_MT_UTILITY:
        case UMP_MT_SYSTEM:
        case UMP_MT_MIDI1_CV:
        case UMP_MT_RESERVED_6:
        case UMP_MT_RESERVED_7:
          umpPacket.wordCount = 1;
          break;

        case UMP_MT_DATA_64:
        case UMP_MT_MIDI2_CV:
        case UMP_MT_RESERVED_8:
        case UMP_MT_RESERVED_9:
        case UMP_MT_RESERVED_A:
          umpPacket.wordCount = 2;
          break;

        case UMP_MT_RESERVED_B:
        case UMP_MT_RESERVED_C:
          umpPacket.wordCount = 3;
          break;

        case UMP_MT_DATA_128:
        case UMP_MT_FLEX_128:
        case UMP_MT_STREAM_128:
        case UMP_MT_RESERVED_E:
          umpPacket.wordCount = 4;
          break;

        default:
          // Unhandled or corrupt data, force to move on
          numProcessed++;
          continue;
      }

      // Confirm have enough data for full packet
      if ((numWords - numProcessed) < umpPacket.wordCount)
      {
        // If not, let system populate more
        goto exitWrite;
      }
      // Get rest of words if needed for UMP Packet
      for (int count = 1; count < umpPacket.wordCount; count++)
      {
        umpPacket.umpData.umpWords[count] = words[numProcessed + count];
      }

      // Now that we have full UMP packet, need to convert to USB MIDI 1.0 format
      uint8_t cbl_num = umpPacket.umpData.umpBytes[0] & UMP_GROUP_MASK; // if used, cable num is group block num

      uint8_t mtVal = umpPacket.umpData.umpBytes[0] & UMP_MT_MASK;
      switch (mtVal)
      {
        case UMP_MT_SYSTEM:  // System Common messages
          umpWritePacket.wordCount = 1; // All types are single USB UMP 1.0 message
          // Now need to determine number of bytes for CIN
          switch (umpPacket.umpData.umpBytes[1])
          {
            case UMP_SYSTEM_TUNE_REQ:
            case UMP_SYSTEM_TIMING_CLK:
            case UMP_SYSTEM_START:
            case UMP_SYSTEM_CONTINUE:
            case UMP_SYSTEM_STOP:
            case UMP_SYSTEM_ACTIVE_SENSE:
            case UMP_SYSTEM_RESET:
            case UMP_SYSTEM_UNDEFINED_F4:
            case UMP_SYSTEM_UNDEFINED_F5:
            case UMP_SYSTEM_UNDEFINED_F9:
            case UMP_SYSTEM_UNDEFINED_FD:
              umpWritePacket.umpData.umpBytes[0] = (cbl_num << 4) | MIDI_CIN_SYSEX_END_1BYTE;
              break;

            case UMP_SYSTEM_MTC:
            case UMP_SYSTEM_SONG_SELECT:
              umpWritePacket.umpData.umpBytes[0] = (cbl_num << 4) | MIDI_CIN_SYSCOM_2BYTE;
              break;

            case UMP_SYSTEM_SONG_POS_PTR:
              umpWritePacket.umpData.umpBytes[0] = (cbl_num << 4) | MIDI_CIN_SYSCOM_3BYTE;
              break;

            default:
              umpWritePacket.wordCount = 0;
              break;
          }

          // Copy over actual data
          for (int count = 1; count < 4; count++)
          {
            umpWritePacket.umpData.umpBytes[count] = umpPacket.umpData.umpBytes[count];
          }
          break;

        case UMP_MT_MIDI1_CV:
          umpWritePacket.wordCount = 1;
          umpWritePacket.umpData.umpBytes[0] = (cbl_num << 4) | ((umpPacket.umpData.umpBytes[1] & 0xf0) >> 4);
          for (int count = 1; count < 4; count++)
          {
            umpWritePacket.umpData.umpBytes[count] = umpPacket.umpData.umpBytes[count];
          }
          break;

        case UMP_MT_DATA_64:
          bEnterSysex = false;
          bEndSysex = false;

          umpWritePacket.wordCount = 0;

          // Determine if sysex will end after this message
          switch (umpPacket.umpData.umpBytes[1] & UMP_SYSEX7_STATUS_MASK)
          {
            case UMP_SYSEX7_COMPLETE:
              bEnterSysex = true;
            case UMP_SYSEX7_END:
              bEndSysex = true;
              break;

            case UMP_SYSEX7_START:
              bEnterSysex = true;
            default:
              bEndSysex = false;
              break;
          }

          if (bEnterSysex)
          {
            // Determine if believed already in Sysex and if so, reset converter
            if (ump->midi1OutSysex[cbl_num].inSysex)
            {
              ump->midi1OutSysex[cbl_num].inSysex = false;
            }
          }

          if (bEnterSysex && !ump->midi1OutSysex[cbl_num].inSysex)
          {
            ump->midi1OutSysex[cbl_num].usbMIDI1Head = 0;
            ump->midi1OutSysex[cbl_num].usbMIDI1Tail = 0;
            ump->midi1OutSysex[cbl_num].inSysex = true;
          }

          uint8_t byteStream[SYSEX_BS_RB_SIZE];
          sysexStatus = (umpPacket.umpData.umpBytes[1] >> 4);
          numberBytes = 0;

          if (sysexStatus <= 1 && numberBytes < SYSEX_BS_RB_SIZE)
          {
            byteStream[numberBytes++] = MIDI_STATUS_SYSEX_START;
          }
          for (uint8_t count = 0; count < (umpPacket.umpData.umpBytes[1] & 0xf); count++)
          {
            if (numberBytes < SYSEX_BS_RB_SIZE)
            {
              byteStream[numberBytes++] = umpPacket.umpData.umpBytes[2 + count];
            }
          }
          if ((sysexStatus == 0 || sysexStatus == 3) && numberBytes < SYSEX_BS_RB_SIZE)
          {
            byteStream[numberBytes++] = MIDI_STATUS_SYSEX_END;
          }

          // Move into sysex circular buffer queue
          for (uint8_t count = 0; count < numberBytes; count++)
          {
            ump->midi1OutSysex[cbl_num].sysexBS[ump->midi1OutSysex[cbl_num].usbMIDI1Head++]
                = byteStream[count];
            ump->midi1OutSysex[cbl_num].usbMIDI1Head %= SYSEX_BS_RB_SIZE;
          }

          // How many bytes available in BS
          numberBytes = (ump->midi1OutSysex[cbl_num].usbMIDI1Head > ump->midi1OutSysex[cbl_num].usbMIDI1Tail)
              ? ump->midi1OutSysex[cbl_num].usbMIDI1Head - ump->midi1OutSysex[cbl_num].usbMIDI1Tail
              : (SYSEX_BS_RB_SIZE - ump->midi1OutSysex[cbl_num].usbMIDI1Tail) + ump->midi1OutSysex[cbl_num].usbMIDI1Head;

          while (numberBytes && umpWritePacket.wordCount < 4)
          {
            umpWritePacket.umpData.umpWords[umpWritePacket.wordCount] = 0;

            if (numberBytes > 2)
            {
              uint8_t *pumpBytes = (uint8_t*) & umpWritePacket.umpData.umpWords[umpWritePacket.wordCount];
              for (uint8_t count = 0; count < 3; count++)
              {
                pumpBytes[count + 1] =
                  ump->midi1OutSysex[cbl_num].sysexBS[ump->midi1OutSysex[cbl_num].usbMIDI1Tail++];
                ump->midi1OutSysex[cbl_num].usbMIDI1Tail %= SYSEX_BS_RB_SIZE;
                numberBytes--;
              }
              // Mark cable number and CIN for start / continue SYSEX in USB MIDI 1.0 format
              if (bEndSysex && !numberBytes)
              {
                pumpBytes[0] = (uint8_t)(cbl_num << 4) | MIDI_CIN_SYSEX_END_3BYTE;
              }
              else
              {
                pumpBytes[0] = (uint8_t)(cbl_num << 4) | MIDI_CIN_SYSEX_START;
              }
            }
            else
            {
              // If less than two and we have a word to populate, check if the end of sysex
              if (bEndSysex)
              {
                // Process bytes
                uint8_t* pumpBytes = (uint8_t*)&umpWritePacket.umpData.umpWords[umpWritePacket.wordCount];
                uint8_t count;
                for (count = 0; numberBytes; count++)
                {
                  pumpBytes[count + 1] =
                    ump->midi1OutSysex[cbl_num].sysexBS[ump->midi1OutSysex[cbl_num].usbMIDI1Tail++];
                  ump->midi1OutSysex[cbl_num].usbMIDI1Tail %= SYSEX_BS_RB_SIZE;
                  numberBytes--;
                }
                // Mark cable number and CIN for start / continue SYSEX in USB MIDI 1.0 format
                switch (count)
                {
                  case 1:
                    pumpBytes[0] = (uint8_t)(cbl_num << 4) | MIDI_CIN_SYSEX_END_1BYTE;
                    break;

                  case 2:
                  default:
                    pumpBytes[0] = (uint8_t)(cbl_num << 4) | MIDI_CIN_SYSEX_END_2BYTE;
                    break;
                }
              }
              else
              {
                break;
              }
            }
            umpWritePacket.wordCount++;
          }
          break;

        default:
          // Not handled so ignore
          numProcessed += umpPacket.wordCount;    // ignore this UMP packet as corrupted
          umpWritePacket.wordCount = 0;
      }

      if (umpWritePacket.wordCount)
      {
        numProcessed += umpPacket.wordCount;
      
        tu_fifo_write_n(&ump->tx_ff, (void*)&umpWritePacket.umpData.umpBytes[0],
          umpWritePacket.wordCount*4);
      }
    }
    else
    {
      // Should already be UMP formatted, so just pass along
      uint16_t numAvailable = tu_fifo_remaining(&ump->tx_ff) / 4;
      numProcessed = (numAvailable < numWords) ? numAvailable : numWords;
      tu_fifo_write_n(&ump->tx_ff, (void*)words, numProcessed*4);
    }
  }

exitWrite :

  // Make sure fifo is pushed to endpoint
  write_flush(ump);

  // Let calling routine know how many words processed
  return numProcessed;
}

//--------------------------------------------------------------------+
// USBD Driver API
//--------------------------------------------------------------------+
void umpd_init(void)
{
  tu_memclr((void *)_umpd_itf, sizeof(_umpd_itf));

  for(uint8_t i=0; i<CFG_TUD_UMP; i++)
  {
    umpd_interface_t* ump = &_umpd_itf[i];

    // config fifo
    tu_fifo_config(&ump->rx_ff, ump->rx_ff_buf, CFG_TUD_UMP_RX_BUFSIZE, 1, false);
    tu_fifo_config(&ump->tx_ff, ump->tx_ff_buf, CFG_TUD_UMP_TX_BUFSIZE, 1, false);

    // Default select the first interface
    ump->ump_interface_selected = 0;

    #if CFG_FIFO_MUTEX
    tu_fifo_config_mutex(&ump->rx_ff, NULL, osal_mutex_create(&ump->rx_ff_mutex));
    tu_fifo_config_mutex(&ump->tx_ff, osal_mutex_create(&ump->tx_ff_mutex), NULL);
    #endif
  }
}

bool umpd_deinit(void)
{
  // Need to add to handle cleanup of multiple instances

  return true;
}

void umpd_reset(uint8_t rhport)
{
  (void) rhport;

  for(uint8_t i=0; i<CFG_TUD_UMP; i++)
  {
    umpd_interface_t* ump = &_umpd_itf[i];
    tu_memclr((void *)ump, ITF_MEM_RESET_SIZE);
    tu_fifo_clear(&ump->rx_ff);
    tu_fifo_clear(&ump->tx_ff);

    // Reset any current processing condition
    for(uint8_t grp=0; grp<MAX_NUM_GROUPS_CABLES; grp++)
    {
      ump->midi1IsInSysex[grp] = false;
      ump->midi1OutSysex[grp].inSysex = 0;
      ump->midi1OutSysex[grp].usbMIDI1Head = 0;
      ump->midi1OutSysex[grp].usbMIDI1Tail = 0;
    }

    ump->ump_interface_selected = 0;
  }
}

uint16_t umpd_open(uint8_t rhport, tusb_desc_interface_t const * desc_itf, uint16_t max_len)
{
  // 1st Interface is Audio Control v1
  TU_VERIFY(TUSB_CLASS_AUDIO               == desc_itf->bInterfaceClass    &&
            AUDIO_SUBCLASS_CONTROL         == desc_itf->bInterfaceSubClass &&
            AUDIO_FUNC_PROTOCOL_CODE_UNDEF == desc_itf->bInterfaceProtocol, 0);

  uint16_t drv_len = tu_desc_len(desc_itf);
  uint8_t const * p_desc = tu_desc_next(desc_itf);

  // Skip Class Specific descriptors
  while ( TUSB_DESC_CS_INTERFACE == tu_desc_type(p_desc) && drv_len <= max_len )
  {
    drv_len += tu_desc_len(p_desc);
    p_desc   = tu_desc_next(p_desc);
  }

  // 2nd Interface is MIDI Streaming
  TU_VERIFY(TUSB_DESC_INTERFACE == tu_desc_type(p_desc), 0);
  tusb_desc_interface_t const * desc_ump = (tusb_desc_interface_t const *) p_desc;

  TU_VERIFY(TUSB_CLASS_AUDIO               == desc_ump->bInterfaceClass    &&
            AUDIO_SUBCLASS_MIDI_STREAMING  == desc_ump->bInterfaceSubClass &&
            AUDIO_FUNC_PROTOCOL_CODE_UNDEF == desc_ump->bInterfaceProtocol, 0);

  // Find available interface
  umpd_interface_t * p_ump = NULL;
  for(uint8_t i=0; i<CFG_TUD_UMP; i++)
  {
    if ( _umpd_itf[i].ep_in == 0 && _umpd_itf[i].ep_out == 0 )
    {
      p_ump = &_umpd_itf[i];
      break;
    }
  }
  TU_ASSERT(p_ump);

  p_ump->itf_num = desc_ump->bInterfaceNumber;
  (void) p_ump->itf_num;

  // next descriptor
  drv_len += tu_desc_len(p_desc);
  p_desc   = tu_desc_next(p_desc);

  // Find and open endpoint descriptors
  uint8_t found_endpoints = 0;
  while ( (found_endpoints < desc_ump->bNumEndpoints) && (drv_len <= max_len)  )
  {
    if ( TUSB_DESC_ENDPOINT == tu_desc_type(p_desc) )
    {
      TU_ASSERT(usbd_edpt_open(rhport, (tusb_desc_endpoint_t const *) p_desc), 0);
      uint8_t ep_addr = ((tusb_desc_endpoint_t const *) p_desc)->bEndpointAddress;

      if (tu_edpt_dir(ep_addr) == TUSB_DIR_IN)
      {
        p_ump->ep_in = ep_addr;
      } else {
        p_ump->ep_out = ep_addr;
      }

      // Class Specific MIDI Stream endpoint descriptor
      drv_len += tu_desc_len(p_desc);
      p_desc   = tu_desc_next(p_desc);

      found_endpoints += 1;
    }

    drv_len += tu_desc_len(p_desc);
    p_desc   = tu_desc_next(p_desc);
  }

  // Finish off any further class specific definitions for interface
  while ( TUSB_DESC_CS_INTERFACE == tu_desc_type(p_desc) && drv_len <= max_len )
  {
    drv_len += tu_desc_len(p_desc);
    p_desc   = tu_desc_next(p_desc);
  }

  // See if there is an alternate interface for UMP USB MIDI 2.0
  if ( TUSB_DESC_INTERFACE == tu_desc_type(p_desc) ) drv_len = max_len;

  // Prepare for incoming data
  _prep_out_transaction(p_ump);

  return drv_len;
}

// Invoked when a control transfer occurred on an interface of this class
// Driver response accordingly to the request and the transfer stage (setup/data/ack)
// return false to stall control endpoint (e.g unsupported request)
bool umpd_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const * request)
{
  // nothing to with DATA & ACK stage
  if (stage != CONTROL_STAGE_SETUP) return true;

  umpd_interface_t* ump = &_umpd_itf[rhport];

  switch ( request->bRequest )
  {
    case TUSB_REQ_SET_INTERFACE :
      // Set the interface type for driver operaiton
      ump->ump_interface_selected = tu_u16_low(request->wValue);

      // As we are using still bulk transfer, no reason to close and open endpoints, however we should clear
      // fifos to start from scratch
      tu_fifo_clear(&ump->rx_ff);
      tu_fifo_clear(&ump->tx_ff);

      // invoke set interface callback if available
      if (tud_ump_set_itf_cb) tud_ump_set_itf_cb(tu_u16_low(request->wIndex), ump->ump_interface_selected);

      tud_control_status(rhport, request); // send a status zero length packet

      return true;

    case TUSB_REQ_GET_DESCRIPTOR :
      if ( request->wValue == 0x2601 ) //0x26 - CS_GR_TRM_BLOCK 0x01 - alternate interface setting
      {
        // invoke midi class specific get request callback if available
        if (tud_ump_get_req_itf_cb && tud_ump_get_req_itf_cb(rhport, request)) return true;

        // return default group block descriptor if not handled by client code
        uint16_t length = request->wLength;
        if ( length > sizeof( default_ump_group_terminal_blk_desc ) )
        {
          length = sizeof( default_ump_group_terminal_blk_desc );
        }
        tud_control_xfer(rhport, request, (void *)default_ump_group_terminal_blk_desc, length );
        return true;
      }
      else
        return false;

    default :
      return false;
  }
}

bool umpd_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes)
{
  (void) result;
  (void) rhport;

  uint8_t itf;
  umpd_interface_t* p_ump;

  // Identify which interface to use
  for (itf = 0; itf < CFG_TUD_UMP; itf++)
  {
    p_ump = &_umpd_itf[itf];
    if ( ( ep_addr == p_ump->ep_out ) || ( ep_addr == p_ump->ep_in ) ) break;
  }
  TU_ASSERT(itf < CFG_TUD_UMP);

  // receive new data
  if ( ep_addr == p_ump->ep_out )
  {
    tu_fifo_write_n(&p_ump->rx_ff, p_ump->epout_buf, xferred_bytes);

    // invoke receive callback if available
    if (tud_ump_rx_cb) tud_ump_rx_cb(itf);

    // prepare for next
    // TODO for now ep_out is not used by public API therefore there is no race condition,
    // and does not need to claim like ep_in
    _prep_out_transaction(p_ump);
  }
  else if ( ep_addr == p_ump->ep_in )
  {
    if (0 == write_flush(p_ump))
    {
      // If there is no data left, a ZLP should be sent if
      // xferred_bytes is multiple of EP size and not zero
      if ( !tu_fifo_count(&p_ump->tx_ff) && xferred_bytes && (0 == (xferred_bytes % CFG_TUD_UMP_EP_BUFSIZE)) )
      {
        if ( usbd_edpt_claim(rhport, p_ump->ep_in) )
        {
          usbd_edpt_xfer(rhport, p_ump->ep_in, NULL, 0);
        }
      }
    }
  }

  return true;
}

bool
tud_USBMIDI1ToUMP(
    uint32_t        usbMidi1Pkt,
    bool*           pbIsInSysex,
    PUMP_PACKET     umpPkt
)
/*++
Routine Description:

    Helper routine to handle conversion of USB MIDI 1.0 32 bit word packet
    to UMP formatted packet. The routine will only populate UMP message
    types 1: System, 2: MIDI 1.0 Channel Voice, and 3: 64 bit data (SYSEX)
    messages. It is rsponsibility of other routines to convert between
    MIDI 2.0 Channel voice if needed.

  NOTE: This routine was refined from the USB MIDI 2.0 Host Driver developed as open
  source to be included in Windows by the Association of Musical Electronics Industry.

Copyright 2023 Association of Musical Electronics Industry
Copyright 2023 Microsoft
Driver source code developed by AmeNote. Some components Copyright 2023 AmeNote Inc.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

Arguments:

    usbMidi1Pkt:    The USB MIDI 1.0 packet presented as a 32 bit word
    pbIsInSysex:    Reference to boolean variable where to store if processing
                    a SYSEX message or not. Note that calling funciton is
                    responsible to maintain this state for each USB MIDI 1.0
                    data stream to process. The intial value should be false.
    umpPkt:         Reference to data location to create UMP packet into. Note
                    that for optimizations, the data beyond wordCount is not
                    cleared, therefore calling function should not process beyond
                    wordCount or zero any additional dataspace.

Return Value:

    bool:           Indicates true of umpPkt is ready, false otherwise.

--*/
{
    // Checked passed parameters
    if (!usbMidi1Pkt || !pbIsInSysex || !umpPkt)
    {
        return false;
    }

    uint8_t* pBuffer = (uint8_t*)&usbMidi1Pkt;
    umpPkt->wordCount = 0;

    // Determine packet cable number from group
    uint8_t cbl_num = (pBuffer[0] & 0xf0) >> 4;

    // USB MIDI 1.0 uses a CIN as an identifier for packet, grab CIN.
    uint8_t code_index = pBuffer[0] & 0x0f;

    // Handle special case of single byte data
    if (code_index == MIDI_CIN_1BYTE_DATA && (pBuffer[1] & 0x80))
    {
        switch (pBuffer[1])
        {
        case UMP_SYSTEM_TUNE_REQ:
        case UMP_SYSTEM_TIMING_CLK:
        case UMP_SYSTEM_START:
        case UMP_SYSTEM_CONTINUE:
        case UMP_SYSTEM_STOP:
        case UMP_SYSTEM_ACTIVE_SENSE:
        case UMP_SYSTEM_RESET:
        case UMP_SYSTEM_UNDEFINED_F4:
        case UMP_SYSTEM_UNDEFINED_F5:
        case UMP_SYSTEM_UNDEFINED_F9:
        case UMP_SYSTEM_UNDEFINED_FD:
            code_index = MIDI_CIN_SYSEX_END_1BYTE;
            break;

        default:
            break;
        }
    }

    uint8_t firstByte = 1;
    uint8_t lastByte = 4;
    uint8_t copyPos;

    switch (code_index)
    {
    case MIDI_CIN_SYSEX_START: // or continue
 
        if (!*pbIsInSysex)
        {
            // SYSEX Start means first byte should be SYSEX start
            if (pBuffer[1] != MIDI_STATUS_SYSEX_START) return false;
            firstByte = 2;
            lastByte = 4;

            // As this is start of SYSEX, need to set status to indicate so and copy 2 bytes of data
            // as first byte of MIDI_STATUS_SYSEX_START
            umpPkt->umpData.umpBytes[1] = UMP_SYSEX7_START | 2;

            // Set that in SYSEX
            *pbIsInSysex = true;
        }
        else
        {
            firstByte = 1;
            lastByte = 4;

            // As this is in SYSEX, then need to indicate continue
            umpPkt->umpData.umpBytes[1] = UMP_SYSEX7_CONTINUE | 3;
        }

        // Capture Cable number
        umpPkt->umpData.umpBytes[0] = UMP_MT_DATA_64 | cbl_num;   // Message Type and group

        umpPkt->wordCount = 2;
        // Transfer in bytes
        copyPos = firstByte;
        for (uint8_t count = 2; count < 8; count++)
        {
            umpPkt->umpData.umpBytes[count] = (copyPos < lastByte)
                ? pBuffer[copyPos++] : 0x00;
        }
        break;

    case MIDI_CIN_SYSEX_END_1BYTE: // or single byte System Common
        // Determine if a system common
        if ( (pBuffer[1] & 0x80) // most significant bit set and not sysex ending
            && (pBuffer[1] != MIDI_STATUS_SYSEX_END))
        {
            umpPkt->umpData.umpBytes[0] = UMP_MT_SYSTEM | cbl_num;
            umpPkt->umpData.umpBytes[1] = pBuffer[1];
            firstByte = 1;
            lastByte = 1;
            goto COMPLETE_1BYTE;
        }

        umpPkt->umpData.umpBytes[0] = UMP_MT_DATA_64 | cbl_num;

        // Determine if complete based on if currently in SYSEX
        if (*pbIsInSysex)
        {
            if (pBuffer[1] != MIDI_STATUS_SYSEX_END) return false;
            umpPkt->umpData.umpBytes[1] = UMP_SYSEX7_END | 0;
            *pbIsInSysex = false; // we are done with SYSEX
            firstByte = 1;
            lastByte = 1;
        }
        else
        {
            // should not get here
            return false;
        }

COMPLETE_1BYTE:
        umpPkt->wordCount = 2;
        // Transfer in bytes
        copyPos = firstByte;
        for (uint8_t count = 2; count < 8; count++)
        {
            umpPkt->umpData.umpBytes[count] = (copyPos < lastByte)
                ? pBuffer[copyPos++] : 0x00;
        }
        break;

    case MIDI_CIN_SYSEX_END_2BYTE:
        umpPkt->umpData.umpBytes[0] = UMP_MT_DATA_64 | cbl_num;

        // Determine if complete based on if currently in SYSEX
        if (*pbIsInSysex)
        {
            if (pBuffer[2] != MIDI_STATUS_SYSEX_END) return false;
            umpPkt->umpData.umpBytes[1] = UMP_SYSEX7_END | 1;
            *pbIsInSysex = false; // we are done with SYSEX
            firstByte = 1;
            lastByte = 2;
        }
        else
        {
            umpPkt->umpData.umpBytes[1] = UMP_SYSEX7_COMPLETE | 0;
            *pbIsInSysex = false; // we are done with SYSEX
            firstByte = 1;
            lastByte = 1;
        }

        umpPkt->wordCount = 2;
        // Transfer in bytes
        copyPos = firstByte;
        for (uint8_t count = 2; count < 8; count++)
        {
            umpPkt->umpData.umpBytes[count] = (copyPos < lastByte)
                ? pBuffer[copyPos++] : 0x00;
        }
        break;

    case MIDI_CIN_SYSEX_END_3BYTE:
        umpPkt->umpData.umpBytes[0] = UMP_MT_DATA_64 | cbl_num;

        // Determine if complete based on if currently in SYSEX
        if (*pbIsInSysex)
        {
            if (pBuffer[3] != MIDI_STATUS_SYSEX_END) return false;
            umpPkt->umpData.umpBytes[1] = UMP_SYSEX7_END | 2;
            *pbIsInSysex = false; // we are done with SYSEX
            firstByte = 1;
            lastByte = 3;
        }
        else
        {
            if (pBuffer[1] != MIDI_STATUS_SYSEX_START || pBuffer[3] != MIDI_STATUS_SYSEX_END) return false;
            umpPkt->umpData.umpBytes[1] = UMP_SYSEX7_COMPLETE | 1;
            *pbIsInSysex = false; // we are done with SYSEX
            firstByte = 2;
            lastByte = 3;
        }

        umpPkt->wordCount = 2;
        // Transfer in bytes
        copyPos = firstByte;
        for (uint8_t count = 2; count < 8; count++)
        {
            umpPkt->umpData.umpBytes[count] = (copyPos < lastByte)
                ? pBuffer[copyPos++] : 0x00;
        }
        break;

        // MIDI1 Channel Voice Messages
    case MIDI_CIN_NOTE_ON:
    case MIDI_CIN_NOTE_OFF:
    case MIDI_CIN_POLY_KEYPRESS:
    case MIDI_CIN_CONTROL_CHANGE:
    case MIDI_CIN_PROGRAM_CHANGE:
    case MIDI_CIN_CHANNEL_PRESSURE:
    case MIDI_CIN_PITCH_BEND_CHANGE:
        umpPkt->umpData.umpBytes[0] = UMP_MT_MIDI1_CV | cbl_num; // message type 2
        *pbIsInSysex = false; // ensure we end any current sysex packets, other layers need to handle error

        // Copy in rest of data
        for (int count = 1; count < 4; count++)
        {
            umpPkt->umpData.umpBytes[count] = pBuffer[count];
        }

        umpPkt->wordCount = 1;
        break;

    case MIDI_CIN_SYSCOM_2BYTE:
    case MIDI_CIN_SYSCOM_3BYTE:
        umpPkt->umpData.umpBytes[0] = UMP_MT_SYSTEM | cbl_num;
        for (int count = 1; count < 4; count++)
        {
            umpPkt->umpData.umpBytes[count] = pBuffer[count];
        }
        umpPkt->wordCount = 1;
        break;

    case MIDI_CIN_MISC:
    case MIDI_CIN_CABLE_EVENT:
        // These are reserved for future use and will not be translated, drop data with no processing
    default:
        // Not valid USB MIDI 1.0 transfer or NULL, skip
        return false;
    }

    return true;
}

} // extern "C"
#endif
