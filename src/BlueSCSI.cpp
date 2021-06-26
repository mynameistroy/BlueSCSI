/*  
 *  BlueSCSI
 *  Copyright (c) 2021  Eric Helgeson, Androda
 *  
 *  This file is free software: you may copy, redistribute and/or modify it  
 *  under the terms of the GNU General Public License as published by the  
 *  Free Software Foundation, either version 2 of the License, or (at your  
 *  option) any later version.  
 *  
 *  This file is distributed in the hope that it will be useful, but  
 *  WITHOUT ANY WARRANTY; without even the implied warranty of  
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU  
 *  General Public License for more details.  
 *  
 *  You should have received a copy of the GNU General Public License  
 *  along with this program.  If not, see https://github.com/erichelgeson/bluescsi.  
 *  
 * This file incorporates work covered by the following copyright and  
 * permission notice:  
 *  
 *     Copyright (c) 2019 komatsu   
 *  
 *     Permission to use, copy, modify, and/or distribute this software  
 *     for any purpose with or without fee is hereby granted, provided  
 *     that the above copyright notice and this permission notice appear  
 *     in all copies.  
 *  
 *     THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL  
 *     WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED  
 *     WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE  
 *     AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR  
 *     CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS  
 *     OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,  
 *     NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN  
 *     CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.  
 */

#include <Arduino.h> // For Platform.IO
#include <SdFat.h>

#include "BlueSCSI.h"
#include "scsi_phase.h"
#include "scsi_cmds.h"

#ifdef USE_STM32_DMA
#warning "warning USE_STM32_DMA"
#endif

#if USE_DB2ID_TABLE
/* DB to SCSI-ID translation table */
static const byte db2scsiid[256]={
  0xff,
  0,
  1,1,
  2,2,2,2,
  3,3,3,3,3,3,3,3,
  4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,
  5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,
  6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,
  6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,
  7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
  7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
  7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
  7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7
};
#endif


uint8_t       m_senseKey = 0;               // Sense key
uint16_t      m_additional_sense_code = 0;  // ASC/ASCQ 
volatile bool m_isBusReset = false;         // Bus reset

byte          scsi_id_mask = 0;           // Mask list of responding SCSI IDs
byte          m_id;                   // Currently responding SCSI-ID
byte          m_lun;                  // Logical unit number currently responding
byte          m_sts;                  // Status byte
byte          m_msg;                  // Message bytes
byte          m_buf[MAX_BLOCKSIZE+1] = {0xff}; // General purpose buffer + overrun fetch
int           m_msc;
byte          m_msb[256];             // Command storage bytes

/* Configurable options */
int           m_scsi_delay = 0;           // SCSI timing delay, default is none

BUS_PHASE phase = BUS_FREE;

unsigned _SCSI_BUS_PHASE = PHASE_BUSFREE;

byte phase_log[2048] = {0};
unsigned phase_cycles[2048] = {0};
SdFs SD;
FsFile scsi_file;
unsigned cycle_buffer_length = 2048 * 4;
unsigned phase_idx = 0;


void flashError(uint8_t error);
void onBusReset(void);
void onBSY(void);
void onATN(void);
void onSEL(void);
void onCD(void);
void onIO(void);
void onMSG(void);
void onREQ(void);
void onACK(void);
void onTick(void);

/*
 * Initialization.
 *  Initialize the bus and set the PIN orientation
 */
void setup()
{
  // PA15 / PB3 / PB4 Cannot be used
  // JTAG Because it is used for debugging.
  // Comment out for Debugging in PlatformIO
  // disableDebugPorts();

  // Serial initialization
#if DEBUG
  Serial.begin(9600);
  while (!Serial) {}
#endif

  // PIN initialization
  gpio_mode(LED, GPIO_OUTPUT_OD);
  gpio_write(LED, low);

  //GPIO(SCSI BUS)Initialization
  //Port setting register (lower)
//  GPIOB->regs->CRL |= 0x000000008; // SET INPUT W/ PUPD on PAB-PB0
  //Port setting register (upper)
  //GPIOB->regs->CRH = 0x88888888; // SET INPUT W/ PUPD on PB15-PB8
//  GPIOB->regs->ODR = 0x0000FF00; // SET PULL-UPs on PB15-PB8
  // DB and DP are input modes
  SCSI_DB_INPUT()

  // Input port
  gpio_mode(ATN, GPIO_INPUT_PU);
  gpio_mode(BSY, GPIO_INPUT_PU);
  gpio_mode(ACK, GPIO_INPUT_PU);
  gpio_mode(RST, GPIO_INPUT_PU);
  gpio_mode(SEL, GPIO_INPUT_PU);
  // Output port
  gpio_mode(MSG, GPIO_INPUT_PU);
  gpio_mode(CD,  GPIO_INPUT_PU);
  gpio_mode(REQ, GPIO_INPUT_PU);
  gpio_mode(IO,  GPIO_INPUT_PU);
  // Turn off the output port
  // SCSI_TARGET_INACTIVE()
  
  systick_enable();

    // clock = 36MHz , about 4Mbytes/sec
  if(!SD.begin(SD1_CONFIG)) {
#if DEBUG
    Serial.println("SD initialization failed!");
#endif
  }

  scsi_file = SD.open("SCSI.DMP", O_WRONLY | O_CREAT | O_TRUNC | O_BINARY);

  //Occurs when the RST pin state changes from HIGH to LOW
  attachInterrupt(PIN_MAP[RST].gpio_bit, onBusReset, FALLING); 
}

volatile unsigned pin_status = 0;
volatile unsigned old_pin_status = 0;

#define BIT_BSY 0
#define BIT_SEL 1
#define BIT_ATN 2
#define BIT_MSG 3
#define BIT_CD 4
#define BIT_IO 5
#define BIT_REQ 6
#define BIT_ACK 7

#define TOGGLE_PIN(x) old_pin_status++; pin_status ^= x

/*
 * Bus reset interrupt.
 */
void onBusReset(void)
{
#if SCSI_SELECT == 1
  // SASI I / F for X1 turbo has RST pulse write cycle +2 clock ==
  // I can't filter because it only activates about 1.25us
  {{
#else
  if(isHigh(gpio_read(RST))) {
    // delayMicroseconds(20);
    for(int i = 0; i < 50; i++) {};
    if(isHigh(gpio_read(RST))) {
#endif  
  // BUS FREE is done in the main process
//      gpio_mode(MSG, GPIO_OUTPUT_OD);
//      gpio_mode(CD,  GPIO_OUTPUT_OD);
//      gpio_mode(REQ, GPIO_OUTPUT_OD);
//      gpio_mode(IO,  GPIO_OUTPUT_OD);
      // Should I enter DB and DBP once?
      SCSI_DB_INPUT()
      m_isBusReset = true;
      PHASE_CHANGE(PHASE_BUSFREE);
      pin_status = 0;
      // SCSI_TARGET_INACTIVE() // Turn off BSY, REQ, MSG, CD, IO output
    }
  }
}

/*
 * Read by handshake.
 */
byte readHandshake(void)
{
  // Wait for Target to request
  while( !SCSI_IN(vREQ)) { if(m_isBusReset) return 0; }
  while( !SCSI_IN(vACK)) { if(m_isBusReset) return 0; }

  // Read the byte
  byte r = READ_DATA_BUS();

  // Wait for Initiator to ack

   return r;  
}

/*
 * Write with a handshake.
 */
inline void writeHandshake(byte d)
{
  GPIOB->regs->BSRR = db_bsrr[d]; // setup DB,DBP (160ns)
  SCSI_DB_OUTPUT() // (180ns)
  // ACK.Fall to DB output delay 100ns(MAX)  (DTC-510B)
  SCSI_OUT(vREQ,inactive) // setup wait (30ns)
  SCSI_OUT(vREQ,inactive) // setup wait (30ns)
  SCSI_OUT(vREQ,inactive) // setup wait (30ns)
  SCSI_OUT(vREQ,active)   // (30ns)
  //while(!SCSI_IN(vACK)) { if(m_isBusReset){ SCSI_DB_INPUT() return; }}
  while(!m_isBusReset && !SCSI_IN(vACK));
  // ACK.Fall to REQ.Raise delay 500ns(typ.) (DTC-510B)
  GPIOB->regs->BSRR = DBP(0xff);  // DB=0xFF , SCSI_OUT(vREQ,inactive)
  // REQ.Raise to DB hold time 0ns
  SCSI_DB_INPUT() // (150ns)
  while( SCSI_IN(vACK)) { if(m_isBusReset) return; }
}

/*
 * Data in phase.
 *  Send len bytes of data array p.
 */
void writeDataPhase(int len, const byte* p)
{
  LOGN("DATAIN PHASE");
  SCSI_OUT(vMSG,inactive) //  gpio_write(MSG, low);
  SCSI_OUT(vCD ,inactive) //  gpio_write(CD, low);
  SCSI_OUT(vIO ,  active) //  gpio_write(IO, high);
  for (int i = 0; i < len; i++) {
    if(m_isBusReset) {
      return;
    }
    writeHandshake(p[i]);
  }
}

/*
 * Data out phase.
 *  len block read
 */
void readDataPhase(int len, byte* p)
{
  LOGN("DATAOUT PHASE");
  SCSI_OUT(vMSG,inactive) //  gpio_write(MSG, low);
  SCSI_OUT(vCD ,inactive) //  gpio_write(CD, low);
  SCSI_OUT(vIO ,inactive) //  gpio_write(IO, low);
  for(uint32_t i = 0; i < len; i++)
    p[i] = readHandshake();
}

/*
 * MsgIn2.
 */
void MsgIn2(int msg)
{
  LOGN("MsgIn2");
  SCSI_OUT(vMSG,  active) //  gpio_write(MSG, high);
  SCSI_OUT(vCD ,  active) //  gpio_write(CD, high);
  SCSI_OUT(vIO ,  active) //  gpio_write(IO, high);
  writeHandshake(msg);
}

/*
 * MsgOut2.
 */
void MsgOut2()
{
  LOGN("MsgOut2");
  SCSI_OUT(vMSG,  active) //  gpio_write(MSG, high);
  SCSI_OUT(vCD ,  active) //  gpio_write(CD, high);
  SCSI_OUT(vIO ,inactive) //  gpio_write(IO, low);
  m_msb[m_msc] = readHandshake();
  m_msc++;
  m_msc %= 256;
}

/*
 * Main loop.
 */


void loop() 
{
  //int msg = 0;
  m_msg = 0;
  byte scsi_id;
  byte len = 0;

  
  if(!scsi_file.isOpen())
  {
    while(1)
    {

    delayMicroseconds(10000);
    LOGN("Open failed?????");
    LOGN(scsi_file.getError());
    }
  }
  
  while(1)
  {
  old_pin_status = pin_status;
  pin_status =
    SCSI_IN(vBSY) << BIT_BSY |
    SCSI_IN(vSEL) << BIT_SEL |
    SCSI_IN(vATN) << BIT_ATN |
    SCSI_IN(vMSG) << BIT_MSG |
    SCSI_IN(vCD) << BIT_CD |
    SCSI_IN(vIO) << BIT_IO
    ;
    if(old_pin_status != pin_status)
    {
      //LOGHEX(pin_status); LOG(":");
      phase_cycles[phase_idx] = systick_get_count();
      phase_log[phase_idx++] = pin_status;
    }
    else
    {
      if(phase_idx)
      {
      LOGN("Breather");
      scsi_file.write(phase_log, phase_idx);
      scsi_file.write(phase_cycles, (phase_idx) * 4);
      scsi_file.sync();
      phase_idx = 0;
      }
    }
    if(phase_idx == 2048)
    {
      LOGN("Flush");
      scsi_file.write(phase_log, phase_idx);
      scsi_file.write(phase_cycles, cycle_buffer_length);
      scsi_file.sync();
      phase_idx = 0;
    }
#if 0
    if(counter)
    {
      for(i = 0; i < counter; i++)
      {
        LOGHEX(log[i]); LOG(":");
      }
      LOGN("");
      counter = 0;
    }
    BusFree:
    // Wait until RST = H, BSY = H, SEL = L
    if( PHASE_CHECK(PHASE_BUSFREE) && !SCSI_IN(vSEL) && SCSI_IN(vBSY));
    {  
      PHASE_CHANGE(PHASE_SELECTION);
      log[counter++] = PHASE_SELECTION;
      log[counter++] = READ_DATA_BUS();
      PHASE_CHANGE(MESSAGE_OUT);
    }
    
    if( PHASE_CHECK(PHASE_MESSAGE_OUT) )
    {
      log[counter++] = PHASE_MESSAGE_OUT;
      PHASE_CHANGE(PHASE_BUSFREE);
      goto BusFree;
    }


    if(!SCSI_IN(vMSG) && !SCSI_IN(vCD) && !SCSI_IN(vIO) && !SCSI_IN(vBSY))
    {
      PHASE_CHANGE(PHASE_BUSFREE);
      if(!m_isBusReset)
      {
        m_isBusReset = true;
        log[counter++] = PHASE_BUSFREE;
        continue;
      }
      goto BusFree;
    }

  // INFORMATION TRANSFER PHASE
    if(& SCSI_IN(vBSY) && !SCSI_IN(vSEL))
    {
      // Information Transfer Phases

      byte info_xfer_phase = 0;
      
      info_xfer_phase = (byte)(SCSI_IN(vMSG) << 2);
      info_xfer_phase |= (byte)(SCSI_IN(vCD) << 1);
      info_xfer_phase |= (byte)(SCSI_IN(vIO));

      switch(info_xfer_phase)
      {
        case INFO_XFER_DATA_OUT:
          phase = DATA_OUT;
          break;

        case INFO_XFER_DATA_IN:
          phase = DATA_IN;
          break;
        
        case INFO_XFER_COMMAND:
          phase = COMMAND;
          break;

        case INFO_XFER_MSG_OUT:
          phase = MESSAGE_OUT;
          break;

        case INFO_XFER_MSG_IN:
          phase = MESSAGE_IN;
          break;

        default:
          LOG("Unknown Info Xfer Phase:"); LOGN(info_xfer_phase);
      }

      if(SCSI_IN(vATN))
      {
          phase = MESSAGE_OUT;
      }
    }

    switch(phase)
    {
      case DATA_OUT:    onDataOut(len); break;
      case DATA_IN:     onDataIn(len); break;
      case COMMAND:     len = onCommand(); break;
      case MESSAGE_IN:  onMessageIn(); break;
      case MESSAGE_OUT: onMessageOut(); break;
    }
  
  
    //LOGN("Sts");
    SCSI_OUT(vMSG,inactive) // gpio_write(MSG, low);
    SCSI_OUT(vCD ,  active) // gpio_write(CD, high);
    SCSI_OUT(vIO ,  active) // gpio_write(IO, high);
    writeHandshake(m_sts);
    if(m_isBusReset) {
      goto BusFree;
    }

    //LOGN("MsgIn");
    SCSI_OUT(vMSG,  active) // gpio_write(MSG, high);
    SCSI_OUT(vCD ,  active) // gpio_write(CD, high);
    SCSI_OUT(vIO ,  active) // gpio_write(IO, high);
    writeHandshake(m_msg);
    #endif

  } // while(1)
}

/*
 * IO read.
 */

inline byte readIO(void)
{
  // Port input data register
  uint32_t ret = GPIOB->regs->IDR;
  byte bret = (byte)((~ret)>>8);
#if READ_PARITY_CHECK
  if((db_bsrr[bret]^ret)&1)
    m_sts |= 0x01; // parity error
#endif

  return bret;
}

void flashError(uint8_t error)
{
  while(true) {
    for(uint8_t i = 0; i < error; i++) {
      gpio_write(LED, high);
      delay(250);
      gpio_write(LED, low);
      delay(250);
    }
    delay(3000);
  }
}

#if 0
/* 
 * Data in phase.
 *  Send len block while reading from SD card.
 */
void writeDataPhaseSD(uint32_t adds, uint32_t len)
{
  LOGN("DATAIN PHASE(SD)");
  uint32_t pos = adds * m_img->m_blocksize;
  m_img->m_file.seek(pos);

  SCSI_OUT(vMSG,inactive) //  gpio_write(MSG, low);
  SCSI_OUT(vCD ,inactive) //  gpio_write(CD, low);
  SCSI_OUT(vIO ,  active) //  gpio_write(IO, high);

  for(uint32_t i = 0; i < len; i++) {
      // Asynchronous reads will make it faster ...
    m_img->m_file.read(m_buf, m_img->m_blocksize);

#if READ_SPEED_OPTIMIZE

//#define REQ_ON() SCSI_OUT(vREQ,active)
#define REQ_ON() (*db_dst = BITMASK(vREQ)<<16)
#define FETCH_SRC()   (src_byte = *srcptr++)
#define FETCH_BSRR_DB() (bsrr_val = bsrr_tbl[src_byte])
#define REQ_OFF_DB_SET(BSRR_VAL) *db_dst = BSRR_VAL
#define WAIT_ACK_ACTIVE()   while(!m_isBusReset && !SCSI_IN(vACK))
#define WAIT_ACK_INACTIVE() do{ if(m_isBusReset) return; }while(SCSI_IN(vACK)) 

    SCSI_DB_OUTPUT()
    register byte *srcptr= m_buf;                 // Source buffer
    register byte *endptr= m_buf +  m_img->m_blocksize; // End pointer

    /*register*/ byte src_byte;                       // Send data bytes
    register const uint32_t *bsrr_tbl = db_bsrr;  // Table to convert to BSRR
    register uint32_t bsrr_val;                   // BSRR value to output (DB, DBP, REQ = ACTIVE)
    register volatile uint32_t *db_dst = &(GPIOB->regs->BSRR); // Output port

    // prefetch & 1st out
    FETCH_SRC();
    FETCH_BSRR_DB();
    REQ_OFF_DB_SET(bsrr_val);
    // DB.set to REQ.F setup 100ns max (DTC-510B)
    // Maybe there should be some weight here
    //　WAIT_ACK_INACTIVE();
    do{
      // 0
      REQ_ON();
      FETCH_SRC();
      FETCH_BSRR_DB();
      WAIT_ACK_ACTIVE();
      // ACK.F  to REQ.R       500ns typ. (DTC-510B)
      REQ_OFF_DB_SET(bsrr_val);
      WAIT_ACK_INACTIVE();
      // 1
      REQ_ON();
      FETCH_SRC();
      FETCH_BSRR_DB();
      WAIT_ACK_ACTIVE();
      REQ_OFF_DB_SET(bsrr_val);
      WAIT_ACK_INACTIVE();
      // 2
      REQ_ON();
      FETCH_SRC();
      FETCH_BSRR_DB();
      WAIT_ACK_ACTIVE();
      REQ_OFF_DB_SET(bsrr_val);
      WAIT_ACK_INACTIVE();
      // 3
      REQ_ON();
      FETCH_SRC();
      FETCH_BSRR_DB();
      WAIT_ACK_ACTIVE();
      REQ_OFF_DB_SET(bsrr_val);
      WAIT_ACK_INACTIVE();
      // 4
      REQ_ON();
      FETCH_SRC();
      FETCH_BSRR_DB();
      WAIT_ACK_ACTIVE();
      REQ_OFF_DB_SET(bsrr_val);
      WAIT_ACK_INACTIVE();
      // 5
      REQ_ON();
      FETCH_SRC();
      FETCH_BSRR_DB();
      WAIT_ACK_ACTIVE();
      REQ_OFF_DB_SET(bsrr_val);
      WAIT_ACK_INACTIVE();
      // 6
      REQ_ON();
      FETCH_SRC();
      FETCH_BSRR_DB();
      WAIT_ACK_ACTIVE();
      REQ_OFF_DB_SET(bsrr_val);
      WAIT_ACK_INACTIVE();
      // 7
      REQ_ON();
      FETCH_SRC();
      FETCH_BSRR_DB();
      WAIT_ACK_ACTIVE();
      REQ_OFF_DB_SET(bsrr_val);
      WAIT_ACK_INACTIVE();
    }while(srcptr < endptr);
    SCSI_DB_INPUT()
#else
    for(int j = 0; j < m_img->m_blocksize; j++) {
      if(m_isBusReset) {
        return;
      }
      writeHandshake(m_buf[j]);
    }
#endif
  }
}
/*
 * Data out phase.
 *  Write to SD card while reading len block.
 */
void readDataPhaseSD(uint32_t adds, uint32_t len)
{
  LOGN("DATAOUT PHASE(SD)");
  uint32_t pos = adds * m_img->m_blocksize;
  m_img->m_file.seek(pos);
  SCSI_OUT(vMSG,inactive) //  gpio_write(MSG, low);
  SCSI_OUT(vCD ,inactive) //  gpio_write(CD, low);
  SCSI_OUT(vIO ,inactive) //  gpio_write(IO, low);
  for(uint32_t i = 0; i < len; i++) {
#if WRITE_SPEED_OPTIMIZE
  register byte *dstptr= m_buf;
	register byte *endptr= m_buf + m_img->m_blocksize;

    for(dstptr=m_buf;dstptr<endptr;dstptr+=8) {
      dstptr[0] = readHandshake();
      dstptr[1] = readHandshake();
      dstptr[2] = readHandshake();
      dstptr[3] = readHandshake();
      dstptr[4] = readHandshake();
      dstptr[5] = readHandshake();
      dstptr[6] = readHandshake();
      dstptr[7] = readHandshake();
      if(m_isBusReset) {
        return;
      }
    }
#else
    for(int j = 0; j <  m_img->m_blocksize; j++) {
      if(m_isBusReset) {
        return;
      }
      m_buf[j] = readHandshake();
    }
#endif
    m_img->m_file.write(m_buf, m_img->m_blocksize);
  }
  m_img->m_file.flush();
}
#endif