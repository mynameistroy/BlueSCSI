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

#define DEBUG            0      // 0:No debug information output
                                // 1: Debug information output available
#define VERSION "1.0-jokker-refactor"
#define LOG_FILENAME "LOG.txt"

#include "BlueSCSI.h"
#include "scsi_cmds.h"
#include "scsi_sense.h"
#include "scsi_status.h"

#ifdef USE_STM32_DMA
#warning "warning USE_STM32_DMA"
#endif




FsFile LOG_FILE;
SdFs SD;

HDDIMG  img[NUM_SCSIID][NUM_SCSILUN]; // Maximum number

uint8_t       m_senseKey = 0;               // Sense key
uint16_t      m_additional_sense_code = 0;  // ASC/ASCQ 
volatile bool m_isBusReset = false;         // Bus reset

byte          scsi_id_mask;           // Mask list of responding SCSI IDs
byte          m_id;                   // Currently responding SCSI-ID
byte          m_lun;                  // Logical unit number currently responding
byte          m_sts;                  // Status byte
byte          m_msg;                  // Message bytes
HDDIMG       *m_img;                  // HDD image for current SCSI-ID, LUN
byte          m_buf[MAX_BLOCKSIZE] = {0xff}; // General purpose buffer + overrun fetch
unsigned      m_msc;
byte          m_msb[256];             // Command storage bytes

/* Configurable options */
unsigned      m_scsi_delay = 0;           // SCSI timing delay, default is none

static byte onUnimplemented(const byte *arg)
{
  // does nothing!
  if(Serial)
  {
    Serial.print("Unimplemented SCSI command: ");
    Serial.println(arg[0], 16);
  }

  m_senseKey = SCSI_SENSE_ILLEGAL_REQUEST;
  m_additional_sense_code = SCSI_ASC_INVALID_OPERATION_CODE;
  return SCSI_STATUS_CHECK_CONDITION;
}

static byte onNOP(const byte *arg) { return 0; }

// function table
byte (*scsi_command_table[0xff])(const byte *);

// scsi command functions
static byte onRequestSense(const byte *cmd);
static byte onRead6(const byte *cmd);
static byte onRead10(const byte *cmd);
static byte onWrite6(const byte *cmd);
static byte onWrite10(const byte *cmd);
static byte onInquiry(const byte *cmd);
static byte onModeSense(const byte *cmd);
static byte onReadCapacity(const byte *cmd);
static byte onModeSense(const byte *cmd);
static byte onModeSelect(const byte *cmd);
static byte onReadTOC(const byte *cmd);
static byte onReadDVDStructure(const byte *cmd);
static byte onReadDiscInformation(const byte *cmd);


static void onBusReset(void);
static void initFileLog(void);
static void finalizeFileLog(void);

static uint32_t MSFtoLBA(const byte *msf);
static void LBAtoMSF(const uint32_t lba, byte *msf);

static void flashError(const unsigned error)
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

// If config file exists, read the first three lines and copy the contents.
// File must be well formed or you will get junk in the SCSI Vendor fields.
static void readSCSIDeviceConfig(HDDIMG *h) {
  FsFile config_file = SD.open("scsi-config.txt", O_RDONLY);
  SCSI_INQUIRY_DATA *inquiry_block = &(h->inquiry_block);
  char buffer[64] = {0};
  String key, value;
  unsigned len = 0;

  if (!config_file.isOpen()) {
    return;
  }

  while (config_file.fgets(buffer, sizeof(buffer)))
  {
    key = strtok(buffer, "=");
    if (!key) continue;
    value = strtok(NULL, "=");
    if (!value) continue;

    if(key.equalsIgnoreCase("vendor"))
    {
      len = sizeof(inquiry_block->vendor) < value.length() ? sizeof(inquiry_block->vendor) : value.length() - 1;
      memset(inquiry_block->vendor, 0, sizeof(inquiry_block->vendor));
      memcpy(inquiry_block->vendor, value.c_str(), len);
      LOG_FILE.print("SCSI VENDOR: ");
      LOG_FILE.write(inquiry_block->vendor, len);
      LOG_FILE.println();
    }
    else if(key.equalsIgnoreCase("product"))
    {
      len = sizeof(inquiry_block->product) < value.length() ? sizeof(inquiry_block->product) : value.length() - 1;
      memset(inquiry_block->product, 0, sizeof(inquiry_block->product));
      memcpy(inquiry_block->product, value.c_str(), len);
      LOG_FILE.print("SCSI PRODUCT: ");
      LOG_FILE.write(inquiry_block->product, len);
      LOG_FILE.println();
    }
    else if(key.equalsIgnoreCase("version"))
    {
      len = sizeof(inquiry_block->revision) < value.length() ? sizeof(inquiry_block->revision) : value.length() - 1;
      memset(inquiry_block->revision, 0, sizeof(inquiry_block->revision));
      memcpy(inquiry_block->revision, value.c_str(), len);
      LOG_FILE.print("SCSI REVISION: ");
      LOG_FILE.write(inquiry_block->revision, len);
      LOG_FILE.println();
    }
    else if(key.equalsIgnoreCase("delay"))
    {
      m_scsi_delay = value.toInt();
      LOG_FILE.print("SCSI Delay: ");
      if(m_scsi_delay < 0 || m_scsi_delay > 1500)
      {
        m_scsi_delay = 0;
        LOG_FILE.println("INVALID");
      }
      else
      {
        LOG_FILE.println(value.c_str());
      }
    }
  }
  LOG_FILE.sync();
}

// read SD information and print to logfile
static void readSDCardInfo()
{
  cid_t sd_cid;

  if(SD.card()->readCID(&sd_cid))
  {
    LOG_FILE.print("Sd MID:");
    LOG_FILE.print(sd_cid.mid, 16);
    LOG_FILE.print(" OID:");
    LOG_FILE.print(sd_cid.oid[0]);
    LOG_FILE.println(sd_cid.oid[1]);

    LOG_FILE.print("Sd Name:");
    LOG_FILE.print(sd_cid.pnm[0]);
    LOG_FILE.print(sd_cid.pnm[1]);
    LOG_FILE.print(sd_cid.pnm[2]);
    LOG_FILE.print(sd_cid.pnm[3]);
    LOG_FILE.println(sd_cid.pnm[4]);

    LOG_FILE.print("Sd Date:");
    LOG_FILE.print(sd_cid.mdt_month);
    LOG_FILE.print("/20"); // CID year is 2000 + high/low
    LOG_FILE.print(sd_cid.mdt_year_high);
    LOG_FILE.println(sd_cid.mdt_year_low);

    LOG_FILE.print("Sd Serial:");
    LOG_FILE.println(sd_cid.psn);
    LOG_FILE.sync();
  }
}

/*
 * Open HDD image file
 */

static bool ImageOpen(HDDIMG *h, const char *image_name)
{
  h->m_fileSize = 0;
  h->m_file = SD.open(image_name, O_RDWR);
  
  if(!h->m_file.isOpen())
  {
    return false;
  }

  h->m_fileSize = h->m_file.size();
  LOG_FILE.print("Imagefile: ");
  LOG_FILE.print(image_name);
  if(h->m_fileSize < 1)
  {
    LOG_FILE.println("FileSizeError");
    goto failed;
  }

  if(h->m_type == SCSI_TYPE_CDROM)
  {
    byte header[12] = {0};
    byte sync[12] = {0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0};

    LOG_FILE.print(" CDROM");

    if(!h->m_file.readBytes(header, sizeof(header)))
    {
      LOG_FILE.println("FileReadError");
      goto failed;
    }

    if(memcmp(sync, header, sizeof(header)) == 0)
    {

      // 00,FFx10,00, so it is presumed to be RAW format
      if(!h->m_file.readBytes(header, 4))
      {
        LOG_FILE.println("FileReadError");
        goto failed;
      }

      // Supports MODE1/2048 or MODE1/2352 only
      if(header[3] != 0x01)
      {
        LOG_FILE.println("UnsupportedISOType");
        goto failed;
      }

      h->m_raw = true;
      
      // Size must be a multiple of 2536 and less than 700MB
      if(h->m_fileSize % 0x930 || h->m_fileSize > 912579600)
      {
        LOG_FILE.println("InvalidISO");
        goto failed;
      }

      h->m_blockcount = h->m_fileSize / 0x930;
      h->m_blocksize = 0x930;
    }
    else
    {
      // Size must be a multiple of 2048 and less than 700MB
      if(h->m_fileSize % 0x800 || h->m_fileSize > 0x2bed5000)
      {
        LOG_FILE.println("InvalidISO");
        goto failed;
      }

      h->m_blockcount = h->m_fileSize >> 11;
      h->m_blocksize = 0x800;
    }
  }
  else
  {
    LOG_FILE.print(" HDD");
    h->m_blockcount = h->m_fileSize / h->m_blocksize;
  }

  // check blocksize dummy file
  LOG_FILE.print(" / ");
  LOG_FILE.print(h->m_fileSize);
  LOG_FILE.print("bytes / ");
  LOG_FILE.print(h->m_fileSize / 1024);
  LOG_FILE.print("KiB / ");
  LOG_FILE.print(h->m_fileSize / 1024 / 1024);
  LOG_FILE.println("MiB");
  
  return true; // File opened
  
  failed:
  h->m_file.close();
  h->m_fileSize = h->m_blocksize = 0; // no file
  
  return false;
}

/*
 * Initialization.
 *  Initialize the bus and set the PIN orientation
 */
void setup()
{
  SdFile root, file;

  for(unsigned i = 0xff; i != 0;)
  {
    scsi_command_table[i] = onUnimplemented;
    i = i - 1;
  }

  scsi_command_table[SCSI_TEST_UNIT_READY] = onNOP;
  scsi_command_table[SCSI_REZERO_UNIT] = onNOP;
  scsi_command_table[SCSI_FORMAT_UNIT] = onNOP;
  scsi_command_table[SCSI_REASSIGN_BLOCKS] = onNOP;
  scsi_command_table[SCSI_SEEK6] = onNOP;
  scsi_command_table[SCSI_SEEK10] = onNOP;
  scsi_command_table[SCSI_START_STOP_UNIT] = onNOP;
  scsi_command_table[SCSI_PREVENT_ALLOW_REMOVAL] = onNOP;
  scsi_command_table[SCSI_VERIFY10] = onNOP;

  scsi_command_table[SCSI_REQUEST_SENSE] = onRequestSense;
  scsi_command_table[SCSI_READ6] = onRead6;
  scsi_command_table[SCSI_READ10] = onRead10;
  scsi_command_table[SCSI_WRITE6] = onWrite6;
  scsi_command_table[SCSI_WRITE10] = onWrite10;
  scsi_command_table[SCSI_INQUIRY] = onInquiry;
  scsi_command_table[SCSI_MODE_SELECT6] = onModeSense;
  scsi_command_table[SCSI_READ_CAPACITY] = onReadCapacity;
  scsi_command_table[SCSI_MODE_SENSE6] =  onModeSense;
  scsi_command_table[SCSI_MODE_SENSE10] = onModeSense;
  scsi_command_table[SCSI_MODE_SELECT6] = onModeSelect;
  scsi_command_table[SCSI_MODE_SELECT10] = onModeSelect;
  scsi_command_table[SCSI_READ_TOC] = onReadTOC;
  scsi_command_table[SCSI_READ_DVD_STRUCTURE] = onReadDVDStructure;
  scsi_command_table[SCSI_READ_DISC_INFORMATION] = onReadDiscInformation;

  // PA15 / PB3 / PB4 Cannot be used
  // JTAG Because it is used for debugging.
  // Comment out for Debugging in PlatformIO
  // disableDebugPorts();

  // Serial initialization
#if DEBUG
  Serial.begin(9600);
  // while (!Serial);
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
  gpio_mode(MSG, GPIO_OUTPUT_OD);
  gpio_mode(CD,  GPIO_OUTPUT_OD);
  gpio_mode(REQ, GPIO_OUTPUT_OD);
  gpio_mode(IO,  GPIO_OUTPUT_OD);
  // Turn off the output port
  SCSI_TARGET_INACTIVE()

  //Occurs when the RST pin state changes from HIGH to LOW
  //attachInterrupt(PIN_MAP[RST].gpio_bit, onBusReset, FALLING);

  LED_ON();

  // clock = 36MHz , about 4Mbytes/sec
  if(!SD.begin(SD1_CONFIG)) {
#if DEBUG
    Serial.println("SD initialization failed!");
#endif
    flashError(ERROR_NO_SDCARD);
  }
  initFileLog();
  readSDCardInfo();

  //Sector data overrun byte setting
  m_buf[MAX_BLOCKSIZE] = 0xff; // DB0 all off,DBP off
  //HD image file open
  scsi_id_mask = 0x00;

  // Iterate over the root path in the SD card looking for candidate image files.
  
  root.open("/");
 
   while (1) {
    byte id, lun;
    char name[MAX_FILE_PATH+1];
    
    if (!file.openNext(&root, O_READ)) break;
    
    if(file.isDir()) {
          continue;
    }

    file.getName(name, MAX_FILE_PATH+1);
    file.close();
    String file_name = String(name);
    file_name.toLowerCase();
    
    id  = name[HDIMG_ID_POS] - '0';
    lun = name[HDIMG_LUN_POS] - '0';

    if(id < NUM_SCSIID && lun < NUM_SCSILUN) {
      m_img = &img[id][lun];
      m_img->m_blocksize = name[HDIMG_BLK_POS] - '0';
      
      if(file_name.startsWith("hd") && file_name.endsWith(".hda"))
      {
        m_img->m_type = SCSI_TYPE_HDD;
        
        switch(m_img->m_blocksize)
        {
          case 1:
            m_img->m_blocksize = 1024;
            break;
          case 2:
            m_img->m_blocksize = 256;
            break;
          default:
            m_img->m_blocksize = 512;
        }
      }
      else if(file_name.startsWith("cd") && file_name.endsWith(".iso"))
      {
        m_img->m_type = SCSI_TYPE_CDROM;
      } 
      else
      {
        LOG_FILE.print("Not a recognized image type ");
        LOG_FILE.println(name);
        LOG_FILE.sync();
        continue;
      }

      memset(&m_img->inquiry_block, 0, sizeof(m_img->inquiry_block));
      if(ImageOpen(m_img, name))
      {
        // Marked as a responsive ID
        scsi_id_mask |= 1<<id;

        switch(m_img->m_type)
        {
          case SCSI_TYPE_HDD:
          // default SCSI HDD
          m_img->inquiry_block.ansi_version = 1;
          m_img->inquiry_block.response_format = 1;
          m_img->inquiry_block.additional_length = 31;
          memcpy(m_img->inquiry_block.vendor, "QUANTUM", 7);
          memcpy(m_img->inquiry_block.product, "FIREBALL1", 9);
          memcpy(m_img->inquiry_block.revision, "1.0", 3);
          break;
          
          case SCSI_TYPE_CDROM:
          // default SCSI CDROM
          m_img->inquiry_block.peripheral_device_type = 5;
          m_img->inquiry_block.rmb = 1;
          m_img->inquiry_block.ansi_version = 1;
          m_img->inquiry_block.response_format = 1;
          m_img->inquiry_block.additional_length = 42;
          m_img->inquiry_block.sync = 1;
          memcpy(m_img->inquiry_block.vendor, "BLUESCSI", 8);
          memcpy(m_img->inquiry_block.product, "CD-ROM CDU-55S", 14);
          memcpy(m_img->inquiry_block.revision, "1.9a", 4);
          m_img->inquiry_block.release = 0x20;
          memcpy(m_img->inquiry_block.revision_date, "1995", 4);
          break;
        }

        readSCSIDeviceConfig(m_img);
      }
    }
  }
  root.close();


  // Error if there are 0 image files
  if(scsi_id_mask==0) {
    LOG_FILE.println("ERROR: No valid images found!");
    flashError(ERROR_FALSE_INIT);
  }

  finalizeFileLog();
  LED_OFF();
  //Occurs when the RST pin state changes from HIGH to LOW
  attachInterrupt(PIN_MAP[RST].gpio_bit, onBusReset, FALLING);
}

/*
 * Setup initialization logfile
 */
void initFileLog() {
  LOG_FILE = SD.open(LOG_FILENAME, O_WRONLY | O_CREAT | O_TRUNC);
  LOG_FILE.println("BlueSCSI <-> SD - https://github.com/erichelgeson/BlueSCSI");
  LOG_FILE.print("VERSION: ");
  LOG_FILE.println(VERSION);
  LOG_FILE.print("DEBUG:");
  LOG_FILE.print(DEBUG);
  LOG_FILE.print(" SCSI_SELECT:");
  LOG_FILE.print(SCSI_SELECT);
  LOG_FILE.print(" SDFAT_FILE_TYPE:");
  LOG_FILE.println(SDFAT_FILE_TYPE);
  LOG_FILE.print("SdFat version: ");
  LOG_FILE.println(SD_FAT_VERSION_STR);
  LOG_FILE.print("SdFat Max FileName Length: ");
  LOG_FILE.println(MAX_FILE_PATH);
  LOG_FILE.println("Initialized SD Card - lets go!");
  LOG_FILE.sync();
}

/*
 * Finalize initialization logfile
 */
void finalizeFileLog() {
  // View support drive map
  LOG_FILE.print("ID");
  for(int lun=0;lun<NUM_SCSILUN;lun++)
  {
    LOG_FILE.print(":LUN");
    LOG_FILE.print(lun);
  }
  LOG_FILE.println(":");
  //
  for(int id=0;id<NUM_SCSIID;id++)
  {
    LOG_FILE.print(" ");
    LOG_FILE.print(id);
    for(int lun=0;lun<NUM_SCSILUN;lun++)
    {
      HDDIMG *h = &img[id][lun];
      if( (lun<NUM_SCSILUN) && (h->m_file))
      {
        LOG_FILE.print((h->m_blocksize<1000) ? ": " : ":");
        LOG_FILE.print(h->m_blocksize);
      }
      else      
        LOG_FILE.print(":----");
    }
    LOG_FILE.println(":");
  }
  LOG_FILE.println("Finished initialization of SCSI Devices - Entering main loop.");
  LOG_FILE.sync();
  LOG_FILE.close();
}

/*
 * Initialization failed, blink 3x fast
 */
void onFalseInit(void)
{
  LOG_FILE.sync();
  while(true) {
    for(int i = 0; i < 3; i++) {
      gpio_write(LED, !gpio_read(LED));
      delay(250);
    }
    delay(3000);
  }
}

/*
 * No SC Card found, blink 5x fast
 */
void noSDCardFound(void)
{
  while(true) {
    for(int i = 0; i < 5; i++) {
      gpio_write(LED, !gpio_read(LED));
      delay(250);
    }
    delay(3000);
    LOGN("No SD card found");
  }
}

/*
 * Bus reset interrupt.
 */
static void onBusReset(void)
{

  if(isHigh(gpio_read(RST))) {
    asm("NOP");
    if(isHigh(gpio_read(RST))) {
  // BUS FREE is done in the main process
//      gpio_mode(MSG, GPIO_OUTPUT_OD);
//      gpio_mode(CD,  GPIO_OUTPUT_OD);
//      gpio_mode(REQ, GPIO_OUTPUT_OD);
//      gpio_mode(IO,  GPIO_OUTPUT_OD);
      // Should I enter DB and DBP once?
      SCSI_DB_INPUT()

      LOGN("BusReset!");
      m_isBusReset = true;
    }
  }
}

/*
 * Read by handshake.
 */
static inline byte readHandshake(void)
{
  SCSI_OUT(vREQ,active)
  //SCSI_DB_INPUT()
  while( ! SCSI_IN(vACK)) { if(m_isBusReset) return 0; }
  byte r = READ_DATA_BUS();
  SCSI_OUT(vREQ,inactive)
  while( SCSI_IN(vACK)) { if(m_isBusReset) return 0; }
  return r;  
}

/*
 * Write with a handshake.
 */
static inline void writeHandshake(byte d)
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
static void writeDataPhase(int len, const byte* p)
{
  LOGN("DATAIN PHASE");
 
  SCSI_PHASE_DATA_IN();

  for (int i = 0; i < len; i++) {
    if(m_isBusReset) {
      return;
    }
    writeHandshake(p[i]);
  }
}

/* 
 * Data in phase.
 *  Send len block while reading from SD card.
 */
static void writeDataPhaseSD(uint32_t adds, uint32_t len)
{
  LOGN("DATAIN PHASE(SD)");
  uint32_t pos = adds * m_img->m_blocksize;
  m_img->m_file.seek(pos);

  SCSI_PHASE_DATA_IN();

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
 *  len block read
 */
static void readDataPhase(unsigned len, byte* p)
{
  LOGN("DATAOUT PHASE");

  SCSI_PHASE_DATA_OUT();
  for(unsigned i = 0; i < len; i++)
    p[i] = readHandshake();
}

/*
 * Data out phase.
 *  Write to SD card while reading len block.
 */
static void readDataPhaseSD(uint32_t adds, uint32_t len)
{
  LOGN("DATAOUT PHASE(SD)");
  uint32_t pos = adds * m_img->m_blocksize;
  m_img->m_file.seek(pos);

  uint32_t buffer_ptr = 0;

  SCSI_PHASE_DATA_OUT();
  
  for(uint32_t i = 0; i < len; i++) {
#if WRITE_SPEED_OPTIMIZE
  register byte *dstptr= m_buf + buffer_ptr;
	register byte *endptr= dstptr + m_img->m_blocksize;

    for(;dstptr<endptr;dstptr+=8) {
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
    buffer_ptr += m_img->m_blocksize;
    if(buffer_ptr == sizeof(m_buf))
    {
      m_img->m_file.write(m_buf, sizeof(m_buf));
      m_img->m_file.flush();
      buffer_ptr = 0;
    }
  }
  if(buffer_ptr)
  {
      m_img->m_file.write(m_buf, buffer_ptr);
      m_img->m_file.flush();
  }
}

/*
 * INQUIRY command processing.
 */
static byte onInquiry(const byte *cmd)
{
  LOGN("onInquiry");
  // only write back what was asked for
  writeDataPhase(cmd[4], m_img->inquiry_block.raw);
  return SCSI_STATUS_GOOD;
}

/*
 * REQUEST SENSE command processing.
 */
static byte onRequestSense(const byte *cmd)
{
  LOGN("onRequestSense");
  byte buf[18] = {
    0x70,   //CheckCondition
    0,      //Segment number
    0x00,   //Sense key
    0, 0, 0, 0,  //information
    17 - 7 ,   //Additional data length
    0,
  };
  buf[12] = (byte)m_additional_sense_code >> 8;
  buf[13] = (byte)m_additional_sense_code;
  buf[2] = m_senseKey;
  m_senseKey = 0;
  m_additional_sense_code = 0;
  writeDataPhase(cmd[4] < 18 ? cmd[4] : 18, buf);
  return SCSI_STATUS_GOOD;
}

/*
 * READ CAPACITY command processing.
 */
static byte onReadCapacity(const byte *cmd)
{
  LOGN("onReadCapacity");
  uint8_t buf[8] = {
    m_img->m_blockcount >> 24, m_img->m_blockcount >> 16, m_img->m_blockcount >> 8, m_img->m_blockcount,
    m_img->m_blocksize >> 24, m_img->m_blocksize >> 16, m_img->m_blocksize >> 8, m_img->m_blocksize    
  };
  writeDataPhase(8, buf);
  return SCSI_STATUS_GOOD;
}

/*
 * READ6 / 10 Command processing.
 */
static byte onRead6(const byte *cmd)
{
  unsigned adds = (((uint32_t)cmd[1] & 0x1F) << 16) | ((uint32_t)cmd[2] << 8) | cmd[3];
  unsigned len = (cmd[4] == 0) ? 0x100 : cmd[4];
  /*
  LOGN("onRead6");
  LOG("-R ");
  LOGHEX(adds);
  LOG(":");
  LOGHEXN(len);
  */
  
  gpio_write(LED, high);
  writeDataPhaseSD(adds, len);
  gpio_write(LED, low);
  return SCSI_STATUS_GOOD;
}

static byte onRead10(const byte *cmd)
{
  unsigned adds = ((uint32_t)cmd[2] << 24) | ((uint32_t)cmd[3] << 16) | ((uint32_t)cmd[4] << 8) | cmd[5];
  unsigned len = ((uint32_t)cmd[7] << 8) | cmd[8];
  /*
  LOGN("onRead10");
  LOG("-R ");
  LOGHEX(adds);
  LOG(":");
  LOGHEXN(len);
  */

  gpio_write(LED, high);
  writeDataPhaseSD(adds, len);
  gpio_write(LED, low);
  return SCSI_STATUS_GOOD;
}

/*
 * WRITE6 / 10 Command processing.
 */
static byte onWrite6(const byte *cmd)
{
  unsigned adds = (((uint32_t)cmd[1] & 0x1F) << 16) | ((uint32_t)cmd[2] << 8) | cmd[3];
  unsigned len = (cmd[4] == 0) ? 0x100 : cmd[4];
  /*
  LOGN("onWrite6");
  LOG("-W ");
  LOGHEX(adds);
  LOG(":");
  LOGHEXN(len);
  */

  if(m_img->m_type == SCSI_TYPE_CDROM)
  {
    m_senseKey = SCSI_SENSE_HARDWARE_ERROR;
    m_additional_sense_code = SCSI_ASC_WRITE_PROTECTED; // Write Protect
    return SCSI_STATUS_CHECK_CONDITION;
  }

  gpio_write(LED, high);
  readDataPhaseSD(adds, len);
  gpio_write(LED, low);
  return SCSI_STATUS_GOOD;
}

static byte onWrite10(const byte *cmd)
{
  unsigned adds = ((uint32_t)cmd[2] << 24) | ((uint32_t)cmd[3] << 16) | ((uint32_t)cmd[4] << 8) | cmd[5];
  unsigned len = ((uint32_t)cmd[7] << 8) | cmd[8];
  /*
  LOGN("onWrite10");
  LOG("-W ");
  LOGHEX(adds);
  LOG(":");
  LOGHEXN(len);
  */

  if(m_img->m_type == SCSI_TYPE_CDROM)
  {
    m_senseKey = SCSI_SENSE_HARDWARE_ERROR;
    m_additional_sense_code = SCSI_ASC_WRITE_PROTECTED; // Write Protect
    return SCSI_STATUS_CHECK_CONDITION;
  }

  gpio_write(LED, high);
  readDataPhaseSD(adds, len);
  gpio_write(LED, low);
  return SCSI_STATUS_GOOD;
}

static byte onReadDVDStructure(const byte *cmd)
{
    LOGN("onReadDVDStructure");
    m_senseKey = SCSI_SENSE_ILLEGAL_REQUEST;
    m_additional_sense_code = SCSI_ASC_CANNOT_READ_MEDIUM_INCOMPATIBLE_FORMAT; /* CANNOT READ MEDIUM - INCOMPATIBLE FORMAT */
    return SCSI_STATUS_CHECK_CONDITION;
}

/*
 * MODE SENSE command processing.
 */
static byte onModeSense(const byte *cdb)
{
  LOGN("onModeSense");
  unsigned len = 0;
  byte page_code = cdb[2] & 0x3f;
  byte change = false;
  byte dbd = cdb[1] & 0x08;
  unsigned a = 0;

  if(cdb[0] == SCSI_MODE_SENSE6)
  {
    len = cdb[4];
  }
  else /* SCSI_MODE_SENSE10 */
  {
    len = cdb[7];
    len <<= 8;
    len |= cdb[8];
    if(len > 0x800) { len = 0x800; }
  }

  if((cdb[2] & 0xc0) == 0x40)
  {
    change = true;
  }
 
  memset(m_buf, 0, len);
 
  // mode sense data header
  m_buf[1] = 0x01;
  a += 4;

  if(dbd) {
    m_buf[3] = 0x08; // block descriptor length
      
    m_buf[a + 1] = (byte)m_img->m_blockcount >> 16;
    m_buf[a + 2] = (byte)m_img->m_blockcount >> 8;
    m_buf[a + 3] = (byte)m_img->m_blockcount;
    
    m_buf[a + 5] = (byte)m_img->m_blocksize >> 16;
    m_buf[a + 6] = (byte)m_img->m_blocksize >> 8;
    m_buf[a + 7] = (byte)m_img->m_blocksize;

    a += 8;
  }

  switch(page_code) {
  case 0x3F:
  case 0x01: // error recovery
    m_buf[a + 0] = 0x01;
    m_buf[a + 1] = 0x0A;
    a += 12;
    if(page_code != 0x3F) break;

  case 0x03:  //Drive parameters
  if(m_img->m_type == SCSI_TYPE_HDD)
  {
    m_buf[a + 0] = 0x03; //Page code
    m_buf[a + 1] = 0x16; // Page length
    m_buf[a + 11] = 0x3F;//Number of sectors / track
  
    a += 24;
  }
    if(page_code != 0x3F) break;
    
  case 0x04:  //Drive parameters
  if(m_img->m_type == SCSI_TYPE_HDD)
    {
      m_buf[a + 0] = 0x04; //Page code
      m_buf[a + 1] = 0x16; // Page length
      m_buf[a + 2] = (byte)m_img->m_blockcount >> 16; // Cylinder length
      m_buf[a + 3] = (byte)m_img->m_blockcount >> 8;
      m_buf[a + 4] = (byte)m_img->m_blockcount;
      m_buf[a + 5] = 1;   //Number of heads
      a += 24;
    }
    if(page_code != 0x3F) break;

    case 0x0d:
      if(m_img->m_type == SCSI_TYPE_CDROM)
      {
        m_buf[a + 0] = 0x0d;
        m_buf[a + 1] = 0x06;

        // 2 seconds for inactive timer
        m_buf[a + 3] = 0x05;

        // MSF multiples are 60 and 75
        m_buf[a + 5] = 60;
        m_buf[a + 7] = 75;

        a += 8;
        if(page_code != 0x3f) break;
      }

    case 0x0e:
      if(m_img->m_type == SCSI_TYPE_CDROM)
      {
        m_buf[a + 0] = 0x0e;
        m_buf[a + 1] = 0x0e;

        a += 16;
        if(page_code != 0x3f) break;
      }

    case 0x30: // magic Apple page Thanks to bitsavers for the info
      {
        static const byte apple_magic[0x24] =
        {
          0x23, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x30, 0x16, 0x41, 0x50,
          0x50, 0x4C, 0x45, 0x20, 0x43, 0x4F, 0x4D, 0x50, 0x55, 0x54, 0x45, 0x52, 0x2C, 0x20, 0x49, 0x4E,
          0x43, 0x20, 0x20, 0x20
        };

        memcpy(&m_buf[a], apple_magic, 0x24);
        a += 24;
        LOGN("Apple special MODE SENSE page");
        if(page_code != 0x3f) break;
      }

  default:
    break;
  }
  m_buf[0] = a - 1;
  writeDataPhase(len < a ? len : a, m_buf);
  return SCSI_STATUS_GOOD;
}

static byte onReadTOC(const byte *cmd)
{
    LOGN("onReadTOC");
    unsigned a = 0;
    uint8_t msf = cmd[2] & 0x02;
    uint8_t track = cmd[6];
    unsigned len = ((uint32_t)cmd[7] << 8) | cmd[8];

    m_senseKey = SCSI_SENSE_ILLEGAL_REQUEST;
    m_additional_sense_code = SCSI_ASC_INVALID_FIELD_IN_CDB;
    return SCSI_STATUS_CHECK_CONDITION;
#if 0
    if(track != 0 && track != 0xaa)
    {
      m_senseKey = SCSI_SENSE_ILLEGAL_REQUEST;
      return SCSI_STATUS_CHECK_CONDITION;
    }
    
    if(track != 0xaa)
    {
      // m_buf[0] = 0;
      m_buf[1] = 5;
      m_buf[2] = 1;
      m_buf[3] = 1;
      // track descriptor
      // m_buf[4] = 0;
      a += 4;

      m_buf[a + 1] = 0x14; // data track
      m_buf[a + 2] = 1;
    
      if(msf)
      {
        LBAtoMSF(16, &m_buf[a + 4]);
      }
      else
      {
        m_buf[a + 6] = 0;
        m_buf[a + 7] = 16; 
      }
      
    }
    else
    {
      // leadout track
      m_buf[a + 1] = 0x0a;
      m_buf[2] = 1;
      m_buf[3] = 1;
      a += 4;

      m_buf[a + 0] = 0xaa;
      if(msf)
      {
        LBAtoMSF(m_img->m_blockcount + 1, &m_buf[a + 4]);
      }
      else
      {
        m_buf[a + 6] = (m_img->m_blockcount + 1) >> 8;
        m_buf[a + 7] = (m_img->m_blockcount + 1);
      }
    }
    
    writeDataPhase(len, m_buf);
    return SCSI_STATUS_GOOD;
#endif
}

static byte onModeSelect(const byte *cdb)
{
  unsigned length = 0;

  LOGN("onModeSelect");
  if(m_img->m_type != SCSI_TYPE_HDD && (cdb[1] & 0x01))
  {
    m_senseKey = SCSI_SENSE_ILLEGAL_REQUEST;
    m_additional_sense_code = SCSI_ASC_INVALID_FIELD_IN_CDB;
    return SCSI_STATUS_CHECK_CONDITION;
  }

  if(cdb[0] == SCSI_MODE_SELECT6)
  {
    length = cdb[4];
  }
  else /* SCSI_MODE_SELECT10 */
  {
    length = cdb[7] << 8;
    length |= cdb[8];
    if(length > 0x800) { length = 0x800; }
  }

  writeDataPhase(length, m_buf);
  return SCSI_STATUS_GOOD;
}

static byte onReadDiscInformation(const byte *cmd)
{
  LOGN("onReadDiscInformation");
  writeDataPhase((cmd[7] >> 8) | cmd[8], m_buf);
  return SCSI_STATUS_GOOD;
}

/*
 * MsgIn2.
 */
static void MsgIn2(unsigned msg)
{
  LOGN("MsgIn2");
 
  SCSI_PHASE_MSG_IN();
  writeHandshake(msg);
}

/*
 * MsgOut2.
 */
static void MsgOut2()
{
  LOGN("MsgOut2");
  SCSI_PHASE_MSG_OUT();
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
 // HDD Image selection
  m_img = (HDDIMG *)0; // None

  // Wait until RST = H, BSY = H, SEL = L
  do {} while( SCSI_IN(vBSY) || !SCSI_IN(vSEL) || SCSI_IN(vRST));
  
  // BSY+ SEL-
  // If the ID to respond is not driven, wait for the next
  byte scsiid = READ_DATA_BUS() & scsi_id_mask;
  if((scsiid) == 0) {
    return;
  }
  LOGN("Selection");
  m_isBusReset = false;
  // Set BSY to-when selected
  SCSI_BSY_ACTIVE();     // Turn only BSY output ON, ACTIVE

  // Ask for a TARGET-ID to respond
#if USE_DB2ID_TABLE
  m_id = db2scsiid[scsiid];
  //if(m_id==0xff) return;
#else
  for(m_id=7;m_id>=0;m_id--)
    if(scsiid & (1<<m_id)) break;
  //if(m_id<0) return;
#endif

  // Wait until SEL becomes inactive
  while(isHigh(gpio_read(SEL)) && isLow(gpio_read(BSY))) {
    if(m_isBusReset) {
      goto BusFree;
    }
  }
  SCSI_TARGET_ACTIVE()  // (BSY), REQ, MSG, CD, IO output turned on
  
  if(isHigh(gpio_read(ATN))) {
    bool syncenable = false;
    unsigned syncperiod = 50;
    unsigned syncoffset = 0;
    unsigned loopWait = 0;
    m_msc = 0;
    memset(m_msb, 0x00, sizeof(m_msb));
    while(isHigh(gpio_read(ATN)) && loopWait < 255) {
      MsgOut2();
      loopWait++;
    }
    for(unsigned i = 0; i < m_msc; i++) {
      // ABORT
      if (m_msb[i] == 0x06) {
        goto BusFree;
      }
      // BUS DEVICE RESET
      if (m_msb[i] == 0x0C) {
        syncoffset = 0;
        goto BusFree;
      }
      // IDENTIFY
      if (m_msb[i] >= 0x80) {
        if(m_msb[i] & 0x1f >= NUM_SCSILUN)
        {
          m_senseKey = SCSI_SENSE_ILLEGAL_REQUEST;
          m_additional_sense_code = SCSI_ASC_LOGICAL_UNIT_NOT_SUPPORTED;
          m_sts |= SCSI_STATUS_CHECK_CONDITION;
          goto Status;
        }
      }
      // Extended message
      if (m_msb[i] == 0x01) {
        // Check only when synchronous transfer is possible
        if (!syncenable || m_msb[i + 2] != 0x01) {
          MsgIn2(0x07);
          break;
        }
        // Transfer period factor(50 x 4 = Limited to 200ns)
        syncperiod = m_msb[i + 3];
        if (syncperiod > 50) {
          syncperiod = 50;
        }
        // REQ/ACK offset(Limited to 16)
        syncoffset = m_msb[i + 4];
        if (syncoffset > 16) {
          syncoffset = 16;
        }
        // STDR response message generation
        MsgIn2(0x01);
        MsgIn2(0x03);
        MsgIn2(0x01);
        MsgIn2(syncperiod);
        MsgIn2(syncoffset);
        break;
      }
    }
  }

  // delay from scsiconfig
  delayMicroseconds(m_scsi_delay);
  
  LOG("Command:");
  SCSI_PHASE_COMMAND();
  
  unsigned len;
  byte cmd[12];
  cmd[0] = readHandshake(); if(m_isBusReset) goto BusFree;
  LOGHEX(cmd[0]);
  // Command length selection, reception
  static const int cmd_class_len[8]={6,10,10,6,6,12,6,6};
  len = cmd_class_len[cmd[0] >> 5];
  cmd[1] = readHandshake(); LOG(":");LOGHEX(cmd[1]);
  cmd[2] = readHandshake(); LOG(":");LOGHEX(cmd[2]); 
  cmd[3] = readHandshake(); LOG(":");LOGHEX(cmd[3]);
  cmd[4] = readHandshake(); LOG(":");LOGHEX(cmd[4]); 
  cmd[5] = readHandshake(); LOG(":");LOGHEX(cmd[5]);
  // Receive the remaining commands
  if(len == 6) goto finished_command_bytes;
  cmd[6] = readHandshake(); LOG(":");LOGHEX(cmd[6]);
  cmd[7] = readHandshake(); LOG(":");LOGHEX(cmd[7]); 
  cmd[8] = readHandshake(); LOG(":");LOGHEX(cmd[8]);
  cmd[9] = readHandshake(); LOG(":");LOGHEX(cmd[9]);
  if(len == 10) goto finished_command_bytes;
  cmd[10] = readHandshake(); LOG(":");LOGHEX(cmd[10]);
  cmd[11] = readHandshake(); LOG(":");LOGHEX(cmd[11]);

#if 0
  for(unsigned i = 6; i < len; i++ ) {
    cmd[i] = readHandshake();
    LOG(":");
    LOGHEX(cmd[i]);
    if(m_isBusReset) goto BusFree;
  }
#endif

  finished_command_bytes:
  if(m_isBusReset) goto BusFree;

  // LUN confirmation
  m_sts = cmd[1]&0xe0;      // Preset LUN in status byte
  m_lun = m_sts>>5;

  if(m_lun >= NUM_SCSILUN)
  {
    m_senseKey = SCSI_SENSE_ILLEGAL_REQUEST;
    m_additional_sense_code = SCSI_ASC_LOGICAL_UNIT_NOT_SUPPORTED;
    m_sts |= SCSI_STATUS_CHECK_CONDITION;
    goto Status;
  }

  m_img = &(img[m_id][m_lun]); // There is an image
  if(!(m_img->m_file.isOpen()))
  {
    m_img = (HDDIMG *)0;       // Image absent
    m_senseKey = SCSI_SENSE_ILLEGAL_REQUEST;
    m_additional_sense_code = SCSI_ASC_LOGICAL_UNIT_NOT_SUPPORTED;
    m_sts |= SCSI_STATUS_CHECK_CONDITION;
    goto Status;
  }
  // if(!m_img) m_sts |= 0x02;            // Missing image file for LUN
  //LOGHEX(((uint32_t)m_img));
  
  LOG(":ID ");
  LOG(m_id);
  LOG(":LUN ");
  LOG(m_lun);

  LOGN("");

  m_sts |= scsi_command_table[cmd[0]](cmd);

  if(m_isBusReset) {
     goto BusFree;
  }
Status:
  //LOGN("Sts");
  SCSI_PHASE_STATUS();
  writeHandshake(m_sts);
  if(m_isBusReset) {
     goto BusFree;
  }

  //LOGN("MsgIn");
  SCSI_PHASE_MSG_IN();
  writeHandshake(m_msg);

BusFree:
  //LOGN("BusFree");
  m_isBusReset = false;
  //SCSI_OUT(vREQ,inactive) // gpio_write(REQ, low);
  //SCSI_OUT(vMSG,inactive) // gpio_write(MSG, low);
  //SCSI_OUT(vCD ,inactive) // gpio_write(CD, low);
  //SCSI_OUT(vIO ,inactive) // gpio_write(IO, low);
  //SCSI_OUT(vBSY,inactive)
  SCSI_TARGET_INACTIVE() // Turn off BSY, REQ, MSG, CD, IO output
}


// Thanks RaSCSI :D
//	LBA→MSF Conversion
static inline void LBAtoMSF(const uint32_t lba, byte *msf)
{
	uint32_t m, s, f;

	// 75 and 75*60 get the remainder
	m = lba / (75 * 60);
	s = lba % (75 * 60);
	f = s % 75;
	s /= 75;

	// The base point is M=0, S=2, F=0
	s += 2;
	if (s >= 60) {
		s -= 60;
		m++;
	}

	// Store
	msf[0] = 0x00;
	msf[1] = (byte)m;
	msf[2] = (byte)s;
	msf[3] = (byte)f;
}


static inline uint32_t MSFtoLBA(const byte *msf)
{
	uint32_t lba;

	// 1, 75, add up in multiples of 75*60
	lba = msf[1];
	lba *= 60;
	lba += msf[2];
	lba *= 75;
	lba += msf[3];

	// Since the base point is M=0, S=2, F=0, subtract 150
	lba -= 150;

	return lba;
}
