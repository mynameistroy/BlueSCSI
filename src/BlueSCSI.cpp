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

#include "scsi_cmds.h"

#ifdef USE_STM32_DMA
#warning "warning USE_STM32_DMA"
#endif

#define DEBUG            0      // 0:No debug information output
                                // 1: Debug information output available

#define SCSI_SELECT      0      // 0 for STANDARD
                                // 1 for SHARP X1turbo
                                // 2 for NEC PC98
#define READ_SPEED_OPTIMIZE  1 // Faster reads
#define WRITE_SPEED_OPTIMIZE 1 // Speeding up writes
#define USE_DB2ID_TABLE      1 // Use table to get ID from SEL-DB

// SCSI config
#define NUM_SCSIID  7          // Maximum number of supported SCSI-IDs (The minimum is 0)
#define NUM_SCSILUN 2          // Maximum number of LUNs supported     (The minimum is 0)
#define READ_PARITY_CHECK 0    // Perform read parity check (unverified)

// HDD format
#define MAX_BLOCKSIZE 4096     // Maximum BLOCK size

// SDFAT
#define SD1_CONFIG SdSpiConfig(PA4, DEDICATED_SPI, SD_SCK_MHZ(SPI_FULL_SPEED), &SPI)
SdFs SD;

#if DEBUG
#define LOG(XX)     Serial.print(XX)
#define LOGHEX(XX)  Serial.print(XX, HEX)
#define LOGN(XX)    Serial.println(XX)
#define LOGHEXN(XX) Serial.println(XX, HEX)
#else
#define LOG(XX)     //Serial.print(XX)
#define LOGHEX(XX)  //Serial.print(XX, HEX)
#define LOGN(XX)    //Serial.println(XX)
#define LOGHEXN(XX) //Serial.println(XX, HEX)
#endif

#define active   1
#define inactive 0
#define high 0
#define low 1

#define isHigh(XX) ((XX) == high)
#define isLow(XX) ((XX) != high)

#define gpio_mode(pin,val) gpio_set_mode(PIN_MAP[pin].gpio_device, PIN_MAP[pin].gpio_bit, val);
#define gpio_write(pin,val) gpio_write_bit(PIN_MAP[pin].gpio_device, PIN_MAP[pin].gpio_bit, val)
#define gpio_read(pin) gpio_read_bit(PIN_MAP[pin].gpio_device, PIN_MAP[pin].gpio_bit)

//#define DB0       PB8     // SCSI:DB0
//#define DB1       PB9     // SCSI:DB1
//#define DB2       PB10    // SCSI:DB2
//#define DB3       PB11    // SCSI:DB3
//#define DB4       PB12    // SCSI:DB4
//#define DB5       PB13    // SCSI:DB5
//#define DB6       PB14    // SCSI:DB6
//#define DB7       PB15    // SCSI:DB7
//#define DBP       PB0     // SCSI:DBP
#define ATN       PA8      // SCSI:ATN
#define BSY       PA9      // SCSI:BSY
#define ACK       PA10     // SCSI:ACK
#define RST       PA15     // SCSI:RST
#define MSG       PB3      // SCSI:MSG
#define SEL       PB4      // SCSI:SEL
#define CD        PB5      // SCSI:C/D
#define REQ       PB6      // SCSI:REQ
#define IO        PB7      // SCSI:I/O

#define SD_CS     PA4      // SDCARD:CS
#define LED       PC13     // LED

// GPIO register port
#define PAREG GPIOA->regs
#define PBREG GPIOB->regs

// LED control
#define LED_ON()       gpio_write(LED, high);
#define LED_OFF()      gpio_write(LED, low);

// Virtual pin (Arduio compatibility is slow, so make it MCU-dependent)
#define PA(BIT)       (BIT)
#define PB(BIT)       (BIT+16)
// Virtual pin decoding
#define GPIOREG(VPIN)    ((VPIN)>=16?PBREG:PAREG)
#define BITMASK(VPIN) (1<<((VPIN)&15))

#define vATN       PA(8)      // SCSI:ATN
#define vBSY       PA(9)      // SCSI:BSY
#define vACK       PA(10)     // SCSI:ACK
#define vRST       PA(15)     // SCSI:RST
#define vMSG       PB(3)      // SCSI:MSG
#define vSEL       PB(4)      // SCSI:SEL
#define vCD        PB(5)      // SCSI:C/D
#define vREQ       PB(6)      // SCSI:REQ
#define vIO        PB(7)      // SCSI:I/O
#define vSD_CS     PA(4)      // SDCARD:CS

// SCSI output pin control: opendrain active LOW (direct pin drive)
#define SCSI_OUT(VPIN,ACTIVE) { GPIOREG(VPIN)->BSRR = BITMASK(VPIN)<<((ACTIVE)?16:0); }

// SCSI input pin check (inactive=0,avtive=1)
#define SCSI_IN(VPIN) ((~GPIOREG(VPIN)->IDR>>(VPIN&15))&1)

// GPIO mode
// IN , FLOAT      : 4
// IN , PU/PD      : 8
// OUT, PUSH/PULL  : 3
// OUT, OD         : 1
//#define DB_MODE_OUT 3
#define DB_MODE_OUT 1
#define DB_MODE_IN  8

// Put DB and DP in output mode
#define SCSI_DB_OUTPUT() { PBREG->CRL=(PBREG->CRL &0xfffffff0)|DB_MODE_OUT; PBREG->CRH = 0x11111111*DB_MODE_OUT; }
// Put DB and DP in input mode
#define SCSI_DB_INPUT()  { PBREG->CRL=(PBREG->CRL &0xfffffff0)|DB_MODE_IN ; PBREG->CRH = 0x11111111*DB_MODE_IN;  }

// Turn on the output only for BSY
#define SCSI_BSY_ACTIVE()      { gpio_mode(BSY, GPIO_OUTPUT_OD); SCSI_OUT(vBSY,  active) }
// BSY,REQ,MSG,CD,IO Turn on the output (no change required for OD)
#define SCSI_TARGET_ACTIVE()   { }
// BSY,REQ,MSG,CD,IO Turn off output, BSY is the last input
#define SCSI_TARGET_INACTIVE() { SCSI_OUT(vREQ,inactive); SCSI_OUT(vMSG,inactive); SCSI_OUT(vCD,inactive);SCSI_OUT(vIO,inactive); SCSI_OUT(vBSY,inactive); gpio_mode(BSY, GPIO_INPUT_PU); }

// HDDiamge file
#define HDIMG_ID_POS  2                 // Position to embed ID number
#define HDIMG_LUN_POS 3                 // Position to embed LUN numbers
#define HDIMG_BLK_POS 5                 // Position to embed block size numbers
#define MAX_FILE_PATH 32                // Maximum file name length

#define SCSI_TYPE_HDD     1 << 0
#define SCSI_TYPE_CDROM   1 << 1

struct SCSI_INQUIRY_DATA
{
  union
  {
  struct {
    // bitfields are in REVERSE order for ARM
    // byte 0
    byte peripheral_device_type:5;
    byte peripheral_qualifier:3;
    // byte 1
    byte reserved_byte2:7;
    byte rmb:1;
    // byte 2
    byte ansi_version:3;
    byte always_zero_byte3:5;
    // byte 3
    byte response_format:4;
    byte reserved_byte4:2;
    byte tiop:1;
    byte always_zero_byte4:1;
    // byte 4
    byte additional_length;
    // byte 5-6
    byte reserved_byte5;
    byte reserveded_byte6;
    // byte 7
    byte sync:1;
    byte always_zero_byte7_more:4;
    byte always_zero_byte7:3;
    // byte 8-15
    char vendor[8];
    // byte 16-31
    char product[16];
    // byte 32-35
    char revision[4];
    // byte 36
    byte release;
    // 37-46
    char revision_date[10];
  };
  // raw bytes
  byte raw[46];
  };
};

// HDD image
typedef struct hddimg_struct
{
	FsFile        m_file;                 // File object
	uint64_t      m_fileSize;             // File size
	size_t        m_blocksize;            // SCSI BLOCK size
  uint8_t       m_type;                 // SCSI device type
  unsigned long m_blockcount;           // blockcount
  bool          m_raw;                  // Raw disk
  SCSI_INQUIRY_DATA inquiry_block;      // SCSI information
}HDDIMG;

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
byte          m_buf[MAX_BLOCKSIZE+1] = {0xff}; // General purpose buffer + overrun fetch
int           m_msc;
byte          m_msb[256];             // Command storage bytes

/* Configurable options */
int           m_scsi_delay = 0;           // SCSI timing delay, default is none

/*
 *  Data byte to BSRR register setting value and parity table
*/

// Parity bit generation
#define PTY(V)   (1^((V)^((V)>>1)^((V)>>2)^((V)>>3)^((V)>>4)^((V)>>5)^((V)>>6)^((V)>>7))&1)

// Data byte to BSRR register setting value conversion table
// BSRR[31:24] =  DB[7:0]
// BSRR[   16] =  PTY(DB)
// BSRR[15: 8] = ~DB[7:0]
// BSRR[    0] = ~PTY(DB)

// Set DBP, set REQ = inactive
#define DBP(D)    ((((((uint32_t)(D)<<8)|PTY(D))*0x00010001)^0x0000ff01)|BITMASK(vREQ))

#define DBP8(D)   DBP(D),DBP(D+1),DBP(D+2),DBP(D+3),DBP(D+4),DBP(D+5),DBP(D+6),DBP(D+7)
#define DBP32(D)  DBP8(D),DBP8(D+8),DBP8(D+16),DBP8(D+24)

// BSRR register control value that simultaneously performs DB set, DP set, and REQ = H (inactrive)
static const uint32_t db_bsrr[256]={
  DBP32(0x00),DBP32(0x20),DBP32(0x40),DBP32(0x60),
  DBP32(0x80),DBP32(0xA0),DBP32(0xC0),DBP32(0xE0)
};
// Parity bit acquisition
#define PARITY(DB) (db_bsrr[DB]&1)

// Macro cleaning
#undef DBP32
#undef DBP8
//#undef DBP
//#undef PTY

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

// Log File
#define VERSION "1.0-20210410"
#define LOG_FILENAME "LOG.txt"
FsFile LOG_FILE;



// #define GET_CDB6_LBA(x) ((x[2] & 01f) << 16) | (x[3] << 8) | x[4]

void onFalseInit(void);
void noSDCardFound(void);
void onBusReset(void);
void initFileLog(void);
void finalizeFileLog(void);

inline uint32_t MSFtoLBA(byte *msf);
inline void LBAtoMSF(uint32_t lba, byte *msf);

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

// If config file exists, read the first three lines and copy the contents.
// File must be well formed or you will get junk in the SCSI Vendor fields.
void readSCSIDeviceConfig(HDDIMG *h) {
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
void readSDCardInfo()
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

bool ImageOpen(HDDIMG *h,const char *image_name)
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
    noSDCardFound();
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
    int id, lun, blk, type;  
    char name[MAX_FILE_PATH+1];
    
    if (!file.openNext(&root, O_READ)) break;
    
    if(file.isDir()) {
          continue;
    }

    file.getName(name, MAX_FILE_PATH+1);
    file.close();
    String file_name = String(name);
    file_name.toLowerCase();
    
    if(file_name.startsWith("hd") && file_name.endsWith(".hda"))
    {
      id  = name[HDIMG_ID_POS] - '0';
      lun = name[HDIMG_LUN_POS] - '0';
      blk = name[HDIMG_BLK_POS] - '0';
      type = SCSI_TYPE_HDD;
      
      if(blk == 2) {
        blk = 256;
      } else if(blk == 1) {
        blk = 1024;
      } else {
        blk = 512;
      }
    }
    else if(file_name.startsWith("cd") && file_name.endsWith(".iso"))
    {
      id  = name[HDIMG_ID_POS] - '0';
      lun = name[HDIMG_LUN_POS] - '0';
      blk = name[HDIMG_BLK_POS] - '0';
      type = SCSI_TYPE_CDROM;
    } 
    else
    {
      LOG_FILE.print("Not a recognized image type ");
      LOG_FILE.println(name);
      LOG_FILE.sync();
      continue;
    }

    if(id < NUM_SCSIID && lun < NUM_SCSILUN) {
      HDDIMG *h = &img[id][lun];
      h->m_blocksize = blk;
      memset(&h->inquiry_block, 0, sizeof(h->inquiry_block));
      h->m_type = type;
      if(ImageOpen(h, name))
      {
        // Marked as a responsive ID
        scsi_id_mask |= 1<<id;

        switch(type)
        {
          case SCSI_TYPE_HDD:
          // default SCSI HDD
          h->inquiry_block.ansi_version = 1;
          h->inquiry_block.response_format = 1;
          h->inquiry_block.additional_length = 31;
          strncpy(h->inquiry_block.vendor, "QUANTUM", 7);
          strncpy(h->inquiry_block.product, "FIREBALL1", 9);
          strncpy(h->inquiry_block.revision, "1.0", 3);
          break;
          
          case SCSI_TYPE_CDROM:
          // default SCSI CDROM
          h->inquiry_block.peripheral_device_type = 5;
          h->inquiry_block.rmb = 1;
          h->inquiry_block.ansi_version = 2;
          h->inquiry_block.response_format = 2;
          h->inquiry_block.additional_length = 42;
          h->inquiry_block.sync = 1;
          strncpy(h->inquiry_block.vendor, "BLUESCSI", 8);
          strncpy(h->inquiry_block.product, "CD-ROM CDU-55S", 14);
          strncpy(h->inquiry_block.revision, "1.9a", 4);
          h->inquiry_block.release = 0x20;
          strncpy(h->inquiry_block.revision_date, "1995/02/08", 11);
          break;
        }

        readSCSIDeviceConfig(h);
      }
    }
  }
  root.close();


  // Error if there are 0 image files
  if(scsi_id_mask==0) {
    LOG_FILE.println("ERROR: No valid images found!");
    onFalseInit();
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
void onBusReset(void)
{
#if SCSI_SELECT == 1
  // SASI I / F for X1 turbo has RST pulse write cycle +2 clock ==
  // I can't filter because it only activates about 1.25us
  {{
#else
  if(isHigh(gpio_read(RST))) {
    delayMicroseconds(20);
    if(isHigh(gpio_read(RST))) {
#endif  
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
inline byte readHandshake(void)
{
  SCSI_OUT(vREQ,active)
  //SCSI_DB_INPUT()
  while( ! SCSI_IN(vACK)) { if(m_isBusReset) return 0; }
  byte r = readIO();
  SCSI_OUT(vREQ,inactive)
  while( SCSI_IN(vACK)) { if(m_isBusReset) return 0; }
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

/*
 * INQUIRY command processing.
 */
#if SCSI_SELECT == 2
byte onInquiryCommand(byte len)
{
  byte buf[36] = {
    0x00, //Device type
    0x00, //RMB = 0
    0x01, //ISO,ECMA,ANSI version
    0x01, //Response data format
    35 - 4, //Additional data length
    0, 0, //Reserve
    0x00, //Support function
    'N', 'E', 'C', 'I', 'T', 'S', 'U', ' ',
    'A', 'r', 'd', 'S', 'C', 'S', 'i', 'n', 'o', ' ', ' ',' ', ' ', ' ', ' ', ' ',
    '0', '0', '1', '0',
  };
  writeDataPhase(len < 36 ? len : 36, buf);
  return 0x00;
}
#else
byte onInquiryCommand(byte len)
{
  // only write back what was asked for
  writeDataPhase(len, m_img->inquiry_block.raw);
  return 0x00;
}
#endif

/*
 * REQUEST SENSE command processing.
 */
void onRequestSenseCommand(byte len)
{
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
  writeDataPhase(len < 18 ? len : 18, buf);  
}

/*
 * READ CAPACITY command processing.
 */
byte onReadCapacityCommand(byte pmi)
{
  if(!m_img) return 0x02; // Image file absent
  
  uint32_t bl = m_img->m_blocksize;
  uint32_t bc = m_img->m_blockcount;
  uint8_t buf[8] = {
    bc >> 24, bc >> 16, bc >> 8, bc,
    bl >> 24, bl >> 16, bl >> 8, bl    
  };
  writeDataPhase(8, buf);
  return 0x00;
}

/*
 * READ6 / 10 Command processing.
 */
byte onReadCommand(uint32_t adds, uint32_t len)
{
  LOG("-R ");
  LOGHEX(adds);
  LOG(":");
  LOGHEXN(len);

  if(!m_img) return 0x02; // Image file absent
  
  gpio_write(LED, high);
  writeDataPhaseSD(adds, len);
  gpio_write(LED, low);
  return 0x00; //sts
}

/*
 * WRITE6 / 10 Command processing.
 */
byte onWriteCommand(uint32_t adds, uint32_t len)
{
  LOG("-W ");
  LOGHEX(adds);
  LOG(":");
  LOGHEXN(len);
  
  if(!m_img) return 0x02; // Image file absent
  if(m_img->m_type == SCSI_TYPE_CDROM)
  {
    m_additional_sense_code = 0x2700; // Write Protect
    return 0x04;
  }

  gpio_write(LED, high);
  readDataPhaseSD(adds, len);
  gpio_write(LED, low);
  return 0; //sts
}

/*
 * MODE SENSE command processing.
 */
#if SCSI_SELECT == 2
byte onModeSenseCommand(byte dbd, int cmd2, uint32_t len)
{
  if(!m_img) return 0x02; // Image file absent

  int pageCode = cmd2 & 0x3F;

  // Assuming sector size 512, number of sectors 25, number of heads 8 as default settings
  int size = m_img->m_fileSize;
  int cylinders = (int)(size >> 9);
  cylinders >>= 3;
  cylinders /= 25;
  int sectorsize = 512;
  int sectors = 25;
  int heads = 8;
  // Sector size
 int disksize = 0;
  for(disksize = 16; disksize > 0; --(disksize)) {
    if ((1 << disksize) == sectorsize)
      break;
  }
  // Number of blocks
  uint32_t diskblocks = (uint32_t)(size >> disksize);
  memset(m_buf, 0, sizeof(m_buf)); 
  int a = 4;
  if(dbd == 0) {
    uint32_t bl = m_img->m_blocksize;
    uint32_t bc = m_img->m_fileSize / bl;
    byte c[8] = {
      0,// Density code
      bc >> 16, bc >> 8, bc,
      0, //Reserve
      bl >> 16, bl >> 8, bl
    };
    memcpy(&m_buf[4], c, 8);
    a += 8;
    m_buf[3] = 0x08;
  }
  switch(pageCode) {
  case 0x3F:
  {
    m_buf[a + 0] = 0x01;
    m_buf[a + 1] = 0x06;
    a += 8;
  }
  case 0x03:  // drive parameters
  {
    m_buf[a + 0] = 0x80 | 0x03; // Page code
    m_buf[a + 1] = 0x16; // Page length
    m_buf[a + 2] = (byte)(heads >> 8);// number of sectors / track
    m_buf[a + 3] = (byte)(heads);// number of sectors / track
    m_buf[a + 10] = (byte)(sectors >> 8);// number of sectors / track
    m_buf[a + 11] = (byte)(sectors);// number of sectors / track
    int size = 1 << disksize;
    m_buf[a + 12] = (byte)(size >> 8);// number of sectors / track
    m_buf[a + 13] = (byte)(size);// number of sectors / track
    a += 24;
    if(pageCode != 0x3F) {
      break;
    }
  }
  case 0x04:  // drive parameters
  {
      LOGN("AddDrive");
      m_buf[a + 0] = 0x04; // Page code
      m_buf[a + 1] = 0x12; // Page length
      m_buf[a + 2] = (cylinders >> 16);// Cylinder length
      m_buf[a + 3] = (cylinders >> 8);
      m_buf[a + 4] = cylinders;
      m_buf[a + 5] = heads;   // Number of heads
      a += 20;
    if(pageCode != 0x3F) {
      break;
    }
  }
  default:
    break;
  }
  m_buf[0] = a - 1;
  writeDataPhase(len < a ? len : a, m_buf);
  return 0x00;
}
#else
//byte onModeSenseCommand(byte dbd, int cmd2, uint32_t len)
byte onModeSenseCommand(byte *cdb)
{
  unsigned len = 0;
  byte page_code = cdb[2] & 0x3f;
  byte change = false;
  byte dbd = cdb[1] & 0x08;
  int a = 0;

  if(!m_img) return 0x02; // No image file

  if(cdb[0] == SCSI_MODE_SENSE6)
  {
    len = cdb[4];
  }
  else /* SCSI_MODE_SENSE10 */
  {
    len = cdb[7];
    len <<= 8;
    len |= cdb[8];
    if(len > 0x800) { len == 0x800; }
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


  default:
    break;
  }
  m_buf[0] = a - 1;
  writeDataPhase(len < a ? len : a, m_buf);
  return 0x00;
}
#endif

byte onReadTOC(uint8_t msf, uint8_t track, uint32_t len)
{
    int a = 0;

    if(!m_img) return 0x02; // No image file

    if(track != 0 && track != 0xaa)
    {
      m_senseKey = 5;
      return 0x02;
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
    return 0x00;
}

byte onModeSelect(byte *cdb)
{
  unsigned length;

  if(m_img->m_type != SCSI_TYPE_HDD && (cdb[1] & 0x01))
  {
    m_senseKey = 5;
    m_additional_sense_code = 0x2400;
    return 0x02;
  }

  if(cdb[0] == SCSI_MODE_SELECT6)
  {
    length = cdb[4];
  }
  else /* SCSI_MODE_SELECT10 */
  {
    length == cdb[7] << 8;
    length |= cdb[8];
    if(length > 0x800) { length = 0x800; }
  }

  writeDataPhase(length, m_buf);
  return 0x00;
}

#if SCSI_SELECT == 1
/*
 * dtc510b_setDriveparameter
 */
#define PACKED  __attribute__((packed))
typedef struct PACKED dtc500_cmd_c2_param_struct
{
  uint8_t StepPlusWidth;        // Default is 13.6usec (11)
  uint8_t StepPeriod;         // Default is  3  msec.(60)
  uint8_t StepMode;         // Default is  Bufferd (0)
  uint8_t MaximumHeadAdress;      // Default is 4 heads (3)
  uint8_t HighCylinderAddressByte;  // Default set to 0   (0)
  uint8_t LowCylinderAddressByte;   // Default is 153 cylinders (152)
  uint8_t ReduceWrietCurrent;     // Default is above Cylinder 128 (127)
  uint8_t DriveType_SeekCompleteOption;// (0)
  uint8_t Reserved8;          // (0)
  uint8_t Reserved9;          // (0)
} DTC510_CMD_C2_PARAM;

static void logStrHex(const char *msg,uint32_t num)
{
    LOG(msg);
    LOGHEXN(num);
}

static byte dtc510b_setDriveparameter(void)
{
  DTC510_CMD_C2_PARAM DriveParameter;
  uint16_t maxCylinder;
  uint16_t numLAD;
  //uint32_t stepPulseUsec;
  int StepPeriodMsec;

  // receive paramter
  writeDataPhase(sizeof(DriveParameter),(byte *)(&DriveParameter));
 
  maxCylinder =
    (((uint16_t)DriveParameter.HighCylinderAddressByte)<<8) |
    (DriveParameter.LowCylinderAddressByte);
  numLAD = maxCylinder * (DriveParameter.MaximumHeadAdress+1);
  //stepPulseUsec  = calcStepPulseUsec(DriveParameter.StepPlusWidth);
  StepPeriodMsec = DriveParameter.StepPeriod*50;
  logStrHex (" StepPlusWidth      : ",DriveParameter.StepPlusWidth);
  logStrHex (" StepPeriod         : ",DriveParameter.StepPeriod   );
  logStrHex (" StepMode           : ",DriveParameter.StepMode     );
  logStrHex (" MaximumHeadAdress  : ",DriveParameter.MaximumHeadAdress);
  logStrHex (" CylinderAddress    : ",maxCylinder);
  logStrHex (" ReduceWrietCurrent : ",DriveParameter.ReduceWrietCurrent);
  logStrHex (" DriveType/SeekCompleteOption : ",DriveParameter.DriveType_SeekCompleteOption);
  logStrHex (" Maximum LAD        : ",numLAD-1);
  return  0; // error result
}
#endif

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

  // Wait until RST = H, BSY = H, SEL = L
  do {} while( SCSI_IN(vBSY) || !SCSI_IN(vSEL) || SCSI_IN(vRST));

  // BSY+ SEL-
  // If the ID to respond is not driven, wait for the next
  //byte db = readIO();
  //byte scsiid = db & scsi_id_mask;
  byte scsiid = readIO() & scsi_id_mask;
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
  //  
  if(isHigh(gpio_read(ATN))) {
    bool syncenable = false;
    int syncperiod = 50;
    int syncoffset = 0;
    int loopWait = 0;
    m_msc = 0;
    memset(m_msb, 0x00, sizeof(m_msb));
    while(isHigh(gpio_read(ATN)) && loopWait < 255) {
      MsgOut2();
      loopWait++;
    }
    for(int i = 0; i < m_msc; i++) {
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
  SCSI_OUT(vMSG,inactive) // gpio_write(MSG, low);
  SCSI_OUT(vCD ,  active) // gpio_write(CD, high);
  SCSI_OUT(vIO ,inactive) // gpio_write(IO, low);
  
  int len;
  byte cmd[12];
  cmd[0] = readHandshake(); if(m_isBusReset) goto BusFree;
  LOGHEX(cmd[0]);
  // Command length selection, reception
  static const int cmd_class_len[8]={6,10,10,6,6,12,6,6};
  len = cmd_class_len[cmd[0] >> 5];
  cmd[1] = readHandshake(); LOG(":");LOGHEX(cmd[1]); if(m_isBusReset) goto BusFree;
  cmd[2] = readHandshake(); LOG(":");LOGHEX(cmd[2]); if(m_isBusReset) goto BusFree;
  cmd[3] = readHandshake(); LOG(":");LOGHEX(cmd[3]); if(m_isBusReset) goto BusFree;
  cmd[4] = readHandshake(); LOG(":");LOGHEX(cmd[4]); if(m_isBusReset) goto BusFree;
  cmd[5] = readHandshake(); LOG(":");LOGHEX(cmd[5]); if(m_isBusReset) goto BusFree;
  // Receive the remaining commands
  for(int i = 6; i < len; i++ ) {
    cmd[i] = readHandshake();
    LOG(":");
    LOGHEX(cmd[i]);
    if(m_isBusReset) goto BusFree;
  }
  // LUN confirmation
  m_sts = cmd[1]&0xe0;      // Preset LUN in status byte
  m_lun = m_sts>>5;
  // HDD Image selection
  m_img = (HDDIMG *)0; // None
  if( (m_lun <= NUM_SCSILUN) )
  {
    m_img = &(img[m_id][m_lun]); // There is an image
    if(!(m_img->m_file.isOpen()))
      m_img = (HDDIMG *)0;       // Image absent
  }
  // if(!m_img) m_sts |= 0x02;            // Missing image file for LUN
  //LOGHEX(((uint32_t)m_img));
  
  LOG(":ID ");
  LOG(m_id);
  LOG(":LUN ");
  LOG(m_lun);

  LOGN("");
  switch(cmd[0]) {
  case SCSI_TEST_UNIT_READY:
    LOGN("[Test Unit]");
    break;
  case SCSI_REZERO_UNIT:
    LOGN("[Rezero Unit]");
    break;
  case SCSI_REQUEST_SENSE:
    LOGN("[RequestSense]");
    onRequestSenseCommand(cmd[4]);
    break;
  case SCSI_FORMAT_UNIT:
    LOGN("[FormatUnit]");
    break;
  case 0x06:
    LOGN("[FormatUnit]");
    break;
  case SCSI_REASSIGN_BLOCKS:
    LOGN("[ReassignBlocks]");
    break;
  case SCSI_READ6:
    LOGN("[Read6]");
    m_sts |= onReadCommand((((uint32_t)cmd[1] & 0x1F) << 16) | ((uint32_t)cmd[2] << 8) | cmd[3], (cmd[4] == 0) ? 0x100 : cmd[4]);
    break;
  case SCSI_WRITE6:
    LOGN("[Write6]");
    m_sts |= onWriteCommand((((uint32_t)cmd[1] & 0x1F) << 16) | ((uint32_t)cmd[2] << 8) | cmd[3], (cmd[4] == 0) ? 0x100 : cmd[4]);
    break;
  case SCSI_SEEK6:
    LOGN("[Seek6]");
    break;
  case SCSI_INQUIRY:
    LOGN("[Inquiry]");
    m_sts |= onInquiryCommand(cmd[4]);
    break;
  case SCSI_MODE_SENSE6:
    LOGN("[ModeSense6]");
    //m_sts |= onModeSenseCommand(cmd[1]&0x80, cmd[2], cmd[4]);
    m_sts |= onModeSenseCommand(cmd);
    break;
  case SCSI_START_STOP_UNIT:
    LOGN("[StartStopUnit]");
    break;
  case SCSI_PREVENT_ALLOW_REMOVAL:
    LOGN("[PreAllowMed.Removal]");
    break;
  case SCSI_READ_CAPACITY:
    LOGN("[ReadCapacity]");
    m_sts |= onReadCapacityCommand(cmd[8]);
    break;
  case SCSI_READ10:
    LOGN("[Read10]");
    m_sts |= onReadCommand(((uint32_t)cmd[2] << 24) | ((uint32_t)cmd[3] << 16) | ((uint32_t)cmd[4] << 8) | cmd[5], ((uint32_t)cmd[7] << 8) | cmd[8]);
    break;
  case SCSI_WRITE10:
    LOGN("[Write10]");
    m_sts |= onWriteCommand(((uint32_t)cmd[2] << 24) | ((uint32_t)cmd[3] << 16) | ((uint32_t)cmd[4] << 8) | cmd[5], ((uint32_t)cmd[7] << 8) | cmd[8]);
    break;
  case SCSI_SEEK10:
    LOGN("[Seek10]");
    break;
  case SCSI_MODE_SENSE10:
    LOGN("[ModeSense10]");
    m_sts |= onModeSenseCommand(cmd);
    break;
  case SCSI_MODE_SELECT6:
    LOGN("[ModeSelect6]");
    m_sts |= onModeSelect(cmd);
    break;
  case SCSI_MODE_SELECT10:
    LOGN("[ModeSelect10]");
    m_sts |= onModeSelect(cmd);
    break;
  case SCSI_READ_TOC:
    LOGN("[ReadTOC]");
    m_sts |= onReadTOC(cmd[2] & 0x02, cmd[6], ((uint32_t)cmd[7] << 8) | cmd[8]);
    break;
  case SCSI_READ_DVD_STRUCTURE:
    LOGN("[ReadDVDStructure]");
    m_sts |= 0x02;
    m_senseKey = 5;
    m_additional_sense_code = 0x3002; /* CANNOT READ MEDIUM - INCOMPATIBLE FORMAT */
    break;
  case SCSI_READ_DISC_INFORMATION:
    LOGN("[ReadDiscInformation]");
    writeDataPhase((cmd[7] >> 8) | cmd[8], m_buf);
    break;
#if SCSI_SELECT == 1
  case 0xc2:
    LOGN("[DTC510B setDriveParameter]");
    m_sts |= dtc510b_setDriveparameter();
    break;
#endif    
  default:
    LOGN("[*Unknown]");
    m_sts |= 0x02;
    m_senseKey = 5;
    break;
  }
  if(m_isBusReset) {
     goto BusFree;
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
inline void LBAtoMSF(uint32_t lba, byte *msf)
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


inline uint32_t MSFtoLBA(byte *msf)
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
