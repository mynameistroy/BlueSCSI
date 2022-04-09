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

#define DEBUG            1      // 0:No debug information output
                                // 1: Debug information output available
#define VERSION "jokker-2021-11-14"
#define LOG_FILENAME "LOG.txt"

#include "BlueSCSI.h"
#include "scsi_cmds.h"
#include "scsi_sense.h"
#include "scsi_status.h"
#include <setjmp.h>

#ifdef USE_STM32_DMA
#warning "warning USE_STM32_DMA"
#endif

FsFile LOG_FILE;
SdFs SD;

SCSI_DEVICE scsi_device_list[NUM_SCSIID][MAX_SCSILUN]; // Maximum number

volatile bool m_isBusReset = false;         // Bus reset

byte          scsi_id_mask;           // Mask list of responding SCSI IDs
byte          m_buf[MAX_BLOCKSIZE] = {0xff}; // General purpose buffer + overrun fetch
unsigned      m_msc;
byte          m_msb[256];             // Command storage bytes
volatile bool m_resetJmp = false;   // Call longjmp on reset
jmp_buf m_resetJmpBuf;

static byte onUnimplemented(SCSI_DEVICE *dev, const byte *cdb)
{
  // does nothing!
  if(Serial)
  {
    Serial.print("Unimplemented SCSI command: ");
    Serial.println(cdb[0], 16);
  }

  dev->m_senseKey = SCSI_SENSE_ILLEGAL_REQUEST;
  dev->m_additional_sense_code = SCSI_ASC_INVALID_OPERATION_CODE;
  return SCSI_STATUS_CHECK_CONDITION;
}

static byte onNOP(SCSI_DEVICE *dev, const byte *cdb)
{
  dev->m_senseKey = 0;
  dev->m_additional_sense_code = 0;
  return SCSI_STATUS_GOOD;
}

// function table
byte (*scsi_command_table[0xff])(SCSI_DEVICE *dev, const byte *cdb);

// scsi command functions
static byte onRequestSense(SCSI_DEVICE *dev, const byte *cdb);
static byte onRead6(SCSI_DEVICE *dev, const byte *cdb);
static byte onRead10(SCSI_DEVICE *dev, const byte *cdb);
static byte onWrite6(SCSI_DEVICE *dev, const byte *cdb);
static byte onWrite10(SCSI_DEVICE *dev, const byte *cdb);
static byte onInquiry(SCSI_DEVICE *dev, const byte *cdb);
static byte onModeSense(SCSI_DEVICE *dev, const byte *cdb);
static byte onReadCapacity(SCSI_DEVICE *dev, const byte *cdb);
static byte onModeSense(SCSI_DEVICE *dev, const byte *cdb);
static byte onModeSelect(SCSI_DEVICE *dev, const byte *cdb);
static byte onReadTOC(SCSI_DEVICE *dev, const byte *cdb);
static byte onReadDVDStructure(SCSI_DEVICE *dev, const byte *cdb);
static byte onReadDiscInformation(SCSI_DEVICE *dev, const byte *cdb);

static uint32_t MSFtoLBA(const byte *msf);
static void LBAtoMSF(const uint32_t lba, byte *msf);

static void onBusReset(void);
static void initFileLog(void);
static void finalizeFileLog(void);

static void flashError(const unsigned error)
{
  while(true) {
    for(uint8_t i = 0; i < error; i++) {
      LED_ON();
      delay(250);
      LED_OFF();
      delay(250);
    }
    delay(3000);
  }
}

// If config file exists, read the first three lines and copy the contents.
// File must be well formed or you will get junk in the SCSI Vendor fields.
static void readSCSIDeviceConfig(SCSI_DEVICE *dev) {
  FsFile config_file = SD.open("scsi-config.txt", O_RDONLY);
  SCSI_INQUIRY_DATA *inquiry_block = &(dev->inquiry_block);
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


bool VerifyISOPVD(SCSI_DEVICE *dev, unsigned sector_size, bool mode2)
{ 
  int seek = 16 * sector_size;
  if(sector_size == CDROM_RAW_SECTORSIZE && mode2) seek += 16;
  if(mode2) seek += 24;

  dev->m_file->seekSet(seek);
  dev->m_file->read(m_buf, 2048);

  return ((m_buf[0] == 1 && !strncmp((char *)&m_buf[1], "CD001", 5) && m_buf[6] == 1) ||
          (m_buf[8] == 1 && !strncmp((char *)&m_buf[9], "CDROM", 5) && m_buf[14] == 1));
}
/*
 * Open HDD image file
 */

static bool ImageOpen(SCSI_DEVICE *dev, const char *image_name)
{
  if(!dev)
  {
    return false;
  }

  dev->m_file = new FsFile(SD.open(image_name, O_RDWR));
  
  if(!dev->m_file->isOpen())
  {
    return false;
  }

  dev->m_fileSize = dev->m_file->size();
  LOG_FILE.print("Imagefile: ");
  LOG_FILE.print(image_name);
  if(dev->m_fileSize < 1)
  {
    LOG_FILE.println("FileSizeError");
    goto failed;
  }

  if(dev->m_type == SCSI_TYPE_CDROM)
  {
    LOG_FILE.print(" CDROM");

    // Borrowed from PCEM
    if(VerifyISOPVD(dev, CDROM_COMMON_SECTORSIZE, false))
    {
      dev->m_blocksize = CDROM_COMMON_SECTORSIZE;
      dev->m_mode2 = false;
    }
    else if(VerifyISOPVD(dev, CDROM_RAW_SECTORSIZE, false))
    {
      dev->m_blocksize = CDROM_RAW_SECTORSIZE;
      dev->m_mode2 = false;
      dev->m_raw = true;
    }
    else if(VerifyISOPVD(dev, 2336, true))
    {
      dev->m_blocksize = 2336;
      dev->m_mode2 = true;
    }
    else if(VerifyISOPVD(dev, CDROM_RAW_SECTORSIZE, true))
    {
      dev->m_blocksize = CDROM_RAW_SECTORSIZE;
      dev->m_mode2 = true;
      dev->m_raw = true;
    }
    else
    {
      // Last ditch effort
      byte header[12] = {0};
      byte sync[12] = {0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0};

      dev->m_file->rewind();
      if(!dev->m_file->readBytes(header, sizeof(header)))
      {
        LOG_FILE.println(" FileReadError");
        goto failed;
      }
      
      if(memcmp(sync, header, sizeof(header)) != 0)
      {
        LOG_FILE.println(" InvalidISO");
        goto failed;
      }

      dev->m_raw = true;
      
      // Size must be a multiple of 2536 and less than 700MB
      if(dev->m_fileSize % CDROM_RAW_SECTORSIZE || dev->m_fileSize > 912579600)
      {
        LOG_FILE.println(" InvalidISO");
        goto failed;
      }

      dev->m_blocksize = CDROM_RAW_SECTORSIZE;   
    }
  }
  else
  {
    LOG_FILE.print(" HDD");
  }
  dev->m_blockcount = dev->m_fileSize / dev->m_blocksize;

  // check blocksize dummy file
  LOG_FILE.print(" / ");
  LOG_FILE.print(dev->m_fileSize);
  LOG_FILE.print("bytes / ");
  LOG_FILE.print(dev->m_fileSize / 1024);
  LOG_FILE.print("KiB / ");
  LOG_FILE.print(dev->m_fileSize / 1024 / 1024);
  LOG_FILE.print("MiB");

  if(dev->m_type == SCSI_TYPE_CDROM)
  {
    LOG_FILE.print(" MODE2:");LOG_FILE.print(dev->m_mode2);
    LOG_FILE.print(" BlockSize:");LOG_FILE.println(dev->m_blocksize);
  }
  LOG_FILE.println();

  return true; // File opened
  
  failed:
  dev->m_file->close();
  delete dev->m_file;
  dev->m_file = NULL;
  dev->m_fileSize = dev->m_blocksize = 0; // no file

  return false;
}

/*
 * Initialization.
 *  Initialize the bus and set the PIN orientation
 */
void setup()
{
  SdFile root, file;

  for(unsigned i = 0; i < 0xff; i++)
  {
    scsi_command_table[i] = onUnimplemented;
  }

  // Setup BSSR table
  for (unsigned i = 0; i <= 255; i++)
  {
    db_bsrr[i] = DBP(i);
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
  enableDebugPorts();

  // Serial initialization
#if DEBUG
  Serial.begin(9600);
  // while (!Serial);
#endif

  // PIN initialization
  gpio_mode(LED, GPIO_OUTPUT_OD);
  //gpio_write(LED, low);
  LED_OFF();

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

  
  scsi_id_mask = 0x00;

  // Iterate over the root path in the SD card looking for candidate image files.
  
  root.open("/");
 
   while (1) {
    byte id, lun;
    char name[MAX_FILE_PATH+1];
    SCSI_DEVICE *dev;
    
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
      dev = &scsi_device_list[id][lun];
      dev->m_blocksize = name[HDIMG_BLK_POS] - '0';
      
      if(file_name.startsWith("hd") && file_name.endsWith(".hda"))
      {
        dev->m_type = SCSI_TYPE_HDD;
        
        switch(dev->m_blocksize)
        {
          case 1:
            dev->m_blocksize = 1024;
            break;
          case 2:
            dev->m_blocksize = 256;
            break;
          default:
            dev->m_blocksize = 512;
        }
      }
      else if(file_name.startsWith("cd") && file_name.endsWith(".iso"))
      {
        dev->m_type = SCSI_TYPE_CDROM;
      } 
      else
      {
        LOG_FILE.print("Not a recognized image type ");
        LOG_FILE.println(name);
        LOG_FILE.sync();
        continue;
      }

      memset(&dev->inquiry_block, 0, sizeof(dev->inquiry_block));
      if(ImageOpen(dev, name))
      {
        // Marked as a responsive ID
        scsi_id_mask |= 1<<id;

        switch(dev->m_type)
        {
          case SCSI_TYPE_HDD:
          // default SCSI HDD
          dev->inquiry_block.ansi_version = 2;
          dev->inquiry_block.response_format = 2;
          dev->inquiry_block.additional_length = 31;
          memcpy(dev->inquiry_block.vendor, "QUANTUM", 7);
          memcpy(dev->inquiry_block.product, "FIREBALL1", 9);
          memcpy(dev->inquiry_block.revision, "1.0", 3);
          break;
          
          case SCSI_TYPE_CDROM:
          // default SCSI CDROM
          dev->inquiry_block.peripheral_device_type = 5;
          dev->inquiry_block.rmb = 1;
          dev->inquiry_block.ansi_version = 1;
          dev->inquiry_block.response_format = 1;
          dev->inquiry_block.additional_length = 42;
          dev->inquiry_block.sync = 1;
          memcpy(dev->inquiry_block.vendor, "BLUESCSI", 8);
          memcpy(dev->inquiry_block.product, "CD-ROM CDU-55S", 14);
          memcpy(dev->inquiry_block.revision, "1.9a", 4);
          dev->inquiry_block.release = 0x20;
          memcpy(dev->inquiry_block.revision_date, "1995", 4);
          break;
        }

        readSCSIDeviceConfig(dev);
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
  attachInterrupt(RST, onBusReset, FALLING);
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
      SCSI_DEVICE *dev = &scsi_device_list[id][lun];
      if( (lun<NUM_SCSILUN) && (dev->m_file))
      {
        LOG_FILE.print((dev->m_blocksize<1000) ? ": " : ":");
        LOG_FILE.print(dev->m_blocksize);
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
 * Return from exception and call longjmp
 */
void __attribute__ ((noinline)) longjmpFromInterrupt(jmp_buf jmpb, int retval) __attribute__ ((noreturn));
void longjmpFromInterrupt(jmp_buf jmpb, int retval) {
  // Address of longjmp with the thumb bit cleared
  const uint32_t longjmpaddr = ((uint32_t)longjmp) & 0xfffffffe;
  const uint32_t zero = 0;
  // Default PSR value, function calls don't require any particular value
  const uint32_t PSR = 0x01000000;
  // For documentation on what this is doing, see:
  // https://developer.arm.com/documentation/dui0552/a/the-cortex-m3-processor/exception-model/exception-entry-and-return
  // Stack frame needs to have R0-R3, R12, LR, PC, PSR (from bottom to top)
  // This is being set up to have R0 and R1 contain the parameters passed to longjmp, and PC is the address of the longjmp function.
  // This is using existing stack space, rather than allocating more, as longjmp is just going to unroll the stack even further.
  // 0xfffffff9 is the EXC_RETURN value to return to thread mode.
  asm (
      "str %0, [sp];\
      str %1, [sp, #4];\
      str %2, [sp, #8];\
      str %2, [sp, #12];\
      str %2, [sp, #16];\
      str %2, [sp, #20];\
      str %3, [sp, #24];\
      str %4, [sp, #28];\
      ldr lr, =0xfffffff9;\
      bx lr"
      :: "r"(jmpb),"r"(retval),"r"(zero), "r"(longjmpaddr), "r"(PSR)
  );
}

/*
 * Bus reset interrupt.
 */
static void onBusReset(void)
{

  if(isHigh(gpio_read(RST))) {
    SCSI_RESET_HOLD();
    if(isHigh(gpio_read(RST))) {
      // BUS FREE is done in the main process
      SCSI_DB_INPUT()
      LOGN("BusReset!");
      if (m_resetJmp) {
        m_resetJmp = false;
        // Jumping out of the interrupt handler, so need to clear the interupt source.
        uint8 exti = PIN_MAP[RST].gpio_bit;
        EXTI_BASE->PR = (1U << exti);
        longjmpFromInterrupt(m_resetJmpBuf, 1);
      } else {
        m_isBusReset = true;
      }
    }
  }
}

/*
 * Enable the reset longjmp, and check if reset fired while it was disabled.
 */
void enableResetJmp(void) {
  m_resetJmp = true;
  if (m_isBusReset) {
    longjmp(m_resetJmpBuf, 1);
  }
}





/*
 * Read by handshake.
 */
static inline byte readHandshake(void)
{
  SCSI_OUT(vREQ,active)
  //SCSI_DB_INPUT()
  while( ! SCSI_IN(vACK)) {}
  byte r = READ_DATA_BUS();
  SCSI_OUT(vREQ,inactive)
  while( SCSI_IN(vACK)) {}
  return r;  
}

/*
 * Write with a handshake.
 */
static inline void writeHandshake(byte d)
{
  SCSI_DB_OUTPUT()
  GPIOB->regs->BSRR = db_bsrr[d];
  SCSI_DESKEW(); SCSI_CABLE_SKEW();
  SCSI_OUT(vREQ,active)
  while(!SCSI_IN(vACK));
  SCSI_OUT(vREQ, inactive);
  while( SCSI_IN(vACK)) {}
  SCSI_DB_INPUT()
}

/*
 * Data in phase.
 *  Send len bytes of data array p.
 */
static void writeDataPhase(int len, const byte* p)
{
  LOGN("DATAIN PHASE");
 
  SCSI_PHASE_DATA_IN();
  SCSI_BUS_SETTLE();

  for (int i = 0; i < len; i++) {
    writeHandshake(p[i]);
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
  SCSI_BUS_SETTLE();
  
  for(unsigned i = 0; i < len; i++)
    p[i] = readHandshake();
}

#if 1
#pragma GCC push_options
#pragma GCC optimize ("-Os")
/*
 * This loop is tuned to repeat the following pattern:
 * 1) Set REQ
 * 2) 5 cycles of work/delay
 * 3) Wait for ACK
 * Cycle time tunings are for 72MHz STM32F103
 * Alignment matters. For the 3 instruction wait loops,it looks like crossing
 * an 8 byte prefetch buffer can add 2 cycles of wait every branch taken.
 */
// void writeDataLoop(uint32_t blocksize) __attribute__ ((aligned(8)));
void writeDataLoop(uint32_t blocksize)
{
#define REQ_ON() (port_b->BRR = req_bit);
#define FETCH_BSRR_DB() (bsrr_val = bsrr_tbl[*srcptr++])
#define REQ_OFF_DB_SET(BSRR_VAL) port_b->BSRR = BSRR_VAL;
#define WAIT_ACK_ACTIVE()   while((*port_a_idr>>(vACK&15)&1))
#define WAIT_ACK_INACTIVE() while(!(*port_a_idr>>(vACK&15)&1))

  register byte *srcptr= m_buf;                 // Source buffer
  register byte *endptr= m_buf + blocksize;     // End pointer

  register const uint32_t *bsrr_tbl = db_bsrr;  // Table to convert to BSRR
  register uint32_t bsrr_val;                   // BSRR value to output (DB, DBP, REQ = ACTIVE)

  register uint32_t req_bit = BITMASK(vREQ);
  register gpio_reg_map *port_b = PBREG;
  register volatile uint32_t *port_a_idr = &(GPIOA->regs->IDR);

  // Start the first bus cycle.
  FETCH_BSRR_DB();
  REQ_OFF_DB_SET(bsrr_val);
  REQ_ON();
  FETCH_BSRR_DB();
  WAIT_ACK_ACTIVE();
  REQ_OFF_DB_SET(bsrr_val);
  // Align the starts of the do/while and WAIT loops to an 8 byte prefetch.
  asm("nop.w;nop");
  do{
    WAIT_ACK_INACTIVE();
    REQ_ON();
    // 4 cycles of work
    FETCH_BSRR_DB();
    // Extra 1 cycle delay while keeping the loop within an 8 byte prefetch.
    asm("nop");
    WAIT_ACK_ACTIVE();
    REQ_OFF_DB_SET(bsrr_val);
    // Extra 1 cycle delay, plus 4 cycles for the branch taken with prefetch.
    asm("nop");
  }while(srcptr < endptr);
  WAIT_ACK_INACTIVE();
  // Finish the last bus cycle, byte is already on DB.
  REQ_ON();
  WAIT_ACK_ACTIVE();
  REQ_OFF_DB_SET(bsrr_val);
  WAIT_ACK_INACTIVE();
}
#pragma GCC pop_options

#pragma GCC push_options
#pragma GCC optimize ("-Os")
    
/*
 * See writeDataLoop for optimization info.
 */
// void readDataLoop(uint32_t blockSize) __attribute__ ((aligned(16)));
void readDataLoop(uint32_t blockSize)
{
  register byte *dstptr= m_buf;
  register byte *endptr= m_buf + blockSize - 1;

#define REQ_ON() (port_b->BRR = req_bit);
#define REQ_OFF() (port_b->BSRR = req_bit);
#define WAIT_ACK_ACTIVE()   while((*port_a_idr>>(vACK&15)&1))
#define WAIT_ACK_INACTIVE() while(!(*port_a_idr>>(vACK&15)&1))
  register uint32_t req_bit = BITMASK(vREQ);
  register gpio_reg_map *port_b = PBREG;
  register volatile uint32_t *port_a_idr = &(GPIOA->regs->IDR);
  REQ_ON();
  // Start of the do/while and WAIT are already aligned to 8 bytes.
  do {
    WAIT_ACK_ACTIVE();
    uint32_t ret = port_b->IDR;
    REQ_OFF();
    *dstptr++ = ~(ret >> 8);
    // Move wait loop in to a single 8 byte prefetch buffer
    asm("nop.w;nop");
    WAIT_ACK_INACTIVE();
    REQ_ON();
    // Extra 1 cycle delay
    asm("nop");
  } while(dstptr<endptr);
  WAIT_ACK_ACTIVE();
  uint32_t ret = GPIOB->regs->IDR;
  REQ_OFF();
  *dstptr = ~(ret >> 8);
  WAIT_ACK_INACTIVE();
}
#pragma GCC pop_options

/* 
 * Data in phase.
 *  Send len block while reading from SD card.
 */
static void writeDataPhaseSD(SCSI_DEVICE *dev, uint32_t adds, uint32_t len)
{
  LOGN("DATAIN PHASE(SD)");
  uint32_t pos = adds * dev->m_blocksize;
  //register byte *srcptr;
  //register byte *endptr;
  unsigned block_ptr = 0;

  SCSI_PHASE_DATA_IN();
  dev->m_file->seek(pos);

  SCSI_DB_OUTPUT();

  for(uint32_t i = 0; i < len; i++)
  {
    m_resetJmp = false;
    dev->m_file->read(m_buf, dev->m_blocksize);
    enableResetJmp();
    writeDataLoop(dev->m_blocksize);
  }
  SCSI_DB_INPUT()
}

/*
 * Data out phase.
 *  Write to SD card while reading len block.
 */
static void readDataPhaseSD(SCSI_DEVICE *dev, uint32_t adds, uint32_t len)
{
  LOGN("DATAOUT PHASE(SD)");
  uint32_t pos = adds * dev->m_blocksize;
  uint32_t buffer_ptr = 0;

  SCSI_PHASE_DATA_OUT();
  dev->m_file->seek(pos);

  for(uint32_t i = 0; i < len; i++) {
     m_resetJmp = true;
     readDataLoop(dev->m_blocksize);
     m_resetJmp = false;
    dev->m_file->write(m_buf, dev->m_blocksize);
    if(m_isBusReset) { break; }
  }
  dev->m_file->flush();
  enableResetJmp();
}

#else

#pragma GCC push_options
#pragma GCC optimize ("-Os")
void writeDataLoop(uint32_t blocksize) __attribute__ ((optimize("align-functions=8")));
void writeDataLoop(uint32_t blocksize)
{
   // Asynchronous reads will make it faster ...
    register byte *srcptr = m_buf;         // Source buffer
    register byte *endptr = srcptr + blocksize; // End pointer
    register const uint32_t *bsrr_tbl = db_bsrr;  // Table to convert to BSRR
    register uint32_t req_bit = BITMASK(REQ);
    register gpio_reg_map *port_b = PBREG;
    register volatile uint32_t *port_a_idr = &(GPIOA->regs->IDR);

  #define REQ_ON() (port_b->BRR = req_bit);
  #define WAIT_ACK_ACTIVE() while((*port_a_idr>>(vACK&15)&1));
  #define WAIT_ACK_INACTIVE() while(!(*port_a_idr>>(vACK&15)&1));
  //#define REQ_ON() SCSI_OUT(vREQ,active);
  //#define WAIT_ACK_ACTIVE() while(!SCSI_IN(vACK));
  //#define WAIT_ACK_INACTIVE() while(SCSI_IN(vACK));

    #define DATA_TRANSFER() \
      GPIOB->regs->BSRR = bsrr_tbl[*srcptr++]; \
      SCSI_DESKEW(); SCSI_CABLE_SKEW(); \     
      REQ_ON(); \
      WAIT_ACK_ACTIVE(); \
      SCSI_OUT(vREQ, inactive); \
      WAIT_ACK_INACTIVE();

    do
    {
      // 16 bytes per loop
      DATA_TRANSFER();
      /* DATA_TRANSFER();
      DATA_TRANSFER();
      DATA_TRANSFER();
      DATA_TRANSFER();
      DATA_TRANSFER();
      DATA_TRANSFER();
      DATA_TRANSFER(); */
    } while(srcptr < endptr);
}
#pragma GCC pop_options

/* 
 * Data in phase.
 *  Send len block while reading from SD card.
 */
void writeDataPhaseSD(SCSI_DEVICE *dev, uint32_t adds, uint32_t len)
{
  LOGN("DATAIN PHASE(SD)");
  uint32_t pos = adds * dev->m_blocksize;
  //register byte *srcptr;
  //register byte *endptr;
  unsigned block_ptr = 0;

  SCSI_PHASE_DATA_IN();
  dev->m_file->seek(pos);

  SCSI_DB_OUTPUT();

  for(uint32_t i = 0; i < len; i++)
  {
    m_resetJmp = false;
    dev->m_file->read(m_buf, dev->m_blocksize);
    enableResetJmp();

    writeDataLoop(dev->m_blocksize);
  }
  SCSI_DB_INPUT()
}

/*
 * Data out phase.
 *  Write to SD card while reading len block.
 */
void readDataPhaseSD(SCSI_DEVICE *dev, uint32_t adds, uint32_t len)
{
  LOGN("DATAOUT PHASE(SD)");
  uint32_t pos = adds * dev->m_blocksize;
  uint32_t buffer_ptr = 0;

  SCSI_PHASE_DATA_OUT();
  dev->m_file->seek(pos);

  for(uint32_t i = 0; i < len; i++) {
  register byte *dstptr= m_buf + buffer_ptr;
	register byte *endptr= dstptr + dev->m_blocksize;

    for(;dstptr<endptr;dstptr+=8) {
      dstptr[0] = readHandshake();
      dstptr[1] = readHandshake();
      dstptr[2] = readHandshake();
      dstptr[3] = readHandshake();
      dstptr[4] = readHandshake();
      dstptr[5] = readHandshake();
      dstptr[6] = readHandshake();
      dstptr[7] = readHandshake();
    }
    buffer_ptr += dev->m_blocksize;
    if(buffer_ptr == sizeof(m_buf))
    {
      m_resetJmp = false;
      dev->m_file->write(m_buf, sizeof(m_buf));
      dev->m_file->flush();
      buffer_ptr = 0;
      enableResetJmp();
    }
  }
  if(buffer_ptr)
  {
      m_resetJmp = false;
      dev->m_file->write(m_buf, buffer_ptr);
      dev->m_file->flush();
      enableResetJmp();
  }
}
#endif
/*
 * MsgIn2.
 */
static void MsgIn2(unsigned msg)
{
  LOGN("MsgIn2");
 
  SCSI_PHASE_MSG_IN();
  SCSI_BUS_SETTLE();
  writeHandshake(msg);
}

/*
 * MsgOut2.
 */
static void MsgOut2()
{
  LOGN("MsgOut2");
  SCSI_PHASE_MSG_OUT();
  SCSI_BUS_SETTLE();
  m_msb[m_msc] = readHandshake();
  m_msc++;
  m_msc %= 256;
}

/*
 * Main loop.
 */
void loop() 
{
  byte m_id = 0;                   // Currently responding SCSI-ID
  byte m_lun = 0;                  // Logical unit number currently responding
  byte m_sts = 0;                  // Status byte
  byte m_msg = 0;              // Message bytes
 // HDD Image selection
  SCSI_DEVICE *dev = (SCSI_DEVICE *)0; // HDD image for current SCSI-ID, LUN
  unsigned len = 0;
  byte cmd[12] = {0};
  byte scsiid = 0;
  const int cmd_class_len[8]={6,10,10,6,6,12,6,6}; // SCSI command size lookup table

  // Wait until RST = H, BSY = H, SEL = L
  do {} while( SCSI_IN(vBSY) || !SCSI_IN(vSEL) || SCSI_IN(vRST));
  
  // BSY+ SEL-
  // If the ID to respond is not driven, wait for the next
  scsiid = READ_DATA_BUS() & scsi_id_mask;
  if((scsiid) == 0) {
    SCSI_DISCONNECTION_DELAY();
    return;
  }
  
  LOGN("Selection");
  m_isBusReset = false;
  if (setjmp(m_resetJmpBuf) == 1) {
    LOGN("Reset, going to BusFree");
    goto BusFree;
  }
  enableResetJmp();
  
  // Set BSY to-when selected
  SCSI_BSY_ACTIVE();     // Turn only BSY output ON, ACTIVE

  // Ask for a TARGET-ID to respond
  m_id = 31 - __builtin_clz(scsiid);

  // Wait until SEL becomes inactive
  while(isHigh(gpio_read(SEL)) && isLow(gpio_read(BSY))) {}

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
        m_lun = m_msb[i] & 0x1f;
        if(m_lun >= NUM_SCSILUN)
        {
          SCSI_DEVICE *d = &scsi_device_list[m_id][m_lun];
          d->m_senseKey = SCSI_SENSE_ILLEGAL_REQUEST;
          d->m_additional_sense_code = SCSI_ASC_LOGICAL_UNIT_NOT_SUPPORTED;
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
 
  LOG("Command:");
  SCSI_PHASE_COMMAND();
  SCSI_BUS_SETTLE();
  
  cmd[0] = readHandshake();
  // Command length selection, reception

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

  finished_command_bytes:

  // LUN confirmation
  m_lun = (cmd[1] & 0xe0) >> 5;

  dev = &(scsi_device_list[m_id][m_lun]);
  
  if(m_lun >= NUM_SCSILUN || !dev->m_file)
  {
    // REQUEST SENSE and INQUIRY are handled different with invalid LUNs
    if(cmd[0] != SCSI_REQUEST_SENSE || cmd[0] != SCSI_INQUIRY)
    {
      dev->m_senseKey = SCSI_SENSE_ILLEGAL_REQUEST;
      dev->m_additional_sense_code = SCSI_ASC_LOGICAL_UNIT_NOT_SUPPORTED;
      m_sts = SCSI_STATUS_CHECK_CONDITION;
      goto Status;
    }

    if(cmd[0] == SCSI_INQUIRY)
    {
      // Special INQUIRY handling for invalid LUNs
      LOGN("onInquiry - InvalidLUN");
      dev = &(scsi_device_list[m_id][0]);

      byte temp = dev->inquiry_block.raw[0];

      // If the LUN is invalid byte 0 of inquiry block needs to be 7fh
      dev->inquiry_block.raw[0] = 0x7f;

      // only write back what was asked for
      writeDataPhase(cmd[4], dev->inquiry_block.raw);

      // return it back to normal if it was altered
      dev->inquiry_block.raw[0] = temp;

      m_sts = SCSI_STATUS_GOOD;
      goto Status;
    }
  }

  LOG(":ID ");
  LOG(m_id);
  LOG(":LUN ");
  LOGN(m_lun);

  LED_ON();
  m_sts = scsi_command_table[cmd[0]](dev, cmd);
  
Status:
  //LOGN("Sts");
  SCSI_PHASE_STATUS();
  SCSI_BUS_SETTLE();
  writeHandshake(m_sts);

  //LOGN("MsgIn");
  SCSI_PHASE_MSG_IN();
  SCSI_BUS_SETTLE();
  writeHandshake(m_msg);

BusFree:
  //LOGN("BusFree");
  LED_OFF();
  m_isBusReset = false;
  SCSI_TARGET_INACTIVE() // Turn off BSY, REQ, MSG, CD, IO output
}

/*
 * INQUIRY command processing.
 */
static byte onInquiry(SCSI_DEVICE *dev, const byte *cdb)
{
  LOGN("onInquiry");

  // only write back what was asked for
  writeDataPhase(cdb[4], dev->inquiry_block.raw);

  return SCSI_STATUS_GOOD;
}

/*
 * REQUEST SENSE command processing.
 */
static byte onRequestSense(SCSI_DEVICE *dev, const byte *cdb)
{
  LOGN("onRequestSense");
  byte buf[18] = {
    0x70,   //CheckCondition
    0,      //Segment number
    dev->m_senseKey,   //Sense key
    0, 0, 0, 0,  //information
    10,   //Additional data length
    0, 0, 0, 0, // command specific information bytes
    (byte)(dev->m_additional_sense_code >> 8),
    (byte)dev->m_additional_sense_code,
    0, 0, 0, 0,
  };
  
  // Reset sense data
  dev->m_senseKey = 0;
  dev->m_additional_sense_code = 0;

  writeDataPhase(sizeof(buf), buf);  
  return SCSI_STATUS_GOOD;
}

/*
 * READ CAPACITY command processing.
 */
static byte onReadCapacity(SCSI_DEVICE *dev, const byte *cdb)
{
  LOGN("onReadCapacity");
  uint8_t buf[8] = {
    dev->m_blockcount >> 24, dev->m_blockcount >> 16, dev->m_blockcount >> 8, dev->m_blockcount,
    dev->m_blocksize >> 24, dev->m_blocksize >> 16, dev->m_blocksize >> 8, dev->m_blocksize    
  };
  writeDataPhase(8, buf);
  return SCSI_STATUS_GOOD;
}

/*
 * READ6 / 10 Command processing.
 */
static byte onRead6(SCSI_DEVICE *dev, const byte *cdb)
{
  unsigned adds = (((uint32_t)cdb[1] & 0x1F) << 16) | ((uint32_t)cdb[2] << 8) | cdb[3];
  unsigned len = (cdb[4] == 0) ? 0x100 : cdb[4];
  /*
  LOGN("onRead6");
  LOG("-R ");
  LOGHEX(adds);
  LOG(":");
  LOGHEXN(len);
  */
  
  writeDataPhaseSD(dev, adds, len);
  return SCSI_STATUS_GOOD;
}

static byte onRead10(SCSI_DEVICE *dev, const byte *cdb)
{
  unsigned adds = ((uint32_t)cdb[2] << 24) | ((uint32_t)cdb[3] << 16) | ((uint32_t)cdb[4] << 8) | cdb[5];
  unsigned len = ((uint32_t)cdb[7] << 8) | cdb[8];
  /*
  LOGN("onRead10");
  LOG("-R ");
  LOGHEX(adds);
  LOG(":");
  LOGHEXN(len);
  */

  writeDataPhaseSD(dev, adds, len);
  return SCSI_STATUS_GOOD;
}

/*
 * WRITE6 / 10 Command processing.
 */
static byte onWrite6(SCSI_DEVICE *dev, const byte *cdb)
{
  unsigned adds = (((uint32_t)cdb[1] & 0x1F) << 16) | ((uint32_t)cdb[2] << 8) | cdb[3];
  unsigned len = (cdb[4] == 0) ? 0x100 : cdb[4];
  /*
  LOGN("onWrite6");
  LOG("-W ");
  LOGHEX(adds);
  LOG(":");
  LOGHEXN(len);
  */

  if(dev->m_type == SCSI_TYPE_CDROM)
  {
    dev->m_senseKey = SCSI_SENSE_HARDWARE_ERROR;
    dev->m_additional_sense_code = SCSI_ASC_WRITE_PROTECTED; // Write Protect
    return SCSI_STATUS_CHECK_CONDITION;
  }

  readDataPhaseSD(dev, adds, len);
  return SCSI_STATUS_GOOD;
}

static byte onWrite10(SCSI_DEVICE *dev, const byte *cdb)
{
  unsigned adds = ((uint32_t)cdb[2] << 24) | ((uint32_t)cdb[3] << 16) | ((uint32_t)cdb[4] << 8) | cdb[5];
  unsigned len = ((uint32_t)cdb[7] << 8) | cdb[8];
  /*
  LOGN("onWrite10");
  LOG("-W ");
  LOGHEX(adds);
  LOG(":");
  LOGHEXN(len);
  */

  if(dev->m_type == SCSI_TYPE_CDROM)
  {
    dev->m_senseKey = SCSI_SENSE_HARDWARE_ERROR;
    dev->m_additional_sense_code = SCSI_ASC_WRITE_PROTECTED; // Write Protect
    return SCSI_STATUS_CHECK_CONDITION;
  }

  readDataPhaseSD(dev, adds, len);
  return SCSI_STATUS_GOOD;
}

static byte onReadDVDStructure(SCSI_DEVICE *dev, const byte *cdb)
{
    LOGN("onReadDVDStructure");
    dev->m_senseKey = SCSI_SENSE_ILLEGAL_REQUEST;
    dev->m_additional_sense_code = SCSI_ASC_CANNOT_READ_MEDIUM_INCOMPATIBLE_FORMAT; /* CANNOT READ MEDIUM - INCOMPATIBLE FORMAT */
    return SCSI_STATUS_CHECK_CONDITION;
}

/*
 * MODE SENSE command processing.
 */
static byte onModeSense(SCSI_DEVICE *dev, const byte *cdb)
{
  LOGN("onModeSense");
  unsigned len = 0;
  byte page_code = cdb[2] & 0x3f;
  byte changeable = false;
  byte dbd = cdb[1] & 0x80;
  unsigned a = 0;
  bool unsupported_page_code = false;

  memset(m_buf, 0, len);

  if(cdb[0] == SCSI_MODE_SENSE6)
  {
    len = cdb[4];

    // mode sense data header
    a += 4;
    m_buf[3] = 8; // block descriptor length
  }
  else /* SCSI_MODE_SENSE10 */
  {
    len = cdb[7];
    len <<= 8;
    len |= cdb[8];
    if(len > 0x800) { len = 0x800; }

    // mode sense data header
    a += 8;
    m_buf[7] = 8; // block descriptor length
  }

  if((cdb[2] & 0xc0) == 0x40)
  {
    changeable = true;
  }
 
  if(dbd)
  {   
    m_buf[a + 5] = (byte)dev->m_blocksize >> 16;
    m_buf[a + 6] = (byte)dev->m_blocksize >> 8;
    m_buf[a + 7] = (byte)dev->m_blocksize;

    a += 8;
  }

  // HDD supports page codes 0x1 (Read/Write), 0x2, 0x3, 0x4
  // CDROM supports page codes 0x1 (Read Only), 0x2, 0xD, 0xE, 0x30
  if(dev->m_type == SCSI_TYPE_HDD)
  {
    switch(page_code)
    {
      case 0x3F:
      case 0x01: // Read/Write Error Recovery
        m_buf[a + 0] = 0x01;
        m_buf[a + 1] = 0x0A;
        if(!changeable)
        {
          m_buf[a + 2] = 0x26;
        }
        a += 0x0A;
        if(page_code != 0x3F) break;


      case 0x02: // Disconnect-Reconnect page
        m_buf[a + 0] = 0x02;
        m_buf[a + 1] = 0x0A;
        if(!changeable)
        {
          m_buf[a + 5] = 10;
        }
        a += 0x0A;
        if(page_code != 0x3F) break;

      case 0x03:  //Drive parameters
        m_buf[a + 0] = 0x83; //Page code
        m_buf[a + 1] = 0x16; // Page length
        if(!changeable)
        {
          m_buf[a + 11] = 63;//Number of sectors / track
          m_buf[a + 12] = (byte)dev->m_blocksize >> 8;
          m_buf[a + 13] = (byte)dev->m_blocksize;
          m_buf[a + 15] = 0x1;
          m_buf[a + 20] = 0xC0;
        }
        else
        {
          m_buf[a + 12] = 0xFF;
          m_buf[a + 13] = 0xFF;
        }
        a += 0x16;
        if(page_code != 0x3F) break;
        
      /*
	        cylinder = LBA / (heads_per_cylinder * sectors_per_track)
	        temp = LBA % (heads_per_cylinder * sectors_per_track)
	        head = temp / sectors_per_track
	        sector = temp % sectors_per_track + 1
      */
      case 0x04:  //Drive parameters
      {
        unsigned cylinders = dev->m_blockcount / (16 * 63);
        m_buf[a + 0] = 0x04; //Page code
        m_buf[a + 1] = 0x16; // Page length
        if(!changeable)
        {
          m_buf[a + 2] = (byte)cylinders >> 16; // Cylinder length
          m_buf[a + 3] = (byte)cylinders >> 8;
          m_buf[a + 4] = (byte)cylinders;
          m_buf[a + 5] = 16;   //Number of heads
          m_buf[a + 13] = 1; // step rate
          m_buf[a + 20] = (byte)7200 >> 8;
          m_buf[a + 21] = (byte)7200;
        }
        else
        {
          m_buf[a + 2] = 0xFF; // Cylinder length
          m_buf[a + 3] = 0xFF;
          m_buf[a + 4] = 0xFF;
          m_buf[a + 5] = 16;   //Number of heads
        }
        a += 0x16;
      }
      if(page_code != 0x3F) break;
      
      // This is to always break at the end so that select page codes that are
      // unsupported can return an error
      case 0xFF:
        break;

      default:
        unsupported_page_code = true;
    }
  }
  else // CDROM
  {
    
    if(cdb[0] == SCSI_MODE_SENSE6)
    {
      m_buf[2] = 1 << 7; // WP bit
    }
    else
    {
      m_buf[3] = 1 << 7; // WP bit
    }
    
    switch(page_code)
    {
      case 0x3f:
      case 0x01: // Read error recovery
        m_buf[a + 0] = 0x01;
        m_buf[a + 1] = 0x06;
        if(!changeable)
        {
          m_buf[a + 3] = 0x05;
        }
        a += 0x06;
        if(page_code != 0x3F) break;

      case 0x02: // Disconnect-Reconnect page
        m_buf[a + 0] = 0x02;
        m_buf[a + 1] = 0x0A;
        if(!changeable)
        {
          m_buf[a + 4] = 0x0A;
        }
        a += 0x0C;
        if(page_code != 0x3f) break;

      case 0x0D: // CDROM parameters
        m_buf[a + 0] = 0x0D;
        m_buf[a + 1] = 0x06;
        if(!changeable)
        {
        // 2 seconds for inactive timer
        m_buf[a + 3] = 0x05;

        // MSF multiples are 60 and 75
        m_buf[a + 5] = 60;
        m_buf[a + 7] = 75;
        }
        a += 0x6;
        if(page_code != 0x3f) break;

      case 0x0E: // CDROM audio control parameters
        m_buf[a + 0] = 0x0E;
        m_buf[a + 1] = 0x0E;

        a += 0xE;
        if(page_code != 0x3f) break;

      case 0x30: // magic Apple page Thanks to bitsavers for the info
        {
          const byte apple_magic[0x24] =
          {
            0x23, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x08, 0x00, 0x30, 0x16, 0x41, 0x50,
            0x50, 0x4C, 0x45, 0x20, 0x43, 0x4F, 0x4D, 0x50,
            0x55, 0x54, 0x45, 0x52, 0x2C, 0x20, 0x49, 0x4E,
            0x43, 0x20, 0x20, 0x20
          };
          if(!changeable)
          {
            memcpy(&m_buf[a], apple_magic, 0x24);
          }
          a += 0x24;
          LOGN("Apple special MODE SENSE page");
          if(page_code != 0x3f) break;
        }

      // This is to always break at the end so that select page codes that are
      // unsupported can return an error
      case 0xFF:
        break;

      default:
        unsupported_page_code = true;
    }
  }

  if(unsupported_page_code)
  {
    // We don't support this page code
    dev->m_senseKey = SCSI_SENSE_ILLEGAL_REQUEST;
    dev->m_additional_sense_code = SCSI_ASC_INVALID_FIELD_IN_CDB;
    return SCSI_STATUS_CHECK_CONDITION;
  }

  if(cdb[0] == SCSI_MODE_SENSE10)
  {
    m_buf[0] = (byte)((a - 2) >> 8);
    m_buf[1] = (byte)(a - 2);
  }
  else
  {
    m_buf[0] = a - 1;
  }
  writeDataPhase(len < a ? len : a, m_buf);
  return SCSI_STATUS_GOOD;
}

static byte onReadTOC(SCSI_DEVICE *dev, const byte *cdb)
{
  LOGN("onReadTOC");

  unsigned lba = 0;
  uint8_t msf = cdb[1] & 0x02;
  uint8_t track = cdb[6];
  unsigned len = ((uint32_t)cdb[7] << 8) | cdb[8];
  memset(m_buf, 0, len);
  // Doing just the error seemed to make MacOS unhappy
#if 0
  dev->m_senseKey = SCSI_SENSE_ILLEGAL_REQUEST;
  dev->m_additional_sense_code = SCSI_ASC_INVALID_FIELD_IN_CDB;
  return SCSI_STATUS_CHECK_CONDITION;
#endif
    if(track > 1 || cdb[2] != 0)
    {
      dev->m_senseKey = SCSI_SENSE_ILLEGAL_REQUEST;
      dev->m_additional_sense_code = SCSI_ASC_INVALID_FIELD_IN_CDB;
      return SCSI_STATUS_CHECK_CONDITION;
    }
    
    m_buf[1] = 18; // TOC length LSB
    m_buf[2] = 1; // First Track
    m_buf[3] = 1; // Last Track
    
    // first track
    m_buf[5] = 0x14; // data track
    m_buf[6] = 1; 
    
    // leadout track 
    m_buf[13] = 0x14; // data track
    m_buf[14] = 0xaa; // leadout track
    if(msf)
    {
      LBAtoMSF(dev->m_blockcount, &m_buf[16]);
    }
    else
    {
      m_buf[16] = (byte)(dev->m_blockcount >> 24);
      m_buf[17] = (byte)(dev->m_blockcount >> 16);
      m_buf[18] = (byte)(dev->m_blockcount >> 8);
      m_buf[20] = (byte)(dev->m_blockcount);
    }
    
    writeDataPhase(SCSI_TOC_LENGTH > len ? len : SCSI_TOC_LENGTH, m_buf);
    return SCSI_STATUS_GOOD;
}

static byte onModeSelect(SCSI_DEVICE *dev, const byte *cdb)
{
  unsigned length = 0;

  LOGN("onModeSelect");
  if(dev->m_type != SCSI_TYPE_HDD && (cdb[1] & 0x01))
  {
    dev->m_senseKey = SCSI_SENSE_ILLEGAL_REQUEST;
    dev->m_additional_sense_code = SCSI_ASC_INVALID_FIELD_IN_CDB;
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

  memset(m_buf, 0, length);
  writeDataPhase(length, m_buf);
  return SCSI_STATUS_GOOD;
}

static byte onReadDiscInformation(SCSI_DEVICE *dev, const byte *cdb)
{
  LOGN("onReadDiscInformation");
  writeDataPhase((cdb[7] >> 8) | cdb[8], m_buf);
  return SCSI_STATUS_GOOD;
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
