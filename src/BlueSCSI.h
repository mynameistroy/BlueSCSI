#ifndef __BLUESCSI_H__
#define __BLUESCSI_H__

#include <Arduino.h> // For Platform.IO
#include <SdFat.h>

#define DEBUG            1      // 0:No debug information output
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
#define MAX_BLOCKSIZE 2352     // Maximum BLOCK size

// SDFAT
#define SD1_CONFIG SdSpiConfig(PA4, DEDICATED_SPI, SD_SCK_MHZ(SPI_FULL_SPEED), &SPI)

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

// SCSI input pin check (inactive=0,active=1)
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

#define VERSION "1.0-20210410"
#define ERROR_FALSE_INIT  3
#define ERROR_NO_SDCARD   5

#define READ_DATA_BUS() (byte)((~GPIOB->regs->IDR)>>8)

#define PHASE_BUSFREE         0
#define PHASE_ARBITRATION     1
#define PHASE_SELECTION       0x02
#define PHASE_MESSAGE_OUT     0x04
#define PHASE_COMMAND         0x08
#define PHASE_DATA_IN         0x10
#define PHASE_DATA_OUT        0x20
#define PHASE_STATUS          0x40
#define PHASE_RESELECTION     0x80

extern unsigned _SCSI_BUS_PHASE;
#define PHASE_CHANGE(x) (_SCSI_BUS_PHASE = (~_SCSI_BUS_PHASE) & x)
#define PHASE_CHECK(x) (_SCSI_BUS_PHASE & x ? 1 : 0)


byte readIO(void);
byte readHandshake(void);

#endif