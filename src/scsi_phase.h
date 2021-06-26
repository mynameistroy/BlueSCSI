#ifndef __SCSI_PHASE_H__
#define __SCSI_PHASE_H__

typedef enum _BUS_PHASE
{
  BUS_FREE = 0,
  ARBITRATION,
  SELECTION,
  RESELECTION,
  COMMAND,
  DATA_IN,
  DATA_OUT,
  STATUS,
  MESSAGE_IN,
  MESSAGE_OUT
} BUS_PHASE;

#define INFO_XFER_DATA_OUT      0
#define INFO_XFER_DATA_IN       1
#define INFO_XFER_COMMAND       2
#define INFO_XFER_STATUS        3
// RESERVED                     4
// RESERVED                     5
#define INFO_XFER_MSG_OUT       6
#define INFO_XFER_MSG_IN        7

// Phase helpers
void onBusFree();
void onArbitration();
void onSelection();
void onReselection();
unsigned onCommand();
void onDataIn(unsigned len);
void onDataOut(unsigned len);
void onStatus();
void onMessageIn();
void onMessageOut();

#endif;