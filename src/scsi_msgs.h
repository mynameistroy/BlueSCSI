#ifndef __SCSI_MSGS_H__
#define __SCSI_MSGS_H__

// One byte messages
#define SCSI_MSG_EXTENDED                           0x01
#define SCSI_MSG_ABORT                              0x06
#define SCSI_MSG_ABORT_TAG                          0x0d
#define SCSI_MSG_BUS_DEVICE_RESET                   0x0c
#define SCSI_MSG_BUS_CLEAR_QUEUE                    0x0e
#define SCSI_MSG_COMMAND_COMPLETE                   0x00
#define SCSI_MSG_DISCONNECT                         0x04
#define SCSI_MSG_IDENTIFY                           0x80
#define SCSI_MSG_INITIATE_RECOVERY                  0x0f
#define SCSI_MSG_LINKED_COMMAND_COMPLETE            0x0a
#define SCSI_MSG_LINKED_COMMAND_COMPLETE_WITH_FLAG  0x0b
#define SCSI_MSG_MESSAGE_PARITY_ERROR               0x09
#define SCSI_MSG_MESSAGE_REJECT                     0x07
#define SCSI_MSG_NO_OPERATION                       0x08
#define SCSI_MSG_RELEASE_RECOVERY                   0x10
#define SCSI_MSG_RESTORE_POINTERS                   0x03
#define SCSI_MSG_SAVE_DATA_POINTERS                 0x02
#define SCSI_MSG_TERMINATE_IO_PROCESS               0x11

// Two byte messages
#define SCSI_MSG_IGNORE_WIDE_RESIDUE                0x23
#define SCSI_MSG_HEAD_OF_QUEUE_TAG                  0x21
#define SCSI_MSG_ORDERED_QUEUE_TAG                  0x22
#define SCSI_MSG_SIMPLE_QUEUE_TAG                   0x20

// Extended messages
#define SCSI_EXTENDED_MSG_MODIFY_DATA_POINTER       0x00
#define SCSI_EXTENDED_MSG_SDTR                      0x01
#define SCSI_EXTENDED_MSG_EXTENDED_IDENTIFY         0x02
#define SCSI_EXTENDED_MSG_WDTR                      0x03

#endif