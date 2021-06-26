#include <Arduino.h>

#include "BlueSCSI.h"
#include "scsi_phase.h"
#include "scsi_msgs.h"
#include "scsi_cmds.h"

extern BUS_PHASE phase;
extern volatile bool m_IsBusReset;

void processMessage();

// Phase helpers
void onBusFree()
{
    if(phase != BUS_FREE)
    {
        LOGN("BusFree");
        phase = BUS_FREE;
    }
}

void onArbitration()
{
    LOGN("Abitration");
    phase = ARBITRATION;

    // Read the Initiator SCSI id
    byte scsi_id = READ_DATA_BUS();
    LOG("I:");
    LOGN(scsi_id);
}

void onSelection()
{
    if(phase == SELECTION) { return; }
    LOGN("Selection");

    phase = SELECTION;

    // Read the Initiator and Target SCSI ids
    // while(SCSI_IN(vBSY)) { if(m_IsBusReset) return; }
    // byte scsi_id = READ_DATA_BUS();
    //LOG("I:");
    //LOGN(scsi_id);
    while( isHigh(gpio_read(SEL)) && isLow(gpio_read(BSY))) { if(m_IsBusReset) return; }
}

void onReselection()
{
    LOGN("Reselection");
    phase = RESELECTION;

    // Read the Initiator and Target SCSI ids
    while(isHigh(gpio_read(BSY))) { if(phase == BUS_FREE) return; }
    byte scsi_id = READ_DATA_BUS();
    LOG("I:");
    LOGN(scsi_id);
    while(isHigh(gpio_read(BSY))) { if(phase == BUS_FREE) return; }
}

unsigned onCommand()
{
    LOG("Command ");
    phase = COMMAND;

    unsigned len;
    byte cmd[12];
    byte lun = 0;
    cmd[0] = readHandshake(); if(phase == BUS_FREE) return 0;
    LOGHEXN(cmd[0]);

    return 0;
#if 0
    // Command length selection, reception
    static const byte cmd_class_len[8]={6,10,10,6,6,12,6,6};
    len = cmd_class_len[cmd[0] >> 5];
    cmd[1] = readHandshake(); LOG(":");LOGHEX(cmd[1]); if(phase == BUS_FREE) return 0;
    cmd[2] = readHandshake(); LOG(":");LOGHEX(cmd[2]); if(phase == BUS_FREE) return 0;
    cmd[3] = readHandshake(); LOG(":");LOGHEX(cmd[3]); if(phase == BUS_FREE) return 0;
    cmd[4] = readHandshake(); LOG(":");LOGHEX(cmd[4]); if(phase == BUS_FREE) return 0;
    cmd[5] = readHandshake(); LOG(":");LOGHEX(cmd[5]); if(phase == BUS_FREE) return 0;
    // Receive the remaining commands
    if(len > 6)
    {
        for(int i = 6; i < len; i++ ) {
            cmd[i] = readHandshake();
            LOG(":");
            LOGHEX(cmd[i]);
            if(phase == BUS_FREE) return 0;
        }
    }
    LOGN("");
    // ATN means change phase
    if(isHigh(gpio_read(ATN))) { phase == MESSAGE_OUT; return 0; }

    // LUN confirmation
    lun = (cmd[1] & 0xe0) >> 5;
    
    // get transfer len
    switch(len)
    {
        case 6:
            len = cmd[4];
            break;
        case 10:
            len = cmd[7] << 8;
            len |= cmd[8];
            break;
        case 12:
            LOGN("12");
            break;
        default:
            LOG("Unsupported CDB ");
            LOGN(len);
    }

    LOG(":ID ");
    // LOG(id);
    LOG(":LUN ");
    LOG(lun);

    LOGN("");
    switch(cmd[0])
    {
    case SCSI_TEST_UNIT_READY:
        LOGN("[Test Unit]");
        break;
    case SCSI_REZERO_UNIT:
        LOGN("[Rezero Unit]");
        break;
    case SCSI_REQUEST_SENSE:
        LOGN("[RequestSense]");
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
        break;
    case SCSI_WRITE6:
        LOGN("[Write6]");
        break;
    case SCSI_SEEK6:
        LOGN("[Seek6]");
        break;
    case SCSI_INQUIRY:
        LOGN("[Inquiry]");
        break;
    case SCSI_MODE_SENSE6:
        LOGN("[ModeSense6]");
        break;
    case SCSI_START_STOP_UNIT:
        LOGN("[StartStopUnit]");
        break;
    case SCSI_PREVENT_ALLOW_REMOVAL:
        LOGN("[PreAllowMed.Removal]");
        break;
    case SCSI_READ_CAPACITY:
        LOGN("[ReadCapacity]");
        break;
    case SCSI_READ10:
        LOGN("[Read10]");
        break;
    case SCSI_WRITE10:
        LOGN("[Write10]");
        break;
    case SCSI_SEEK10:
        LOGN("[Seek10]");
        break;
    case SCSI_MODE_SENSE10:
        LOGN("[ModeSense10]");
        break;
    case SCSI_MODE_SELECT6:
        LOGN("[ModeSelect6]");
        break;
    case SCSI_MODE_SELECT10:
        LOGN("[ModeSelect10]");
        break;
    case SCSI_READ_TOC:
        LOGN("[ReadTOC]");
        break;
    case SCSI_READ_DVD_STRUCTURE:
        LOGN("[ReadDVDStructure]");
        break;
    case SCSI_READ_DISC_INFORMATION:
        LOGN("[ReadDiscInformation]");
        break;
    default:
        LOGN("[*Unknown]");
        break;
    }

    return len;
#endif
}

void onDataIn(unsigned len)
{
    LOGN("DataIn");
    phase = DATA_IN;
    byte data;

    while(len || phase == DATA_IN)
    {
        data = READ_DATA_BUS();
        if(isHigh(gpio_read(ATN))) { phase = MESSAGE_OUT; } 
        len--;
    }
}

void onDataOut(unsigned len)
{
    LOGN("DataOut");
    phase = DATA_OUT;
    byte data;

    while(len || phase == DATA_OUT)
    {
        data = readHandshake();
        if(isHigh(gpio_read(ATN))) { phase = MESSAGE_OUT; } 
        len--;
    }
}

void onStatus()
{
    LOGN("Status");
    phase = STATUS;
    byte data;

    data = readHandshake();
    if(isHigh(gpio_read(ATN))) { phase = MESSAGE_OUT; }

}

void onMessageIn()
{
    //LOG("MessageIn ");
    phase = MESSAGE_IN;

    // processMessage();
}


void onMessageOut()
{
    LOGN("MessageOut");
    phase = MESSAGE_OUT;

    // processMessage();

    do {} while(SCSI_IN(vATN));
}

void processMessage()
{
    byte message[255] = {0};
    int i = 0;
    int count = 0;

    while(isHigh(gpio_read(ATN)) && i++ < 255)
    {
        if(phase == BUS_FREE) return;
        message[count++] = readHandshake();
        count %= 256;
    }

    // ATN change phase
    if(isHigh(gpio_read(ATN))) { phase == MESSAGE_OUT; }

    LOG(i); LOG(" bytes ");
    for(int i = 0; i < count; i++)
    {
    switch(message[0])
    {
        case SCSI_MSG_ABORT:
            LOGN("ABORT");
            break;

        case SCSI_MSG_ABORT_TAG:
            LOGN("ABORT_TAG");
            break;
        
        case SCSI_MSG_BUS_CLEAR_QUEUE:
            LOGN("CLEAR_QUEUE");
            break;

        case SCSI_MSG_BUS_DEVICE_RESET:
            LOGN("BUS_DEVICE_RESET");
            break;

        case SCSI_MSG_COMMAND_COMPLETE:
            LOGN("COMMAND_COMPLETE");
            break;

        case SCSI_MSG_DISCONNECT:
            LOGN("DISCONNECT");
            break;

        case SCSI_MSG_IDENTIFY:
            LOGN("IDENTIFY");
            break;

        case SCSI_MSG_IGNORE_WIDE_RESIDUE:
            LOGN("IGNORE_WIDE_RESIDUE");
            break;

        case SCSI_MSG_INITIATE_RECOVERY:
            LOGN("INITIATE_RECOVERY");
            break;

        case SCSI_MSG_LINKED_COMMAND_COMPLETE:
            LOGN("LINKED_COMMAND_COMPLETE");
            break;
        
        case SCSI_MSG_LINKED_COMMAND_COMPLETE_WITH_FLAG:
            LOGN("LINKED_COMMAND_COMPLETE_WITH_FLAG");
            break;
        
        case SCSI_MSG_MESSAGE_PARITY_ERROR:
            LOGN("MESSAGE_PARITY_ERROR");
            break;

        case SCSI_MSG_MESSAGE_REJECT:
            LOGN("MESSAGE_REJECT");
            break;
        
        case SCSI_MSG_NO_OPERATION:
            LOGN("NO_OPERATION");
            break;

        case SCSI_MSG_HEAD_OF_QUEUE_TAG:
            LOGN("HEAD_OF_QUEUE_TAG");
            break;

        case SCSI_MSG_ORDERED_QUEUE_TAG:
            LOGN("ORDERED_QUEUE_TAG");
            break;

        case SCSI_MSG_SIMPLE_QUEUE_TAG:
            LOGN("SIMPLE_QUEUE_TAG");
            break;

        case SCSI_MSG_RELEASE_RECOVERY:
            LOGN("RELEASE_RECOVERY");
            break;

        case SCSI_MSG_RESTORE_POINTERS:
            LOGN("RESTORE_POINTERS");
            break;

        case SCSI_MSG_SAVE_DATA_POINTERS:
            LOGN("SAVE_DATA_POINTERES");
            break;

        case SCSI_MSG_TERMINATE_IO_PROCESS:
            LOGN("TERMINATE_IO_PROCESS");
            break;
            
        case SCSI_MSG_EXTENDED:
            LOG("EXTENDED => ");
            switch(message[1])
            {
                case SCSI_EXTENDED_MSG_EXTENDED_IDENTIFY:
                    LOGN("EXTENDED_INDENTIFY");
                    break;

                case SCSI_EXTENDED_MSG_MODIFY_DATA_POINTER:
                    LOGN("MODIFY_DATA_POINTER");
                    break;
                
                case SCSI_EXTENDED_MSG_SDTR:
                    LOGN("SDTR");
                    break;

                case SCSI_EXTENDED_MSG_WDTR:
                    LOGN("WDTR");
                    break;

                default:
                    LOG("UNKNOWN ");
                    LOGHEXN(message[1]);
            }

        default:
            LOG("UNKNOWN ");
            LOGHEXN(message[0]);
    }
    }
}