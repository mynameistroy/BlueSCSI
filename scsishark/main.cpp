#define _CRT_SECURE_NO_WARNINGS 1

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <sys/types.h>
#include <errno.h>

#define BIT_BSY 0
#define BIT_SEL 1
#define BIT_ATN 2
#define BIT_MSG 3
#define BIT_CD 4
#define BIT_IO 5
#define BIT_REQ 6
#define BIT_ACK 7
#define _FILENAME "SCSI.DMP"

#define MASK(x) (1<<x)

enum _SCSI_PHASE
{
	Arbitration,
	Selection,
	MessageOut,
	MessageIn,
	DataIn,
	DataOut,
	Status,
	BusFree,
	Reselection,
};


int main(int argc, char** argv)
{
	FILE *scsi_log = NULL;
	enum _SCSI_PHASE phase = BusFree;

	scsi_log = fopen(_FILENAME, "rb");
	if (!scsi_log)
	{
		printf("Failed to open %s:%s\n", _FILENAME, strerror(errno));
		return EXIT_FAILURE;
	}

	while (!feof(scsi_log))
	{
		uint8_t phase = 0;
		uint32_t cycleoffset = 0;
		int bsy = 0, sel = 0, atn = 0, msg = 0, cd = 0, io = 0;

		// read phase
		fread(&phase, sizeof(phase), 1, scsi_log);
		bsy = (phase & MASK(BIT_BSY)) > 0;
		sel = (phase & MASK(BIT_SEL)) > 0;
		atn = (phase & MASK(BIT_ATN)) > 0;
		msg = (phase & MASK(BIT_MSG)) > 0;
		cd = (phase & MASK(BIT_CD)) > 0;
		io = (phase & MASK(BIT_IO)) > 0;

		fread(&cycleoffset, sizeof(cycleoffset), 1, scsi_log);
		printf("%8d - BSY:%d SEL:%d ATN:%d MSG:%d CD:%d IO:%d ",
			cycleoffset, bsy, sel, atn, msg, cd, io
		);
		if (phase == 0)
		{ 
			phase = BusFree;
			printf("Busfree\n");
		}
		else if (phase == BusFree && bsy && !sel)
		{
			phase = Arbitration;
			printf("Arbitration\n");
		}
		else if (phase == Arbitration && !bsy && sel)
		{
			phase = Selection;
			printf("Selection\n");
		}
		else if (phase > Arbitration && bsy && !sel)
		{
			uint8_t itp = phase >> 3;
			switch (itp)
			{
				case 0: printf("Data Out\n"); break;
				case 1: printf("Data In\n"); break;
				case 2: printf("Command\n"); break;
				case 3: printf("Status\n"); break;
				case 4:
				case 5:
					printf("Reserved!\n");
					break;
				case 6: printf("Message Out\n"); break;
				case 7: printf("Message In\n"); break;
				default:
					printf("Unknown\n");
			}
		}
		else
		{
			printf("\n");
		}
	}

	fclose(scsi_log);
}