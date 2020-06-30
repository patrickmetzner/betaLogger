/* Includes ------------------------------------------------------------------*/
#include "diskio.h"
#include "stm32_ff_link.h"

static volatile DSTATUS Stat = STA_NOINIT;
static volatile uint32_t diskInitialized = 0;

#define BLOCK_SIZE						512

DSTATUS disk_status(BYTE pdrv)
{
	Stat = STA_NOINIT;

	if (BSP_SD_GetStatus() == MSD_OK) {
		Stat &= ~STA_NOINIT;
	}

	return Stat;
}

DSTATUS disk_initialize(BYTE pdrv)
{
	DSTATUS stat = RES_OK;

	if (isBitClr(diskInitialized, pdrv)) {
		setBit(diskInitialized, pdrv);
		Stat = STA_NOINIT;
		/* Configure the uSD device */
		if (BSP_SD_Init() == MSD_OK) {
			Stat &= ~STA_NOINIT;
		}
		return Stat;
	}
	return stat;
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
	DRESULT res = RES_OK;

	if (BSP_SD_ReadBlocks((uint32_t *)buff,
						  (uint64_t)(sector * BLOCK_SIZE),
						  BLOCK_SIZE,
						  count) != MSD_OK) {
		res = RES_ERROR;
	}

	return res;
}

#if _USE_WRITE == 1
DRESULT disk_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
	DRESULT res = RES_OK;

	if (BSP_SD_WriteBlocks((uint32_t *)buff,
						   (uint64_t)(sector * BLOCK_SIZE),
						   BLOCK_SIZE, count) != MSD_OK) {
		res = RES_ERROR;
	}

	return res;
}
#endif

#if _USE_IOCTL == 1
DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
	DRESULT res = RES_ERROR;
	SD_CardInfo CardInfo;

	if (Stat & STA_NOINIT) {
		return RES_NOTRDY;
	}

	switch (cmd) {
	/* Make sure that no pending write process */
	case CTRL_SYNC :
		res = RES_OK;
		break;

	/* Get number of sectors on the disk (DWORD) */
	case GET_SECTOR_COUNT :
		BSP_SD_GetCardInfo(&CardInfo);
		*(DWORD *)buff = CardInfo.CardCapacity / BLOCK_SIZE;
		res = RES_OK;
		break;

	/* Get R/W sector size (WORD) */
	case GET_SECTOR_SIZE :
		*(WORD *)buff = BLOCK_SIZE;
		res = RES_OK;
		break;

	/* Get erase block size in unit of sector (DWORD) */
	case GET_BLOCK_SIZE :
		*(DWORD *)buff = BLOCK_SIZE;
		break;

	default:
		res = RES_PARERR;
	}

	return res;
}
#endif

__weak DWORD get_fattime(void)
{
	return 0;
}
