#include "stm32_ff_link.h"
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define SD_DUMMY_BYTE					0xFF
#define SD_NO_RESPONSE_EXPECTED			0x80
#define SD_PRESENT						((uint8_t)0x01)
#define SD_NOT_PRESENT					((uint8_t)0x00)
#define SD_MAX_TRY						100		// Number of try 
#define SD_CMD_LENGTH					6

#define SD_CMD_GO_IDLE_STATE							0		 // CMD0 = 0x40	
#define SD_CMD_SEND_OP_COND								1		 // CMD1 = 0x41	
#define SD_CMD_SEND_IF_COND								8		 // CMD8 = 0x48	
#define SD_CMD_SEND_CSD									9		 // CMD9 = 0x49	
#define SD_CMD_SEND_CID									10		// CMD10 = 0x4A 
#define SD_CMD_STOP_TRANSMISSION						12		// CMD12 = 0x4C 
#define SD_CMD_SEND_STATUS								13		// CMD13 = 0x4D 
#define SD_CMD_SET_BLOCKLEN								16		// CMD16 = 0x50 
#define SD_CMD_READ_SINGLE_BLOCK						17		// CMD17 = 0x51 
#define SD_CMD_READ_MULT_BLOCK							18		// CMD18 = 0x52 
#define SD_CMD_SET_BLOCK_COUNT							23		// CMD23 = 0x57 
#define SD_CMD_WRITE_SINGLE_BLOCK						24		// CMD24 = 0x58 
#define SD_CMD_WRITE_MULT_BLOCK							25		// CMD25 = 0x59 
#define SD_CMD_PROG_CSD									27		// CMD27 = 0x5B 
#define SD_CMD_SET_WRITE_PROT							28		// CMD28 = 0x5C 
#define SD_CMD_CLR_WRITE_PROT							29		// CMD29 = 0x5D 
#define SD_CMD_SEND_WRITE_PROT							30		// CMD30 = 0x5E 
#define SD_CMD_SD_ERASE_GRP_START						32		// CMD32 = 0x60 
#define SD_CMD_SD_ERASE_GRP_END							33		// CMD33 = 0x61 
#define SD_CMD_UNTAG_SECTOR								34		// CMD34 = 0x62 
#define SD_CMD_ERASE_GRP_START							35		// CMD35 = 0x63 
#define SD_CMD_ERASE_GRP_END							36		// CMD36 = 0x64 
#define SD_CMD_UNTAG_ERASE_GROUP						37		// CMD37 = 0x65 
#define SD_CMD_ERASE									38		// CMD38 = 0x66 
#define SD_CMD_SD_APP_OP_COND							41		// CMD41 = 0x69 
#define SD_CMD_APP_CMD									55		// CMD55 = 0x77 
#define SD_CMD_READ_OCR									58		// CMD55 = 0x79 

#define SD_TOKEN_START_DATA_SINGLE_BLOCK_READ			0xFE	// Data token start byte, Start Single Block Read 
#define SD_TOKEN_START_DATA_MULTIPLE_BLOCK_READ			0xFE	// Data token start byte, Start Multiple Block Read 
#define SD_TOKEN_START_DATA_SINGLE_BLOCK_WRITE			0xFE	// Data token start byte, Start Single Block Write 
#define SD_TOKEN_START_DATA_MULTIPLE_BLOCK_WRITE		0xFD	// Data token start byte, Start Multiple Block Write 
#define SD_TOKEN_STOP_DATA_MULTIPLE_BLOCK_WRITE			0xFD	// Data toke stop byte, Stop Multiple Block Write 

typedef struct {
	uint8_t r1;
	uint8_t r2;
	uint8_t r3;
	uint8_t r4;
	uint8_t r5;
} SD_CmdAnswer_typedef;

typedef enum {
	SD_ANSWER_R1_EXPECTED,
	SD_ANSWER_R1B_EXPECTED,
	SD_ANSWER_R2_EXPECTED,
	SD_ANSWER_R3_EXPECTED,
	SD_ANSWER_R4R5_EXPECTED,
	SD_ANSWER_R7_EXPECTED,
} SD_Answer_type;

typedef enum {
	// R1 answer value
	SD_R1_NO_ERROR						= (0x00),
	SD_R1_IN_IDLE_STATE					= (0x01),
	SD_R1_ERASE_RESET					= (0x02),
	SD_R1_ILLEGAL_COMMAND				= (0x04),
	SD_R1_COM_CRC_ERROR					= (0x08),
	SD_R1_ERASE_SEQUENCE_ERROR			= (0x10),
	SD_R1_ADDRESS_ERROR					= (0x20),
	SD_R1_PARAMETER_ERROR				= (0x40),

	// R2 answer value
	SD_R2_NO_ERROR						= 0x00,
	SD_R2_CARD_LOCKED					= 0x01,
	SD_R2_LOCKUNLOCK_ERROR				= 0x02,
	SD_R2_ERROR							= 0x04,
	SD_R2_CC_ERROR						= 0x08,
	SD_R2_CARD_ECC_FAILED				= 0x10,
	SD_R2_WP_VIOLATION					= 0x20,
	SD_R2_ERASE_PARAM					= 0x40,
	SD_R2_OUTOFRANGE					= 0x80,

	SD_DATA_OK							= (0x05),
	SD_DATA_CRC_ERROR					= (0x0B),
	SD_DATA_WRITE_ERROR					= (0x0D),
	SD_DATA_OTHER_ERROR					= (0xFF)
} SD_Error;


volatile uint8_t SdStatus = SD_NOT_PRESENT;
uint16_t flag_SDHC = 0;

static void SPIx_Init(void);
static void SPIx_GPIO_Init(void);
static void SPIx_WriteReadData(const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLength);
static void SPIx_WriteData(uint8_t *DataIn, uint16_t DataLength);
static void SPIx_Write(uint8_t Value);
static void SPIx_Error(void);
static void SPIx_FlushFifo(void);

static uint8_t SD_GoIdleState(void);
static SD_CmdAnswer_typedef SD_SendCmd(uint8_t Cmd, uint32_t Arg, uint8_t Crc, uint8_t Answer);
static uint8_t SD_ReadData(void);
static uint8_t SD_GetDataResponse(void);
static uint8_t SD_GetCIDRegister(SD_CID *Cid);
static uint8_t SD_GetCSDRegister(SD_CSD *Csd);
static uint8_t SD_WaitData(uint8_t data);


static void SPIx_Init(void)
{
	if (HAL_SPI_GetState(&SD_SPI_HANDLE) == HAL_SPI_STATE_RESET) {
		SPIx_GPIO_Init();
		/* SPI Config */
		/* SPI baudrate is set to 12 MHz maximum (PCLK1/SPI_BaudRatePrescaler = 48/4 = 12 MHz)
		 to verify these constraints:
		    - SD card SPI interface max baudrate is 25MHz for write/read
		    - PCLK1 max frequency is 48 MHz
		 */
		SD_SPI_HANDLE.Init.BaudRatePrescaler = SD_SPI_BAUD_RATE;
		SD_SPI_HANDLE.Init.CLKPhase = SPI_PHASE_2EDGE;
		SD_SPI_HANDLE.Init.CLKPolarity = SPI_POLARITY_HIGH;
		SD_SPI_HANDLE.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
// SD_SPI_HANDLE.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
// SD_SPI_HANDLE.Init.CRCPolynomial = 7;
		SD_SPI_HANDLE.Init.DataSize = SPI_DATASIZE_8BIT;
		SD_SPI_HANDLE.Init.Direction = SPI_DIRECTION_2LINES;
		SD_SPI_HANDLE.Init.FirstBit = SPI_FIRSTBIT_MSB;
		SD_SPI_HANDLE.Init.Mode = SPI_MODE_MASTER;
		SD_SPI_HANDLE.Init.NSS = SPI_NSS_SOFT;
// SD_SPI_HANDLE.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
		SD_SPI_HANDLE.Init.TIMode = SPI_TIMODE_DISABLE;
		SD_SPI_HANDLE.Instance = SD_SPI_INSTANCE;

		HAL_SPI_Init(&SD_SPI_HANDLE);
	}
}

static void SPIx_GPIO_Init(void)
{
	GPIO_InitTypeDef gpioinitstruct = {0};

	/*** Configure the GPIOs ***/
	/* Enable GPIO clock */
	SD_SCLK_GPIO_CLK_ENABLE();
	SD_MISO_GPIO_CLK_ENABLE();
	SD_MOSI_GPIO_CLK_ENABLE();
	SD_CS_GPIO_CLK_ENABLE();

	/* Configure SPI SCLK */
	gpioinitstruct.Pin = SD_SCLK_PIN;
	gpioinitstruct.Mode = GPIO_MODE_AF_PP;
// gpioinitstruct.Pull  = GPIO_PULLUP;
	gpioinitstruct.Speed = GPIO_SPEED_FREQ_HIGH;
// gpioinitstruct.Alternate = SD_SCLK_AF;
	HAL_GPIO_Init(SD_SCLK_GPIO_PORT, &gpioinitstruct);

	/* Configure SPI MISO */
	gpioinitstruct.Pin = SD_MISO_PIN;
	gpioinitstruct.Mode = GPIO_MODE_INPUT;
	gpioinitstruct.Pull  = GPIO_NOPULL;
	HAL_GPIO_Init(SD_MISO_GPIO_PORT, &gpioinitstruct);

	/* Configure SPI MOSI */
	gpioinitstruct.Pin = SD_MOSI_PIN;
	gpioinitstruct.Mode = GPIO_MODE_AF_PP;
	gpioinitstruct.Pull  = GPIO_NOPULL;
	HAL_GPIO_Init(SD_MOSI_GPIO_PORT, &gpioinitstruct);

	/*** Configure the SPI peripheral ***/
	/* Enable SPI clock */
	SD_SPI_CLK_ENABLE();
}

static void SPIx_WriteReadData(const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLength)
{
	HAL_StatusTypeDef status = HAL_OK;

	status = HAL_SPI_TransmitReceive(&SD_SPI_HANDLE, (uint8_t *) DataIn, DataOut, DataLength, SD_SPI_TIMEOUT_MAX);

	/* Check the communication status */
	if (status != HAL_OK) {
		/* Execute user timeout callback */
		SPIx_Error();
	}
}

static void SPIx_WriteData(uint8_t *DataIn, uint16_t DataLength)
{
	HAL_StatusTypeDef status = HAL_OK;

	status = HAL_SPI_Transmit(&SD_SPI_HANDLE, DataIn, DataLength, SD_SPI_TIMEOUT_MAX);

	/* Check the communication status */
	if (status != HAL_OK) {
		/* Execute user timeout callback */
		SPIx_Error();
	}
}

static void SPIx_Write(uint8_t Value)
{
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t data;

	status = HAL_SPI_TransmitReceive(&SD_SPI_HANDLE, (uint8_t *) &Value, &data, 1, SD_SPI_TIMEOUT_MAX);

	/* Check the communication status */
	if (status != HAL_OK) {
		/* Execute user timeout callback */
		SPIx_Error();
	}
}

static void SPIx_Error(void)
{
	/* De-initialize the SPI communication BUS */
	HAL_SPI_DeInit(&SD_SPI_HANDLE);

	/* Re-Initiaize the SPI communication BUS */
	SPIx_Init();
}

static void SPIx_FlushFifo(void)
{
	HAL_SPIEx_FlushRxFifo(&SD_SPI_HANDLE);
}

/********************************* LINK SD ************************************/

void SD_IO_Init(void)
{
	uint8_t counter = 0;

	/*------------Put SD in SPI mode--------------*/
	/* SD SPI Config */
	SPIx_Init();

	/* SD chip select high */
	SD_CS_HIGH();

	/* Send dummy byte 0xFF, 10 times with CS high */
	/* Rise CS and MOSI for 80 clocks cycles */
	for (counter = 0; counter <= 9; counter++) {
		/* Send dummy byte 0xFF */
		SD_IO_WriteByte(SD_DUMMY_BYTE);
	}
}

void SD_IO_WriteReadData(const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLength)
{
	/* Send the byte */
	SPIx_WriteReadData(DataIn, DataOut, DataLength);
}

uint8_t SD_IO_WriteByte(uint8_t Data)
{
	uint8_t tmp;

	/* Send the byte */
	SPIx_WriteReadData(&Data, &tmp, 1);
	return tmp;
}

void SD_IO_ReadData(uint8_t *DataOut, uint16_t DataLength)
{
	/* Send the byte */
	SD_IO_WriteReadData(DataOut, DataOut, DataLength);
}

void SD_IO_WriteData(const uint8_t *Data, uint16_t DataLength)
{
	/* Send the byte */
	SPIx_WriteData((uint8_t *)Data, DataLength);
	SPIx_FlushFifo();
}

uint8_t BSP_SD_Init(void)
{
	// Configure IO functionalities for SD pin
	SD_IO_Init();

	// SD detection pin is not physically mapped on the Adafruit shield
	SdStatus = SD_PRESENT;

	// SD initialized and set to SPI mode properly
	return SD_GoIdleState();
}

uint8_t BSP_SD_ReadBlocks(uint32_t *pData, uint32_t ReadAddr, uint16_t BlockSize, uint32_t NumberOfBlocks)
{
	uint32_t offset = 0;
	uint8_t retr = BSP_SD_ERROR;
	uint8_t *ptr = NULL;
	SD_CmdAnswer_typedef response;

	// Send CMD16 (SD_CMD_SET_BLOCKLEN) to set the size of the block and
	// Check if the SD acknowledged the set block length command: R1 response (0x00: no errors)
	response = SD_SendCmd(SD_CMD_SET_BLOCKLEN, BlockSize, 0xFF, SD_ANSWER_R1_EXPECTED);
	SD_CS_HIGH();
	SD_IO_WriteByte(SD_DUMMY_BYTE);
	if (response.r1 != SD_R1_NO_ERROR) {
		goto error;
	}

	ptr = malloc(sizeof(uint8_t) * BlockSize);
	if (ptr == NULL) {
		goto error;
	}
	memset(ptr, SD_DUMMY_BYTE, sizeof(uint8_t)*BlockSize);

	// Data transfer
	while (NumberOfBlocks--) {
		// Send CMD17 (SD_CMD_READ_SINGLE_BLOCK) to read one block
		// Check if the SD acknowledged the read block command: R1 response (0x00: no errors)
		response = SD_SendCmd(SD_CMD_READ_SINGLE_BLOCK, (ReadAddr + offset) / (flag_SDHC == 1 ? BlockSize : 1), 0xFF,
							  SD_ANSWER_R1_EXPECTED);
		if (response.r1 != SD_R1_NO_ERROR) {
			goto error;
		}

		// Now look for the data token to signify the start of the data
		if (SD_WaitData(SD_TOKEN_START_DATA_SINGLE_BLOCK_READ) == BSP_SD_OK) {
			// Read the SD block data : read NumByteToRead data
			SD_IO_WriteReadData(ptr, (uint8_t *)pData + offset, BlockSize);

			// Set next read address
			offset += BlockSize;
			// get CRC bytes (not really needed by us, but required by SD)
			SD_IO_WriteByte(SD_DUMMY_BYTE);
			SD_IO_WriteByte(SD_DUMMY_BYTE);
		} else {
			goto error;
		}

		// End the command data read cycle
		SD_CS_HIGH();
		SD_IO_WriteByte(SD_DUMMY_BYTE);
	}

	retr = BSP_SD_OK;

error :
	// Send dummy byte: 8 Clock pulses of delay
	SD_CS_HIGH();
	SD_IO_WriteByte(SD_DUMMY_BYTE);
	if (ptr != NULL) {
		free(ptr);
	}

	// Return the reponse
	return retr;
}

uint8_t BSP_SD_WriteBlocks(uint32_t *pData, uint32_t WriteAddr, uint16_t BlockSize, uint32_t NumberOfBlocks)
{
	uint32_t offset = 0;
	uint8_t retr = BSP_SD_ERROR;
	uint8_t *ptr = NULL;
	SD_CmdAnswer_typedef response;

	// Send CMD16 (SD_CMD_SET_BLOCKLEN) to set the size of the block and
	// Check if the SD acknowledged the set block length command: R1 response (0x00: no errors)
	response = SD_SendCmd(SD_CMD_SET_BLOCKLEN, BlockSize, 0xFF, SD_ANSWER_R1_EXPECTED);
	SD_CS_HIGH();
	SD_IO_WriteByte(SD_DUMMY_BYTE);
	if (response.r1 != SD_R1_NO_ERROR) {
		goto error;
	}

	ptr = malloc(sizeof(uint8_t) * BlockSize);
	if (ptr == NULL) {
		goto error;
	}

	// Data transfer
	while (NumberOfBlocks--) {
		// Send CMD24 (SD_CMD_WRITE_SINGLE_BLOCK) to write blocks	and
		// Check if the SD acknowledged the write block command: R1 response (0x00: no errors)
		response = SD_SendCmd(SD_CMD_WRITE_SINGLE_BLOCK, (WriteAddr + offset) / (flag_SDHC == 1 ? BlockSize : 1), 0xFF,
							  SD_ANSWER_R1_EXPECTED);
		if (response.r1 != SD_R1_NO_ERROR) {
			goto error;
		}

		// Send dummy byte for NWR timing : one byte between CMDWRITE and TOKEN
		SD_IO_WriteByte(SD_DUMMY_BYTE);
		SD_IO_WriteByte(SD_DUMMY_BYTE);

		// Send the data token to signify the start of the data
		SD_IO_WriteByte(SD_TOKEN_START_DATA_SINGLE_BLOCK_WRITE);

		// Write the block data to SD
		SD_IO_WriteReadData((uint8_t *)pData + offset, ptr, BlockSize);

		// Set next write address
		offset += BlockSize;

		// Put CRC bytes (not really needed by us, but required by SD)
		SD_IO_WriteByte(SD_DUMMY_BYTE);
		SD_IO_WriteByte(SD_DUMMY_BYTE);

		// Read data response
		if (SD_GetDataResponse() != SD_DATA_OK) {
			// Set response value to failure
			goto error;
		}

		SD_CS_HIGH();
		SD_IO_WriteByte(SD_DUMMY_BYTE);
	}
	retr = BSP_SD_OK;

error :
	if (ptr != NULL) {
		free(ptr);
	}
	// Send dummy byte: 8 Clock pulses of delay
	SD_CS_HIGH();
	SD_IO_WriteByte(SD_DUMMY_BYTE);

	// Return the reponse
	return retr;
}

uint8_t BSP_SD_GetCardInfo(SD_CardInfo *pCardInfo)
{
	uint8_t status;

	status = SD_GetCSDRegister(&(pCardInfo->Csd));
	status |= SD_GetCIDRegister(&(pCardInfo->Cid));
	if (flag_SDHC == 1) {
		pCardInfo->CardBlockSize = 512;
		pCardInfo->CardCapacity = (pCardInfo->Csd.version.v2.DeviceSize + 1) * pCardInfo->CardBlockSize;
	} else {
		pCardInfo->CardCapacity = (pCardInfo->Csd.version.v1.DeviceSize + 1) ;
		pCardInfo->CardCapacity *= (1 << (pCardInfo->Csd.version.v1.DeviceSizeMul + 2));
		pCardInfo->CardBlockSize = 1 << (pCardInfo->Csd.RdBlockLen);
		pCardInfo->CardCapacity *= pCardInfo->CardBlockSize;
	}

	return status;
}

uint8_t BSP_SD_GetStatus(void)
{
	SD_CmdAnswer_typedef retr;

	// Send CMD13 (SD_SEND_STATUS) to get SD status
	retr = SD_SendCmd(SD_CMD_SEND_STATUS, 0, 0xFF, SD_ANSWER_R2_EXPECTED);
	SD_CS_HIGH();
	SD_IO_WriteByte(SD_DUMMY_BYTE);

	// Find SD status according to card state
	if ((retr.r1 == SD_R1_NO_ERROR) && (retr.r2 == SD_R2_NO_ERROR)) {
		return BSP_SD_OK;
	}

	return BSP_SD_ERROR;
}

uint8_t BSP_SD_Erase(uint32_t StartAddr, uint32_t EndAddr)
{
	uint8_t retr = BSP_SD_ERROR;
	SD_CmdAnswer_typedef response;

	// Send CMD32 (Erase group start) and check if the SD acknowledged the erase command: R1 response (0x00: no errors)
	response = SD_SendCmd(SD_CMD_SD_ERASE_GRP_START, StartAddr, 0xFF, SD_ANSWER_R1_EXPECTED);
	SD_CS_HIGH();
	SD_IO_WriteByte(SD_DUMMY_BYTE);
	if (response.r1 == SD_R1_NO_ERROR) {
		// Send CMD33 (Erase group end) and Check if the SD acknowledged the erase command: R1 response (0x00: no errors)
		response = SD_SendCmd(SD_CMD_SD_ERASE_GRP_END, EndAddr, 0xFF, SD_ANSWER_R1_EXPECTED);
		SD_CS_HIGH();
		SD_IO_WriteByte(SD_DUMMY_BYTE);
		if (response.r1 == SD_R1_NO_ERROR) {
			// Send CMD38 (Erase) and Check if the SD acknowledged the erase command: R1 response (0x00: no errors)
			response = SD_SendCmd(SD_CMD_ERASE, 0, 0xFF, SD_ANSWER_R1B_EXPECTED);
			if (response.r1 == SD_R1_NO_ERROR) {
				retr = BSP_SD_OK;
			}
			SD_CS_HIGH();
			SD_IO_WriteByte(SD_DUMMY_BYTE);
		}
	}

	// Return the reponse
	return retr;
}

static uint8_t SD_GoIdleState(void)
{
	SD_CmdAnswer_typedef response;
	volatile uint8_t counter = 0;
	// Send CMD0 (SD_CMD_GO_IDLE_STATE) to put SD in SPI mode and
	// wait for In Idle State Response (R1 Format) equal to 0x01
	do {
		counter++;
		response = SD_SendCmd(SD_CMD_GO_IDLE_STATE, 0, 0x95, SD_ANSWER_R1_EXPECTED);
		SD_CS_HIGH();
		SD_IO_WriteByte(SD_DUMMY_BYTE);
		if (counter >= SD_MAX_TRY) {
			return BSP_SD_ERROR;
		}
	} while (response.r1 != SD_R1_IN_IDLE_STATE);


	// Send CMD8 (SD_CMD_SEND_IF_COND) to check the power supply status
	// and wait until response (R7 Format) equal to 0xAA and
	response = SD_SendCmd(SD_CMD_SEND_IF_COND, 0x1AA, 0x87, SD_ANSWER_R7_EXPECTED);
	SD_CS_HIGH();
	SD_IO_WriteByte(SD_DUMMY_BYTE);
	if ((response.r1	& SD_R1_ILLEGAL_COMMAND) == SD_R1_ILLEGAL_COMMAND) {
		// initialise card V1
		do {
			// initialise card V1
			// Send CMD55 (SD_CMD_APP_CMD) before any ACMD command: R1 response (0x00: no errors)
			response = SD_SendCmd(SD_CMD_APP_CMD, 0x00000000, 0xFF, SD_ANSWER_R1_EXPECTED);
			SD_CS_HIGH();
			SD_IO_WriteByte(SD_DUMMY_BYTE);

			// Send ACMD41 (SD_CMD_SD_APP_OP_COND) to initialize SDHC or SDXC cards: R1 response (0x00: no errors)
			response = SD_SendCmd(SD_CMD_SD_APP_OP_COND, 0x00000000, 0xFF, SD_ANSWER_R1_EXPECTED);
			SD_CS_HIGH();
			SD_IO_WriteByte(SD_DUMMY_BYTE);
		} while (response.r1 == SD_R1_IN_IDLE_STATE);
		flag_SDHC = 0;
	} else if (response.r1 == SD_R1_IN_IDLE_STATE) {
		// initialise card V2
		do {

			// Send CMD55 (SD_CMD_APP_CMD) before any ACMD command: R1 response (0x00: no errors)
			response = SD_SendCmd(SD_CMD_APP_CMD, 0, 0xFF, SD_ANSWER_R1_EXPECTED);
			SD_CS_HIGH();
			SD_IO_WriteByte(SD_DUMMY_BYTE);

			// Send ACMD41 (SD_CMD_SD_APP_OP_COND) to initialize SDHC or SDXC cards: R1 response (0x00: no errors)
			response = SD_SendCmd(SD_CMD_SD_APP_OP_COND, 0x40000000, 0xFF, SD_ANSWER_R1_EXPECTED);
			SD_CS_HIGH();
			SD_IO_WriteByte(SD_DUMMY_BYTE);
		} while (response.r1 == SD_R1_IN_IDLE_STATE);

		if ((response.r1 & SD_R1_ILLEGAL_COMMAND) == SD_R1_ILLEGAL_COMMAND) {
			do {
				// Send CMD55 (SD_CMD_APP_CMD) before any ACMD command: R1 response (0x00: no errors)
				response = SD_SendCmd(SD_CMD_APP_CMD, 0, 0xFF, SD_ANSWER_R1_EXPECTED);
				SD_CS_HIGH();
				SD_IO_WriteByte(SD_DUMMY_BYTE);
				if (response.r1 != SD_R1_IN_IDLE_STATE) {
					return BSP_SD_ERROR;
				}
				// Send ACMD41 (SD_CMD_SD_APP_OP_COND) to initialize SDHC or SDXC cards: R1 response (0x00: no errors)
				response = SD_SendCmd(SD_CMD_SD_APP_OP_COND, 0x00000000, 0xFF, SD_ANSWER_R1_EXPECTED);
				SD_CS_HIGH();
				SD_IO_WriteByte(SD_DUMMY_BYTE);
			} while (response.r1 == SD_R1_IN_IDLE_STATE);
		}

		// Send CMD58 (SD_CMD_READ_OCR) to initialize SDHC or SDXC cards: R3 response (0x00: no errors)
		response = SD_SendCmd(SD_CMD_READ_OCR, 0x00000000, 0xFF, SD_ANSWER_R3_EXPECTED);
		SD_CS_HIGH();
		SD_IO_WriteByte(SD_DUMMY_BYTE);
		if (response.r1 != SD_R1_NO_ERROR) {
			return BSP_SD_ERROR;
		}
		flag_SDHC = (response.r2 & 0x40) >> 6;
	} else {
		return BSP_SD_ERROR;
	}

	return BSP_SD_OK;
}

static SD_CmdAnswer_typedef SD_SendCmd(uint8_t Cmd, uint32_t Arg, uint8_t Crc, uint8_t Answer)
{
	uint8_t frame[SD_CMD_LENGTH], frameout[SD_CMD_LENGTH];
	SD_CmdAnswer_typedef retr = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

	// R1 Lenght = NCS(0)+ 6 Bytes command + NCR(min1 max8) + 1 Bytes answer + NEC(0) = 15bytes
	// R1b identical to R1 + Busy information
	// R2 Lenght = NCS(0)+ 6 Bytes command + NCR(min1 max8) + 2 Bytes answer + NEC(0) = 16bytes

	// Prepare Frame to send
	frame[0] = (Cmd | 0x40);				 // Construct byte 1
	frame[1] = (uint8_t)(Arg >> 24); // Construct byte 2
	frame[2] = (uint8_t)(Arg >> 16); // Construct byte 3
	frame[3] = (uint8_t)(Arg >> 8);	// Construct byte 4
	frame[4] = (uint8_t)(Arg);			 // Construct byte 5
	frame[5] = (Crc | 0x01);				 // Construct byte 6

	// Send the command
	SD_CS_LOW();
	SD_IO_WriteReadData(frame, frameout, SD_CMD_LENGTH); // Send the Cmd bytes

	switch (Answer) {
	case SD_ANSWER_R1_EXPECTED :
		retr.r1 = SD_ReadData();
		break;
	case SD_ANSWER_R1B_EXPECTED :
		retr.r1 = SD_ReadData();
		retr.r2 = SD_IO_WriteByte(SD_DUMMY_BYTE);
		// Set CS High
		SD_CS_HIGH();
		HAL_Delay(1);
		// Set CS Low
		SD_CS_LOW();

		// Wait IO line return 0xFF
		while (SD_IO_WriteByte(SD_DUMMY_BYTE) != 0xFF);
		break;
	case SD_ANSWER_R2_EXPECTED :
		retr.r1 = SD_ReadData();
		retr.r2 = SD_IO_WriteByte(SD_DUMMY_BYTE);
		break;
	case SD_ANSWER_R3_EXPECTED :
	case SD_ANSWER_R7_EXPECTED :
		retr.r1 = SD_ReadData();
		retr.r2 = SD_IO_WriteByte(SD_DUMMY_BYTE);
		retr.r3 = SD_IO_WriteByte(SD_DUMMY_BYTE);
		retr.r4 = SD_IO_WriteByte(SD_DUMMY_BYTE);
		retr.r5 = SD_IO_WriteByte(SD_DUMMY_BYTE);
		break;
	default :
		break;
	}
	return retr;
}

static uint8_t SD_ReadData(void)
{
	uint8_t timeout = 0x08;
	uint8_t readvalue;

	// Check if response is got or a timeout is happen
	do {
		readvalue = SD_IO_WriteByte(SD_DUMMY_BYTE);
		timeout--;

	} while ((readvalue == SD_DUMMY_BYTE) && timeout);

	// Right response got
	return readvalue;
}

static uint8_t SD_GetCSDRegister(SD_CSD *Csd)
{
	uint16_t counter = 0;
	uint8_t CSD_Tab[16];
	uint8_t retr = BSP_SD_ERROR;
	SD_CmdAnswer_typedef response;

	// Send CMD9 (CSD register) or CMD10(CSD register) and Wait for response in the R1 format (0x00 is no errors)
	response = SD_SendCmd(SD_CMD_SEND_CSD, 0, 0xFF, SD_ANSWER_R1_EXPECTED);
	if (response.r1 == SD_R1_NO_ERROR) {
		if (SD_WaitData(SD_TOKEN_START_DATA_SINGLE_BLOCK_READ) == BSP_SD_OK) {
			for (counter = 0; counter < 16; counter++) {
				// Store CSD register value on CSD_Tab
				CSD_Tab[counter] = SD_IO_WriteByte(SD_DUMMY_BYTE);
			}

			// Get CRC bytes (not really needed by us, but required by SD)
			SD_IO_WriteByte(SD_DUMMY_BYTE);
			SD_IO_WriteByte(SD_DUMMY_BYTE);

			// CSD header decoding

			// Byte 0
			Csd->CSDStruct = (CSD_Tab[0] & 0xC0) >> 6;
			Csd->Reserved1 =	CSD_Tab[0] & 0x3F;

			// Byte 1
			Csd->TAAC = CSD_Tab[1];

			// Byte 2
			Csd->NSAC = CSD_Tab[2];

			// Byte 3
			Csd->MaxBusClkFrec = CSD_Tab[3];

			// Byte 4/5
			Csd->CardComdClasses = (CSD_Tab[4] << 4) | ((CSD_Tab[5] & 0xF0) >> 4);
			Csd->RdBlockLen = CSD_Tab[5] & 0x0F;

			// Byte 6
			Csd->PartBlockRead	 = (CSD_Tab[6] & 0x80) >> 7;
			Csd->WrBlockMisalign = (CSD_Tab[6] & 0x40) >> 6;
			Csd->RdBlockMisalign = (CSD_Tab[6] & 0x20) >> 5;
			Csd->DSRImpl				 = (CSD_Tab[6] & 0x10) >> 4;

			// CSD v1/v2 decoding

			if (flag_SDHC == 0) {
				Csd->version.v1.Reserved1 = ((CSD_Tab[6] & 0x0C) >> 2);

				Csd->version.v1.DeviceSize =	((CSD_Tab[6] & 0x03) << 10)
												|	(CSD_Tab[7] << 2)
												| ((CSD_Tab[8] & 0xC0) >> 6);
				Csd->version.v1.MaxRdCurrentVDDMin = (CSD_Tab[8] & 0x38) >> 3;
				Csd->version.v1.MaxRdCurrentVDDMax = (CSD_Tab[8] & 0x07);
				Csd->version.v1.MaxWrCurrentVDDMin = (CSD_Tab[9] & 0xE0) >> 5;
				Csd->version.v1.MaxWrCurrentVDDMax = (CSD_Tab[9] & 0x1C) >> 2;
				Csd->version.v1.DeviceSizeMul = ((CSD_Tab[9] & 0x03) << 1)
												| ((CSD_Tab[10] & 0x80) >> 7);
			} else {
				Csd->version.v2.Reserved1 = ((CSD_Tab[6] & 0x0F) << 2) | ((CSD_Tab[7] & 0xC0) >> 6);
				Csd->version.v2.DeviceSize = ((CSD_Tab[7] & 0x3F) << 16) | (CSD_Tab[8] << 8) | CSD_Tab[9];
				Csd->version.v2.Reserved2 = ((CSD_Tab[10] & 0x80) >> 8);
			}

			Csd->EraseSingleBlockEnable = (CSD_Tab[10] & 0x40) >> 6;
			Csd->EraseSectorSize	 = ((CSD_Tab[10] & 0x3F) << 1)
									   | ((CSD_Tab[11] & 0x80) >> 7);
			Csd->WrProtectGrSize	 = (CSD_Tab[11] & 0x7F);
			Csd->WrProtectGrEnable = (CSD_Tab[12] & 0x80) >> 7;
			Csd->Reserved2				 = (CSD_Tab[12] & 0x60) >> 5;
			Csd->WrSpeedFact			 = (CSD_Tab[12] & 0x1C) >> 2;
			Csd->MaxWrBlockLen		 = ((CSD_Tab[12] & 0x03) << 2)
									   | ((CSD_Tab[13] & 0xC0) >> 6);
			Csd->WriteBlockPartial = (CSD_Tab[13] & 0x20) >> 5;
			Csd->Reserved3				 = (CSD_Tab[13] & 0x1F);
			Csd->FileFormatGrouop	= (CSD_Tab[14] & 0x80) >> 7;
			Csd->CopyFlag					= (CSD_Tab[14] & 0x40) >> 6;
			Csd->PermWrProtect		 = (CSD_Tab[14] & 0x20) >> 5;
			Csd->TempWrProtect		 = (CSD_Tab[14] & 0x10) >> 4;
			Csd->FileFormat				= (CSD_Tab[14] & 0x0C) >> 2;
			Csd->Reserved4				 = (CSD_Tab[14] & 0x03);
			Csd->crc							 = (CSD_Tab[15] & 0xFE) >> 1;
			Csd->Reserved5				 = (CSD_Tab[15] & 0x01);

			retr = BSP_SD_OK;
		}
	}

	// Send dummy byte: 8 Clock pulses of delay
	SD_CS_HIGH();
	SD_IO_WriteByte(SD_DUMMY_BYTE);

	// Return the reponse
	return retr;
}

static uint8_t SD_GetCIDRegister(SD_CID *Cid)
{
	uint32_t counter = 0;
	uint8_t retr = BSP_SD_ERROR;
	uint8_t CID_Tab[16];
	SD_CmdAnswer_typedef response;

	// Send CMD10 (CID register) and Wait for response in the R1 format (0x00 is no errors)
	response = SD_SendCmd(SD_CMD_SEND_CID, 0, 0xFF, SD_ANSWER_R1_EXPECTED);
	if (response.r1 == SD_R1_NO_ERROR) {
		if (SD_WaitData(SD_TOKEN_START_DATA_SINGLE_BLOCK_READ) == BSP_SD_OK) {
			// Store CID register value on CID_Tab
			for (counter = 0; counter < 16; counter++) {
				CID_Tab[counter] = SD_IO_WriteByte(SD_DUMMY_BYTE);
			}

			// Get CRC bytes (not really needed by us, but required by SD)
			SD_IO_WriteByte(SD_DUMMY_BYTE);
			SD_IO_WriteByte(SD_DUMMY_BYTE);

			// Byte 0
			Cid->ManufacturerID = CID_Tab[0];

			// Byte 1
			Cid->OEM_AppliID = CID_Tab[1] << 8;

			// Byte 2
			Cid->OEM_AppliID |= CID_Tab[2];

			// Byte 3
			Cid->ProdName1 = CID_Tab[3] << 24;

			// Byte 4
			Cid->ProdName1 |= CID_Tab[4] << 16;

			// Byte 5
			Cid->ProdName1 |= CID_Tab[5] << 8;

			// Byte 6
			Cid->ProdName1 |= CID_Tab[6];

			// Byte 7
			Cid->ProdName2 = CID_Tab[7];

			// Byte 8
			Cid->ProdRev = CID_Tab[8];

			// Byte 9
			Cid->ProdSN = CID_Tab[9] << 24;

			// Byte 10
			Cid->ProdSN |= CID_Tab[10] << 16;

			// Byte 11
			Cid->ProdSN |= CID_Tab[11] << 8;

			// Byte 12
			Cid->ProdSN |= CID_Tab[12];

			// Byte 13
			Cid->Reserved1 |= (CID_Tab[13] & 0xF0) >> 4;
			Cid->ManufactDate = (CID_Tab[13] & 0x0F) << 8;

			// Byte 14
			Cid->ManufactDate |= CID_Tab[14];

			// Byte 15
			Cid->CID_CRC = (CID_Tab[15] & 0xFE) >> 1;
			Cid->Reserved2 = 1;

			retr = BSP_SD_OK;
		}
	}

	// Send dummy byte: 8 Clock pulses of delay
	SD_CS_HIGH();
	SD_IO_WriteByte(SD_DUMMY_BYTE);

	// Return the reponse
	return retr;
}

static uint8_t SD_GetDataResponse(void)
{
	uint8_t dataresponse;
	uint8_t rvalue = SD_DATA_OTHER_ERROR;

	dataresponse = SD_IO_WriteByte(SD_DUMMY_BYTE);
	SD_IO_WriteByte(SD_DUMMY_BYTE); // read the busy response byte

	// Mask unused bits
	switch (dataresponse & 0x1F) {
	case SD_DATA_OK:
		rvalue = SD_DATA_OK;

		// Set CS High
		SD_CS_HIGH();
		// Set CS Low
		SD_CS_LOW();

		// Wait IO line return 0xFF
		while (SD_IO_WriteByte(SD_DUMMY_BYTE) != 0xFF);
		break;
	case SD_DATA_CRC_ERROR:
		rvalue =	SD_DATA_CRC_ERROR;
		break;
	case SD_DATA_WRITE_ERROR:
		rvalue = SD_DATA_WRITE_ERROR;
		break;
	default:
		break;
	}

	// Return response
	return rvalue;
}

static uint8_t SD_WaitData(uint8_t data)
{
	uint16_t timeout = 0xFFFF;
	uint8_t readvalue;

	// Check if response is got or a timeout is happen

	do {
		readvalue = SD_IO_WriteByte(SD_DUMMY_BYTE);
		timeout--;
	} while ((readvalue != data) && timeout);

	if (timeout == 0) {
		// After time out
		return BSP_SD_TIMEOUT;
	}

	// Right response got
	return BSP_SD_OK;
}
