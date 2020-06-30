
#include "..\Drivers\STM32F1xx_HAL_Driver\Inc\stm32f1xx_hal.h"
//#include "..\Drivers\CMSIS\Device\ST\STM32F1xx\Include\stm32f103x.h"

enum {
	BSP_SD_OK = 0x00,
	MSD_OK = 0x00,
	BSP_SD_ERROR = 0x01,
	BSP_SD_TIMEOUT
};

typedef struct {
	uint8_t  Reserved1: 2;              // Reserved
	uint16_t DeviceSize: 12;            // Device Size
	uint8_t  MaxRdCurrentVDDMin: 3;     // Max. read current @ VDD min
	uint8_t  MaxRdCurrentVDDMax: 3;     // Max. read current @ VDD max
	uint8_t  MaxWrCurrentVDDMin: 3;     // Max. write current @ VDD min
	uint8_t  MaxWrCurrentVDDMax: 3;     // Max. write current @ VDD max
	uint8_t  DeviceSizeMul: 3;          // Device size multiplier
} struct_v1;

typedef struct {
	uint8_t  Reserved1: 6;              // Reserved
	uint32_t DeviceSize: 22;            // Device Size
	uint8_t  Reserved2: 1;              // Reserved
} struct_v2;

typedef struct {
	// Header part
	uint8_t  CSDStruct: 2;           // CSD structure
	uint8_t  Reserved1: 6;           // Reserved
	uint8_t  TAAC: 8;                // Data read access-time 1
	uint8_t  NSAC: 8;                // Data read access-time 2 in CLK cycles
	uint8_t  MaxBusClkFrec: 8;       // Max. bus clock frequency
	uint16_t CardComdClasses: 12;     // Card command classes
	uint8_t  RdBlockLen: 4;          // Max. read data block length
	uint8_t  PartBlockRead: 1;       // Partial blocks for read allowed
	uint8_t  WrBlockMisalign: 1;     // Write block misalignment
	uint8_t  RdBlockMisalign: 1;     // Read block misalignment
	uint8_t  DSRImpl: 1;             // DSR implemented

	// v1 or v2 struct
	union csd_version {
		struct_v1 v1;
		struct_v2 v2;
	} version;

	uint8_t  EraseSingleBlockEnable: 1; // Erase single block enable
	uint8_t  EraseSectorSize: 7;        // Erase group size multiplier
	uint8_t  WrProtectGrSize: 7;        // Write protect group size
	uint8_t  WrProtectGrEnable: 1;      // Write protect group enable
	uint8_t  Reserved2: 2;              // Reserved
	uint8_t  WrSpeedFact: 3;            // Write speed factor
	uint8_t  MaxWrBlockLen: 4;          // Max. write data block length
	uint8_t  WriteBlockPartial: 1;      // Partial blocks for write allowed
	uint8_t  Reserved3: 5;              // Reserved
	uint8_t  FileFormatGrouop: 1;       // File format group
	uint8_t  CopyFlag: 1;               // Copy flag (OTP)
	uint8_t  PermWrProtect: 1;          // Permanent write protection
	uint8_t  TempWrProtect: 1;          // Temporary write protection
	uint8_t  FileFormat: 2;             // File Format
	uint8_t  Reserved4: 2;              // Reserved
	uint8_t  crc: 7;                    // Reserved
	uint8_t  Reserved5: 1;              // always 1
} SD_CSD;

typedef struct {
	volatile uint8_t  ManufacturerID;       // ManufacturerID
	volatile uint16_t OEM_AppliID;          // OEM/Application ID
	volatile uint32_t ProdName1;            // Product Name part1
	volatile uint8_t  ProdName2;            // Product Name part2
	volatile uint8_t  ProdRev;              // Product Revision
	volatile uint32_t ProdSN;               // Product Serial Number
	volatile uint8_t  Reserved1;            // Reserved1
	volatile uint16_t ManufactDate;         // Manufacturing Date
	volatile uint8_t  CID_CRC;              // CID CRC
	volatile uint8_t  Reserved2;            // always 1
} SD_CID;

typedef struct {
	SD_CSD Csd;
	SD_CID Cid;
	uint32_t CardCapacity;  // Card Capacity
	uint32_t CardBlockSize; // Card Block Size
} SD_CardInfo;

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;

// SPI configuration
#define SD_SPI_BAUD_RATE				SPI_BAUDRATEPRESCALER_4
#define SD_SPI_HANDLE					hspi1
#define SD_SPI_INSTANCE					SPI1
#define SD_SPI_CLK_ENABLE()				__HAL_RCC_SPI1_CLK_ENABLE()
#define SD_SPI_TIMEOUT_MAX				1000

// Pinout
#define SD_CS_PIN						GPIO_PIN_5
#define SD_CS_GPIO_PORT					GPIOB
#define SD_CS_GPIO_CLK_ENABLE()			__HAL_RCC_GPIOB_CLK_ENABLE()
#define SD_CS_GPIO_CLK_DISABLE()		__HAL_RCC_GPIOB_CLK_DISABLE()

#define SD_SCLK_AF
#define SD_SCLK_GPIO_PORT				GPIOB
#define SD_SCLK_PIN						GPIO_PIN_5
#define SD_SCLK_GPIO_CLK_ENABLE()		__HAL_RCC_GPIOA_CLK_ENABLE()
#define SD_SCLK_GPIO_CLK_DISABLE()		__HAL_RCC_GPIOA_CLK_DISABLE()

#define SD_MISO_AF
#define SD_MISO_GPIO_PORT				GPIOA
#define SD_MISO_PIN						GPIO_PIN_6
#define SD_MISO_GPIO_CLK_ENABLE()		__HAL_RCC_GPIOA_CLK_ENABLE()
#define SD_MISO_GPIO_CLK_DISABLE()		__HAL_RCC_GPIOA_CLK_DISABLE()

#define SD_MOSI_AF
#define SD_MOSI_GPIO_PORT				GPIOA
#define SD_MOSI_PIN						GPIO_PIN_7
#define SD_MOSI_GPIO_CLK_ENABLE()		__HAL_RCC_GPIOA_CLK_ENABLE()
#define SD_MOSI_GPIO_CLK_DISABLE()		__HAL_RCC_GPIOA_CLK_DISABLE()

// DO NOT EDIT
#define SD_CS_LOW()						HAL_GPIO_WritePin(SD_CS_GPIO_PORT, SD_CS_PIN, GPIO_PIN_RESET)
#define SD_CS_HIGH()					HAL_GPIO_WritePin(SD_CS_GPIO_PORT, SD_CS_PIN, GPIO_PIN_SET)

void SD_IO_Init(void);
void SD_IO_WriteReadData(const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLength);
uint8_t SD_IO_WriteByte(uint8_t Data);
void SD_IO_ReadData(uint8_t *DataOut, uint16_t DataLength);
void SD_IO_WriteData(const uint8_t *Data, uint16_t DataLength);
// uint8_t SD_IO_ReadByte(void);

uint8_t BSP_SD_Init(void);
uint8_t BSP_SD_ReadBlocks(uint32_t *pData, uint32_t ReadAddr, uint16_t BlockSize, uint32_t NumberOfBlocks);
uint8_t BSP_SD_WriteBlocks(uint32_t *pData, uint32_t WriteAddr, uint16_t BlockSize, uint32_t NumberOfBlocks);
uint8_t BSP_SD_Erase(uint32_t StartAddr, uint32_t EndAddr);
uint8_t BSP_SD_GetCardInfo(SD_CardInfo *pCardInfo);
uint8_t BSP_SD_GetStatus(void);
