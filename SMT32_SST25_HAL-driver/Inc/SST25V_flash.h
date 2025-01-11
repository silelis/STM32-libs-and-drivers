#ifndef __SST25V_FLASH_H
#define __SST25V_FLASH_H

#include "stdint.h"
#include "SST25V_flash_CONFIG.h"

#if (_SST25_USE_FREERTOS ==1)
	#error "Nie masz napisanej funkcji delayns() dla FREERTOS"
#else
	#define SST25_delay_ns(x)		HAL_Delay(x);
	//TODO: Do napisania porz¹dna funkcja delay'u w nanosekundach
	#warning "Do napisania porzadnia funkcja delay"
#endif // _SST25_USE_FREERTOS =1 

#if (_SST25_DEBUG ==1)
	#error "Nie masz napisanej obs³ugi debug"
#endif

typedef union
{
	uint8_t StatuRegister_byte;
	struct
	{
		uint8_t BUSY	: 1;      	//1 = Internal Write operation is in progress
								//0 = No internal Write operation is in progress
		uint8_t WEL		: 1;      	//1 = Device is memory Write enabled
								//0 = Device is not memory Write enabled
		uint8_t BP0		: 1;      	//Indicate current level of block write protection (See Table 4-3)
		uint8_t BP1		: 1;      	//Indicate current level of block write protection (See Table 4-3)
		uint8_t BP2		: 1;      	//Indicate current level of block write protection (See Table 4-3)
		uint8_t BP3		: 1;      	//Indicate current level of block write protection (See Table 4-3)
								/*
								|---------------------------------------------------------------------------------|	
								|					|	Status Register  Bit(**)	 |	Protected Memory Address  |	
								|	Protection Level|	BP3	 |  BP2  |  BP1  |  BP0  |					16 Mbit	  |
								|---------------------------------------------------------------------------------|
								|    None			|	 X	 |	 0	 |	 0	 |	 0	 |		None				  |
								|    Upper 1/32		|	 X	 |	 0	 |	 0	 |	 1	 |		1F0000H-1FFFFFH		  |
								|    Upper 1/16		|	 X	 |	 0	 |	 1	 |	 0	 |		1E0000H-1FFFFFH		  |    
								|    Upper 1/8		|	 X	 |	 0	 |	 1	 |	 1	 |		1C0000H-1FFFFFH		  |
								|    Upper 1/4		|	 X	 |	 1	 |	 0	 |	 0	 |		180000H-1FFFFFH		  |
								|    Upper 1/2		|	 X	 |	 1	 |	 0	 |	 1	 |		100000H-1FFFFFH		  |
								|    All Blocks  	|	 X	 |	 1	 |	 1	 |	 0	 |		000000H-1FFFFFH		  |
								|    All Blocks		|	 X	 |	 1	 |	 1	 |	 1	 |		000000H-1FFFFFH		  |
								-----------------------------------------------------------------------------------
								*  - X = Don’t Care (RESERVED) default is “0
								** - Default at power-up for BP2, BP1, and BP0 is ‘111’. (All Blocks Protected)
								*/
		uint8_t AAI		: 1;      	//Auto Address Increment Programming status
								//1 = AAI programming mode
								//0 = Byte - Program mode
		uint8_t BPL		: 1;      	//1 = BP3, BP2, BP1, BP0 are read-only bits
								//0 = BP3, BP2, BP1, BP0 are read / writable
	};
}	SST25_STATUS_REGISTER;

typedef union
{
	uint32_t product_identyfication;
	struct
	{
		uint8_t Manufacturers_ID;
		uint8_t Device_ID;
		uint8_t Manufacturers_ID_; 
		uint8_t Device_ID_;
	};
}	SST25_RDID;

typedef union
{
	uint32_t JEDEC_READ_ID;
	struct
	{
		uint8_t Memory_Capacity; 
		uint8_t Memory_Type; 
		uint8_t Manufacturers_ID;
	};
	
} SST25_JEDEC_READ_ID;





static void SST25_start(void);
static void SST25_end(void);
static void SST25_wait_while_busy(void);
static void SST25_write_status(uint8_t);
static void flash_SPI_write(uint8_t*, uint16_t);
static void flash_SPI_read(uint8_t *, uint16_t);
static uint8_t flash_SPI_read_write_byte(uint8_t);
static void SST25_write_enable(void);
static void SST25_write_disable(void);
static void SST25_StartCommandEnd_sequense(uint8_t command2send);
static void SST25_enable_hardware_EOW(void);
static void SST25_disable_hardware_EOW(void);
static void SST25_chip_erase_sequence(void);
static void SST25_enable_write_status(void);
static void SST25_sector_erase(uint32_t, uint8_t);



SST25_STATUS_REGISTER SST25_RDSR(void);      //READ-STATUS-REGISTER (RDSR) SEQUENCE
uint8_t SST25_get_status(void);
void SST25_init(void);
void SST25_sector_erase_4K(uint32_t);
void SST25_sector_erase_32K(uint32_t);
void SST25_sector_erase_64K(uint32_t);
void SST25_sector_erase_ChipErase(void);
void SST25_read(uint32_t, uint8_t *, uint16_t);
void SST25_read_slow(uint32_t, uint8_t *, uint16_t);
void SST25_write_byte(uint32_t, uint8_t);
void SST25_write(uint32_t, uint8_t*, uint16_t);
SST25_RDID SST25_read_id(void);
SST25_JEDEC_READ_ID SST25_jedec_read_id(void);
#endif
