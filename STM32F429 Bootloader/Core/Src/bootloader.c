/*
 * bootloader.c
 *
 *  Created on: Jul 18, 2024
 *      Author: Soulaimane Oulad Belayachi
 */


#include "bootloader.h"


const uint8_t BL_Version[2] = { MAJOR, MINOR };


const uint8_t supported_command[] = { BL_GET_VER,
									  BL_GET_HELP,
									  BL_GET_CID,
									  BL_GET_RDP_STATUS,
									  BL_GO_TO_ADDR,
									  BL_FLASH_ERASE,
									  BL_MEM_WRITE,
									  BL_EN_R_W_PROTECT,
									  BL_MEM_READ,
									  BL_READ_SECTOR_STATUS,
									  BL_OTP_READ,
									  BL_DIS_R_W_PROTECT
									};

extern CRC_HandleTypeDef hcrc;

const char* BOOTLOADER_BANNER =
    " ______   _____   _____  _______         _____  _______ ______   _______  ______\r\n"
    " |_____] |     | |     |    |    |      |     | |_____| |     \\ |______  |_____/\r\n"
    " |_____] |_____| |_____|    |    |_____ |_____| |     | |_____/ |______  |    \\_\r\n"
    "                                                                               \r\n"
    "          							by Soulaimane_OuladBelayachi\r\n";

uint8_t is_valid_app_present(uint32_t address) {
	uint32_t sp = *((volatile uint32_t*) address);
	uint32_t pc = *((volatile uint32_t*) (address + 4));

	PRINT_DEBUG("		>> BL_DEBUG_MSG : SP = 0x%08lx, PC = 0x%08lx\r\n", sp, pc);

	uint8_t sp_valid = (sp >= 0x20000000) && (sp <= 0x20030000 + 0x40000); // Adjust the upper bound based on your actual RAM size
	uint8_t pc_valid = (pc >= APP_BASE_ADDR) && (pc <= 0x08100000); // Adjust the upper bound based on your actual Flash size

	PRINT_DEBUG("		>> BL_DEBUG_MSG : SP valid: %d, PC valid: %d\r\n", sp_valid,
			pc_valid);

	return sp_valid && pc_valid;
}


void launch_bootloader_banner(void){
	printf("%s\r\n", BOOTLOADER_BANNER);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_SET);
	printf("                                |--------------------------------|                                \r\n");
	printf("================================| Bootloader Diagnostic Messages |================================\r\n");
	printf("                                |--------------------------------|                                \r\n\n");
	printf("		>> BL_DEBUG_MSG : Loading Bootloader Version %d.%dv \r\n",BL_Version[0],BL_Version[1]);

	HAL_Delay(2000);
}



uint8_t get_bootloader_version(){
	return (uint8_t)BOOTLOADER_VER;
}

void bootloader_uart_send_data(uint8_t* pBuffer,uint8_t len){
	HAL_UART_Transmit(COMMAND_UART,pBuffer,len,HAL_MAX_DELAY);
}




void bootloader_send_ack(uint8_t command_code, uint8_t follow_len) {

	//Send 2 bytes.. 1st byte is ACK and the 2nd byte is "length to follow"
	uint8_t ack_buf[2];
	ack_buf[0] = BL_ACK;
	ack_buf[1] = follow_len;
	HAL_UART_Transmit(COMMAND_UART, ack_buf, 2, HAL_MAX_DELAY);
}

void bootloader_send_nack(void) {
	uint8_t nack = BL_NACK;
	HAL_UART_Transmit(COMMAND_UART, &nack, 1, HAL_MAX_DELAY);
}


uint8_t bootloader_verify_crc(uint8_t *pData, uint32_t len, uint32_t crc_host) {
    // Initialize CRC value
    uint32_t uwCRCValue = 0xFF;

    // Calculate CRC for each byte of data
    for(uint32_t i = 0; i < len; i++) {
        uint32_t i_data = pData[i];
        uwCRCValue = HAL_CRC_Accumulate(&hcrc, &i_data, 1);
    }

    /* Reset CRC Calculation Unit */
    __HAL_CRC_DR_RESET(&hcrc);

    // Compare calculated CRC with host-provided CRC
    if(uwCRCValue == crc_host) {
        return VERIFY_CRC_SUCCESS;
    }

    return VERIFY_CRC_FAIL;
}


uint8_t get_flash_rdp_level() {
    // Define pointer to the Option Bytes address
    uint32_t* pOB_addr = (uint32_t*) 0x1FFFC000;

    // Extract RDP level from the Option Byte
    // Shift right by 8 bits to get the RDP byte and cast to uint8_t
    uint8_t rdp_level = (uint8_t) (*pOB_addr >> 8);

    // Return the RDP level
    return rdp_level;
}


uint8_t is_valid_address(uint32_t address) {
    // Check if address is within Flash memory range
    if (address >= FLASH_BASE && address < (FLASH_BASE + FLASH_SIZE)) {
        return ADDR_VALID;
    }

    // Check if address is within SRAM1 range
    if (address >= SRAM1_BASE && address < (SRAM1_BASE + SRAM1_SIZE)) {
        return ADDR_VALID;
    }

    // Check if address is within SRAM2 range
    if (address >= SRAM2_BASE && address < (SRAM2_BASE + SRAM2_SIZE)) {
        return ADDR_VALID;
    }

    // Check if address is within CCM (Core Coupled Memory) range
    if (address >= CCMDATARAM_BASE && address < (CCMDATARAM_BASE + CCMDATARAM_SIZE)) {
        return ADDR_VALID;
    }

    // Check if address is within backup SRAM range
    if (address >= BKPSRAM_BASE && address < (BKPSRAM_BASE + BKPSRAM_SIZE)) {
        return ADDR_VALID;
    }

    // If none of the above conditions are met, the address is invalid
    return ADDR_INVALID;
}


uint8_t flash_erase(uint8_t sector_number, uint8_t number_of_sectors) {
    // There are 24 sectors in STM32F429 MCU (Sector[0:23])
    // number_of_sectors must be in the range of 0 to 24
    // if sector_number = 0xFF, that means mass erase

    FLASH_EraseInitTypeDef flashErase_handle;
    uint32_t sectorError;
    HAL_StatusTypeDef status;
    uint32_t bank;

    // Validate number_of_sectors
    if(number_of_sectors > 24)
        return INVALID_SECTOR;

    // Check if sector_number is valid for mass erase or individual sector erase
    if((sector_number == 0xFF) || (sector_number <= 23)) {

        if(sector_number == (uint8_t) 0xFF) {
            // Mass erase: erase all sectors in both banks
            flashErase_handle.TypeErase = FLASH_TYPEERASE_MASSERASE;
            bank = FLASH_BANK_1 | FLASH_BANK_2;
        } else {
            // Individual sector erase: calculate the number of sectors to erase
            uint8_t remaining_sector = 24 - sector_number;

            // Adjust number_of_sectors if it exceeds the remaining sectors
            if(number_of_sectors > remaining_sector) {
                number_of_sectors = remaining_sector;
            }

            flashErase_handle.TypeErase = FLASH_TYPEERASE_SECTORS;
            flashErase_handle.Sector = sector_number; // The initial sector
            flashErase_handle.NbSectors = number_of_sectors;
            bank = (sector_number < 12) ? FLASH_BANK_1 : FLASH_BANK_2;
        }

        flashErase_handle.Banks = bank;

        // Gain access to modify the FLASH registers
        HAL_FLASH_Unlock();
        flashErase_handle.VoltageRange = FLASH_VOLTAGE_RANGE_3;
        status = (uint8_t) HAL_FLASHEx_Erase(&flashErase_handle, &sectorError);
        HAL_FLASH_Lock();

        return status;
    }

    return INVALID_SECTOR;
}


static uint32_t GetSector(uint32_t Address)
{
    uint32_t sector = 0;

    if((Address < 0x08003FFF) && (Address >= 0x08000000))
    {
        sector = FLASH_SECTOR_0;
    }
    else if((Address < 0x08007FFF) && (Address >= 0x08004000))
    {
        sector = FLASH_SECTOR_1;
    }
    else if((Address < 0x0800BFFF) && (Address >= 0x08008000))
    {
        sector = FLASH_SECTOR_2;
    }
    else if((Address < 0x0800FFFF) && (Address >= 0x0800C000))
    {
        sector = FLASH_SECTOR_3;
    }
    else if((Address < 0x0801FFFF) && (Address >= 0x08010000))
    {
        sector = FLASH_SECTOR_4;
    }
    else if((Address < 0x0803FFFF) && (Address >= 0x08020000))
    {
        sector = FLASH_SECTOR_5;
    }
    else if((Address < 0x0805FFFF) && (Address >= 0x08040000))
    {
        sector = FLASH_SECTOR_6;
    }
    else if((Address < 0x0807FFFF) && (Address >= 0x08060000))
    {
        sector = FLASH_SECTOR_7;
    }
    else if((Address < 0x0809FFFF) && (Address >= 0x08080000))
    {
        sector = FLASH_SECTOR_8;
    }
    else if((Address < 0x080BFFFF) && (Address >= 0x080A0000))
    {
        sector = FLASH_SECTOR_9;
    }
    else if((Address < 0x080DFFFF) && (Address >= 0x080C0000))
    {
        sector = FLASH_SECTOR_10;
    }
    else if((Address < 0x080FFFFF) && (Address >= 0x080E0000))
    {
        sector = FLASH_SECTOR_11;
    }
    else if((Address < 0x08103FFF) && (Address >= 0x08100000))
    {
        sector = FLASH_SECTOR_12;
    }
    else if((Address < 0x08107FFF) && (Address >= 0x08104000))
    {
        sector = FLASH_SECTOR_13;
    }
    else if((Address < 0x0810BFFF) && (Address >= 0x08108000))
    {
        sector = FLASH_SECTOR_14;
    }
    else if((Address < 0x0810FFFF) && (Address >= 0x0810C000))
    {
        sector = FLASH_SECTOR_15;
    }
    else if((Address < 0x0811FFFF) && (Address >= 0x08110000))
    {
        sector = FLASH_SECTOR_16;
    }
    else if((Address < 0x0813FFFF) && (Address >= 0x08120000))
    {
        sector = FLASH_SECTOR_17;
    }
    else if((Address < 0x0815FFFF) && (Address >= 0x08140000))
    {
        sector = FLASH_SECTOR_18;
    }
    else if((Address < 0x0817FFFF) && (Address >= 0x08160000))
    {
        sector = FLASH_SECTOR_19;
    }
    else if((Address < 0x0819FFFF) && (Address >= 0x08180000))
    {
        sector = FLASH_SECTOR_20;
    }
    else if((Address < 0x081BFFFF) && (Address >= 0x081A0000))
    {
        sector = FLASH_SECTOR_21;
    }
    else if((Address < 0x081DFFFF) && (Address >= 0x081C0000))
    {
        sector = FLASH_SECTOR_22;
    }
    else if((Address < 0x081FFFFF) && (Address >= 0x081E0000))
    {
        sector = FLASH_SECTOR_23;
    }

    return sector;
}

// Helper function to get the flash bank for a given address
static uint32_t GetBank(uint32_t address)
{
    return (address < 0x08100000) ? FLASH_BANK_1 : FLASH_BANK_2;
}

uint8_t flash_memory_write(uint8_t *pBuffer, uint32_t mem_address, uint32_t len) {
	uint8_t status = HAL_OK;
    uint32_t address = mem_address;
    uint32_t end_address = mem_address + len;

    // Unlock the Flash
    HAL_FLASH_Unlock();

    // Clear all FLASH flags
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);



    // Program the flash byte by byte
    for (uint32_t i = 0; i < len && status == HAL_OK; i++) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, address, pBuffer[i]);
        address += 1; // Move to next byte
    }

    // Lock the Flash
    HAL_FLASH_Lock();

    return status;
}

uint16_t read_OB_rw_protection_status(void) {
	//This structure is defined in ST Flash driver to hold the OB(Option Byte) contents
	FLASH_OBProgramInitTypeDef OBInit;

	//Unlock the OB(Option Byte) memory access
	HAL_FLASH_OB_Unlock();
	//Get the OB configuration details
	HAL_FLASHEx_OBGetConfig(&OBInit);
	//Lock the OB memory access
	HAL_FLASH_OB_Lock();

	//Only interested in R/W protection status of the sectors
	return (uint16_t)OBInit.WRPSector;
}

uint8_t flash_config_sector_rw_protection(uint8_t sector_details, uint8_t protection_mode, uint8_t disable) {
	//First configure the protection mode
	//protection_mode = 1, means write protect of the user flash sectors
	//protection_mode = 2, means read/write protect of the user flash sectors
	//According to RM of stm32f446xx TABLE 9, We have to modify the address 0x1FFF C008 bit 15(SPRMOD)

	//Flash option control register (OPTCR)
	volatile uint32_t *pOPTCR = (uint32_t *) 0x40023C14;

	if(disable) {
	//DISABLE all r/w protection on sectors
		//Option byte configuration unlock
		HAL_FLASH_OB_Unlock();

		//Wait until no active operations on flash
		while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

		//Clear the 31st bit (default state) - Refer to: Flash Option Control Register (FLASH_OPTCR) in RM
		*pOPTCR &= ~(1 << 31);

		//Clear the protection: Make all bits belonging to sectors as 1
		*pOPTCR |= (0xFF << 16);

		//Set the Option Start Bit (OPTSTRT) in the FLASH_OPTCR register
		*pOPTCR |= (1 << 1);

		//Wait until no active operations on flash
		while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

		HAL_FLASH_OB_Lock();

		return 0;
	}

	if(protection_mode == (uint8_t) 1) {
	//Put Write Protection on the sectors encoded in sector_details argument
		//Option Byte Configuration unlock
		HAL_FLASH_OB_Unlock();

		//Wait until no active operations on flash
		while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

		//Set Write Protection for the sectors
		//Clear the 31st bit (default state) - Refer to: Flash Option Control Register (FLASH_OPTCR) in RM
		*pOPTCR &= ~(1 << 31);

		//Set WP on sectors
		*pOPTCR &= ~(sector_details << 16);

		//Set the Option Start Bit (OPTSTRT) in the FLASH_OPTCR register
		*pOPTCR |= (1 << 1);

		//Wait until no active operations on flash
		while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

		HAL_FLASH_OB_Lock();

	} else if(protection_mode == (uint8_t) 2) {
	//Put Read and Write Protections on the sectors encoded in sector_details argument
		//Option Byte Configuration unlock
		HAL_FLASH_OB_Unlock();

		//Wait until no active operations on flash
		while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

		//Setting Read and Write Protections for the sectors
		//Set the 31st bit - Refer to: Flash Option Control Register (FLASH_OPTCR) in RM
		*pOPTCR |= (1 << 31);

		//Set R&W Protections on sectors
		*pOPTCR &= ~(0xFF << 16);
		*pOPTCR |= (sector_details << 16);

		//Set the Option Start Bit (OPTSTRT) in the FLASH_OPTCR register
		*pOPTCR |= (1 << 1);

		//Wait until no active operations on flash
		while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

		HAL_FLASH_OB_Lock();
	}

	return 0;
}




void bootloader_handle_getver_cmd(uint8_t* bl_rx_buffer){
	uint8_t bl_version;

	//1. Verify the Checksum
	PRINT_DEBUG("		>> BL_DEBUG_MSG : Run bootloader_handle_getver \n\r");

	//Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0] + 1;

	//Extract the CRC32 sent by the host
	uint32_t host_crc = *((uint32_t *) (bl_rx_buffer + command_packet_len - 4));

	if(!bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len - 4, host_crc)) {
		PRINT_DEBUG("		>> BL_DEBUG_MSG: checksum success!!\n\r");
		//checksum is correct
		bootloader_send_ack(bl_rx_buffer[0], 1);
		bl_version = get_bootloader_version();
		PRINT_DEBUG("		>> BL_DEBUG_MSG : BL_VER: 0x%02lx\n\r", bl_version);
		bootloader_uart_send_data(&bl_version, 1);
	} else {
		PRINT_DEBUG("		>> BL_DEBUG_MSG: checksum fail!!\n\r");
		//checksum is wrong
		bootloader_send_nack();
	}
}
void bootloader_handle_gethelp_cmd(uint8_t* bl_rx_buffer){
	PRINT_DEBUG("		>> BL_DEBUG_MSG : Run bootloader_handle_gethelp \n\r");

	//Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0] + 1;

	//Extract the CRC32 sent by the host
	uint32_t host_crc = *((uint32_t *) (bl_rx_buffer + command_packet_len - 4));

	if(!bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len - 4, host_crc)) {
		PRINT_DEBUG("		>> BL_DEBUG_MSG: checksum success!!\n\r");
		//checksum is correct
		bootloader_send_ack(bl_rx_buffer[0], 12);
		bootloader_uart_send_data((uint8_t*)supported_command,12);
	} else {
		PRINT_DEBUG("		>> BL_DEBUG_MSG: checksum fail!!\n\r");
		//checksum is wrong
		bootloader_send_nack();
	}

}
void bootloader_handle_getcid_cmd(uint8_t* bl_rx_buffer){
	uint32_t cid[3] = {0,0,0};


	PRINT_DEBUG("		>> BL_DEBUG_MSG : Run bootloader_handle_getcid \n\r");

	//Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0] + 1;

	//Extract the CRC32 sent by the host
	uint32_t host_crc = *((uint32_t *) (bl_rx_buffer + command_packet_len - 4));
	//1. Verify the Checksum
	if(!bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len - 4, host_crc)) {
		PRINT_DEBUG("		>> BL_DEBUG_MSG: checksum success!!\n\r");
		//checksum is correct
		bootloader_send_ack(bl_rx_buffer[0], 12);
		cid[0] = HAL_GetUIDw0();
		cid[1] = HAL_GetUIDw1();
		cid[2] = HAL_GetUIDw2();
		PRINT_DEBUG("		>> BL_DEBUG_MSG : CID: 0x%08lx  0x%08lx  0x%08lx\n\r", cid[0],cid[1],cid[2]);
		bootloader_uart_send_data((uint8_t*)cid, 12);
	} else {
		PRINT_DEBUG("		>> BL_DEBUG_MSG: checksum fail!!\n\r");
		//checksum is wrong
		bootloader_send_nack();
	}
}
void bootloader_handle_getrdp_cmd(uint8_t* bl_rx_buffer){
	uint8_t rdp_level;


	PRINT_DEBUG("		>> BL_DEBUG_MSG : Run bootloader_handle_getrdp \n\r");

	//Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0] + 1;

	//Extract the CRC32 sent by the host
	uint32_t host_crc = *((uint32_t *) (bl_rx_buffer + command_packet_len - 4));

	if(!bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len - 4, host_crc)) {
		PRINT_DEBUG("		>> BL_DEBUG_MSG: checksum success!!\n\r");
		//checksum is correct
		bootloader_send_ack(bl_rx_buffer[0], 1);
		rdp_level = get_flash_rdp_level();
		PRINT_DEBUG("		>> BL_DEBUG_MSG : RDP level: 0x%02lx\n\r", rdp_level);
		bootloader_uart_send_data(&rdp_level, 1);
	} else {
		PRINT_DEBUG("		>> BL_DEBUG_MSG: checksum fail!!\n\r");
		//checksum is wrong
		bootloader_send_nack();
	}
}
void bootloader_handle_goaddr_cmd(uint8_t* bl_rx_buffer){

	uint32_t go_address;
	uint8_t valid_addr = ADDR_VALID;
	uint8_t invalid_addr = ADDR_INVALID;

	PRINT_DEBUG("		>> BL_DEBUG_MSG : Run bootloader_handle_goaddr \n\r");

	//Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0] + 1;

	//Extract the CRC32 sent by the host
	uint32_t host_crc = *((uint32_t *) (bl_rx_buffer + command_packet_len - 4));

	if(!bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len - 4, host_crc)) {

		PRINT_DEBUG("		>> BL_DEBUG_MSG: checksum success!!\n\r");
		//checksum is correct
		bootloader_send_ack(bl_rx_buffer[0], 1);

		go_address = *((uint32_t*)&bl_rx_buffer[2]);

		PRINT_DEBUG("		>> BL_DEBUG_MSG : Go to address : 0x%08lx\n\r", go_address);

		if(is_valid_address(go_address) == ADDR_VALID){
			bootloader_uart_send_data(&valid_addr,1);
			PRINT_DEBUG("		>> BL_DEBUG_MSG : Address : 0x%08lx is valid\n\r", go_address);

			go_address |= (1 << 0);
			void (*jump_to_addr) (void) = (void*) go_address;

			PRINT_DEBUG("		>> BL_DEBUG_MSG : Jumping to go address !\n\r");
			jump_to_addr();

		}else{
			PRINT_DEBUG("		>> BL_DEBUG_MSG : Address : 0x%08lx is invalid\n\r", go_address);
			bootloader_uart_send_data(&invalid_addr,1);
		}
	} else {
		PRINT_DEBUG("		>> BL_DEBUG_MSG: checksum fail!!\n\r");
		//checksum is wrong
		bootloader_send_nack();
	}


}
void bootloader_handle_eraseflash_cmd(uint8_t* bl_rx_buffer){
	PRINT_DEBUG("		>> BL_DEBUG_MSG : bootloader_handle_eraseflash \n\r");

	uint8_t erase_status = 0x00;

	//Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0] + 1;

	//Extract the CRC32 sent by the host
	uint32_t host_crc = *((uint32_t *) (bl_rx_buffer + command_packet_len - 4));

	if(!bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len - 4, host_crc)) {
		PRINT_DEBUG("		>> BL_DEBUG_MSG: checksum success!!\n\r");
		//checksum is correct
		bootloader_send_ack(bl_rx_buffer[0], 1);
		PRINT_DEBUG("		>> BL_DEBUG_MSG: initial_sector: %d no_of_sectors: %d\n\r", bl_rx_buffer[2], bl_rx_buffer[3]);
		PRINT_DEBUG("		>> BL_DEBUG_MSG: Start Flash erasing ...\n\r");
		erase_status = flash_erase(bl_rx_buffer[2], bl_rx_buffer[3]);
		PRINT_DEBUG("		>> BL_DEBUG_MSG: Flash erasing completed !\n\r");

		bootloader_uart_send_data(&erase_status, 1);
	} else {
		PRINT_DEBUG("		>> BL_DEBUG_MSG: checksum fail!!\n\r");
		//checksum is wrong
		bootloader_send_nack();
	}

}
void bootloader_handle_memwrite_cmd(uint8_t* bl_rx_buffer){

	PRINT_DEBUG("		>> BL_DEBUG_MSG : bootloader_handle_memwrite \n\r");

	uint8_t write_status = 0x00;

	uint8_t payload_len = bl_rx_buffer[6];

	uint32_t mem_address = *((uint32_t *)(&bl_rx_buffer[2]));

	//Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0] + 1;

	//Extract the CRC32 sent by the host
	uint32_t host_crc = *((uint32_t *) (bl_rx_buffer + command_packet_len - 4));

	if(!bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len - 4, host_crc)) {
		PRINT_DEBUG("		>> BL_DEBUG_MSG: checksum success!!\n\r");
		PRINT_DEBUG("		>> BL_DEBUG_MSG : Memory write address : 0x%08lx\r\n", mem_address);
		bootloader_send_ack(bl_rx_buffer[0], 1);
		if(is_valid_address(mem_address) == ADDR_VALID){
			PRINT_DEBUG("		>> BL_DEBUG_MSG: This address is valid.\n\r");
			PRINT_DEBUG("		>> BL_DEBUG_MSG: Start flash writing ... \n\r");
			write_status = flash_memory_write(&bl_rx_buffer[7], mem_address, payload_len);
			PRINT_DEBUG("		>> BL_DEBUG_MSG: Flash writing completed !\n\r");

			bootloader_uart_send_data(&write_status, 1);

		}else{
			PRINT_DEBUG("		>> BL_DEBUG_MSG: This address is invalid.\n\r");
			write_status = ADDR_INVALID;

			bootloader_uart_send_data(&write_status, 1);

		}

	} else {
		PRINT_DEBUG("		>> BL_DEBUG_MSG: checksum fail!!\n\r");
		//checksum is wrong
		bootloader_send_nack();
	}


}
void bootloader_handle_enrwprotect_cmd(uint8_t* bl_rx_buffer){
	uint8_t status = 0x00;


	PRINT_DEBUG("		>> BL_DEBUG_MSG : Run bootloader_handle_enrwprotect \n\r");

	//Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0] + 1;

	//Extract the CRC32 sent by the host
	uint32_t host_crc = *((uint32_t *) (bl_rx_buffer + command_packet_len - 4));

	if(!bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len - 4, host_crc)) {
		PRINT_DEBUG("		>> BL_DEBUG_MSG: checksum success!!\n\r");
		//checksum is correct
		bootloader_send_ack(bl_rx_buffer[0], 1);
		status = flash_config_sector_rw_protection(bl_rx_buffer[2], bl_rx_buffer[3], 0);
		PRINT_DEBUG("		>> BL_DEBUG_MSG : Enable Flash protection status: 0x%02lx\n\r", status);
		bootloader_uart_send_data(&status, 1);
	} else {
		PRINT_DEBUG("		>> BL_DEBUG_MSG: checksum fail!!\n\r");
		//checksum is wrong
		bootloader_send_nack();
	}

}

void bootloader_handle_readsector_protection_cmd(uint8_t* bl_rx_buffer){
	uint16_t status;
	PRINT_DEBUG("		>> BL_DEBUG_MSG : bootloader_handle_readsector_protection \n\r");

	//Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0] + 1;

	//Extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t *)(bl_rx_buffer + command_packet_len - 4));

	if(!bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len - 4, host_crc)) {
		PRINT_DEBUG("		>> BL_DEBUG_MSG: checksum success!!\n\r");
		bootloader_send_ack(bl_rx_buffer[0], 2);

		status = read_OB_rw_protection_status();
		PRINT_DEBUG("		>> BL_DEBUG_MSG : nWRP status: 0x%02lx\n\r", status);
		bootloader_uart_send_data((uint8_t *)&status, 2);

	} else {
	PRINT_DEBUG("		>> BL_DEBUG_MSG: checksum fail!!\n\r");
    bootloader_send_nack();
	}
}

void bootloader_handle_disrwprotect_cmd(uint8_t* bl_rx_buffer){
	uint8_t status = 0x00;


	PRINT_DEBUG("		>> BL_DEBUG_MSG : Run bootloader_handle_disrwprotect \n\r");

	//Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0] + 1;

	//Extract the CRC32 sent by the host
	uint32_t host_crc = *((uint32_t *) (bl_rx_buffer + command_packet_len - 4));

	if(!bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len - 4, host_crc)) {
		PRINT_DEBUG("		>> BL_DEBUG_MSG: checksum success!!\n\r");
		//checksum is correct
		bootloader_send_ack(bl_rx_buffer[0], 1);
		status = flash_config_sector_rw_protection(0, 0, 1);
		PRINT_DEBUG("		>> BL_DEBUG_MSG : Disable Flash protection status: 0x%02lx\n\r", status);
		bootloader_uart_send_data(&status, 1);
	} else {
		PRINT_DEBUG("		>> BL_DEBUG_MSG: checksum fail!!\n\r");
		//checksum is wrong
		bootloader_send_nack();
	}

}

