/*
 * bootloader.h
 *
 *  Created on: Jul 18, 2024
 *      Author: Soulaimane Oulad Belayachi
 */

#ifndef INC_BOOTLOADER_H_
#define INC_BOOTLOADER_H_

#include <stdio.h>
#include <stdint.h>
#include "main.h"

#ifdef BT_DEBUG
#define	PRINT_DEBUG				printf
#endif

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart1;


#define DEBUG_UART			&huart4
#define COMMAND_UART		&huart1

#define BOOTLOADER_VER		 0x10
#define MAJOR 					1   // BL Major version Number
#define MINOR 					0   // BL Minor version Number


#define APP_BASE_ADDR			0x08040000U

/*Supported Bootloader Commands*/
#define BL_GET_VER						0x30 	/*This command is used to read the bootloader version from the MCU*/
#define BL_GET_HELP						0x31	/*This command is used to know what are the commands supported by the bootloader*/
#define BL_GET_CID						0x32	/*This command is used to read the MCU chip identification number */
#define BL_GET_RDP_STATUS				0x33	/*This command is used to read the FLASH Read Protection level.*/
#define BL_GO_TO_ADDR					0x34	/*This command is used to jump bootloader to specified address. */
#define BL_FLASH_ERASE					0x35	/*This command is used to mass erase or sector erase of the user flash . */
#define BL_MEM_WRITE					0x36	/*This command is used to write data in to different memories of the MCU*/
#define BL_EN_R_W_PROTECT 				0x37	/*This command is used to enable read/write protect on different sectors of the user flash .*/
#define BL_READ_SECTOR_STATUS			0x38	/*This command is used to read all the sector protection status. */
#define BL_DIS_R_W_PROTECT				0x39	/*This command is used to disableread/write protection on different sectors of the user flash . This command takes the protection status to default state. */



#define VERIFY_CRC_SUCCESS				0
#define VERIFY_CRC_FAIL					1

#define BL_ACK							0x50
#define BL_NACK							0x51


#define ADDR_VALID						0x00
#define ADDR_INVALID					0x01

#define INVALID_SECTOR 					0x04


// Define memory regions
#define FLASH_BASE          0x08000000UL
#define FLASH_SIZE          (2 * 1024 * 1024)  // 2MB for STM32F429

#define SRAM1_BASE          0x20000000UL
#define SRAM1_SIZE          (112 * 1024)       // 112KB

#define SRAM2_BASE          0x2001C000UL
#define SRAM2_SIZE          (16 * 1024)        // 16KB

#define CCMDATARAM_BASE     0x10000000UL
#define CCMDATARAM_SIZE     (64 * 1024)        // 64KB

#define BKPSRAM_BASE        0x40024000UL
#define BKPSRAM_SIZE        (4 * 1024)         // 4KB


uint8_t is_valid_app_present(uint32_t address);
uint8_t get_bootloader_version(void);
void bootloader_uart_send_data(uint8_t* pBuffer,uint8_t len);

void launch_bootloader_banner(void);



// Handle Get Version command
void bootloader_handle_getver_cmd(uint8_t* bl_rx_buffer);

// Handle Get Help command
void bootloader_handle_gethelp_cmd(uint8_t* bl_rx_buffer);

// Handle Get Chip ID command
void bootloader_handle_getcid_cmd(uint8_t* bl_rx_buffer);

// Handle Get Read Protection Status command
void bootloader_handle_getrdp_cmd(uint8_t* bl_rx_buffer);

// Handle Go To Address command
void bootloader_handle_goaddr_cmd(uint8_t* bl_rx_buffer);

// Handle Erase Flash command
void bootloader_handle_eraseflash_cmd(uint8_t* bl_rx_buffer);

// Handle Memory Write command
void bootloader_handle_memwrite_cmd(uint8_t* bl_rx_buffer);

// Handle Enable Read/Write Protection command
void bootloader_handle_enrwprotect_cmd(uint8_t* bl_rx_buffer);

// Handle Memory Read command
void bootloader_handle_memread_cmd(uint8_t* bl_rx_buffer);

// Handle Read Sector Protection Status command
void bootloader_handle_readsector_protection_cmd(uint8_t* bl_rx_buffer);

// Handle Disable Read/Write Protection command
void bootloader_handle_disrwprotect_cmd(uint8_t* bl_rx_buffer);



#endif /* INC_BOOTLOADER_H_ */
