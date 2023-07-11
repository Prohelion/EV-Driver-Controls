/*
 * Tritium internal flash interface header
 * Copyright (c) 2015, Tritium Pty Ltd.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *	- Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer 
 *	  in the documentation and/or other materials provided with the distribution.
 *	- Neither the name of Tritium Pty Ltd nor the names of its contributors may be used to endorse or promote products 
 *	  derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
 * IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 * OF SUCH DAMAGE. 
 *
 * Last Modified: A.Rudzki, Tritium Pty Ltd, 13 January 2014
 *
 * - Implements the following flash interface functions
 *	- flash_init
 *	- flash_erase
 *	- flash_read
 *	- flash_write
 *
 */

#ifndef FLASH_H
#define FLASH_H

// Public function prototypes
void flash_init(void);
void flash_erase(void);
void flash_read(unsigned char *ptr, unsigned char size );
void flash_write(unsigned char *ptr, unsigned char size );

// Public variables
typedef struct _flash_config {
	unsigned long 	serial;
	unsigned int 	can_id;
	unsigned int	can_bitrate;
	//Add any other flash configuration items you want after can_bitrate
} flash_config;

extern flash_config flash;

// Flash segment address
#define FLASH_BASE_ADDR			0x1000

// Flash identifiers for comms etc
// Mandatory Config Items and Locations for Bootloader
#define FLASH_SERIAL				0x01	//1
#define FLASH_CAN_ID				0x02	//2
#define FLASH_CAN_BITRATE			0x03	//3
//Add any other flash configuration item accessor IDs here. Dont forget to add to switch cases in tri86.c CAN packet reception section
//
#define FLASH_WRITE_TRIGGER			0xFF	//255

// Flash commands
#define FLASH_CMD_WRITE		0			// Data from PC, to insert into flash memory
#define FLASH_CMD_READ		1			// Request from PC, to read a location in flash memory
#define FLASH_CMD_REPLY		2			// The reply to the PC in response to a FLASH_CMD_READ, with the data

#endif
