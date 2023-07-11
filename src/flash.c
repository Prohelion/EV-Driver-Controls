/*
 * Tritium internal flash Interface
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

// Include files
#include <msp430x24x.h>
#include "tri86.h"
#include "flash.h"

// Public variables
flash_config	flash;


/**************************************************************************************************
 * PUBLIC FUNCTIONS
 *************************************************************************************************/

/*
 * Initialises MSP430 internal flash controller
 *	- Timing is assuming a 16MHz clock
 */
void flash_init( void )
{
	// Clock source for flash timing generator
	FCTL2 = FWKEY + FSSEL_1 + 0x002B;		// MCLK/44 clock source for flash timing generator
}

/*
 * Erase the flash segment
 *	- Erases ALL DATA in the segment
 *	- Disable watchdog and interrupts before executing this routine
 */
void flash_erase( void )
{
	unsigned char *flash_ptr;

	// Set up segment to erase
	flash_ptr = (char *)(FLASH_BASE_ADDR);
	// Perform the erase
	FCTL3 = FWKEY;
	FCTL1 = FWKEY + ERASE;
	*flash_ptr = 0;
	// Finish write and lock flash
	FCTL3 = FWKEY + LOCK;
}

/*
 * Read config area from flash into RAM
 *	- Pass in pointer to config struct in RAM, and size of struct to read from flash
 */
void flash_read( unsigned char *ptr, unsigned char size )
{
	unsigned char i;
	unsigned char *flash_ptr;

	flash_ptr = (char *)(FLASH_BASE_ADDR);
	for(i = 0; i < size; i++)
	{
		*ptr++ = *flash_ptr++;
	}
}


/*
 *	Write config area from RAM into flash
 *	- Disable watchdog and interrupts before executing this routine
 *	- Flash address being written MUST have already been erased previously
 */
void flash_write( unsigned char *ptr, unsigned char size )
{
	unsigned char i;
	unsigned char *flash_ptr;

	// Set up segment to write
	flash_ptr = (char *)(FLASH_BASE_ADDR);
	FCTL3 = FWKEY;
	FCTL1 = FWKEY + WRT;
	// Write data
	for(i = 0; i < size; i++)
	{
		*flash_ptr++ = *ptr++;
	}
	// Finish write and lock flash
	FCTL1 = FWKEY;
	FCTL3 = FWKEY + LOCK;
}
