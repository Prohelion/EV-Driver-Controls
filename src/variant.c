/*
 * Hardware info version recovery from bootloader
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
 */

#include "variant.h"
#include "buildnum.h"
#include "can.h"
#include "flash.h"

hware_info hware;

#define __hware_version__ __attribute__((section(".hware_version")))
__hware_version__ const hware_info bl_hware_info;

/*
 * Pull hardware info in from bootloader sector
 *	- Fills in either data from flash, or default value if no flash values found
 *	- Return values:
 *		1 = Valid data found in flash
 *		0 = Invalid or no data found, filled in defaults
 */
UInt8 variant_retreive_hware( void )
{
	// Check for valid info (structure has actually been loaded with something)
	if (( bl_hware_info.device_id != 0x00000000 ) && ( bl_hware_info.device_id != 0xFFFFFFFF ))
	{
		hware = bl_hware_info;
		return 1;
	} 
	// No valid data found, fill in defaults
	else
	{
		hware.device_id = 0x00001002;
		hware.hardware_version = 4;
		hware.model_id = 0;
		hware.bootloader_version = 5;
		hware.unallocated = 0;
		return 0;
	}	
}

/*
 *	Function handles all TRITIUM PRODUCT VERSION REPORTING SYSTEM requests
 *
 *	-By default a single CAN packet is sent with the requested information per function call
 *	-Calling this function with the 'request all' ID will trigger a re-entrant loop until all available information is sent
 *	-Any IDs that are to be handled externally (ie: jump to bootloader) must still be handled in here to have the function return early
 *	 with the specific ID, otherwise they will be treated as an unknown request..
 *
 */
UInt8 variant_devinfo_req(UInt8 req)
{
	UInt8 i;
	//Setup CAN packet to send
	//Clear all data so dont have any ugly left over data sent allong with requested value
	can_push_ptr->data.data_u64 = 0;
	can_push_ptr->address = can_addr;
	can_push_ptr->status = 8;
	can_push_ptr->data.data_u32[0] = hware.device_id;
	can_push_ptr->data.data_u8[3] = req;

	switch (req)
	{
		case DEV_INFO_REQ_ALL:
			//Send a reply containing the number of info packets that will be sent
			can_push_ptr->data.data_u32[1] = DEV_INFO_TTL_REQ;
			can_push();

			//Send a reply for every info request available
			for (i = DEV_INFO_REQ_DEVID; i <= DEV_INFO_REQ_HWMOD; i++)
			{
				variant_devinfo_req(i);
			}

			//Special case - All packets already added to tx queue, so dont want to call can_push() again
			return 0;
			break;

		case DEV_INFO_REQ_DEVID:
			can_push_ptr->data.data_u32[1] = hware.device_id;
			break;

		case DEV_INFO_REQ_HWVER:
			can_push_ptr->data.data_u32[1] = hware.hardware_version;
			break;

		case DEV_INFO_REQ_MODEL:
			can_push_ptr->data.data_u32[1] = hware.model_id;
			break;

		case DEV_INFO_REQ_BLVER:
			can_push_ptr->data.data_u32[1] = hware.bootloader_version;
			break;

		case DEV_INFO_REQ_BLRSVD:
			can_push_ptr->data.data_u32[1] = hware.unallocated;
			break;

		case DEV_INFO_REQ_SERIAL:
			can_push_ptr->data.data_u32[1] = flash.serial;
			break;

		case DEV_INFO_REQ_BNUM:
			can_push_ptr->data.data_u32[1] = BUILD_NUMBER;
			break;

		case DEV_INFO_REQ_HWMOD:
			//??
			break;

		case DEV_INFO_REQ_BLOAD:
			//Signal that request was not handled, but is known
			//Special case - dont want a reply sent for this ID itself
			return req;
			break;

		case DEV_INFO_REQ_HB:
			//Special case - catch this so an unknown request reply is not sent
			return 0;
			break;

		default:
			//Signal that request is unknown
			can_push_ptr->data.data_u8[3] = DEV_INFO_REQ_UNKNWN;
			can_push_ptr->data.data_u32[1] = req;
			break;
	}
	can_push();
	//Signal that request was handled
	return 0;
}
