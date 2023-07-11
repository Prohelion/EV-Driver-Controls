/*
 * Hardware info version recovery from bootloader flash sector
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

#ifndef VARIANT_H
#define VARIANT_H

#include "tri86.h"

//This is the 1st version of the Bootloader which expects a jump to it from user code instead of a WD reset
#define NEWBL_JUMP_MIN_VER	5

//TRITIUM PRODUCT VERSION REPORTING SYSTEM COMMAND IDs
#define DEV_INFO_REQ_HB		0x00	//Device normal HB reserved
#define DEV_INFO_REQ_ALL	0x01	//Request all device information available
#define DEV_INFO_REQ_DEVID	0x02	//Device ID				-----Begin BL device info
#define DEV_INFO_REQ_HWVER	0x03	//HardWare version		-----
#define DEV_INFO_REQ_MODEL	0x04	//Model ID				-----
#define DEV_INFO_REQ_BLVER	0x05	//Bootloader version	-----
#define DEV_INFO_REQ_BLRSVD	0x06	//Reserved				-----End BL device info
#define DEV_INFO_REQ_SERIAL	0x07	//Serial Number
#define DEV_INFO_REQ_BNUM	0x08	//Firmware build number
#define DEV_INFO_REQ_HWMOD	0x09	//Is this a dupe of HWVER??? 
//
#define DEV_INFO_EVT_MSG	0xFD	//A system event has occured (Error, etc...)
#define DEV_INFO_REQ_UNKNWN	0xFE	//Unknown request - REPLY ONLY
#define DEV_INFO_REQ_BLOAD	0xFF	//Jump to bootloader
//
#define DEV_INFO_TTL_REQ	(DEV_INFO_REQ_HWMOD - DEV_INFO_REQ_ALL)	//Total number of items that get sent with 'DEV_INFO_REQ_ALL'

// 8-byte struct inside bootloader sector
typedef struct hware_info_	
{
	UInt32	device_id;				// 4 bytes
	UInt8	hardware_version;		// 1 byte
	UInt8	model_id;				// 1 byte
	UInt8	bootloader_version;		// 1 byte
	UInt8	unallocated;			// 1 byte
} hware_info;

extern hware_info hware;

UInt8 variant_retreive_hware(void);
UInt8 variant_devinfo_req(UInt8 req);

#endif // VARIANT_H
