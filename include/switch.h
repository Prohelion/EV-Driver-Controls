/*
 * Tritium switch input interface
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

#ifndef SWITCH_H
#define SWITCH_H

// Public function prototypes
void switch_init( unsigned int *state );
void switch_update( unsigned int *state );

// Public variables
// Switch inputs - same bitfield positions as CAN packet spec
extern unsigned int switches;

// Driver controls switch position packet bitfield positions
#define SW_MODE_R			0x0001
#define SW_MODE_N			0x0002
#define SW_MODE_B			0x0004
#define SW_MODE_D			0x0008
#define SW_IGN_ACC			0x0010
#define SW_IGN_ON			0x0020
#define SW_IGN_START		0x0040
#define SW_BRAKE			0x0080
#define SW_FUEL				0x0100
#define SW_SPARE1			0x0200
#define SW_SPARE2			0x0400
#define SW_SPARE3			0x0800
#define SW_ACCEL_FAULT		0x1000
#define SW_CAN_FAULT		0x2000
#define SW_BRAKE_FAULT		0x4000
#define SW_REV_FAULT		0x8000

// Input de-glitching filter length
#define SW_FILTER_LENGTH	5

#endif	// SWITCH_H
