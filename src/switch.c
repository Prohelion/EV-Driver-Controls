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

// Include files
#include <msp430x24x.h>
#include "tri86.h"
#include "switch.h"

// Public variables
unsigned int switches = 0x0000;

// Public functions
void switch_init( unsigned int *state )
{
}

/*
 * Collect switch inputs from hardware and fill out current state
 *	- Inverts active low switches so that all bits in the state register are active high
 *	- Provides de-glitching functionality
 *	- This routine should be called at a regular rate, preferably at least 
 *	  SW_FILTER_LENGTH faster than the switch positions are actually required	
 */
void switch_update( unsigned int *state )
{
	static char switch_count[9] = {0,0,0,0,0,0,0,0,0};
	unsigned char ii;
	
	// Grab current state of pins (invert if necessary)
	// Keep track of how many update cycles it's been active or inactive
	if ( P2IN & IN_GEAR_4 ) switch_count[0]++;		// Active high
	else switch_count[0]--;

	if ( P2IN & IN_GEAR_3 ) switch_count[1]++;		// Active high
	else switch_count[1]--;

	if ( P2IN & IN_GEAR_2 ) switch_count[2]++;		// Active high
	else switch_count[2]--;

	if ( P2IN & IN_GEAR_1 ) switch_count[3]++;		// Active high
	else switch_count[3]--;
	
	if ( P1IN & IN_IGN_ACCn ) switch_count[4]--;		// Active low
	else switch_count[4]++;
	
	if ( P1IN & IN_IGN_ONn ) switch_count[5]--;		// Active low
	else switch_count[5]++;

	if ( P1IN & IN_IGN_STARTn ) switch_count[6]--;	// Active low
	else  switch_count[6]++;

	if ( P1IN & IN_BRAKEn ) switch_count[7]--;		// Active low
	else  switch_count[7]++;

	if ( P1IN & IN_FUEL ) switch_count[8]++;			// Active high
	else switch_count[8]--;

	// Saturate switch counts (determines length of 'filter')
	for( ii = 0; ii < 9; ii++ )
	{
		if ( switch_count[ii] > SW_FILTER_LENGTH ) switch_count[ii] = SW_FILTER_LENGTH;
		else if ( switch_count[ii] < 0 ) switch_count[ii] = 0;
	}

	// Update bitfield for rest of program based on counts
	if ( switch_count[0] == SW_FILTER_LENGTH ) *state |= SW_MODE_R;
	else if ( switch_count[0] == 0 ) *state &= ~SW_MODE_R;

	if ( switch_count[1] == SW_FILTER_LENGTH ) *state |= SW_MODE_N;
	else if ( switch_count[1] == 0 ) *state &= ~SW_MODE_N;

	if ( switch_count[2] == SW_FILTER_LENGTH ) *state |= SW_MODE_B;
	else if ( switch_count[2] == 0 ) *state &= ~SW_MODE_B;
	
	if ( switch_count[3] == SW_FILTER_LENGTH ) *state |= SW_MODE_D;
	else if ( switch_count[3] == 0 ) *state &= ~SW_MODE_D;
	
	if ( switch_count[4] == SW_FILTER_LENGTH ) *state |= SW_IGN_ACC;
	else if ( switch_count[4] == 0 ) *state &= ~SW_IGN_ACC;
	
	if ( switch_count[5] == SW_FILTER_LENGTH ) *state |= SW_IGN_ON;
	else if ( switch_count[5] == 0 ) *state &= ~SW_IGN_ON;
	
	if ( switch_count[6] == SW_FILTER_LENGTH ) *state |= SW_IGN_START;
	else if ( switch_count[6] == 0 ) *state &= ~SW_IGN_START;
	
	if ( switch_count[7] == SW_FILTER_LENGTH ) *state |= SW_BRAKE;
	else if ( switch_count[7] == 0 ) *state &= ~SW_BRAKE;
	
	if ( switch_count[8] == SW_FILTER_LENGTH ) *state |= SW_FUEL;
	else if ( switch_count[8] == 0 ) *state &= ~SW_FUEL;
}


