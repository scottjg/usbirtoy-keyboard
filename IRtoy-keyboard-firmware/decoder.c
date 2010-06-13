/*
*
*	USB infrared remote control receiver transmitter firmware v1.0
*	License: creative commons - attribution, share-alike 
*	Copyright Ian Lesnet 2010
*	http://dangerousprototypes.com
*
*/
//IR signal decoder. uses irToy.s[] array of pulse lengths (1 = .1ms)

#include "HardwareProfile.h"
extern struct _irtoy irToy;



//Decodes NEC style IR pulses
//format is:
// start pulse | address | more address or address crc | command | command crc
//             |  8-bits |  8-bits                     |  8-bits |  8-bits 
//start pulse is ~9ms high, ~4.5ms low
//rest is LSB first: 1 = ~.5ms high, ~1.7ms low
//                   0 = ~.5ms high, ~.5ms low
//
int decodeNec(unsigned char *addr, unsigned char *data) 
{
	int i, bound;
	unsigned char address = 0, command=0, crc=0;
	unsigned char b = 1;

	static unsigned char prev_address = 0, prev_command=0;

	//repeat code
	if(irToy.samplecount == 3) {	
		*addr = prev_address;
		*data = prev_command;
		return 2;
	} else if(irToy.samplecount < 0x43) //otherwise, probably just noise
		return 0;

	//get address (skip first high-low pulses)
	for(i = 2+1; i < 3+(8*2); i+=2) {
		if(irToy.s[i] > 10)
			address |= b;
		b <<= 1;
	}

	//get command
	b=1;
	i+=16;
	bound = i + 16;
	for(; i < bound; i+=2) {
		if(irToy.s[i] > 10)
			command|= b;
		b <<= 1;
	}

	//get checksum for command
	b=1;
	bound = i + 16;
	for(; i < bound; i+=2) {
		if(irToy.s[i] > 10)
			crc |= b;
		b <<= 1;
	}

	//verify checksum
	if(command != ~crc)
		return 0;

	*addr = address;
	*data = command;

	prev_command = command;
	prev_address = address;

	return 1;
}

//Phillips RC5 infrared decoder code
// I realize this is completely illegible. Most of the IR formats
// seemed to use a variable pulse width, but RC5 uses a modified 
// manchester encoding. So, now we have this array of distances 
// between transitions, and we really just want to get logic level at
// certain times. Oh well, the distance list really made NEC easy at least.
int decodeRC5(unsigned char *addr, unsigned char *data) 
{
	int i;
	unsigned short b = 0x1000;//1 << 12;
	unsigned short bits = 0;
	unsigned char address, command;	
	unsigned char toggle = 0;
	unsigned char repeat = 0;

	static unsigned char prev_address = 0, prev_command = 0, prev_toggle = 0;

	if(irToy.samplecount < 13) //probably just noise
		return 0;

	i=0;
	while(i < irToy.samplecount) {
		unsigned char logic = (i & 1) ? 0 : 1;
		if(logic == 0) {
			if(irToy.s[i] > 12) {
				bits |= b;
			} else
				i++;		
		} else if(logic == 1) {
			if(irToy.s[i] <= 12) {
				bits |= b;
				i++;
			}
		}
		b>>=1;
		i++;
		if(b == 0) break;
	}

	//first 5 bits address, last 6 data, 
	address = (bits >> 6) & 0x1F;
	command = bits & 0x3F;

	//repeats will use the same toggle bit value. 
	//if the key was actually pressed again, toggle will be inverted
	toggle = bits & 0x800 ? 1 : 0;
	if(prev_address == address && prev_command == command) {
		if(prev_toggle == toggle) {
			repeat = 1;
		}
	}

	prev_address = address;
	prev_command = command;
	prev_toggle = toggle;	

	*addr = address;
	*data = command;

	if(!repeat)
		return 1;
	else
		return 2;
}