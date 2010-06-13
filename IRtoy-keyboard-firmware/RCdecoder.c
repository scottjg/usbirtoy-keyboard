/*
*
*	USB infrared remote control receiver transmitter firmware v1.0
*	License: creative commons - attribution, share-alike 
*	Copyright Ian Lesnet 2010
*	http://dangerousprototypes.com
*
*/
//
// IRman compatible RC decoder for RC5, RC5x
// Sends a 6 byte packet:
// RC5 address byte (0)
// RC5 command byte (1) *bit 6 is RC5x bit
// Bytes 2-5 are 0 padding
//
//#ifdef RC_DECODER
#include "HardwareProfile.h"

//USB stack
#include "usb_config.h" //download these files from Microchip
#include "./USB/usb.h"
#include "./USB/usb_device.h" 
#include "./USB/usb_function_hid.h"

#include "decoder.h"

extern struct _irtoy irToy;
extern USB_HANDLE lastINTransmission;
static enum _RC5STATE{
	IDLE=0,
	HALF_PERIOD,
	BIT_PERIOD_0,
	BIT_PERIOD_1,
	SEND,
}decoderState=IDLE;

static struct _RC5decoder{
	unsigned char bp0;//manchester period 0
	unsigned char bp1;//manchester period 1
	unsigned char bcnt;//bit count
}RC5;

void SetupRC5(void){
	decoderState=IDLE;
	CCP1CON=0; //disable any PWM
	T2CON=0;
	T2IE=0; //disable any Timer 2 interrupt
	T2IF=0;
	IRRX_IF=0;
	IRRX_IE=1; //enable RX interrupts for data ACQ

}


void ProcessIR(void){   
	static unsigned char i;
	//static unsigned char rc5x, toggle, addr, cmd;
	static unsigned char need_keyup = 0;

	if((decoderState==SEND) && !HIDTxHandleBusy(lastINTransmission)){
	
		//process IR data (decode RC5)
		//rc5x=irToy.s[0]; //0 second start bit, use below
		//toggle=irToy.s[1]; //1 toggle bit, discard

		unsigned char addr;
		unsigned char data;
		unsigned char key;
		if(irToy.s[0] > 0x50) {  //if start pulse is more than 50 samples
			if(decodeNec(&addr, &data) != 1) //try to decode it as a NEC remote
				goto end;
		} else { //otherwise it might be RC5?
			if(decodeRC5(&addr, &data) != 1)
				goto end;
		}

		if(addr != 0x05) goto end;

		switch(data) {
			case 0x20: 
				key = 0x52; //up
				break;
			case 0x21:
				key = 0x51; //down
				break;
			case 0x15:
				key = 0x50; //left
				break;
			case 0x16:
				key = 0x4f; //right
				break;
			case 0x0b: //enter
				key = 0x28;
				break;
			case 0x1d: //menu
				key = 0x10; //'m'
				break;
			case 0x3c: //info
				key = 0x06; //'c'
				break;
			case 0x31: //exit
				key = 0x29; //esc
				break;
			case 0x35: //play
				key = 0x04 + ('p' - 'a'); //'p'
				break;
			case 0x29: //pause
				key = 0x2c;
				break;
			case 0x36: //stop
				key = 0x04 + ('x' - 'a'); //'x'
				break;
			case 0x32: //rew
				key = 0x04 + ('r' - 'a'); //'r'
				break;
			case 0x34: //ff
				key = 0x04 + ('f' - 'a'); //'f'
			default:
				key = 0;
		}

		hid_report_in[0]=0x00;
		hid_report_in[1]=0x00;
		hid_report_in[2]=key;
		hid_report_in[3]=0x00;
		hid_report_in[4]=0x00;
		hid_report_in[5]=0x00;
		hid_report_in[6]=0x00;
		hid_report_in[7]=0x00;

		lastINTransmission = HIDTxPacket(HID_EP, (BYTE*)hid_report_in, 0x08);
		need_keyup = 1;
end:
		decoderState=IDLE;//wait for more RC commands....
		IRRX_IE=1;//interrupts back on	

	} else if(need_keyup && !HIDTxHandleBusy(lastINTransmission)) { 
		hid_report_in[0]=0x00;
		hid_report_in[1]=0x00;
		hid_report_in[2]=0x00;
		hid_report_in[3]=0x00;
		hid_report_in[4]=0x00;
		hid_report_in[5]=0x00;
		hid_report_in[6]=0x00;
		hid_report_in[7]=0x00;

		lastINTransmission = HIDTxPacket(HID_EP, (BYTE*)hid_report_in, 0x08);
		need_keyup = 0;
	}

}	

//high priority interrupt routine
#pragma interrupt IRdecoderInterruptHandlerHigh
void IRdecoderInterruptHandlerHigh (void){
	unsigned char transitioned = 0, donesampling = 0;

	//RC5 decoder
	if(IRRX_IE==1 && IRRX_IF == 1){ //if RB Port Change Interrupt	
	  if(decoderState==IDLE && ((IRRX_PORT & IRRX_PIN)==0) ){//only if idle and pin state == 0
			IRRX_IE=0;	//disable port b interrupt
  			LED_LAT |= LED_PIN; //LED ON! (off after .5 bit period)
			//T2_RC5halfbitperiod(); //setup to hit in the middle of next bit period
			T2_RXsampleperiod();
			T2IF=0;//clear the interrupt flag
			T2IE=1; //able interrupts...
			T2ON=1;

			irToy.samplecount = 0;
			irToy.s[irToy.samplecount] = 0;
			decoderState=BIT_PERIOD_0;
		}	
    	IRRX_IF = 0;    //Reset the RB Port Change Interrupt Flag bit  

  	}else if(T2IE==1 && T2IF==1){
		switch(decoderState){
			case BIT_PERIOD_0:
				if((IRRX_PORT & IRRX_PIN)!=0) {
					transitioned = 1;
					decoderState=BIT_PERIOD_1;
				} else {
				}
				break;
			case BIT_PERIOD_1:
				if((IRRX_PORT & IRRX_PIN)==0) {
					transitioned = 1;
					decoderState=BIT_PERIOD_0;
				}
				break;
		}//switch

		if(transitioned) {
			irToy.samplecount++;
			if(irToy.samplecount == 0x80) { //overflow
				decoderState=IDLE;
				T2ON=0; //turn off timer
				IRRX_IE=1; //reenable ir rx interrupt
				LED_LAT &= (~LED_PIN); //LED OFF!
			} else {
				irToy.s[irToy.samplecount] = 0;
			}

		} else if(!donesampling) {
			irToy.s[irToy.samplecount]++;			
			//finished sampling because we got a lot of idle time
			if(irToy.s[irToy.samplecount] == 100) {
				donesampling = 1;
			}
		}

		if(donesampling) {
			decoderState=SEND;
			T2ON=0; //turn off timer
			LED_LAT &= (~LED_PIN); //LED OFF!
		}

		T2IF=0;//clear the interrupt flag
	}//if   
}
//#endif //RC_DECODER

