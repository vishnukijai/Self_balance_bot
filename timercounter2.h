//Micricontroller		:All 8-bit AVR Micricontrollers with Similar Register Configuration with ATmega16
//System Clock			:1 MHz - 16 MHz
//Header File Version 		:1.1
//Author			:Arun Kumar Garg 
//				:ABLab Solutions
//				:www.ablab.in
//				:arun@ablab.in
//Date				:1st August 2011

/*Copyright (c) 2011 ABLab Solutions All rights reserved.
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the 	following disclaimer in the documentation 
     and/or other materials provided with the distribution.
   * Neither the name of the copyright holders nor the names of contributors may be used to endorse or promote products derived from this software without 
     specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */


/*The functions and macros defined in this header file are for Timer Counter 2 of all 8-bit AVR Micricontrollers with Similar Register Configuration with ATmega16*/

#ifndef	_TIMERCOUNTER2_H_
#define	_TIMERCOUNTER2_H_	1



#include<avr/io.h>



/*This function is declared to set the mode of the timer counter 2. The following values sets the following modes:-
mode=0 Normal mode
     1 Phase correct PWM mode
     2 CTC mode
     3 Fast PWM mode.*/

void set_timercounter2_mode(unsigned char mode);



/*This function is declared to set the prescaller(frequency of timer counter 2) of the timer counter 2. The following values sets the following prescallers:-
prescaler=0 no clock
	  1 1
	  2 8
	  3 64
	  4 256
	  5 1024
	  6 external falling edge
	  7 external rising edge.*/

void set_timercounter2_prescaler(unsigned char prescaler_value);



/*This function is declared to set the output mode of the timer counter 2.The following values sets the following output mode:-*/
output mode for non-PWM
		=0 normal port operation 
		=1 toggle on compare match
		=2 clear on compare match
		=3 set on compare match
Phase correct PWM
		=0 normal port operation
		=1 reserved
		=2 clear on on compare match whe up counting and set on on compare match when down counting
		=3 set on on compare match whe up counting and clear on on compare match when down counting
Fast PWM
		=0 normal port operation
		=1 reserved
		=2 clear on on compare match 
		=3 set on on compare match.*/

void set_timercounter2_output_mode(unsigned char output_mode);



/*This function is declared to enable the interrupt of timer counter 2. The following values enables the following interrupt:-*/
interrupt type=0 for overflow
	      =1 for compare match.*/

void enable_timercounter0_interrupt(unsigned char interrupt_type);



/*This function is declared to disable the interrupt of timer counter 2. The following values disables the following interrupt:-*/
interrupt type=0 for overflow
	      =1 for compare match.*/

void disable_timercounter2_interrupt(unsigned char interrupt_type);



/*This function declared to set the Output Compare Register 2(OCR2) Register value of timer counter 2.*/

void set_timercounter2_compare_value(unsigned char compare_value);



/*Function definations*/

void set_timercounter2_mode(unsigned char mode)
{
	if(mode==0)
	{
		TCCR2|=0;
	}
	else if(mode==1)
	{
		TCCR2|=_BV(WGM00);
	}
	else if(mode==2)
	{
		TCCR2|=_BV(WGM01);
	}
	else if(mode==3)
	{
		TCCR2|=_BV(WGM00)|_BV(WGM01);
	}
	else
		;

}

void set_timercounter2_prescaler(unsigned char prescaler_value)
{
	TCCR2|=prescaler_value;
}

void set_timercounter2_output_mode(unsigned char output_mode)
{
	TCCR2|=output_mode<<4;
}

void enable_timercounter2_interrupt(unsigned char interrupt_type)
{
	TIMSK|=1<<(interrupt_type+6);
}

void disable_timercounter2_interrupt(unsigned char interrupt_type)
{
	TIMSK&=~(1<<(interrupt_type+6));
}

void set_timercounter2_compare_value(unsigned char compare_value)
{
	OCR2=compare_value;
}
#endif





