/*
 * Copyright (c) 2013, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "fsl_device_registers.h"
#include "Declarations.h"
#include "HD44780.h"

int main(void)
{
		uint8_t text[8]="hello !";
    init_hardware();
	
		LCD_Init();	
		LCD_WriteText(text);
		
    while(1)
    {

    }    
}

static void init_hardware(void)
{    
	SIM->SCGC5 |= ( SIM_SCGC5_PORTA_MASK
								| SIM_SCGC5_PORTB_MASK
								| SIM_SCGC5_PORTC_MASK
								| SIM_SCGC5_PORTD_MASK
								| SIM_SCGC5_PORTE_MASK );

	SIM->SOPT2 |= 	SIM_SOPT2_PLLFLLSEL_MASK; 				// set PLLFLLSEL to select the PLL for this clock source 

	PORTB->PCR[18] = PORT_PCR_MUX(1);              		// Set Pin B18 to GPIO function
	PORTB->PCR[19] = PORT_PCR_MUX(1);              		// Set Pin B19 to GPIO function
	PORTD->PCR[1]  = PORT_PCR_MUX(1);               	// Set Pin D1 to GPIO function

	FPTB->PDDR |= (1<<LED_R);                        	// Red LED, Negative Logic (0=on, 1=off)
	FPTB->PDDR |= (1<<LED_G);                      		// Green LED, Negative Logic (0=on, 1=off)
	FPTD->PDDR |= (1<<LED_B);                        	// Blue LED, Negative Logic (0=on, 1=off)

	FPTB->PCOR |= (1<<LED_R);													// Turn off leds
	FPTB->PCOR |= (1<<LED_G);
	FPTD->PCOR |= (1<<LED_B);

	PORTE->PCR[5]  = PORT_PCR_MUX(1);            			// Set Pins to GPIO function
	PORTE->PCR[20] = PORT_PCR_MUX(1);									// TODO: U can do it better !
	PORTE->PCR[21] = PORT_PCR_MUX(1);
	PORTE->PCR[22] = PORT_PCR_MUX(1);
	PORTE->PCR[23] = PORT_PCR_MUX(1);
	PORTE->PCR[29] = PORT_PCR_MUX(1);
	PORTE->PCR[30] = PORT_PCR_MUX(1);

	PTE->PDDR	|= MASK(D4)| MASK(D5) 									// Set output
						|  MASK(D6)| MASK(D7)
						|  MASK(E) | MASK(RS)
						|  MASK(RW);
							
		RGB(1,0,0);		
		_delay_ms(200);
		RGB(0,1,0);
		_delay_ms(200);
		RGB(0,0,1);
		_delay_ms(200);
		RGB(0,0,0);
		_delay_ms(200);
		RGB(1,1,1);		
		_delay_ms(400);
		RGB(0,0,0);		
		_delay_ms(200);		
		RGB(1,1,1);		
		_delay_ms(400);
		RGB(0,0,0);		
		_delay_ms(200);
		
}

void RGB(uint16_t Red,uint16_t Green,uint16_t Blue)
{
  (void)(Red   ? (FPTB->PCOR |= MASK(LED_R)) : (FPTB->PSOR |= MASK(LED_R)));
	(void)(Green ? (FPTB->PCOR |= MASK(LED_G)) : (FPTB->PSOR |= MASK(LED_G)));
	(void)(Blue  ? (FPTD->PCOR |= MASK(LED_B)) : (FPTD->PSOR |= MASK(LED_B)));
}

void _delay_ms(unsigned delayTicks)
{    
	delayTicks *=(uint64_t)SystemCoreClock/10000u;
    while(delayTicks--)
    {
        __ASM("nop");
    }
}
