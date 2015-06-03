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

    RGB(0,1,0);		
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
    
    SIM->SOPT2 |= SIM_SOPT2_PLLFLLSEL_MASK; 			// set PLLFLLSEL to select the PLL for this clock source 

    PORTB->PCR[18] = PORT_PCR_MUX(1);              		// Set Pin B18 to GPIO function
    PORTB->PCR[19] = PORT_PCR_MUX(1);              		// Set Pin B19 to GPIO function
    PORTD->PCR[1] = PORT_PCR_MUX(1);               		// Set Pin D1 to GPIO function
    FPTB->PDDR |= (1<<18);                         		// Red LED, Negative Logic (0=on, 1=off)
    FPTB->PDDR |= (1<<19);                        		// Green LED, Negative Logic (0=on, 1=off)
    FPTD->PDDR |= (1<<1);                          		// Blue LED, Negative Logic (0=on, 1=off)
}

void RGB(int Red,int Green,int Blue)
{
    if (Red == 1)
        FPTB->PCOR |= (1<<18);
    else
        FPTB->PSOR |= (1<<18);
    
    if (Green == 1)
        FPTB->PCOR |= (1<<19);
    else
        FPTB->PSOR |= (1<<19);
    
    if (Blue == 1)
        FPTD->PCOR |= (1<<1);
    else
        FPTD->PSOR |= (1<<1);
}

void _delay_ms(unsigned delayTicks)
{    
	delayTicks *=(uint64_t)SystemCoreClock/10000u;
    while(delayTicks--)
    {
        __ASM("nop");
    }
}
