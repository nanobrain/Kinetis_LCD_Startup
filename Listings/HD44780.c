/** ###################################################################
**     Filename  : HD44780.c
**     Processor : MKL25Z
**     Date/Time : 2015-31-05, 20:34
** ###################################################################*/
//----1----2----3----4----5----6----7-10----11--12--13--14--15----16--LCD
//-------------------E05--E20--E21-NC--NC---E22-E23-E29-E30-----------Kinetis
//---Vss--Vcc--Vee---Rs---Re---E---DB0-DB3--DB4-DB5-DB6-DB7-LED+--LED-

#include "fsl_device_registers.h"
#include "Declarations.h"
#include "HD44780.h"

#define D5	(22) // PORT E
#define D6	(23)
#define D7	(29)
#define D8	(30)
#define E		(21)
#define RS	(5)
#define RW	(20)
#define MASK(x) (1UL << (x))

#define E_PutVal(value) 	PTE->PDOR  =  (value ? PTE->PDOR | MASK(E): PTE->PDOR & ~MASK(E))
#define RS_PutVal(value) 	PTE->PDOR  =  (value ? PTE->PDOR | MASK(RW): PTE->PDOR & ~MASK(RW))
#define RW_PutVal(value) 	PTE->PDOR  =  (value ? PTE->PDOR | MASK(RS): PTE->PDOR & ~MASK(RS))
#define Set_Input  				PTE->PDDR &= ~MASK(D5) & ~MASK(D6) & ~MASK(D7) & ~MASK(D8)
#define Set_Output 				PTE->PDDR |=  MASK(D5) |  MASK(D6) |  MASK(D7)|  MASK(D8)

void LCD_IO_PutVal(uint8_t val)
{
	PTE->PDOR=(val&(MASK(D5)) ? PTE->PDOR | MASK(D5): PTE->PDOR & ~MASK(D5));
	PTE->PDOR=(val&(MASK(D6)) ? PTE->PDOR | MASK(D6): PTE->PDOR & ~MASK(D6));
	PTE->PDOR=(val&(MASK(D7)) ? PTE->PDOR | MASK(D7): PTE->PDOR & ~MASK(D7));
	PTE->PDOR=(val&(MASK(D8)) ? PTE->PDOR | MASK(D8): PTE->PDOR & ~MASK(D8));
}
uint8_t LCD_IO_GetVal()
{
	uint8_t temp;
	temp  = (PTE->PDIR & MASK(D5))<<7;
	temp |= (PTE->PDIR & MASK(D6))<<6;
	temp |= (PTE->PDIR & MASK(D7))<<5;
	temp |= (PTE->PDIR & MASK(D8))<<4;
	E_PutVal(0);
	E_PutVal(1);
	temp = (PTE->PDIR & MASK(D5))<<3;
	temp |= (PTE->PDIR & MASK(D6))<<2;
	temp |= (PTE->PDIR & MASK(D7))<<1;
	temp |= (PTE->PDIR & MASK(D8));

	return temp;
}

uint8_t LCD_ReadNibble(void) {

	uint8_t temp=0;
	
	Set_Input;
	E_PutVal(1);
	temp = //read;
	E_PutVal(0);
	Set_Output;	
	return temp;
}
  
uint8_t LCD_CheckStatus(void){

	uint8_t status = 0;
	RS_PutVal(0);
	RW_PutVal(1);
	status |= (LCD_ReadNibble() << 4);
	status |=  LCD_ReadNibble();
	return status;

}

void LCD_WriteNibble(uint8_t Data){

	E_PutVal(1);
	LCD_IO_PutVal((uint8_t)(Data));
	E_PutVal(0);

}

void LCD_WriteData(uint8_t DataToWrite){

	RS_PutVal(1);
	RW_PutVal(0);
	
	LCD_WriteNibble(DataToWrite >> 4);
	LCD_WriteNibble(DataToWrite & 0x0F);
	
	while(LCD_CheckStatus() & 0x80);
}

void LCD_WriteText(uint8_t * text) {

	while(*text) {
	  LCD_WriteData(*text);
	  text++;
}

}

void LCD_WriteCommand(uint8_t command){

	E_PutVal(0);
	RS_PutVal(0);
	RW_PutVal(0); 
	LCD_WriteNibble(command >> 4);
	LCD_WriteNibble(command & 0x0F);

}

void LCD_Init(void){

                
	_delay_ms(50);                            // 50ms dla ustabilizowania siê napiêcia zasilania
	
	PORTE->PCR[5] = PORT_PCR_MUX(1);            		// Set Pins to GPIO function
	PORTE->PCR[20] = PORT_PCR_MUX(1);								// TODO: U can do it better !
	PORTE->PCR[21] = PORT_PCR_MUX(1);
	PORTE->PCR[22] = PORT_PCR_MUX(1);
	PORTE->PCR[23] = PORT_PCR_MUX(1);
	PORTE->PCR[29] = PORT_PCR_MUX(1);
	PORTE->PCR[30] = PORT_PCR_MUX(1);

	PTE->PDDR	|= MASK(D5)| MASK(D6) 					// Set output
						|  MASK(D7)| MASK(D8)
						|  MASK(E) | MASK(RS)
						|  MASK(RW);
	                                           // start initialization
	                                                 
	LCD_WriteCommand((uint8_t)LCD_INIT);
	_delay_ms(1);                              // 1ms, minimum 39us
	
	LCD_WriteCommand((uint8_t)LCD_FSET);
	_delay_ms(1);                              // 1ms, minimum 39us
	
	LCD_WriteCommand((uint8_t)LCD_FSET);
	_delay_ms(1);                              // 1ms, minimum 37us
	
	LCD_WriteCommand((uint8_t)LCD_OFFON);
	_delay_ms(1);                              // 1ms, minimum 37us
	
	LCD_WriteCommand((uint8_t)LCD_CLR);
	_delay_ms(2);                              // 2ms, minimum 1.54ms
	
	LCD_WriteCommand((uint8_t)LCD_EMODE);
	_delay_ms(1);                              // 1ms (O tym dokumentacja nie wspomina³a....)
	
	                                                // stop initialization
}
