/** ###############HD44780.c###########################################
**     Filename  : HD44780.c
**     Processor : MKL25Z
**     Date/Time : 2015-31-05, 20:34
** ###################################################################
----1----2----3----4----5----6----7-10----11--12--13--14--15----16--LCD
-------------------E05--E20--E21-NC--NC---E22-E23-E29-E30-----------Kinetis
---Vss--Vcc--Vee---Rs---Rw---E---DB0-DB3--DB4-DB5-DB6-DB7-LED+--LED-
*/

#include "fsl_device_registers.h"
#include "Declarations.h"
#include "HD44780.h"

void LCD_IO_PutVal(uint8_t val){
	
	(void)((val & MASK(0)) ? (PTE->PSOR |= MASK(D4)): (PTE->PCOR |= MASK(D4)));
	(void)((val & MASK(1)) ? (PTE->PSOR |= MASK(D5)): (PTE->PCOR |= MASK(D5)));
	(void)((val & MASK(2)) ? (PTE->PSOR |= MASK(D6)): (PTE->PCOR |= MASK(D6)));
	(void)((val & MASK(3)) ? (PTE->PSOR |= MASK(D7)): (PTE->PCOR |= MASK(D7)));
}

uint8_t LCD_IO_GetVal(){

	uint8_t temp;
	
	(void)((PTE->PDIR & MASK(D7)) ? (temp |=MASK(3)): (temp &=~MASK(3)));
	(void)((PTE->PDIR & MASK(D6)) ? (temp |=MASK(2)): (temp &=~MASK(2)));
	(void)((PTE->PDIR & MASK(D5)) ? (temp |=MASK(1)): (temp &=~MASK(1)));
	(void)((PTE->PDIR & MASK(D4)) ? (temp |=MASK(0)): (temp &=~MASK(0)));
	
	return temp;
}

uint8_t LCD_ReadNibble(void) {
	
	uint8_t temp=0;
	
	Set_Input;
	E_PutVal(1);
	temp = LCD_IO_GetVal();
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
	LCD_IO_PutVal(Data);
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

	_delay_ms(50);														// 50ms dla ustabilizowania siê napiêcia zasilania
	
																						// start initialization	                                                 
	LCD_WriteCommand((uint8_t)LCD_INIT);
	_delay_ms(1);                             // 1ms, minimum 39us
	
	LCD_WriteCommand((uint8_t)LCD_FSET);
	_delay_ms(1);                             // 1ms, minimum 39us
	
	LCD_WriteCommand((uint8_t)LCD_FSET);
	_delay_ms(1);                             // 1ms, minimum 37us
	
	LCD_WriteCommand((uint8_t)LCD_OFFON);
	_delay_ms(1);                             // 1ms, minimum 37us
	
	LCD_WriteCommand((uint8_t)LCD_CLR);
	_delay_ms(2);                             // 2ms, minimum 1.54ms
	
	LCD_WriteCommand((uint8_t)LCD_EMODE);
	_delay_ms(1);                             // 1ms (O tym dokumentacja nie wspomina³a....)
	
	                                          // stop initialization
}
