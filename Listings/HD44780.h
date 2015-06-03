/** ###################################################################
**     Filename  : HD44780.h
**     Processor : Universal
**     Date/Time : 2015-31-05, 20:34
** ###################################################################*/

#ifndef __HD44780_H
#define __HD44780_H

#define LCD_INIT    0x30                    // function set
#define LCD_FSET    0x28                    // 4-bit, two lines, 5x7 font
#define LCD_OFFON   0x0C                    // display on, cursor off
#define LCD_CLR     0x01                    // display clear
#define LCD_HOME    0x02                    // return cursor to home; disable shift
#define LCD_2Line   0xC0                    // moves cursor to second line
#define LCD_1Ctr    0x08                    // moves cursor to center of second line
#define LCD_2Ctr    0xC8                    // moves cursor to center of second line
#define LCD_EMODE   0x06                    // entry mode. Shift disable, cursor increment

void LCD_SetInput(void);
void LCD_SetOutput(void);
uint8_t LCD_ReadNibble(void);
uint8_t LCD_CheckStatus(void);
void LCD_WriteNibble(uint8_t Data);
void LCD_WriteData(uint8_t DataToWrite);
void LCD_WriteText(uint8_t* text);
void LCD_WriteCommand(uint8_t command);
void LCD_Init(void);
#endif
