/** ###################################################################
**     Filename  : Declarations.h
**     Processor : Universal
**     Date/Time : 2015-31-05, 20:34
** ###################################################################*/
#define MASK(x) (1UL << (x))
void _delay_ms(unsigned delayTicks);
void RGB(int Red,int Green,int Blue);				// RGB-LED Control: 1=on, 0=off, for each of the 3 colors
static void init_hardware(void);
