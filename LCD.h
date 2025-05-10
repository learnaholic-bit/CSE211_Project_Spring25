// LCD_Interface.h
#ifndef LCD
#define LCD

#include "TM4C123.h"
#include "Bit_Utilies.h"

// Function Prototypes
void lcd_init();
void delay(long d);
void printdata(unsigned char data);
//interfacing functions
void lcd_data(unsigned char data);
void lcd_cmd(unsigned char cmd);
void lcd_clear();
void lcd_string(unsigned char *str, int c) ;
void lcd_float(float f, int c);


#endif
