//16x2 LCD in 8 bit mode Driver
//just use 
//lcd_string(unsigned char *str, int c) to print string on LCD, 0 print in first line, 1 print in second line 
//lcd_float(float f, int c) to print float on LCD, 0 print in first line, 1 print in second line 
//lcd_clear() to clear display
#include "tm4c123gh6pm.h"
#include "Bit_Utilies.h"
#include <string.h>
#include "LCD.h"
#include <stdio.h>
//assume lcd pin assignment
// RS = PD0
// RW = PD1
// EN = PD2

// D0 = PA7
// D1 = PA6
// D2 = PD3
// D3 = PB4
// D4 = PE5
// D5 = PE4
// D6 = PB1
// D7 = PB0
void delay(long d) {
    while(d--) {}
}



/**
 * @brief Prints a character on the LCD.
 *
 * @param[in] data 8 bit hexadecimal data to be printed on the display.
 *
 * @details This function takes an 8 bit hexadecimal data as argument and prints the corresponding character on the LCD.
 * The LCD is connected to Port A, B, D and E as follows:
 * D0 = PA7
 * D1 = PA6
 * D2 = PD3
 * D3 = PB4
 * D4 = PE5
 * D5 = PE4
 * D6 = PB1
 * D7 = PB0
 */
void printdata(unsigned char data) // data = 8 bit hexadecimal data, this prints a character on lcd 
{
    if(GET_BIT(data,0)) SET_BIT(GPIO_PORTA_DATA_R,7); else CLR_BIT(GPIO_PORTA_DATA_R,7);
    if(GET_BIT(data,1)) SET_BIT(GPIO_PORTA_DATA_R,6); else CLR_BIT(GPIO_PORTA_DATA_R,6);
    if(GET_BIT(data,2)) SET_BIT(GPIO_PORTD_DATA_R,3); else CLR_BIT(GPIO_PORTD_DATA_R,3);
    if(GET_BIT(data,3)) SET_BIT(GPIO_PORTB_DATA_R,4); else CLR_BIT(GPIO_PORTB_DATA_R,4);
    if(GET_BIT(data,4)) SET_BIT(GPIO_PORTE_DATA_R,5); else CLR_BIT(GPIO_PORTE_DATA_R,5);
    if(GET_BIT(data,5)) SET_BIT(GPIO_PORTE_DATA_R,4); else CLR_BIT(GPIO_PORTE_DATA_R,4);
    if(GET_BIT(data,6)) SET_BIT(GPIO_PORTB_DATA_R,1); else CLR_BIT(GPIO_PORTB_DATA_R,1);
    if(GET_BIT(data,7)) SET_BIT(GPIO_PORTB_DATA_R,0); else CLR_BIT(GPIO_PORTB_DATA_R,0);
}

void lcd_data(unsigned char data)           //this is used to pass a character to data register of lcd
{
    printdata(data);                    // pass the 8Bit data to the datalines of lcd
    CLR_BIT(GPIO_PORTD_DATA_R,1);      // Turn off the R/W for write operation in lcd
    SET_BIT(GPIO_PORTD_DATA_R,0);      // Turn On the RS for writing to the data register of lcd
    SET_BIT(GPIO_PORTD_DATA_R,2);      // turn on the En of lcd for enabling the clock of lcd
    delay(10000);                      // wait for sometime
    CLR_BIT(GPIO_PORTD_DATA_R,2);      // Turn off the En of lcd
}

/**
 * @brief Sends a command to the LCD's command register.
 * 
 * This function is used to pass instructions to the command register
 * of the LCD, allowing control over its operation such as clearing
 * the display, setting the cursor position, or configuring display
 * settings.
 * 
 * @param cmd The command byte to be sent to the LCD. This should be
 *            a valid instruction as per the LCD's datasheet.
 */
void lcd_cmd(unsigned char cmd)           //this is used to pass instructions to command register of lcd
{
    printdata(cmd);                    // pass the 8Bit data to the datalines of lcd
    CLR_BIT(GPIO_PORTD_DATA_R,1);      // Turn off the R/W for write operation in lcd
    CLR_BIT(GPIO_PORTD_DATA_R,0);      // Turn off the RS for writing to the command register of lcd
    SET_BIT(GPIO_PORTD_DATA_R,2);      // turn on the En of lcd for enabling the clock of lcd
    delay(10000);                      // wait for sometime
    CLR_BIT(GPIO_PORTD_DATA_R,2);      // Turn off the En of lcd
}


/**
 * @brief Prints a string on the LCD.
 *
 * @param[in] str Pointer to a string to be printed on the LCD.
 * @param[in] c   If true, the string is printed on the second line of the LCD.
 *
 * @details This function takes a pointer to a string and prints it on the LCD.
 * If @p c is true, the string is printed on the second line of the LCD by setting
 * the cursor position to the beginning of the second line before printing the
 * string. If @p c is false, the string is printed starting from the current
 * cursor position.
 */
void lcd_string(unsigned char *str, int c) 
{ 
    unsigned char i;
    unsigned char len = strlen((char *)str);  // Compute the length of the string using strlen
    if(c) 
        lcd_cmd(0xC0);
    for(i = 0; i < len; i++) {
        lcd_data(str[i]);  // Send each character to the LCD
    }
}

/**
 * @brief Prints a floating-point number on the LCD.
 *
 * @param[in] f The floating-point number to be displayed.
 * @param[in] c If true, the number is printed on the second line of the LCD.
 *
 * @details This function formats the given floating-point number into a string
 * and displays it on the LCD. If @p c is true, the number is printed on the
 * second line by setting the cursor position to the beginning of the second
 * line before printing. If @p c is false, the number is printed starting from
 * the current cursor position.
 */
void lcd_float(float f, int c) {
    char speedbuffer[7];
    snprintf(speedbuffer, 20, "%.5f", f);
    lcd_string((unsigned char*)speedbuffer,c);
}
/**
 * @brief Clears the LCD display.
 *
 * @details This function sends a command to the LCD to clear the display.
 * The LCD is cleared by setting the cursor position to the beginning of the
 * first line and then writing a blank character at that position. This
 * effectively clears the entire display.
 */
void lcd_clear() {
    lcd_cmd(0x01);
}

/**
 * @brief Initializes the LCD module.
 * 
 * This function sets up the LCD for operation by configuring the necessary
 * hardware settings, such as data pins, control pins, and any required
 * initialization sequences. It must be called before using any other LCD
 * functions.
 * 
 * @note Ensure that the hardware connections are properly configured
 *       before calling this function.
 */
void lcd_init() {
    //Clock of PORTA, PORTB, PORTD, PORTE
    SET(SYSCTL_RCGCGPIO_R, (1<<0) | (1<<1) | (1<<3) | (1<<4));
    while(GET_BIT(SYSCTL_PRGPIO_R,0)== 0){};
    while(GET_BIT(SYSCTL_PRGPIO_R,1)== 0){};
    while(GET_BIT(SYSCTL_PRGPIO_R,3)== 0){};
    while(GET_BIT(SYSCTL_PRGPIO_R,4)== 0){};

    // Digitalise the datapins and config pins
    SET(GPIO_PORTA_DEN_R, (1<<6)|(1<<7));
    SET(GPIO_PORTB_DEN_R, (1<<0)|(1<<1)|(1<<4));
    SET(GPIO_PORTD_DEN_R, (1<<0)|(1<<1)|(1<<2)|(1<<3));
    SET(GPIO_PORTE_DEN_R, (1<<4)|(1<<5));

    // Direction of datapins and config pins -> Output Pins
    SET(GPIO_PORTA_DIR_R, (1<<6) | (1<<7)); 
    SET(GPIO_PORTB_DIR_R, (1<<0) | (1<<1) | (1<<4));
    SET(GPIO_PORTD_DIR_R, (1<<0) | (1<<1) | (1<<2)|(1<<3)); 
    SET(GPIO_PORTE_DIR_R, (1<<4) | (1<<5));
    //delay
    delay(10000);

    //initialize lcd
    /* LCD Initialization Command Sequence */
    lcd_cmd(0x38);  // Function Set: 
    // - 8-bit data mode 
    // - 2-line display 
    // - 5x8 character font

    lcd_cmd(0x06);  // Entry Mode Set:
    // - Increment cursor automatically after each write
    // - No display shift

    lcd_cmd(0x0C);  // Display Control:
    // - Display ON
    // - Cursor OFF
    // - Blinking OFF

    lcd_cmd(0x01);  // Clear Display:
    // - Clears entire display
    // - Returns cursor to home position (line1,column1)
}
