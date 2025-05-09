#include "GPIO.h"
#include "Bit_Utilies.h"
#include "tm4c123gh6pm.h"
#include "TM4C123.h"



// ************************************//BUZZER//*********************************
// GPIO PORTD3 TO BUZZER
//assume pin allignment
//PE1

void GPIO_initPortE_BUZZER(){
    SET_BIT(SYSCTL_RCGCGPIO_R, 4); // Enable Port E clock
    while(GET_BIT(SYSCTL_PRGPIO_R, 4) == 0); // Wait for clock
    GPIO_PORTE_CR_R |= 0x02;         // Allow changes to PE0-PE7
    GPIO_PORTE_AMSEL_R &= ~0x02;     // Disable analog on PE0-PE7
    GPIO_PORTE_PCTL_R = 0;           // GPIO mode for PE0-PE7
    GPIO_PORTE_DIR_R |= 0x02;        // PE0-PE7 as outputs
    GPIO_PORTE_AFSEL_R &= ~0x02;     // Disable alt functions on PE0-PE7
    GPIO_PORTE_DEN_R |= 0x02;        // Enable digital I/O on PE0-PE7
}

// ************************************//Switches & LEDs//*********************************
// GPIO PORTF TO Switches & LEDs
//assume pin allignment (same as on board)
//PF0		SW1
//PF1		LED_RED
//PF2		LED_BLUE
//PF3		LED_GREEN
//PF4		SW2

void GPIO_initPortF(void){
		SET_BIT(SYSCTL_RCGCGPIO_R,5);						// Enable Port F clock
    while(GET_BIT(SYSCTL_PRGPIO_R,5)== 0);	// Wait for clock
    GPIO_PORTF_LOCK_R= GPIO_LOCK_KEY;				// Unlock Port F
    GPIO_PORTF_CR_R |= 0x1F;								// Allow changes to PF0-Pf4
    GPIO_PORTF_AFSEL_R &=0x00;							// Disable alt functions on PF0-Pf4							
    GPIO_PORTF_AMSEL_R &=0x00;							// Disable analog on PF0-Pf4
    GPIO_PORTF_DEN_R |= 0x1F;								// Enable digital I/O on PF0-Pf4
    GPIO_PORTF_PCTL_R &=0x0;								// GPIO mode for PF0-Pf4
    GPIO_PORTF_PUR_R |= 0x11;								//Enable Pull Up Resistance for PF0, PF4
    GPIO_PORTF_DIR_R |= 0x0E;								// PF1-Pf3 as outputs;  PF0, PF4 as inputs; 
}

/*
void GPIO_initPortE_LED(){
    SET_BIT(SYSCTL_RCGCGPIO_R, 4); // Enable Port E clock
    while(GET_BIT(SYSCTL_PRGPIO_R, 4) == 0); // Wait for clock
    GPIO_PORTE_CR_R |= 0x0E;         // Allow changes to PE0-PE7
    GPIO_PORTE_AMSEL_R &= ~0x0E;     // Disable analog on PE0-PE7
    GPIO_PORTE_PCTL_R = 0;           // GPIO mode for PE0-PE7
    GPIO_PORTE_DIR_R |= 0x0E;        // PE0-PE7 as outputs
    GPIO_PORTE_AFSEL_R &= ~0x0E;     // Disable alt functions on PE0-PE7
    GPIO_PORTE_DEN_R |= 0x0E;        // Enable digital I/O on PE0-PE7
}
*/

unsigned char GPIO_getSwitchesValue(unsigned char sw){
    switch (sw){
        case GPIO_SW1 : return GET_BIT(GPIO_PORTF_DATA_R,4); break;
        case GPIO_SW2 : return GET_BIT(GPIO_PORTF_DATA_R,0); break;

        default : return 0;
    }
}

void GPIO_setLedValue(unsigned char ledColor, unsigned char ledState){ 
    // two parameters one to select led second to on/off
    switch (ledColor){
        case GPIO_RED_LED : switch(ledState)
        {
            case GPIO_LED_OFF: CLR_BIT(GPIO_PORTF_DATA_R,1); break;
            case GPIO_LED_ON : SET_BIT(GPIO_PORTF_DATA_R,1); break;
            default : break;
        } 
        break;

        case GPIO_BLUE_LED : switch(ledState)
        {
            case GPIO_LED_OFF: CLR_BIT(GPIO_PORTF_DATA_R,2); break;
            case GPIO_LED_ON : SET_BIT(GPIO_PORTF_DATA_R,2); break;
            default : break;
        }
        break;

        case GPIO_GREEN_LED : switch(ledState)
        {
            case GPIO_LED_OFF: CLR_BIT(GPIO_PORTF_DATA_R,3); break;
            case GPIO_LED_ON : SET_BIT(GPIO_PORTF_DATA_R,3); break;
            default : break;
        }
        break;

        default : break;
    }
}

void buzzerOn(void) {
    GPIO_PORTE_DATA_R |= 0x02;
}

void buzzerOff(void) {
    GPIO_PORTE_DATA_R &= ~0x02;
}
/*
void GPIO_PORTB_setPort(unsigned char sevenSegmentValues){

    GPIO_PORTB_DATA_R = sevenSegmentValues;

}
*/
void GPIO_printSevenSegment(unsigned char data) // data = 8 bit hexadecimal data, this prints a character on 7segmentdisplay 
{
    if(GET_BIT(data,0)) SET_BIT(GPIO_PORTA_DATA_R,2); else CLR_BIT(GPIO_PORTA_DATA_R,2); 	//A		PA2
    if(GET_BIT(data,1)) SET_BIT(GPIO_PORTA_DATA_R,3); else CLR_BIT(GPIO_PORTA_DATA_R,3);	//B		PA3
    if(GET_BIT(data,2)) SET_BIT(GPIO_PORTA_DATA_R,4); else CLR_BIT(GPIO_PORTA_DATA_R,4);	//C		PA4
    if(GET_BIT(data,3)) SET_BIT(GPIO_PORTB_DATA_R,6); else CLR_BIT(GPIO_PORTB_DATA_R,6);	//D		PB6
    if(GET_BIT(data,4)) SET_BIT(GPIO_PORTB_DATA_R,7); else CLR_BIT(GPIO_PORTB_DATA_R,7);	//E		PB7
    if(GET_BIT(data,5)) SET_BIT(GPIO_PORTD_DATA_R,6); else CLR_BIT(GPIO_PORTD_DATA_R,6);	//F		PD6
    if(GET_BIT(data,6)) SET_BIT(GPIO_PORTD_DATA_R,7); else CLR_BIT(GPIO_PORTD_DATA_R,7);	//G		PD7
}








////
/*#include "GPIO.h"
#include "Bit_Utilies.h"
#include "TM4C123.h"

void GPIO_initPortB(){
    SET_BIT(SYSCTL_RCGCGPIO_R,1); // BIT 1 --> PORT B
    while(GET_BIT(SYSCTL_PRGPIO_R,1)== 0); // Check that Clock is ready
    GPIO_PORTB_CR_R |= 0xFF; // allow changes to PB7-PB0
    GPIO_PORTB_AMSEL_R &= ~0x0F; // Disable Analog
    GPIO_PORTB_PCTL_R &= ~0x0F; // PCTL GPIO on PB7-PB0
    GPIO_PORTB_DIR_R |= 0xFF; // PF4 PF0 --> in, PB7-PB0--> out
    GPIO_PORTB_AFSEL_R &= ~0x00F0; // Disable alt funct on PB7-PB0
    GPIO_PORTB_DEN_R |= 0xFF; // Enable digital I/O on PF4-0
}

void GPIO_initPortC(){
    SET_BIT(SYSCTL_RCGCGPIO_R, 2); // Bit 2 --> Port C
    while(GET_BIT(SYSCTL_PRGPIO_R, 2) == 0); // Wait until Port C is ready

    GPIO_PORTC_CR_R |= 0xFF;        // Allow changes to PC7-PC0
    GPIO_PORTC_AMSEL_R &= ~0xFF;    // Disable analog functionality on PC7-PC0
    GPIO_PORTC_PCTL_R &= ~0xFFFFFFFF; // Clear PCTL register to select GPIO on PC7-PC0
    GPIO_PORTC_DIR_R |= 0xFF;       // Set PC7-PC0 as output
    GPIO_PORTC_AFSEL_R &= ~0xFF;    // Disable alternate functions on PC7-PC0
    GPIO_PORTC_DEN_R |= 0xFF;       // Enable digital functionality on PC7-PC0
}


void GPIO_initPortF(void){
    SET_BIT(SYSCTL_RCGCGPIO_R,5);
    while(GET_BIT(SYSCTL_PRGPIO_R,5)== 0);
    GPIO_PORTF_LOCK_R= GPIO_LOCK_KEY;
    GPIO_PORTF_CR_R |= 0x1F;
    GPIO_PORTB_AFSEL_R &=0x00;
    GPIO_PORTF_AMSEL_R &=0x00;
    GPIO_PORTF_DEN_R |= 0x1F;
    GPIO_PORTF_PCTL_R &=0x0;
    GPIO_PORTF_PUR_R |= 0x11;
    GPIO_PORTF_DIR_R |= 0x0E;
}

unsigned char GPIO_getSwitchesValue(unsigned char sw){
    switch (sw){
        case GPIO_SW1 : return GET_BIT(GPIO_PORTF_DATA_R,4); break;
        case GPIO_SW2 : return GET_BIT(GPIO_PORTF_DATA_R,0); break;

        default : return 0;
    }
}

void GPIO_setLedValue(unsigned char ledColor, unsigned char ledState){ 
    // two parameters one to select led second to on/off
    switch (ledColor){
        case GPIO_RED_LED : switch(ledState)
        {
            case GPIO_LED_OFF: CLR_BIT(GPIO_PORTF_DATA_R,1); break;
            case GPIO_LED_ON : SET_BIT(GPIO_PORTF_DATA_R,1); break;
            default : break;
        } 
        break;

        case GPIO_BLUE_LED : switch(ledState)
        {
            case GPIO_LED_OFF: CLR_BIT(GPIO_PORTF_DATA_R,2); break;
            case GPIO_LED_ON : SET_BIT(GPIO_PORTF_DATA_R,2); break;
            default : break;
        }
        break;

        case GPIO_GREEN_LED : switch(ledState)
        {
            case GPIO_LED_OFF: CLR_BIT(GPIO_PORTF_DATA_R,3); break;
            case GPIO_LED_ON : SET_BIT(GPIO_PORTF_DATA_R,3); break;
            default : break;
        }
        break;

        default : break;
    }
}


void GPIO_PORTB_setPort(unsigned char sevenSegmentValues){

    GPIO_PORTB_DATA_R = sevenSegmentValues;

}
*/







////// General Intiallize function that intiallizes all pins (0xFF) in a port DON'T USE IT!
/*
void GPIO_initPortA(){
    SET_BIT(SYSCTL_RCGCGPIO_R, 0); // Enable Port A clock
    while(GET_BIT(SYSCTL_PRGPIO_R, 0) == 0); // Wait for clock
    GPIO_PORTA_CR_R |= 0xFF;         // Allow changes to PA0-PA7
    GPIO_PORTA_AMSEL_R &= ~0xFF;     // Disable analog on PA0-PA7
    GPIO_PORTA_PCTL_R = 0;           // GPIO mode for PA0-PA7
    GPIO_PORTA_DIR_R |= 0xFF;        // PA0-PA7 as outputs
    GPIO_PORTA_AFSEL_R &= ~0xFF;     // Disable alt functions on PA0-PA7
    GPIO_PORTA_DEN_R |= 0xFF;        // Enable digital I/O on PA0-PA7
    // Note: PA0-PA1 likely used for UART0 (RX/TX). Setting as outputs will break UART.
}

void GPIO_initPortB(){
    SET_BIT(SYSCTL_RCGCGPIO_R, 1); // Enable Port B clock
    while(GET_BIT(SYSCTL_PRGPIO_R, 1) == 0); // Wait for clock
    GPIO_PORTB_CR_R |= 0xFF;         // Allow changes to PB0-PB7
    GPIO_PORTB_AMSEL_R &= ~0xFF;     // Disable analog on PB0-PB7
    GPIO_PORTB_PCTL_R = 0;           // GPIO mode for PB0-PB7
    GPIO_PORTB_DIR_R |= 0xFF;        // PB0-PB7 as outputs
    GPIO_PORTB_AFSEL_R &= ~0xFF;     // Disable alt functions on PB0-PB7
    GPIO_PORTB_DEN_R |= 0xFF;        // Enable digital I/O on PB0-PB7
}

void GPIO_initPortC(){
    SET_BIT(SYSCTL_RCGCGPIO_R, 2); // Enable Port C clock
    while(GET_BIT(SYSCTL_PRGPIO_R, 2) == 0); // Wait for clock
    GPIO_PORTC_LOCK_R = 0x4C4F434B;  // Unlock Port C (for JTAG pins PC0-PC3)
    GPIO_PORTC_CR_R |= 0xFF;         // Allow changes to PC0-PC7
    GPIO_PORTC_AMSEL_R &= ~0xFF;     // Disable analog on PC0-PC7
    GPIO_PORTC_PCTL_R = 0;           // GPIO mode for PC0-PC7
    GPIO_PORTC_DIR_R |= 0xFF;        // PC0-PC7 as outputs
    GPIO_PORTC_AFSEL_R &= ~0xFF;     // Disable alt functions on PC0-PC7
    GPIO_PORTC_DEN_R |= 0xFF;        // Enable digital I/O on PC0-PC7
    // Note: PC0-PC3 are JTAG/SWD pins by default. This disables debugging.
}

void GPIO_initPortD(){
    SET_BIT(SYSCTL_RCGCGPIO_R, 3); // Enable Port D clock
    while(GET_BIT(SYSCTL_PRGPIO_R, 3) == 0); // Wait for clock
    GPIO_PORTD_LOCK_R = 0x4C4F434B;  // Unlock Port D (for PD7 NMI)
    GPIO_PORTD_CR_R |= 0xFF;         // Allow changes to PD0-PD7
    GPIO_PORTD_AMSEL_R &= ~0xFF;     // Disable analog on PD0-PD7
    GPIO_PORTD_PCTL_R = 0;           // GPIO mode for PD0-PD7
    GPIO_PORTD_DIR_R |= 0xFF;        // PD0-PD7 as outputs
    GPIO_PORTD_AFSEL_R &= ~0xFF;     // Disable alt functions on PD0-PD7
    GPIO_PORTD_DEN_R |= 0xFF;        // Enable digital I/O on PD0-PD7
}

void GPIO_initPortE(){
    SET_BIT(SYSCTL_RCGCGPIO_R, 4); // Enable Port E clock
    while(GET_BIT(SYSCTL_PRGPIO_R, 4) == 0); // Wait for clock
    GPIO_PORTE_CR_R |= 0xFF;         // Allow changes to PE0-PE7
    GPIO_PORTE_AMSEL_R &= ~0xFF;     // Disable analog on PE0-PE7
    GPIO_PORTE_PCTL_R = 0;           // GPIO mode for PE0-PE7
    GPIO_PORTE_DIR_R |= 0xFF;        // PE0-PE7 as outputs
    GPIO_PORTE_AFSEL_R &= ~0xFF;     // Disable alt functions on PE0-PE7
    GPIO_PORTE_DEN_R |= 0xFF;        // Enable digital I/O on PE0-PE7
}
*/