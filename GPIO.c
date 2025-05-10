#include "GPIO.h"
#include "Bit_Utilies.h"
#include "tm4c123gh6pm.h"
#include "TM4C123.h"



// ************************************//BUZZER//*********************************
/**
 * @brief Initializes Port E GPIO for the buzzer.
 *
 * The buzzer is connected to PE1 (Port E, Pin 1).
 * This function enables the clock for Port E, configures PE1 as a digital output,
 * and sets the appropriate control registers.
 */

void GPIO_initPortE_BUZZER(){
    SET_BIT(SYSCTL_RCGCGPIO_R, 4); // Enable Port E clock
    while(GET_BIT(SYSCTL_PRGPIO_R, 4) == 0); // Wait for clock
    GPIO_PORTE_CR_R |= 0x02;         
    GPIO_PORTE_AMSEL_R &= ~0x02;     
    GPIO_PORTE_PCTL_R = 0;           
    GPIO_PORTE_DIR_R |= 0x02;        
    GPIO_PORTE_AFSEL_R &= ~0x02;     
    GPIO_PORTE_DEN_R |= 0x02;        
}

// ************************************//Switches & LEDs//*********************************
/**  GPIO PORTF TO Switches & LEDs
// @brief Initializes Port F GPIO for Switches and LEDs
//assume pin allignment (same as on board)
//PF0		SW1
//PF1		LED_RED
//PF2		LED_BLUE
//PF3		LED_GREEN
//PF4		SW2
*/
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


/**
 * @brief Get the value of a specific switch on the board
 * 
 * @param sw One of GPIO_SW1, GPIO_SW2
 */

unsigned char GPIO_getSwitchesValue(unsigned char sw){
    switch (sw){
        case GPIO_SW1 : return GET_BIT(GPIO_PORTF_DATA_R,4); break;
        case GPIO_SW2 : return GET_BIT(GPIO_PORTF_DATA_R,0); break;

        default : return 0;
    }
}

/**
 * @brief Set the state of a specific LED on the board
 * 
 * @param ledColor One of GPIO_RED_LED, GPIO_BLUE_LED, GPIO_GREEN_LED
 * @param ledState One of GPIO_LED_OFF, GPIO_LED_ON
 */
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


/**
 * @brief Turns the buzzer on.
 *
 * The buzzer is connected to PE1 (Port E, Pin 1)
 */
void buzzerOn(void) {
    GPIO_PORTE_DATA_R |= 0x02;
}


/**
 * @brief Turns the buzzer off.
 *
 * The buzzer is connected to PE1 (Port E, Pin 1)
 */
void buzzerOff(void) {
    GPIO_PORTE_DATA_R &= ~0x02;
}


/**
 * @brief Prints a character on a 7 segment display using GPIO.
 *
 * @param[in] data 8 bit hexadecimal data to be printed on the display.
 *
 * @details This function takes an 8 bit hexadecimal data as argument and prints the corresponding character on a 7 segment display.
 * The 7 segment display is connected to Port A, B and D as follows:
 * A		PA2
 * B		PA3
 * C		PA4
 * D		PB6
 * E		PB7
 * F		PD6
 * G		PD7
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
