#include "SEVEN_SEG.h"
#include "tm4c123gh6pm.h"
#include "GPIO.h"
#include "Bit_Utilies.h"


	//assume pin allignment of SEVEN_SEG
	//A		PA2
	//B		PA3
	//C		PA4
	//D		PB6
	//E		PB7
	//F		PD6
	//G		PD7
	
	
unsigned seg1, seg2, seg3;
const unsigned char seven_segmentArray[10] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x27, 0x7F, 0x6F};	//0-9
const unsigned char seven_segment_lowercase[26] = {																													//a-z
    0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71, 0x6F, 0x74, 0x30, 0x1E,
    0x76, 0x38, 0x55, 0x54, 0x5C, 0x73, 0x67, 0x50, 0x6D, 0x78,
    0x1C, 0x3E, 0x3D, 0x76, 0x6E, 0x5B
};
const unsigned char seven_segment_uppercase[26] = {																													//A-Z
    0x77, 0x7F, 0x39, 0x3F, 0x79, 0x71, 0x6F, 0x76, 0x30, 0x1E,
    0x76, 0x38, 0x55, 0x54, 0x3F, 0x73, 0x67, 0x50, 0x6D, 0x78,
    0x3E, 0x3E, 0x3D, 0x76, 0x6E, 0x5B
};



void GPIO_initSevenSegment() {
		/*
		GPIO_initPortA();
		GPIO_initPortB();
		GPIO_initPortD();
	*/
    // Enable clocks for Ports A, B, D
    SET_BIT(SYSCTL_RCGCGPIO_R, 0); // Port A
    SET_BIT(SYSCTL_RCGCGPIO_R, 1); // Port B
    SET_BIT(SYSCTL_RCGCGPIO_R, 3); // Port D
    while (!(GET_BIT(SYSCTL_PRGPIO_R, 0) && GET_BIT(SYSCTL_PRGPIO_R, 1) && GET_BIT(SYSCTL_PRGPIO_R, 3)));

    // Port A (PA2-PA4 for A, B, C)
    GPIO_PORTA_CR_R |= 0x1C;      // Allow changes to PA2-PA4
    GPIO_PORTA_AMSEL_R &= ~0x1C;  // Disable analog on PA2-PA4
    GPIO_PORTA_PCTL_R = 0;        // GPIO mode
    GPIO_PORTA_DIR_R |= 0x1C;     // PA2-PA4 as outputs
    GPIO_PORTA_AFSEL_R &= ~0x1C;  // Disable alt functions on PA2-PA4
    GPIO_PORTA_DEN_R |= 0x1C;     // Enable digital on PA2-PA4

    // Port B (PB6-PB7 for D, E)
    GPIO_PORTB_CR_R |= 0xC0;      // Allow changes to PB6-PB7
    GPIO_PORTB_AMSEL_R &= ~0xC0;  // Disable analog on PB6-PB7
    GPIO_PORTB_PCTL_R = 0;        // GPIO mode
    GPIO_PORTB_DIR_R |= 0xC0;     // PB6-PB7 as outputs
    GPIO_PORTB_AFSEL_R &= ~0xC0;  // Disable alt functions on PB6-PB7
    GPIO_PORTB_DEN_R |= 0xC0;     // Enable digital on PB6-PB7

    // Port D (PD6-PD7 for F, G)
    GPIO_PORTD_LOCK_R = 0x4C4F434B; // Unlock Port D (for PD7 NMI)
    GPIO_PORTD_CR_R |= 0xC0;       // Allow changes to PD6-PD7
    GPIO_PORTD_AMSEL_R &= ~0xC0;   // Disable analog on PD6-PD7
    GPIO_PORTD_PCTL_R = 0;         // GPIO mode
    GPIO_PORTD_DIR_R |= 0xC0;      // PD6-PD7 as outputs
    GPIO_PORTD_AFSEL_R &= ~0xC0;   // Disable alt functions on PD6-PD7
    GPIO_PORTD_DEN_R |= 0xC0;      // Enable digital on PD6-PD7
}





void setSevenSegment(unsigned char c){
		unsigned char pattern = 0x00;
		if (c >= '0' && c <= '9') {
            pattern = seven_segmentArray[c - '0'];
        } else if (c >= 'a' && c <= 'z') {
            pattern = seven_segment_lowercase[c - 'a'];
        } else if (c >= 'A' && c <= 'Z') {
            pattern = seven_segment_uppercase[c - 'A'];
        }
    GPIO_printSevenSegment(pattern);
}

void SplitDistance(double distance){
    int d = (int)distance;

    seg1 = d % 10;
    d /= 10;
    seg2 = d % 10;
    d /= 10;
    seg3 = d % 10;
    d /= 10;
}



/*void show(int digit, char value){
    GPIO_PORTC_DATA_R = 0x00;       // Turns off LEDs
    GPIO_PORTC_DATA_R = digit;      // Selects segment to turn on PE1, PE2, PE3
    setSevenSegment(value);
}*/