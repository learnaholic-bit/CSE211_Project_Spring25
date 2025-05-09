#include "tm4c123gh6pm.h"
#include "Bit_Utilies.h"
#include "stdint.h"
#include <stdio.h>
void UART0_Init(void) {
    SYSCTL_RCGCUART_R |= (1 << 0);    // Enable UART0
    SYSCTL_RCGCGPIO_R |= (1 << 0);    // Enable Port A
    while ((SYSCTL_PRGPIO_R & 0x01) == 0); // Wait until Port A is ready

    UART0_CTL_R &= ~(1 << 0);         // Disable UART

    UART0_IBRD_R = 104;               // Integer = 104
    UART0_FBRD_R = 11;                // Fraction = 11

    UART0_LCRH_R = (0x3 << 5) | (1 << 4); // 8-bit, enable FIFO
    UART0_CTL_R |= (1 << 0) | (1 << 8) | (1 << 9); // Enable UART, TX, RX

    GPIO_PORTA_AFSEL_R |= 0x03;       // PA0, PA1 alt functions
    GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R & ~0xFF) | 0x11;
    GPIO_PORTA_DEN_R |= 0x03;         // Digital enable
    GPIO_PORTA_AMSEL_R &= ~0x03;      // Disable analog
}

void UART0_SendChar(char c) {
    while (UART0_FR_R & (1 << 5));    // Wait while TX FIFO is full
    UART0_DR_R = c;
}

void UART0_SendString(const char *str) {
    while (*str) {
        UART0_SendChar(*str++);
    }
}

char UART0_GetChar(void) {
    while (UART0_FR_R & (1 << 4));    // Wait while RX FIFO is empty
    return (char)(UART0_DR_R & 0xFF);
}

void UART0_SendFloat(float value) {
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "%.2f", value);  // Format to 2 decimal places
    UART0_SendString(buffer);
}


void UART_Init(void) {
    SYSCTL_RCGCUART_R |= (1 << 3);    // Enable UART3
    SYSCTL_RCGCGPIO_R |= (1 << 2);    // Enable Port C (UART3 on PC6, PC7)
    while ((SYSCTL_PRGPIO_R & (1 << 2)) == 0); // Wait until Port C is ready

    UART3_CTL_R &= ~(1 << 0);         // Disable UART3

    UART3_IBRD_R = 104;               // Integer part of BRD
    UART3_FBRD_R = 11;                // Fractional part of BRD

    UART3_LCRH_R = (0x3 << 5) | (1 << 4); // 8-bit, enable FIFO
    UART3_CTL_R |= (1 << 0) | (1 << 8) | (1 << 9); // Enable UART3, TX, RX

    GPIO_PORTC_AFSEL_R |= (1 << 6) | (1 << 7);   // Enable alt functions on PC6, PC7
    GPIO_PORTC_PCTL_R = (GPIO_PORTC_PCTL_R & ~0xFF000000) | 0x11000000;
    GPIO_PORTC_DEN_R |= (1 << 6) | (1 << 7);     // Digital enable
    GPIO_PORTC_AMSEL_R &= ~((1 << 6) | (1 << 7));// Disable analog
}

void UART_SendChar(char c) {
    while (UART3_FR_R & (1 << 5));    // Wait while TX FIFO is full
    UART3_DR_R = c;
}

void UART_SendString(const char *str) {
    while (*str) {
        UART_SendChar(*str++);
    }
}

char UART_GetChar(void) {
    while (UART3_FR_R & (1 << 4));    // Wait while RX FIFO is empty
    return (char)(UART3_DR_R & 0xFF);
}


// Function to initialize UART4
#include "tm4c123gh6pm.h" // Include the header file provided

void UART2_Init(void) {
    // 1. Clock Gating
    SYSCTL_RCGCUART_R |= (1 << 2);  // Enable UART2 clock
    SYSCTL_RCGCGPIO_R |= (1 << 3);  // Enable Port D clock
    while ((SYSCTL_PRGPIO_R & (1 << 3)) == 0); // Wait until Port D is ready

    // --- UART2 Configuration ---
    UART2_CTL_R &= ~(1 << 0);      // Disable UART2 during configuration

    // Configure Baud Rate (assuming 16MHz system clock for 9600 baud)
    // BRD = 16,000,000 / (16 * 9600) = 104.16667
    // UARTFBRD[DIVFRAC] = integer(0.16667 * 64 + 0.5) = 11
    UART2_IBRD_R = 104;           // Integer part of Baud Rate Divisor
    UART2_FBRD_R = 11;            // Fractional part of Baud Rate Divisor

    // Configure Line Control (8-bit data, 1 stop bit, no parity, FIFO enabled)
    UART2_LCRH_R = (UART_LCRH_WLEN_8 | UART_LCRH_FEN); // 8-bit, enable FIFO

    // Configure UART Control (Enable UART, TX, RX)
    // Note: Ensure UARTEN (bit 0), TXE (bit 8), RXE (bit 9) are set
    UART2_CTL_R |= (UART_CTL_UARTEN | UART_CTL_TXE | UART_CTL_RXE); // Enable UART, TX, RX

    // --- GPIO Port D Configuration for UART2 (PD6=U2RX, PD7=U2TX) ---

    // PD7 might be locked, need to unlock it first
    GPIO_PORTD_LOCK_R = GPIO_LOCK_KEY; // Unlock Port D commit register
    GPIO_PORTD_CR_R |= (1 << 7);       // Allow changes to PD7
    // GPIO_PORTD_LOCK_R = 0;        // Optional: Relock commit register (usually not necessary)

    // Configure PD6 and PD7 as UART pins
    GPIO_PORTD_AFSEL_R |= (1 << 6) | (1 << 7); // Enable alternate function for PD6, PD7

    // Configure the PCTL register for PD6 and PD7 to select UART function (U2RX, U2TX)
    // U2RX (PD6) and U2TX (PD7) typically use PCTL value 1
    GPIO_PORTD_PCTL_R = (GPIO_PORTD_PCTL_R & ~0xFF000000) | (GPIO_PCTL_PD7_U2TX | GPIO_PCTL_PD6_U2RX); // Set PCTL for PD6, PD7 to UART

    GPIO_PORTD_DEN_R |= (1 << 6) | (1 << 7);   // Enable digital function for PD6, PD7
    GPIO_PORTD_AMSEL_R &= ~((1 << 6) | (1 << 7)); // Disable analog function for PD6, PD7
}

void UART2_SendChar(char c) {
    // Wait until the Transmit FIFO is not full
    while (UART2_FR_R & UART_FR_TXFF); // Check TXFF flag (bit 5)
    UART2_DR_R = c;                    // Write the character to the Data Register
}

void UART2_SendString(const char *str) {
    while (*str) {
        UART2_SendChar(*str++);
    }
}

char UART2_GetChar(void) {
    // Wait until the Receive FIFO is not empty
    while (UART2_FR_R & UART_FR_RXFE); // Check RXFE flag (bit 4)
    return (char)(UART2_DR_R & 0xFF); // Read the character from the Data Register
}



/*
// Assumes a 80 MHz bus clock, creates 115200 baud rate
void UART_Init(void) { // should be called only once
    SYSCTL_RCGCUART_R |= 0x00000002; // activate UART1
    SYSCTL_RCGCGPIO_R |= 0x00000004; // activate port C
		while (GET_BIT (SYSCTL_RCGCUART_R, 1)==0);	
		while (GET_BIT (SYSCTL_PRGPIO_R, 4)==0);
    UART1_CTL_R &= ~0x00000001; // disable UART
    UART1_IBRD_R = 104; // IBRD = int(80,000,000/(16*115,200)) = int(43.40278)
    UART1_FBRD_R = 11; // FBRD = round(0.40278 * 64) = 26
    UART1_LCRH_R = 0x00000070; // 8 bit, no parity bits, one stop, FIFOs
    UART1_CTL_R |= 0x00000001; // enable UART

    GPIO_PORTC_AFSEL_R |= 0x30; // enable alt funct on PC5-4
    GPIO_PORTC_DEN_R |= 0x30; // configure PC5-4 as UART1
    GPIO_PORTC_PCTL_R = (GPIO_PORTC_PCTL_R & 0xFF0FFFFF) + 0x00220000;
    GPIO_PORTC_AMSEL_R &= ~0x30; // disable analog on PC5-4
}

// Wait for new input,
// then return ASCII code
char UART_GetChar(void) {
    while ((UART1_FR_R & 0x00010) != 0); // wait until RXFE is 0
    return (char)(UART1_DR_R & 0xFF);
}

// Wait for buffer to be not full,
// then output
void UART_SendChar(char data) {
    while ((UART1_FR_R & 0x00020) != 0); // wait until TXFF is 0
    UART1_DR_R = data;
}

// Assumes a 80 MHz bus clock, creates 115200 baud rate
void UART0_Init(void) { // should be called only once
    SYSCTL_RCGCUART_R |= 0x00000001; // activate UART1
    SYSCTL_RCGCGPIO_R |= 0x00000001; // activate port C
		
		while (GET_BIT (SYSCTL_RCGCUART_R, 0)==0);	
		while (GET_BIT (SYSCTL_PRGPIO_R, 0)==0);
	
    UART0_CTL_R &= ~0x00000001; // disable UART
    UART0_IBRD_R = 104; // IBRD = int(80,000,000/(16*115,200)) = int(43.40278)
    UART0_FBRD_R = 11; // FBRD = round(0.40278 * 64) = 26
    UART0_LCRH_R = 0x00000070; // 8 bit, no parity bits, one stop, FIFOs
    UART0_CTL_R |= 0x00000001; // enable UART

    GPIO_PORTA_AFSEL_R |= 0x30; // enable alt funct on PC5-4
    GPIO_PORTA_DEN_R |= 0x30; // configure PC5-4 as UART1
    GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R & 0xFF00FFFF) + 0x00220000;
    GPIO_PORTA_AMSEL_R &= ~0x30; // disable analog on PC5-4
}

// Wait for new input,
// then return ASCII code
char UART0_GetChar(void) {
    while ((UART0_FR_R & 0x00010) != 0); // wait until RXFE is 0
    return (char)(UART0_DR_R & 0xFF);
}

// Wait for buffer to be not full,
// then output


void UART0_SendChar(uint8_t data) {
    while ((UART0_FR_R & 0x00020) != 0); // wait until TXFF is 0
    UART0_DR_R = data;
}


void UART0_SendChar(char c) {
    while (UART0_FR_R & (1 << 5)); // Wait until TXFF is 0 (transmit FIFO not full)
    UART0_DR_R = c;
}

void UART0_SendString(const char *str) {
    while (*str) {
        UART0_SendChar(*str++);
}
}
*/






/*


//UART INIT

void UART_Init(){
SET_BIT (SYSCTL_RCGCUART_R , 4);// CLOCK --> UART3
	while (GET_BIT (SYSCTL_PRUART_R,4)==0);
SET_BIT (SYSCTL_RCGCGPIO_R , 4);
	while (GET_BIT (SYSCTL_PRGPIO_R, 0)==0) ;
// Safety
// SET (GPIO_PORTD_LOCK_R, 0x4C4F434B) ;		// UNLOCK PORTD
// SET (GPIO_PORTD_CR_R , 0xc0 ) ;			// PD6 ,PD7 ONLY affected by AFSEL , PUR , PDR ,DEN


// UART BAUDRATE = 9600
// [ (16x10^6)/ (Prescale=16) ]/ [BAUDRATE = 9600] --> 104.1667
	
CLR(UART0_CTL_R ,UART_CTL_UARTEN);				//DISABLE UART
UART0_IBRD_R = 104;												// 104
UART0_FBRD_R = 11 ;												// (0.1667*64)+0.5
// Write Desired Serial Parameters
SET (UART0_LCRH_R,UART_LCRH_WLEN_8);			// WORDLENGTH = 8
SET (UART0_LCRH_R,UART_LCRH_FEN) ;				// FIFO ENABLE
//Enable UART , TXE ,RXE
	SET(UART0_CTL_R,UART_CTL_UARTEN);				//ENABLE UART
	SET(UART0_CTL_R,UART_CTL_TXE);					//ENABLE TX
	SET(UART0_CTL_R,UART_CTL_RXE);					//ENABLE RX
// GPIO ENANLE
SET(GPIO_PORTD_AFSEL_R, 0xc0);						// Alter func for PD6, Pd7
//SELECT TYPE OF ALTERNATE FUNC
CLR(GPIO_PORTA_PCTL_R,0x000000FF);				//Alter func for PD6 , PD7
SET(GPIO_PORTA_PCTL_R,GPIO_PCTL_PA0_U0RX);//PD6-->RX
SET(GPIO_PORTA_PCTL_R,GPIO_PCTL_PA1_U0TX);//PD7 -->TX
SET(GPIO_PORTA_DEN_R,0x3);								//PD0 , PD1 --> DIGITAL

}

// CHECK IF THERE IS CHAR RECIEVED
char UART_IsCharAvail(){
    return((UART0_FR_R & UART_FR_RXFE) == UART_FR_RXFE);
}

// GET CHAR
char UART_GetChar(){
    while ((UART0_FR_R & UART_FR_RXFE) != 0);
    return (char) GET_REG(UART0_DR_R);  // & 0xff
}

void UART_SendChar(uint16_t data) {
    while (GET_BIT(UART3_FR_R, 5));  // Wait until TX FIFO not full (bit 5)
    UART3_DR_R = data & 0xFF;        // Write lower 8 bits to transmit
}

*/


















/*
#include "uart.h"
#include "tm4c123gh6pm.h" // Include the device-specific register definitions
#include <stdbool.h>      // Include for bool type used in UART_ReadLine
#include "stdint.h"
// --- Configuration ---
// Using UART Module 1
#define GPS_UART_MODULE             UART1_BASE          // Base address for UART1 registers (0x4000D000)
#define GPS_UART_SYSCTL_BIT         SYSCTL_RCGCUART_R1  // System Control clock gating bit for UART1 (Value: 0x00000002)

// Using GPIO Port B for UART1 pins
#define GPS_GPIO_PORT_BASE          GPIO_PORTB_BASE     // Base address for GPIO Port B registers (0x40005000)
#define GPS_GPIO_SYSCTL_BIT         SYSCTL_RCGCGPIO_R1  // System Control clock gating bit for Port B (Value: 0x00000002)
#define GPS_PIN_RX                  0                   // Pin 0 (PB0) is used for UART1 Receive (Connected to GPS TX)
#define GPS_PIN_TX                  1                   // Pin 1 (PB1) is used for UART1 Transmit (Connected to GPS RX)

// GPIO Alternate Function Configuration (PCTL) - Values specific to TM4C123GH6PM Port B, Pins 0/1 for U1RX/U1TX
#define GPS_PCTL_RX_FUNCTION_VAL    1                   // The PCTL value for U1RX on Pin 0
#define GPS_PCTL_TX_FUNCTION_VAL    1                   // The PCTL value for U1TX on Pin 1
#define GPS_PCTL_RX_REGISTER_MASK   GPIO_PCTL_PB0_M     // Mask for Pin 0 PCTL field (0x0000000F)
#define GPS_PCTL_TX_REGISTER_MASK   GPIO_PCTL_PB1_M     // Mask for Pin 1 PCTL field (0x000000F0)
#define GPS_PCTL_RX_REGISTER_SHIFT  0                   // Bit shift for Pin 0 PCTL field (PB0)
#define GPS_PCTL_TX_REGISTER_SHIFT  4                   // Bit shift for Pin 1 PCTL field (PB1)


// Clock and Baud Rate
#define SYSTEM_CLOCK_HZ             16000000            // Assume 16 MHz system clock (default internal PIOSC)
                                                        // IMPORTANT: Change if using PLL or external crystal!
#define BAUD_RATE                   9600                // Standard GPS module baud rate (verify with your module)

// --- End Configuration ---


//----------------------- UART_Init -----------------------
// Initialize UART1 for communication using standard peripheral ready checks.
// Assumes SYSTEM_CLOCK_HZ is correctly defined.
void UART_Init(void) {
    // --- Variable Declarations ---
    float brd;
    uint16_t ibrd;
    uint8_t fbrd;
    volatile uint32_t delay;

    // Baud Rate Calculation
    brd = (float)SYSTEM_CLOCK_HZ / (16 * BAUD_RATE);
    ibrd = (uint16_t)brd;
    fbrd = (uint8_t)(((brd - ibrd) * 64) + 0.5);

    // 1. Enable the UART Module and GPIO Port Clocks
    SYSCTL_RCGCGPIO_R |= GPS_GPIO_SYSCTL_BIT; // Enable GPIO Port B Clock
    while ((SYSCTL_PRGPIO_R & GPS_GPIO_SYSCTL_BIT) == 0) {}; // Wait for GPIO ready
    SYSCTL_RCGCUART_R |= GPS_UART_SYSCTL_BIT; // Enable UART1 Clock

    // Delay to allow UART clock to stabilize (simulator-friendly)
    //for (delay = 0; delay < 100; delay++);
		while ((SYSCTL_PRUART_R & GPS_UART_SYSCTL_BIT) == 0) {}; // Wait for UART ready
    // 2. Configure GPIO Pins (Port B, Pins 0 & 1)
    GPIO_PORTB_AFSEL_R |= (1 << GPS_PIN_RX) | (1 << GPS_PIN_TX);
    GPIO_PORTB_PCTL_R &= ~(GPS_PCTL_RX_REGISTER_MASK | GPS_PCTL_TX_REGISTER_MASK);
    GPIO_PORTB_PCTL_R |= (GPS_PCTL_RX_FUNCTION_VAL << GPS_PCTL_RX_REGISTER_SHIFT) |
                         (GPS_PCTL_TX_FUNCTION_VAL << GPS_PCTL_TX_REGISTER_SHIFT);
    GPIO_PORTB_DEN_R |= (1 << GPS_PIN_RX) | (1 << GPS_PIN_TX);
    GPIO_PORTB_AMSEL_R &= ~((1 << GPS_PIN_RX) | (1 << GPS_PIN_TX));

    // 3. Configure UART Module (UART1)
    UART1_CTL_R &= ~UART_CTL_UARTEN;
    UART1_IBRD_R = ibrd;
    UART1_FBRD_R = fbrd;
    UART1_LCRH_R = (UART_LCRH_WLEN_8 | UART_LCRH_FEN);
    UART1_CC_R = UART_CC_CS_SYSCLK;
    UART1_CTL_R |= (UART_CTL_UARTEN | UART_CTL_RXE | UART_CTL_TXE);
}

//----------------------- UART_ReadChar -----------------------
// Wait for new input from UART1 (RX FIFO not empty), then return ASCII code
char UART_GetChar(void) {
    // Wait until the Receiver FIFO is not empty (UART_FR_RXFE flag is 0)
    while ((UART1_FR_R & UART_FR_RXFE) != 0);
    // Read the 8-bit data from the UART data register
    return (char)(UART1_DR_R & 0xFF);
}

//----------------------- UART_SendChar -----------------------
// Wait for buffer space (TX FIFO not full), then output char to UART1
void UART_SendChar(char data) {
    // Wait until the Transmit FIFO is not full (UART_FR_TXFF flag is 0)
    while ((UART1_FR_R & UART_FR_TXFF) != 0);
    // Write the 8-bit data to the UART data register
    UART1_DR_R = data;
}

//----------------------- UART_SendString -----------------------
// Outputs a null-terminated string to UART1 character by character
void UART_SendString(const char *ptr) {
  while(*ptr != '\0') { // Loop until null terminator is found
    UART_SendChar(*ptr);
    ptr++;
  }
}

//----------------------- UART_ReadLine -----------------------
// Reads a line of text (up to '\n') from UART1 into the provided buffer.
// Handles CR ('\r') characters by ignoring them.
// Stops reading if the buffer becomes full (leaving space for null terminator).
// Returns true if a newline was found before the buffer filled, false otherwise.
// The buffer is always null-terminated.
bool UART_ReadLine(char* buffer, uint32_t buffer_size) {
    // Declarations at top for C89/C90
    uint32_t count = 0;
    char character;

    if (buffer_size == 0) { // Prevent buffer overflow on zero-sized buffer
        return false;
    }

    // Ensure buffer is initially null-terminated
    buffer[0] = '\0';

    while (count < (buffer_size - 1)) { // Ensure space for null terminator '\0'
        character = UART_GetChar(); // Read the next character (waits if none available)

        if (character == '\r') {
            // Ignore carriage return, just continue waiting for the next char (likely '\n')
            continue;
        } else if (character == '\n') {
            // End of line detected
            buffer[count] = '\0'; // Null terminate the string at the current position
            return true; // Line successfully read
        } else if (character >= 32 && character <= 126) { // Store only standard printable ASCII
            buffer[count] = character;
            count++;
        }
        // Optional: Handle backspace or other control characters if needed
        // Other non-printable characters are currently ignored
    }

    // Loop terminated because buffer is full (count reached buffer_size - 1)
    buffer[count] = '\0'; // Null terminate the string with the characters received so far
    return false; // Indicate that the buffer is full, line might be incomplete
}
*/
/*
#include "TM4C123.h"
#include "Bit_Utilies.h"

//UART INIT

void UART_Init(){
SET_BIT (SYSCTL_RCGCUART_R , 0);// CLOCK --> UART2
	while (GET_BIT (SYSCTL_PRUART_R,0)==0);
SET_BIT (SYSCTL_RCGCGPIO_R , 0);
	while (GET_BIT (SYSCTL_PRGPIO_R, 0)==0) ;
// Safety
// SET (GPIO_PORTD_LOCK_R, 0x4C4F434B) ;		// UNLOCK PORTD
// SET (GPIO_PORTD_CR_R , 0xc0 ) ;			// PD6 ,PD7 ONLY affected by AFSEL , PUR , PDR ,DEN


// UART BAUDRATE = 9600
// [ (16x10^6)/ (Prescale=16) ]/ [BAUDRATE = 9600] --> 104.1667
	
CLR(UART0_CTL_R ,UART_CTL_UARTEN);				//DISABLE UART
UART0_IBRD_R = 104;												// 104
UART0_FBRD_R = 11 ;												// (0.1667*64)+0.5
// Write Desired Serial Parameters
SET (UART0_LCRH_R,UART_LCRH_WLEN_8);			// WORDLENGTH = 8
SET (UART0_LCRH_R,UART_LCRH_FEN) ;				// FIFO ENABLE
//Enable UART , TXE ,RXE
	SET(UART0_CTL_R,UART_CTL_UARTEN);				//ENABLE UART
	SET(UART0_CTL_R,UART_CTL_TXE);					//ENABLE TX
	SET(UART0_CTL_R,UART_CTL_RXE);					//ENABLE RX
// GPIO ENANLE
SET(GPIO_PORTD_AFSEL_R, 0xc0);						// Alter func for PD6, Pd7
//SELECT TYPE OF ALTERNATE FUNC
CLR(GPIO_PORTA_PCTL_R,0x000000FF);				//Alter func for PD6 , PD7
SET(GPIO_PORTA_PCTL_R,GPIO_PCTL_PA0_U0RX);//PD6-->RX
SET(GPIO_PORTA_PCTL_R,GPIO_PCTL_PA1_U0TX);//PD7 -->TX
SET(GPIO_PORTA_DEN_R,0x3);								//PD0 , PD1 --> DIGITAL

}

// CHECK IF THERE IS CHAR RECIEVED
char UART_IsCharAvail(){
    return((UART0_FR_R & UART_FR_RXFE) == UART_FR_RXFE);
}

// GET CHAR
char UART_GetChar(){
    while ((UART0_FR_R & UART_FR_RXFE) != 0);
    return (char) GET_REG(UART0_DR_R); // & 0xff
}
void UART_SendChar(char data) {
    while ((UART1_FR_R & 0x00020) != 0); // wait until TXFF is 0
    UART1_DR_R = data;
}
*/






