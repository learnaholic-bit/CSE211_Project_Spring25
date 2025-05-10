#include "tm4c123gh6pm.h"
#include "Bit_Utilies.h"
#include "stdint.h"
#include <stdio.h>
/**
 * @brief Initializes UART0 with a baud rate of 9600, 8-bit data, no parity,
 *        1 stop bit, and enables both TX and RX.
 * 
 * This function performs the following steps:
 * - Enables the clock for UART0 and Port A.
 * - Waits for Port A to be ready.
 * - Configures the integer and fractional parts of the baud rate divisor.
 * - Sets up the line control for 8-bit data and enables the FIFO.
 * - Enables UART0 along with its transmit and receive functionalities.
 * - Configures PA0 and PA1 for alternative functions and digital enable.
 * - Disables the analog functionality on PA0 and PA1.
 */
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

/**
 * @brief Transmits a single character via UART0.
 *
 * This function waits until the UART0 transmit FIFO is not full, 
 * then writes the character to the data register to send it.
 *
 * @param c The character to be transmitted.
 */
void UART0_SendChar(char c) {
    while (UART0_FR_R & (1 << 5));    // Wait while TX FIFO is full
    UART0_DR_R = c;
}

/**
 * @brief Sends a string of characters via UART0.
 *
 * This function takes a C-style string (i.e. a null-terminated string)
 * and sends each character in order via UART0. It is a convenience
 * function that iteratively calls UART0_SendChar for each character in
 * the string.
 *
 * @param str The string to be sent.
 */
void UART0_SendString(const char *str) {
    while (*str) {
        UART0_SendChar(*str++);
    }
}

/**
 * @brief Receives a single character from UART0.
 *
 * This function continuously checks if the UART0 receive FIFO is not empty.
 * Once a character is available, it reads and returns the character from 
 * the data register.
 *
 * @return The character received via UART0.
 */

char UART0_GetChar(void) {
    while (UART0_FR_R & (1 << 4));    // Wait while RX FIFO is empty
    return (char)(UART0_DR_R & 0xFF);
}

/**
 * @brief Sends a floating-point number via UART0.
 *
 * This function formats the given float to two decimal places and
 * sends it as a string using UART0. It acts as a convenience method
 * by converting the float into a string representation and transmitting
 * it over the UART communication.
 *
 * @param value The floating-point number to be sent.
 */

void UART0_SendFloat(float value) {
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "%.2f", value);  // Format to 2 decimal places
    UART0_SendString(buffer);
}


/**
 * @brief Initializes UART3 with a baud rate of 9600, 8-bit data, no parity,
 *        1 stop bit, and enables both TX and RX.
 *
 * This function performs the following steps:
 * - Enables the clock for UART3 and Port C.
 * - Waits for Port C to be ready.
 * - Configures the integer and fractional parts of the baud rate divisor.
 * - Sets up the line control for 8-bit data and enables the FIFO.
 * - Enables UART3 along with its transmit and receive functionalities.
 * - Configures PC6 and PC7 for alternative functions and digital enable.
 * - Disables the analog functionality on PC6 and PC7.
 */
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

/**
 * @brief Send a single character to the UART3 transmitter.
 *
 * This function is blocking, and will wait until the transmit FIFO is not
 * full before sending the character.
 */
void UART_SendChar(char c) {
    while (UART3_FR_R & (1 << 5));    // Wait while TX FIFO is full
    UART3_DR_R = c;
}

/**
 * @brief Sends a string of characters via UART3.
 *
 * This function takes a C-style string (i.e. a null-terminated string)
 * and sends each character in order via UART3. It is a convenience
 * function that iteratively calls UART_SendChar for each character in
 * the string.
 *
 * @param str The string to be sent.
 */
void UART_SendString(const char *str) {
    while (*str) {
        UART_SendChar(*str++);
    }
}

/**
 * @brief Receives a single character from UART3.
 *
 * This function waits until there is data available in the receive FIFO
 * of UART3, and then reads the character from the data register.
 *
 * @return The received character.
 */
char UART_GetChar(void) {
    while (UART3_FR_R & (1 << 4));    // Wait while RX FIFO is empty
    return (char)(UART3_DR_R & 0xFF);
}



/**
 * @brief Initializes UART2 for communication.
 *
 * This function sets up UART2 on the TM4C123 microcontroller for serial 
 * communication. It enables the clock for UART2 and the associated GPIO port, 
 * configures the baud rate for 9600 bps using a 16MHz system clock, and sets 
 * the line control for 8-bit data, 1 stop bit, no parity, with FIFO enabled. 
 * It also enables the UART, TX, and RX functionalities and configures the 
 * necessary GPIO pins (PD6 as U2RX and PD7 as U2TX) for UART operation.
 */

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

/**
 * @brief Transmits a single character via UART2.
 *
 * This function blocks execution until the UART2 transmit FIFO is 
 * not full, ensuring there's space to send the character. Once space 
 * is available, it writes the character to the data register, 
 * initiating the transmission.
 *
 * @param c The character to be transmitted.
 */
void UART2_SendChar(char c) {
    // Wait until the Transmit FIFO is not full
    while (UART2_FR_R & UART_FR_TXFF); // Check TXFF flag (bit 5)
    UART2_DR_R = c;                    // Write the character to the Data Register
}

/**
 * @brief Sends a string of characters via UART2.
 *
 * This function takes a C-style string (i.e. a null-terminated string)
 * and sends each character in order via UART2. It is a convenience
 * function that iteratively calls UART2_SendChar for each character in
 * the string.
 *
 * @param str The string to be sent.
 */
void UART2_SendString(const char *str) {
    while (*str) {
        UART2_SendChar(*str++);
    }
}

/**
 * @brief Receive a single character via UART2.
 *
 * This function blocks execution until the UART2 receive FIFO is not
 * empty, ensuring there's a character to receive. Once a character is
 * available, it reads the character from the Data Register, returning
 * it to the caller.
 *
 * @return The received character.
 */
char UART2_GetChar(void) {
    // Wait until the Receive FIFO is not empty
    while (UART2_FR_R & UART_FR_RXFE); // Check RXFE flag (bit 4)
    return (char)(UART2_DR_R & 0xFF); // Read the character from the Data Register
}