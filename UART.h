// UART_Interface.h

#ifndef UART
#define UART
#include "TM4C123.h"
#include "Bit_Utilies.h"

// Function Prototypes
void UART_Init(void);
char UART_IsCharAvail(void);
char UART_GetChar(void);
void UART_SendChar(char);
// Function Prototypes
void UART0_Init(void);
char UART0_IsCharAvail(void);
char UART0_GetChar(void);
void UART0_SendChar(char);
void UART0_SendString(const char *str);
void UART0_SendFloat(float value);
void UART2_Init();
void UART2_SendChar(char c);
void UART2_SendString(const char *str);
char UART2_GetChar(void);
#endif
