#include "tm4c123gh6pm.h"
#include "Bit_Utilies.h"
#include <string.h>
#include "LCD.h"
#include "UART.h"
#include "GPS.h"
#include "Bit_Utilies.h"
#include "GPIO.h"

/**
 * @brief Main function for GPS-based navigation and alert system.
 *
 * This function initializes various hardware components including GPIO, UART, and LCD.
 * It continuously reads GPS data, processes it to find the nearest landmark, and displays
 * status information or time, date, and speed based on user input from switches.
 * It also provides visual and auditory alerts using LEDs and a buzzer based on the distance
 * to the nearest landmark.
 */
int main(void)
{

	float distance;
	Landmark Nearest;
	GPIO_initPortE_BUZZER();
	UART_Init();
	UART0_Init();
	UART2_Init();
	lcd_init();
	GPIO_initPortF();
	while (1)
	{
		GPS_read();
		GPS_format();

		Nearest = GPS_FindNearestLandmark();
		if (GPIO_getSwitchesValue(GPIO_SW1) == 0)
			Display_status();
		else if (GPIO_getSwitchesValue(GPIO_SW2) == 0)
		{
			Display_TIME();
			Display_DATE();
			DisplaySpeed();
		}
		else
		{
			distance = Nearest.distance;
			if (distance < 30.0)
			{
				GPIO_setLedValue(GPIO_GREEN_LED, GPIO_LED_ON);
				buzzerOn();
				delay_ms(500);
				GPIO_setLedValue(GPIO_GREEN_LED, GPIO_LED_OFF);
				buzzerOff();
			}
			else if ((30.0 < distance) && (distance < 50.0))
			{
				GPIO_setLedValue(GPIO_BLUE_LED, GPIO_LED_ON);
				delay_ms(500);
				GPIO_setLedValue(GPIO_BLUE_LED, GPIO_LED_OFF);
			}
			else if ((50.0 < distance) && (distance < 80.0))
			{
				GPIO_setLedValue(GPIO_RED_LED, GPIO_LED_ON);
				delay_ms(500);
				GPIO_setLedValue(GPIO_RED_LED, GPIO_LED_OFF);
			}
			else
			{
				GPIO_setLedValue(GPIO_RED_LED, GPIO_LED_ON);
				GPIO_setLedValue(GPIO_GREEN_LED, GPIO_LED_ON);
				GPIO_setLedValue(GPIO_BLUE_LED, GPIO_LED_ON);
				delay_ms(500);
				GPIO_setLedValue(GPIO_RED_LED, GPIO_LED_OFF);
				GPIO_setLedValue(GPIO_GREEN_LED, GPIO_LED_OFF);
				GPIO_setLedValue(GPIO_BLUE_LED, GPIO_LED_OFF);
			}

			lcd_string((unsigned char *)Nearest.name, 0);
			delay_ms(500);
			lcd_clear();
			GPIO_setLedValue(GPIO_RED_LED, GPIO_LED_OFF);
			GPIO_setLedValue(GPIO_GREEN_LED, GPIO_LED_OFF);
			GPIO_setLedValue(GPIO_BLUE_LED, GPIO_LED_OFF);
		}

		UART2_ESP();
	}
}
// ESP8266 Web Server
// http://192.168.155.89/