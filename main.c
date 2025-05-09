#include "tm4c123gh6pm.h" 
#include "Bit_Utilies.h"
#include <string.h>
#include "LCD.h"
#include "UART.h"
#include "GPS.h"
#include "Bit_Utilies.h"
#include "GPIO.h"

int main(void) {

		float distance;
		Landmark Nearest;
		GPIO_initPortE_BUZZER();
		UART_Init();
		UART0_Init();
		UART2_Init();
		lcd_init();
		GPIO_initPortF();
    while (1){
					//UART0_SendChar(UART2_GetChar());
				GPS_read();
				GPS_format();
			
				Nearest = GPS_FindNearestLandmark();
				if (GPIO_getSwitchesValue(GPIO_SW1)==0)
								Display_status ();
				else if (GPIO_getSwitchesValue(GPIO_SW2)==0){
					Display_TIME ();
					Display_DATE ();
					DisplaySpeed();
				}
					else {		
						//	if (strcmp(Nearest.name, "error") == 0){/
									distance = Nearest.distance;
									if (distance<30.0){
												GPIO_setLedValue(GPIO_GREEN_LED,GPIO_LED_ON);
												buzzerOn();
												delay_ms(500);
												GPIO_setLedValue(GPIO_GREEN_LED,GPIO_LED_OFF);
												buzzerOff();
										}
									else if ((30.0<distance)&&(distance<50.0)){
												GPIO_setLedValue(GPIO_BLUE_LED,GPIO_LED_ON);
												delay_ms(500);
												GPIO_setLedValue(GPIO_BLUE_LED,GPIO_LED_OFF);
										}
									else if ((50.0<distance)&&(distance<80.0)){
											GPIO_setLedValue(GPIO_RED_LED,GPIO_LED_ON);
											delay_ms(500);
											GPIO_setLedValue(GPIO_RED_LED,GPIO_LED_OFF);
												}	
									else{
												GPIO_setLedValue(GPIO_RED_LED,GPIO_LED_ON);
												GPIO_setLedValue(GPIO_GREEN_LED,GPIO_LED_ON);
												GPIO_setLedValue(GPIO_BLUE_LED,GPIO_LED_ON);
												delay_ms(500);
												GPIO_setLedValue(GPIO_RED_LED,GPIO_LED_OFF);
												GPIO_setLedValue(GPIO_GREEN_LED,GPIO_LED_OFF);
												GPIO_setLedValue(GPIO_BLUE_LED,GPIO_LED_OFF);
								//	}						
							}
							
												lcd_string((unsigned char*)Nearest.name,0);
												delay_ms(500);
												lcd_clear();
												GPIO_setLedValue(GPIO_RED_LED,GPIO_LED_OFF);
												GPIO_setLedValue(GPIO_GREEN_LED,GPIO_LED_OFF);
												GPIO_setLedValue(GPIO_BLUE_LED,GPIO_LED_OFF);

							
					}
					
					UART2_ESP();

	}

}
/*	//test
	////////////////////
		float latitude = 40.71;
		float longitude = -74.01;
		float currentLong= -114.03;
		float currentLat= 51.09;
		float distance;
		UART0_Init();
		lcd_init();
		GPIO_initPortF();

		distance = GPS_getDistance (longitude , latitude ,currentLong ,currentLat);
		UART0_SendString("distance:");
		UART0_SendFloat(distance);
		lcd_string((unsigned char*)"basm el salib",0);
		delay_ms(2500);
		lcd_clear();

		///////////////////////// */


/*       //LEDs Test 
		GPIO_initPortF();
	
		GPIO_setLedValue(GPIO_RED_LED,GPIO_LED_ON);
		delay_ms(500);
		GPIO_setLedValue(GPIO_RED_LED,GPIO_LED_OFF);
		
		GPIO_setLedValue(GPIO_BLUE_LED,GPIO_LED_ON);
		delay_ms(500);
		GPIO_setLedValue(GPIO_BLUE_LED,GPIO_LED_OFF);
	
		
		GPIO_setLedValue(GPIO_GREEN_LED,GPIO_LED_ON);
		delay_ms(500);
		GPIO_setLedValue(GPIO_GREEN_LED,GPIO_LED_OFF);
	*/


     /////////////
/*
	    
		GPIO_initPortF();
		GPIO_initPortE_BUZZER();
		
		while(1) {
				if (GPIO_getSwitchesValue(GPIO_SW1)==0) {
					GPIO_setLedValue(GPIO_RED_LED,GPIO_LED_ON);
					buzzerOn();
					delay_ms(500);
					buzzerOff();
					GPIO_setLedValue(GPIO_RED_LED,GPIO_LED_OFF);
					
					GPIO_setLedValue(GPIO_BLUE_LED,GPIO_LED_ON);
					buzzerOn();
					delay_ms(500);
					buzzerOff();
					GPIO_setLedValue(GPIO_BLUE_LED,GPIO_LED_OFF);
				
					
					GPIO_setLedValue(GPIO_GREEN_LED,GPIO_LED_ON);
					buzzerOn();
					delay_ms(500);
					buzzerOff();
					GPIO_setLedValue(GPIO_GREEN_LED,GPIO_LED_OFF);
			}
			else if (GPIO_getSwitchesValue(GPIO_SW2)==0){
					GPIO_setLedValue(GPIO_GREEN_LED,GPIO_LED_ON);
					buzzerOn();
					delay_ms(500);
					buzzerOff();
					GPIO_setLedValue(GPIO_GREEN_LED,GPIO_LED_OFF);
					
					GPIO_setLedValue(GPIO_BLUE_LED,GPIO_LED_ON);
					buzzerOn();
					delay_ms(500);
					buzzerOff();
					GPIO_setLedValue(GPIO_BLUE_LED,GPIO_LED_OFF);
					
					GPIO_setLedValue(GPIO_RED_LED,GPIO_LED_ON);
					buzzerOn();
					delay_ms(500);
					buzzerOff();
					GPIO_setLedValue(GPIO_RED_LED,GPIO_LED_OFF);
			}
		}
		*/
		//http://192.168.155.89/