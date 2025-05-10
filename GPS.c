#include "GPS.h"
#include "Bit_Utilies.h"
#include "UART.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "LCD.h"
#include "GPIO.h"
#include <stdio.h>
/**
 * @file GPS.c
 * @brief This file contains variables and constants used for GPS data processing.
 *
 * @details
 * - `current_longitude` and `current_latitude`: Strings to store the current longitude and latitude in text format.
 * - `currentLong` and `currentLat`: Doubles to store the current longitude and latitude in numeric format.
 * - `EARTH_RADIUS`: A constant representing the Earth's radius in meters, used for distance calculations.
 * - `GPS`: A character array to store raw GPS data.
 * - `GPS_logName`: A string containing the prefix for GPS log data, specifically "$GPRMC,".
 * - `GPS_formated`: A 2D character array to store formatted GPS data split into fields.
 * - `token`: A pointer used for tokenizing GPS data strings.
 * - `nearest`: An integer to store the index of the nearest location.
 * - `CurrentNearestName`: A string to store the name of the nearest location.
 * - `CurrentLatitude` and `CurrentLongitude`: Floats to store the current latitude and longitude in numeric format.
 * - `min_dist`: A float to store the minimum distance to a location.
 */
// double currentLong ,currentLat;
char current_longitude[16];
char current_latitude[16];
double currentLong, currentLat;
const double EARTH_RADIUS = 6371000;
char GPS[80];
char GPS_logName[] = "$GPRMC,";
char GPS_formated[12][20];
char *token;
int nearest;
char CurrentNearestName[20];
float CurrentLatitude;
float CurrentLongitude;
float min_dist;

// LCD
char buffer[16];
float s;
//
static Landmark landmarks[6];
static char var[20];

void GPS_InitLandmarks()
{
	strcpy(landmarks[0].name, "CREDIT"); // CREDIT
	landmarks[0].latitude = 3003.82247;	 // 3003.8;//9019/300.0;//3005.22;//3003.48////30.089444444;
	landmarks[0].longitude = 3116.70711; // 3116.7;//18767/600.0;//3118.53;//3116.42////31.314722;

	strcpy(landmarks[1].name, "STADIUM");
	landmarks[1].latitude = 3003.84237;	 // 3003.833333333;//10823/360.0;//3003.50;
	landmarks[1].longitude = 3116.75010; // 3116.7333333;//28151/900.0;//3116.44;

	strcpy(landmarks[2].name, "LIBRARY");
	landmarks[2].latitude = 3003.91033;	 // 3003.9;//6013/200.0;//3003.54;
	landmarks[2].longitude = 3116.80179; // 3116.8;//782/25.0;//3116.48;

	strcpy(landmarks[3].name, "HALL_A");
	landmarks[3].latitude = 3003.86148;	 // 3003.86159;//3003.85919;//36077/1200.0;//3003.51;
	landmarks[3].longitude = 3116.81583; // 03116.80921;//03116.80865;//782/25.0;//3116.48;

	strcpy(landmarks[4].name, "HALL_C");
	landmarks[4].latitude = 3003.82510;	 // 3003.81693;//9019/300.0;//3003.49;3003.79089;
	landmarks[4].longitude = 3116.83927; // 03116.84672;//3116.50;//03116.84328;

	strcpy(landmarks[5].name, "error");
	landmarks[5].latitude = 5;
	landmarks[5].longitude = 0;
}
/**
 * @brief Find the nearest landmark to the current location.
 *
 * @details This function iterates through the list of landmarks and calculates the distance
 *          to each one using the GPS_getDistance function. The landmark with the smallest
 *          distance is returned.
 *
 * @return The nearest landmark to the current location.
 */
Landmark GPS_FindNearestLandmark()
{
	int i;
	min_dist = 10000;
	nearest = 5;

	GPS_InitLandmarks();
	for (i = 0; i < 5; i++)
	{
		double dist = GPS_getDistance(currentLong, currentLat, landmarks[i].longitude, landmarks[i].latitude);
		if (dist < min_dist)
		{
			min_dist = dist;
			nearest = i;
		}
	}
	landmarks[nearest].distance = GPS_getDistance(currentLong, currentLat, landmarks[nearest].longitude, landmarks[nearest].latitude);
	return landmarks[nearest];
}
/**
 * @brief Reads GPS data from the UART interface.
 *
 * @details This function reads characters from the UART interface and checks if the
 *          received data matches the specified GPS log prefix defined in `GPS_logName`.
 *          Once the correct log is received, it initializes the `GPS` array and continues
 *          to read and store characters into the `GPS` array until the '*' character is
 *          encountered, indicating the end of the log.
 */
void GPS_read()
{
	char recievedChar;
	char fillGPScounter = 0;
	char flag = 1;
	char i;
	do
	{
		flag = 1;
		for (i = 0; i < 7; i++)
		{ 
			if (UART_GetChar() != GPS_logName[i])
			{ 
				flag = 0;
				break;
			}
		}
	} while (flag == 0);
	// Here I make sure that I recieved the correct log
	strcpy(GPS, ""); // Init GPS Array
	do
	{
		recievedChar = UART_GetChar();
		GPS[fillGPScounter++] = recievedChar;
	} while (recievedChar != '*');

}

/**
 * @brief Converts an angle in the format of degrees and minutes to decimal degrees.
 *
 * @param angle The angle to be converted, where the integer part represents degrees
 *              and the decimal part represents minutes.
 * @return The angle converted to decimal degrees.
 */
float ToDegree(float angle)
{
	int degree = (int)(angle / 100);
	float minutes = angle - (float)degree * 100;
	return (degree + (minutes / 60));
}

/**
 * @brief Converts an angle in the format of degrees to radians.
 *
 * @param angle The angle to be converted in degrees.
 * @return The angle converted to radians.
 */
float ToRad(float angle)
{ 
	return angle * PI / 180.0;
}

/**
 * @brief Calculates the great-circle distance between two points on the Earth's surface.
 *
 * This function uses the Haversine formula to compute the distance between
 * two geographic coordinates specified in degrees of latitude and longitude.
 *
 * @param current_Long The longitude of the current location in degrees.
 * @param current_Lat The latitude of the current location in degrees.
 * @param previous_Long The longitude of the previous location in degrees.
 * @param previous_Lat The latitude of the previous location in degrees.
 * @return The distance between the current and previous locations in meters.
 */
double GPS_getDistance(double current_Long, double current_Lat, double previous_Long, double previous_Lat)
{
	double prev_lat_rad = ToRad(ToDegree(previous_Lat));
	double prev_long_rad = ToRad(ToDegree(previous_Long));
	double current_lat_rad = ToRad(ToDegree(current_Lat));
	double current_long_rad = ToRad(ToDegree(current_Long));
	double lat_diff = current_lat_rad - prev_lat_rad;
	double long_diff = current_long_rad - prev_long_rad;

	double a = sin(lat_diff / 2) * sin(lat_diff / 2) + cos(prev_lat_rad) * cos(current_lat_rad) * sin(long_diff / 2) * sin(long_diff / 2); // Haversine formula: a = sin (?f/2) + cos f1   cos f2   sin (??/2)
	double c = 2 * atan2(sqrt(a), sqrt(1 - a));
	return (EARTH_RADIUS * c); // distance in meters
}
/**
 * @brief Parses the GPS data from the UART interface and stores it into the
 *        global variables `currentLat` and `currentLong`.
 *
 * @details This function takes the GPS data from the UART interface and
 *          parses it into the individual fields using the `strtok` function.
 *          The function then checks if the received data is valid and if so,
 *          it stores the current latitude and longitude in the global
 *          variables `currentLat` and `currentLong` respectively.
 */
void GPS_format()
{
	float _long = ToDegree(currentLong);
	float _lat = ToDegree(currentLat);
	char noOfTokenStrings = 0;
	token = strtok(GPS, ",");
	do
	{
		strcpy(GPS_formated[noOfTokenStrings], token);
		token = strtok(NULL, ",");
		noOfTokenStrings++;
	} while (token != NULL);
	if (strcmp(GPS_formated[1], "A") == 0)
	{
		// Valid   strcmp--> return zero if equal
		if (strcmp(GPS_formated[3], "N") == 0)

			currentLat = atof(GPS_formated[2]);

		else
			currentLat = -atof(GPS_formated[2]);

		if (strcmp(GPS_formated[5], "E") == 0)
			currentLong = atof(GPS_formated[4]);
		else
			currentLong = -atof(GPS_formated[4]);
	}
	snprintf(current_latitude, sizeof(current_latitude), "%.6f", _lat);
	snprintf(current_longitude, sizeof(current_longitude), "%.6f", _long);

	UART0_EXCEL();
}

/**
 * @brief Constructs a time string in the format HH:MM:SS from GPS data.
 *
 * @details This function extracts characters from the `GPS_formated` array 
 *          and arranges them into a time string. The time is formatted as 
 *          hours, minutes, and seconds separated by colons.
 *
 * @return A pointer to a string containing the formatted time.
 */
char *TIME()
{ //***
	var[0] = GPS_formated[0][0];
	var[1] = GPS_formated[0][1];
	var[2] = ':';
	var[3] = GPS_formated[0][2];
	var[4] = GPS_formated[0][3];
	var[5] = ':';
	var[6] = GPS_formated[0][4];
	var[7] = GPS_formated[0][5];
	var[8] = GPS_formated[0][6];
	var[9] = GPS_formated[0][7];
	var[10] = '\0';
	return var;
}

/**
 * @brief Constructs a status string "DATA VALID" or "DATA INVALID" based on GPS data.
 *
 * @details This function checks the second field of the `GPS_formated` array to
 *          see if the GPS data is valid. If so, it copies the string "DATA VALID"
 *          into the `var` array; otherwise, it copies "DATA INVALID".
 *
 * @return A pointer to the string containing the status.
 */
char *status()
{
	if (strcmp(GPS_formated[1], "A") == 0)
	{
		strcpy(var, "DATA VALID");
		return var;
	}
	else
	{
		strcpy(var, "DATA INVALID");
		return var;
	}
}

/**
 * @brief Calculates the speed from the GPS data.
 *
 * @details This function converts the speed value from knots to meters per second
 *          by multiplying it by a conversion factor (0.514444).
 *
 * @return The speed in meters per second.
 */
float Speed()
{
	return atof(GPS_formated[6]) * 0.514444;
}
/**
 * @brief Constructs a date string in the format DD/MM/YYYY from GPS data.
 *
 * @details This function extracts characters from the `GPS_formated` array 
 *          and arranges them into a date string. The date is formatted as 
 *          day, month, and year separated by slashes.
 *
 * @return A pointer to a string containing the formatted date.
 */
char *DATE()
{ 
	var[0] = GPS_formated[7][0];
	var[1] = GPS_formated[7][1];
	var[2] = '/';
	var[3] = GPS_formated[7][2];
	var[4] = GPS_formated[7][3];
	var[5] = '/';
	var[6] = '2';
	var[7] = '0';
	var[8] = GPS_formated[7][4];
	var[9] = GPS_formated[7][5];
	var[10] = '\0';
	return var;
}

/**
 * @brief Delays the program execution.
 */
void delay_ms(unsigned int ms)
{
	volatile unsigned int count;
	while (ms--)
	{
		count = 5333; // Adjusted for ~16,000 cycles per ms at 16 MHz (assuming 3 cycles per iteration)
		while (count--){};
	}
}

/**
 * @brief Displays the GPS status on an LCD and controls LED indications.
 *
 * @details This function checks if the GPS data is valid by examining the 
 *          `GPS_formated` array. If the data is valid, it displays "DATA VALID" 
 *          on the LCD and turns on the green LED for a brief period. If the 
 *          data is invalid, it enters a loop where it continuously displays 
 *          "DATA INVALID" on the LCD, flashes the red LED, and attempts to 
 *          re-read and format the GPS data until valid data is obtained.
 */
void Display_status()
{
	if (strcmp(GPS_formated[1], "A") == 0)
	{
		lcd_string((unsigned char *)status(), 0);
		GPIO_setLedValue(GPIO_GREEN_LED, GPIO_LED_ON);
		delay_ms(2500);
		lcd_clear();
		GPIO_setLedValue(GPIO_GREEN_LED, GPIO_LED_OFF);
	}
	else
	{
		while (!(strcmp(GPS_formated[1], "A") == 0))
		{
			lcd_string((unsigned char *)status(), 0);
			GPIO_setLedValue(GPIO_RED_LED, GPIO_LED_ON);
			delay_ms(500);
			GPIO_setLedValue(GPIO_RED_LED, GPIO_LED_OFF);
			lcd_clear();
			GPS_read();
			GPS_format();
		}
	}
}

/**
 * @brief Displays the current time on an LCD.
 *
 * @details This function displays the label "TIME:" followed by the current
 *          time retrieved from the TIME() function on an LCD. The time is 
 *          displayed for 2500 ms before the LCD is cleared.
 */
void Display_TIME()
{
	lcd_string((unsigned char *)"TIME:", 0);
	lcd_string((unsigned char *)TIME(), 1);
	delay_ms(2500);
	lcd_clear();
}


/**
 * @brief Displays the current date on an LCD.
 *
 * @details This function displays the label "DATE:" followed by the current
 *          date retrieved from the DATE() function on an LCD. The date is 
 *          displayed for 2500 ms before the LCD is cleared.
 */
void Display_DATE()
{
	lcd_string((unsigned char *)"DATE:", 0);
	lcd_string((unsigned char *)DATE(), 1);
	delay_ms(2500);
	lcd_clear();
}

/**
 * @brief Displays the current speed on an LCD.
 *
 * @details This function displays the label "Speed:" followed by the current
 *          speed retrieved from the Speed() function on an LCD. The speed is
 *          displayed for 2500 ms before the LCD is cleared.
 */
void DisplaySpeed()
{
	char speedbuffer[7];
	float s;
	s = Speed();
	snprintf(speedbuffer, 20, "%.5f m/s", s);
	lcd_string((unsigned char *)"Speed:", 0);
	lcd_string((unsigned char *)speedbuffer, 1);
	delay_ms(2500);
	lcd_clear();
}

/**
 * @brief Sends the current latitude and longitude over UART0.
 *
 * @details This function transmits the current latitude and longitude
 *          as strings through UART0, followed by a newline character.
 *          The latitude and longitude are separated by a comma.
 */
void UART0_EXCEL()
{
	//  UART0_SendString(TIME ());
	// UART0_SendChar(',');                // separate column
	UART0_SendString(current_latitude);
	UART0_SendChar(',');
	UART0_SendString(current_longitude);
	UART0_SendChar('\n');
}

/**
 * @brief Transmits the current position and nearest landmark over UART2.
 *
 * @details This function transmits the current latitude and longitude as strings
 *          through UART2, followed by a comma, the name of the nearest landmark,
 *          and a newline character. The function does not transmit anything if the
 *          distance to the nearest landmark is greater than 10000 meters.
 */
void UART2_ESP()
{
	if (min_dist >= 10000)
		return;
	UART2_SendString(current_latitude);
	UART2_SendChar(',');
	UART2_SendString(current_longitude);
	UART2_SendChar(',');
	UART2_SendString(landmarks[nearest].name);
	UART2_SendChar('\n');
}