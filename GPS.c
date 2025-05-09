#include "GPS.h"
#include "Bit_Utilies.h"
#include "UART.h"
#include <math.h> 
#include <string.h>
#include <stdlib.h>
#include "LCD.h"
#include "GPIO.h"
#include <stdio.h>
//double currentLong ,currentLat;
char current_longitude[16] ;
char current_latitude[16];
double currentLong,currentLat;
const double EARTH_RADIUS = 6371000;
char GPS [80];///////////
char GPS_logName[]="$GPRMC,";
char GPS_formated[12][20];
char * token;
int nearest;
char CurrentNearestName[20];
float CurrentLatitude;
float CurrentLongitude;
float min_dist;



//LCD
		char buffer[16];
		float s;
//
static Landmark landmarks[6];
static char var[20]; 

void GPS_InitLandmarks() {//***
    strcpy(landmarks[0].name, "CREDIT");//CREDIT
    landmarks[0].latitude = 3003.82247;//3003.8;//9019/300.0;//3005.22;//3003.48////30.089444444;
    landmarks[0].longitude = 3116.70711;//3116.7;//18767/600.0;//3118.53;//3116.42////31.314722;

    strcpy(landmarks[1].name, "STADIUM");
    landmarks[1].latitude = 3003.84237;//3003.833333333;//10823/360.0;//3003.50;
    landmarks[1].longitude = 3116.75010;//3116.7333333;//28151/900.0;//3116.44;

    strcpy(landmarks[2].name, "LIBRARY");
    landmarks[2].latitude = 3003.91033;//3003.9;//6013/200.0;//3003.54;
    landmarks[2].longitude = 3116.80179;//3116.8;//782/25.0;//3116.48;

    strcpy(landmarks[3].name, "HALL_A");
    landmarks[3].latitude = 3003.86148;//3003.86159;//3003.85919;//36077/1200.0;//3003.51;
    landmarks[3].longitude = 3116.81583;//03116.80921;//03116.80865;//782/25.0;//3116.48;

    strcpy(landmarks[4].name, "HALL_C");
    landmarks[4].latitude = 3003.82510;//3003.81693;//9019/300.0;//3003.49;3003.79089;
    landmarks[4].longitude = 3116.83927;//03116.84672;//3116.50;//03116.84328;
	
    strcpy(landmarks[5].name, "error");
    landmarks[5].latitude = 5;
    landmarks[5].longitude=0;
}
Landmark GPS_FindNearestLandmark() {//***
  int i;  
	min_dist =10000;
   	 
		nearest = 5;
	
		GPS_InitLandmarks();
    for (i = 0; i < 5; i++) {

        double dist = GPS_getDistance(currentLong, currentLat,landmarks[i].longitude,landmarks[i].latitude);
/*			UART0_SendString("dist");
				UART0_SendChar('\n');
				UART0_SendFloat(dist);
				lcd_string((unsigned char*)"Distance",0);
				lcd_float(dist, 1); 
				delay_ms(500);
				lcd_clear(); */
			
			if (dist < min_dist) {
            min_dist = dist;
            nearest = i;
        }

    }
		/*
					 lcd_string((unsigned char*)"MinimunDistance",0);
						lcd_float(min_dist, 1); 
						delay_ms(500);
						lcd_clear();*/
/*    if (min_dist <=20)
         return landmarks[nearest] ;

    else 
		{*/
				landmarks[nearest].distance=GPS_getDistance(currentLong, currentLat,landmarks[nearest].longitude,landmarks[nearest].latitude);
        return landmarks[nearest] ;
		//}
	}
void GPS_read(){//***
    char recievedChar;
		char fillGPScounter = 0;
    char flag = 1;
		char i;
    do {
        flag = 1;
        for( i = 0; i < 7; i++){//$dhf
            if(UART_GetChar() != GPS_logName[i]){//if(UART_GetChar() != GPS_logName[i]){
                flag = 0;
								break;
            }
        }
    } while(flag == 0);
    // Here I make sure that I recieved the correct log
		strcpy(GPS , ""); // Init GPS Array
do {
    recievedChar = UART_GetChar();
    GPS[fillGPScounter++] = recievedChar;
} while (recievedChar != '*');
//UART0_SendString(GPS);//////////////////////////////////////////////////run2
//UART0_SendChar('\n');
}
float ToDegree (float angle) {
int degree = (int) (angle / 100);
float minutes = angle - (float) degree* 100;
return (degree+ (minutes/ 60) ) ;
}
float ToRad (float angle){//***
return angle * PI / 180.0 ;
}
//float GPS_getDistance (float currentLong , float currentLat ,float destLong , float destLat){//***

	//function to calculate distance between two locations using longitude and latitude in Radians//
//  Haversine formula: a = sin (?f/2) + cos f1   cos f2   sin (??/2)
//  c = 2 * atan2(sqrt(a), sqrt(1 - a))
// distance = Earth radius *c
double GPS_getDistance(double current_Long,double current_Lat,double previous_Long,double previous_Lat){
    double prev_lat_rad = ToRad(ToDegree(previous_Lat));
    double prev_long_rad = ToRad(ToDegree(previous_Long));
    double current_lat_rad = ToRad(ToDegree(  current_Lat));
    double current_long_rad = ToRad(ToDegree(current_Long));
    double lat_diff = current_lat_rad - prev_lat_rad;
    double long_diff = current_long_rad - prev_long_rad;

    double a= sin(lat_diff/2)*sin(lat_diff/2)+ cos(prev_lat_rad)*cos(current_lat_rad)*sin(long_diff/2)*sin(long_diff/2); // Haversine formula: a = sin (?f/2) + cos f1   cos f2   sin (??/2)
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return (EARTH_RADIUS*c);   //distance in meters
}
	/*	// Get Radian Angle
	float currentLongRad = ToRad (ToDegree (currentLong) ) ;
	float currentLatRad = ToRad (ToDegree (currentLat )) ;
	float destLongRad = ToRad (destLong) ;//float destLongRad = ToRad (ToDegree (destLong) ) ;
	float destLatRad = ToRad (destLat) ;//float destLatRad = ToRad (ToDegree (destLat)) ;
// *	float currentLongRad = ToRad (ToDegree (currentLong))  ;//float currentLongRad = ToRad (ToDegree (currentLong) ) ;
	float currentLatRad = ToRad (ToDegree (currentLat));//float currentLatRad = ToRad (ToDegree (currentLat) ) ;
	float destLongRad = ToRad (destLong ) ;//float destLongRad = ToRad (ToDegree (destLong) ) ;
	float destLatRad = ToRad (destLat) ;//float destLatRad = ToRad (ToDegree (destLat)) ;* /
	// Get Difference
	float longDiff = destLongRad - currentLongRad;
	float latDiff = destLatRad - currentLatRad;
	// Calculate Distance
//	float a = (sin(latDiff/2)sin(latDiff/2))+cos(currentLatRad)*cos(destLatRad)(sin(longDiff/2)*sin(longDiff/2)); //haversine formula 
//	double c= 2* atan2 (sqrt(a),sqrt(1-a));
	float a = pow(sin(latDiff/2),2)+cos(currentLatRad)*cos(destLatRad)*pow(sin(longDiff/2),2);//haversine formula 
	double c= 2* atan2 (sqrt(a),sqrt(1-a));
	return EARTH_RADIUS * c ;
}*/
void GPS_format(){//***
		float _long = ToDegree(currentLong);
	  float _lat = ToDegree(currentLat);
    char noOfTokenStrings = 0;
    token = strtok(GPS , ",");
    do {
        strcpy(GPS_formated[noOfTokenStrings], token);
        token = strtok(NULL , ",");
        noOfTokenStrings++;
    } while(token != NULL);
    if(strcmp(GPS_formated[1], "A") == 0) { 
			// Valid   strcmp--> return zero if equal
		//	current_latitude = GPS_formated[2];
		//	current_longitude = GPS_formated[4];
        if(strcmp(GPS_formated[3], "N") == 0)
				 					
            currentLat = atof(GPS_formated[2]);
	         			
				  else
            currentLat = -atof(GPS_formated[2]);

        if(strcmp(GPS_formated[5], "E") == 0)
            currentLong = atof(GPS_formated[4]);
        else
            currentLong = -atof(GPS_formated[4]);
    } //else { 
	//	Display_status ();
		
		
	//	}
		
/*		UART0_SendString(" Time: ");//////////////////////////////////////////////////run1
    UART0_SendString(GPS_formated[0]);
		UART0_SendChar(',');
		UART0_SendString(" ,Data valid: ");
    UART0_SendString(GPS_formated[1]);
		UART0_SendChar(',');
		UART0_SendString(" Lat : ");
    UART0_SendString(GPS_formated[2]);
    UART0_SendChar(',');	
		UART0_SendString(" dir : ");
    UART0_SendString(GPS_formated[3]);
		UART0_SendChar(',');
		UART0_SendString(" Long : ");
		UART0_SendString(GPS_formated[4]);
		UART0_SendChar(',');
		UART0_SendString(" direction : ");
    UART0_SendString(GPS_formated[5]);
		UART0_SendChar('\n');*/
		
		/*
		//UART0_SendString(" end : ");/////////////////////////////////////////////comment it
		//UART0_SendString(GPS_formated[6]);///////////////////////////////////////comment it
		*/
	  snprintf(current_latitude,sizeof(current_latitude),"%.6f",_lat);
	  snprintf(current_longitude,sizeof(current_longitude),"%.6f",_long);
		
		UART0_EXCEL();
}

char* TIME (){//***
	var [0] = GPS_formated[0][0];
	var [1] = GPS_formated[0][1];
	var [2] = ':';
	var [3] = GPS_formated[0][2];
	var [4] = GPS_formated[0][3];
	var [5] = ':';
	var [6] = GPS_formated[0][4];
	var [7] = GPS_formated[0][5];
	var [8] = GPS_formated[0][6];
	var [9] = GPS_formated[0][7];
	var [10] = '\0';
	return var;
}
char* status (){//***
	if(strcmp(GPS_formated[1], "A") == 0) {
    strcpy(var, "DATA VALID");
    return var;
	}
  else
	{
    strcpy(var, "DATA INVALID");
    return var;
	}
}
float Speed (){//***
	return atof(GPS_formated[6])*0.514444;
}
char* DATE (){//***
	var [0] = GPS_formated[7][0];
	var [1] = GPS_formated[7][1];
	var [2] = '/';
	var [3] = GPS_formated[7][2];
	var [4] = GPS_formated[7][3];
	var [5] = '/';
	var [6] = '2';
	var [7] = '0';
	var [8] = GPS_formated[7][4];
	var [9] = GPS_formated[7][5];
	var [10] = '\0';
	return var;
}
void delay_ms(unsigned int ms) {//***
    volatile unsigned int count;
    while (ms--) {
        count = 5333; // Adjusted for ~16,000 cycles per ms at 16 MHz (assuming 3 cycles per iteration)
        while (count--) {// Each iteration consumes approximately 3 cycles
        }
    }
}
 
void Display_status (){//***
		if(strcmp(GPS_formated[1], "A") == 0)
			{
			lcd_string((unsigned char*)status (),0);
			GPIO_setLedValue(GPIO_GREEN_LED,GPIO_LED_ON);
			delay_ms(2500);
			lcd_clear();
			GPIO_setLedValue(GPIO_GREEN_LED,GPIO_LED_OFF);
			}
		else
			{
				while(!(strcmp(GPS_formated[1], "A")== 0))
				{
						lcd_string((unsigned char*)status (),0);
						GPIO_setLedValue(GPIO_RED_LED,GPIO_LED_ON);
						delay_ms(500);
						GPIO_setLedValue(GPIO_RED_LED,GPIO_LED_OFF);
						lcd_clear();	
						GPS_read();
						GPS_format();
				}  
				/*
		UART0_SendString(" Time: ");
    UART0_SendString(GPS_formated[0]);
		UART0_SendString(" Data valid: ");
    UART0_SendString(GPS_formated[1]);
		UART0_SendString(" Lat : ");
    UART0_SendString(GPS_formated[2]);
		UART0_SendString(" dir : ");
    UART0_SendString(GPS_formated[3]);
		UART0_SendString(" Long : ");
    UART0_SendString(GPS_formated[4]);
		UART0_SendString(" dir : ");
    UART0_SendString(GPS_formated[5]);
		UART0_SendString(" end : ");
    UART0_SendString(GPS_formated[6]);						
			*/
			}
}
void Display_TIME (){//***
			lcd_string((unsigned char*)"TIME:",0);
			lcd_string((unsigned char*)TIME (),1);
			delay_ms(2500);
			lcd_clear();
}
void Display_DATE (){//***
			lcd_string((unsigned char*)"DATE:",0);
			lcd_string((unsigned char*)DATE (),1);
			delay_ms(2500);
			lcd_clear();
}
void DisplaySpeed() {//***
char speedbuffer[7];
float s;
    s = Speed();
    snprintf(speedbuffer, 20, "%.5f m/s", s);
		lcd_string((unsigned char*)"Speed:",0);
    lcd_string((unsigned char*)speedbuffer,1);
//		lcd_string((unsigned char*)" m/s",1);
    delay_ms(2500);
    lcd_clear();
}
void UART0_EXCEL(){
      //  UART0_SendString(TIME ());
  		// UART0_SendChar(',');                // separate column
				UART0_SendString(current_latitude);
				UART0_SendChar(',');
				UART0_SendString(current_longitude);			
				UART0_SendChar('\n');
}

void UART2_ESP(){
      //  UART0_SendString(TIME ());
  		// UART0_SendChar(',');                // separate column
	
				if (min_dist >=10000)
						return;
				UART2_SendString(current_latitude);
				UART2_SendChar(',');
				UART2_SendString(current_longitude);
				UART2_SendChar(',');
				UART2_SendString(landmarks[nearest].name);
				UART2_SendChar('\n');
}





