#ifndef GPS_H
#define GPS_H
#define PI 3.14159265358979323846

typedef struct {
    char name[20];
    float latitude;
    float longitude;	
		float distance;
} Landmark;


typedef struct {
    char message [20];
} VAR;
void GPS_InitLandmarks(void) ;
Landmark GPS_FindNearestLandmark() ;
void GPS_read();


float ToDegree (float angle) ;

float ToRad (float angle);
double GPS_getDistance (double currentLong , double currentLat ,double destLong , double destLat);
void GPS_format();
char* TIME ();
char* status ();
float Speed ();
char* DATE ();
void delay_ms(unsigned int ms);
void Display_status ();
void Display_TIME ();
void Display_DATE ();
void DisplaySpeed();
void UART0_EXCEL();
void UART2_ESP();
#endif