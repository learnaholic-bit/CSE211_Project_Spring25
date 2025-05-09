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

/*
// GPS.h
#ifndef GPS
#define GPS

#define PI 3.14159265358979323846

// External Variables
extern char GPS[80];
extern char GPS_formated[12][20];
extern float currentLong, currentLat, speed, finalLat;

// Function Prototypes
void GPS_read(void);
void GPS_format(void);
float ToDegree(float angle);
float ToRad(float angle);
float GPS_getDistance(float currentLong, float currentLat, float destLong, float destLat);

#endif // GPS*/