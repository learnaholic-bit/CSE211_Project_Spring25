////*PortF Switches*////
#define GPIO_SW1 0
#define GPIO_SW2 1
#define GPIO_SW_NOT_PRESSED 1

////*PortF LED color*////
#define GPIO_RED_LED 0
#define GPIO_GREEN_LED 1
#define GPIO_BLUE_LED 2

////*PortF LED state on/off*////
#define GPIO_LED_ON 1
#define GPIO_LED_OFF 0


//////////*functions*/////////
void GPIO_initPortE_BUZZER();

void GPIO_initPortF(void);

unsigned char GPIO_getSwitchesValue(unsigned char sw);

void GPIO_setLedValue(unsigned char ledColor, unsigned char ledState);

void buzzerOn(void);

void buzzerOff(void);

void GPIO_printSevenSegment(unsigned char data);