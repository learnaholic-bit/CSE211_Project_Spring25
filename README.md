# Embeded_project

## Overview

This project is a GPS-based navigation and alert system implemented on a TM4C123GH6PM microcontroller. It's designed to read GPS data, identify the nearest predefined landmark, and provide real-time information and alerts to the user. Key information such as current location, time, date, speed, and nearest landmark is displayed on an LCD. The system also features visual (LEDs) and auditory (buzzer) alerts based on proximity to landmarks.

Additionally, the project integrates an ESP8266 Wi-Fi module to serve a web page, allowing users to view the live GPS location and path on a map through a web browser.

## Features

*   **Real-time GPS Tracking**: Acquires and parses NMEA `$GPRMC` sentences from a GPS module.
*   **Landmark Navigation**:
    *   Stores a predefined list of landmarks (e.g., "CREDIT", "STADIUM", "LIBRARY", "HALL_A", "HALL_C") with their coordinates.
    *   Calculates the distance to these landmarks using the Haversine formula.
    *   Identifies and displays the name of the nearest landmark.
*   **User Interface & Display**:
    *   Character LCD for displaying status (e.g., "DATA VALID"), time, date, speed, and nearest landmark information.
    *   Two push-button switches (SW1 on PF4, SW2 on PF0) for user interaction to cycle through display modes.
*   **Proximity Alerts**:
    *   **Visual Alerts**: RGB LED on PortF indicates distance to the nearest landmark:
        *   Green LED + Buzzer: < 30 meters (approaching destination)
        *   Blue LED: 30-50 meters
        *   Red LED: 50-80 meters
        *   All LEDs (White effect): > 80 meters or default state.
    *   **Auditory Alerts**: Buzzer on PortE (PE1) activates when very close to a landmark.
*   **Data Logging**: Outputs current latitude and longitude via UART0, suitable for logging or interfacing with tools like Excel.
*   **Live Web Map**:
    *   Transmits current latitude, longitude, and nearest landmark name to an ESP8266 module via UART2.
    *   The ESP8266 acts as a Wi-Fi Access Point.
    *   Serves an HTML page with a Leaflet.js map displaying the live location and tracked path.

## System Architecture

The system primarily consists of two main components:

1.  **TM4C123GH6PM Microcontroller**: This is the core controller responsible for:
    *   Interfacing with the GPS module (via UART3).
    *   Processing GPS data.
    *   Managing user input from switches (PortF).
    *   Displaying information on the LCD.
    *   Controlling LEDs (PortF) and the buzzer (PortE) for alerts.
    *   Sending processed location data to the ESP8266 (via UART2).
2.  **ESP8266 Wi-Fi Module**: This module:
    *   Receives location data from the TM4C123 (via its Serial/UART input).
    *   Creates a Wi-Fi Access Point.
    *   Hosts a web server that provides an HTML page with a live map.

## Hardware Components

*   TM4C123GH6PM LaunchPad Development Board
*   GPS Module (interfaced via UART)
*   Character LCD (interfaced with GPIO)
*   RGB LED (on-board PortF LED: PF1-Red, PF2-Blue, PF3-Green)
*   Push-button Switches (on-board PortF: SW1-PF4, SW2-PF0)
*   Buzzer (interfaced with GPIO on PortE, specifically PE1)
*   ESP8266 Wi-Fi Module (e.g., ESP-01, NodeMCU)
*   Connecting Wires

## Software Modules

The project firmware is divided into several modules:

### TM4C123 Microcontroller Firmware (C Language)

*   **`main.c`**:
    *   Main application logic.
    *   Initializes all peripherals (`GPIO_initPortE_BUZZER`, `UART_Init` for UART3, `UART0_Init`, `UART2_Init`, `lcd_init`, `GPIO_initPortF`).
    *   Contains the main loop for reading GPS (`GPS_read`), processing data (`GPS_format`, `GPS_FindNearestLandmark`), handling user input (`GPIO_getSwitchesValue`), updating LCD (`Display_status`, `Display_TIME`, etc.), and managing alerts (`GPIO_setLedValue`, `buzzerOn`/`buzzerOff`).
    *   Coordinates data transmission to the ESP8266 (`UART2_ESP`).
*   **`GPS.h` / `GPS.c`**:
    *   Handles communication with the GPS module (via UART3, using `UART_GetChar` from `UART.c`).
    *   Parses `$GPRMC` NMEA sentences.
    *   Stores and manages landmark data (see `GPS_InitLandmarks` in `GPS.c`).
    *   Calculates distances using the Haversine formula (`GPS_getDistance` in `GPS.c`).
    *   Formats GPS data (time, date, speed, coordinates via functions like `TIME()`, `DATE()`, `Speed()` in `GPS.c`).
    *   Provides `GPS_FindNearestLandmark` function.
    *   Sends data to UART0 (for logging via `UART0_EXCEL` in `GPS.c`) and UART2 (for ESP8266 via `UART2_ESP` in `GPS.c`).
*   **`UART.h` / `UART.c`**:
    *   Provides drivers for UART0 (PA0/PA1), UART2 (PD6/PD7), and UART3 (PC6/PC7) peripherals.
    *   Functions for initialization, sending/receiving characters and strings.
    *   UART0: Used for data logging.
    *   UART2: Used for sending data to the ESP8266 module.
    *   UART3: Used for receiving data from the GPS module (via `UART_Init` and `UART_GetChar`).
*   **`LCD.h` / `LCD.c`** :
    *   Driver for the character LCD. Pin connections detailed in `LCD.c` comments (RS=PD0, RW=PD1, EN=PD2, D0=PA7, D1=PA6, D2=PD3, D3=PB4, D4=PE5, D5=PE4, D6=PB1, D7=PB0).
    *   Functions for initialization (`lcd_init`), sending commands (`lcd_cmd`), writing data/strings (`lcd_data`, `lcd_string`), clearing display (`lcd_clear`), and printing floats (`lcd_float`).
*   **`GPIO.h` / `GPIO.c`**:
    *   Manages General Purpose Input/Output pins.
    *   Initializes PortF for on-board switches (SW1-PF4, SW2-PF0) and RGB LED (PF1-Red, PF2-Blue, PF3-Green) via `GPIO_initPortF`.
    *   Initializes PortE for the buzzer (PE1) via `GPIO_initPortE_BUZZER`.
    *   Provides functions to read switch states (`GPIO_getSwitchesValue`), control LED colors (`GPIO_setLedValue`), and activate/deactivate the buzzer (`buzzerOn`, `buzzerOff`).
    *   Includes `GPIO_printSevenSegment` for 7-segment display control (though not used in `main.c`).
*   **`SEVEN_SEG.h` / `SEVEN_SEG.c`**:
    *   Contains functions (`setSevenSegment`, `GPIO_initSevenSegment`) and definitions for controlling a 7-segment display. Pin connections detailed in `SEVEN_SEG.c` comments (A=PA2, B=PA3, C=PA4, D=PB6, E=PB7, F=PD6, G=PD7).
    *   Includes patterns for numbers and letters.
    *   *Note: This module does not appear to be actively used in the `main.c` logic in the current version.*
*   **`bit_utilies.h`**:
    *   A utility header file with macros for common bit manipulation operations (e.g., `SET_BIT`, `CLR_BIT`, `GET_BIT`). (Note: included as `Bit_Utilies.h` in C files).
*   **`tm4c123gh6pm.h` / `TM4C123.h`**:
    *   Device-specific header files for the TM4C123GH6PM microcontroller, providing register definitions.

### ESP8266 Wi-Fi Module Firmware (C++/Arduino)

*   **`ESP8266WiFi.h` (Sketch for ESP8266)**:
    *   Configures the ESP8266 to connect to an existing Wi-Fi network (SSID: "OPPO F11", Password: "m1042004" - *remember to change this for security if needed!*).
        *   Initializes a web server on port 80.
        *   Handles HTTP requests:
            *   `/`: Serves an HTML page with a Leaflet.js map.
            *   `/coords`: Provides current GPS coordinates and location description as JSON.
        *   Receives latitude, longitude, and location description string from the TM4C123 via its `Serial` port (baud rate 9600, connected to TM4C's UART2 TX).
        *   Updates the map marker and path dynamically.

## Setup and Usage

### TM4C123 Setup

1.  **Hardware Connections**:
    *   Connect the GPS module's TX pin to the TM4C123's UART3 RX pin (PC6).
    *   Connect the Character LCD to the TM4C123 GPIO pins:
        *   RS: PD0
        *   RW: PD1
        *   EN: PD2
        *   D0: PA7, D1: PA6, D2: PD3, D3: PB4, D4: PE5, D5: PE4, D6: PB1, D7: PB0
    *   Connect the buzzer to TM4C123's PE1 pin.
    *   Connect the TM4C123's UART2 TX pin (PD7) to the ESP8266's RX pin.
2.  **Software**:
    *   Compile the TM4C123 C code using an appropriate IDE (e.g., Keil MDK, TI Code Composer Studio, IAR Embedded Workbench).
    *   Flash the compiled binary to the TM4C123 microcontroller.

### ESP8266 Setup

1.  **Hardware Connections**:
    *   Connect the ESP8266's RX pin to the TM4C123's UART2 TX pin (PD7). Ensure proper voltage level shifting if necessary (though both TM4C123 and ESP8266 are typically 3.3V logic).
    *   Power the ESP8266 module.
2.  **Software**:
    *   Open the content of `ESP8266WiFi.h` as a sketch in the Arduino IDE (with ESP8266 board support installed).
    *   Modify the `ssid` and `password` variables in the sketch if you want to change the Access Point credentials.
    *   Upload the sketch to the ESP8266 module (ensure Serial baud rate in sketch matches TM4C UART2, currently 9600).
    *   Monitor the Serial Monitor in Arduino IDE (baud rate 9600) when the ESP8266 boots up to see the IP address.

### Running the System

1.  Power on both the TM4C123 and the ESP8266.
2.  The TM4C123 will start acquiring GPS data. Ensure the GPS module has a clear view of the sky for a satellite fix. The LCD will show "DATA INVALID" and the red LED may flash until a fix is obtained.
3.  The LCD will display information based on switch inputs or proximity alerts once GPS data is valid.
4.  To view the live map:
    *   Connect your computer or smartphone to the Wi-Fi network (e.g., "OPPO F11").
    *   Open a web browser and navigate to the IP address of the ESP8266 
    *   The web page will display the live location and path.

## Project Structure (Key Files)

```
.
├── main.c                 // Main application logic for TM4C123
├── GPS.c                  // GPS data handling
├── GPS.h                  // Header for GPS module
├── UART.c                 // UART peripheral driver
├── UART.h                 // Header for UART driver
├── LCD.c                  // LCD driver
├── LCD.h                  // Header for LCD driver
├── GPIO.c                 // GPIO driver (switches, LEDs, buzzer)
├── GPIO.h                 // Header for GPIO driver
├── SEVEN_SEG.c            // 7-segment display functions
├── SEVEN_SEG.h            // Header for 7-segment display
├── bit_utilies.h          // Bit manipulation macros (Note: included as Bit_Utilies.h)
├── tm4c123gh6pm.h         // TM4C123 MCU specific definitions
├── TM4C123.h              // Alternative/additional TM4C123 MCU definitions
├── ESP8266WiFi.h          // ESP8266 C++ sketch for WiFi and web server
└── README.md              // This file
```