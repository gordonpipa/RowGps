





// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial

// Set serial for AT commands (to the module)
// Use Hardware Serial on Mega, Leonardo, Micro
#define SerialAT Serial1

#define TINY_GSM_MODEM_SIM7000
#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb
#define SerialAT Serial1

// See all AT commands, if wanted
// #define DUMP_AT_COMMANDS

// set GSM PIN, if any
#define GSM_PIN ""

// Your GPRS credentials, if any
const char apn[]  = "YOUR-APN";     //SET TO YOUR APN
const char gprsUser[] = "";
const char gprsPass[] = "";

#include <TinyGsmClient.h>
#include <SPI.h>
#include <SD.h>
#include <Ticker.h>

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

#define uS_TO_S_FACTOR      1000000ULL  // Conversion factor for micro seconds to seconds
#define TIME_TO_SLEEP       60          // Time ESP32 will go to sleep (in seconds)

#define UART_BAUD           9600
#define PIN_DTR             25
#define PIN_TX              27
#define PIN_RX              26
#define PWR_PIN             4

#define SD_MISO             2
#define SD_MOSI             15
#define SD_SCLK             14
#define SD_CS               13
#define LED_PIN             12


void enableGPS(void)
{
    // Set SIM7000G GPIO4 LOW ,turn on GPS power
    // CMD:AT+SGPIO=0,4,1,1
    // Only in version 20200415 is there a function to control GPS power
    modem.sendAT("+SGPIO=0,4,1,1");
    if (modem.waitResponse(10000L) != 1) {
        DBG(" SGPIO=0,4,1,1 false ");
    }
    modem.enableGPS();


}

void disableGPS(void)
{
    // Set SIM7000G GPIO4 LOW ,turn off GPS power
    // CMD:AT+SGPIO=0,4,1,0
    // Only in version 20200415 is there a function to control GPS power
    modem.sendAT("+SGPIO=0,4,1,0");
    if (modem.waitResponse(10000L) != 1) {
        DBG(" SGPIO=0,4,1,0 false ");
    }
    modem.disableGPS();
}

void modemPowerOn()
{
    pinMode(PWR_PIN, OUTPUT);
    digitalWrite(PWR_PIN, LOW);
    delay(1000);    //Datasheet Ton mintues = 1S
    digitalWrite(PWR_PIN, HIGH);
}

void modemPowerOff()
{
    pinMode(PWR_PIN, OUTPUT);
    digitalWrite(PWR_PIN, LOW);
    delay(1500);    //Datasheet Ton mintues = 1.2S
    digitalWrite(PWR_PIN, HIGH);
}


void modemRestart()
{
    modemPowerOff();
    delay(1000);
    modemPowerOn();
}

void setup()
{
    // Set console baud rate
    SerialMon.begin(115200);
    Serial.println ("esp32");

    delay(10);

    // Set LED OFF
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    modemPowerOn();

    SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);

    Serial.println("/**********************************************************/");
    Serial.println("To initialize the network test, please make sure your GPS");
    Serial.println("antenna has been connected to the GPS port on the board.");
    Serial.println("/**********************************************************/\n\n");

    delay(10000);
}

void loop()
{
    if (!modem.testAT()) {
        Serial.println("Failed to restart modem, attempting to continue without restarting");
        modemRestart();
        return;
    }

    Serial.println("Start positioning . Make sure to locate outdoors.");
    Serial.println("The blue indicator light flashes to indicate positioning.");

    enableGPS();

    


    float lat      = 0;
    float lon      = 0;
    float speed    = 0;
    float alt     = 0;
    int   vsat     = 0;
    int   usat     = 0;
    float accuracy = 0;
    int   year     = 0;
    int   month    = 0;
    int   day      = 0;
    int   hour     = 0;
    int   min      = 0;
    int   sec      = 0;
    int   i        = 0;
    while (i<100) {
      
        SerialMon.println("Requesting current GPS/GNSS/GLONASS location " + String(i, 8));
        if (modem.getGPS(&lat, &lon, &speed, &alt, &vsat, &usat, &accuracy,
                     &year, &month, &day, &hour, &min, &sec)) {
            i =i+1;
            if (i<2) {
              Serial.println("The location has been locked, the latitude and longitude are:");
              SerialMon.println("Latitude: " + String(lat, 8) + "\tLongitude: " + String(lon, 8));
              SerialMon.println("Speed: " + String(speed) + "\tAltitude: " + String(alt));
              SerialMon.println("Visible Satellites: " + String(vsat) + "\tUsed Satellites: " + String(usat));
              SerialMon.println("Accuracy: " + String(accuracy));
              SerialMon.println("Year: " + String(year) + "\tMonth: " + String(month) + "\tDay: " + String(day));
              SerialMon.println("Hour: " + String(hour) + "\tMinute: " + String(min) + "\tSecond: " + String(sec));
              digitalWrite(LED_PIN, !digitalRead(LED_PIN));

            }
            else {
              SerialMon.println("Sp: " + String(speed) + "\tAlt: " + String(alt) +"\tLat: " + String(lat, 8) + "\tLon: " + String(lon, 8));
              digitalWrite(LED_PIN, !digitalRead(LED_PIN));
              
            }            
      
        }
        else {
          digitalWrite(LED_PIN, !digitalRead(LED_PIN));
          delay(1000);
          digitalWrite(LED_PIN, !digitalRead(LED_PIN));
          delay(1000);
          
        }
        
        delay(200);
    }

    disableGPS();

    Serial.println("/**********************************************************/");
    Serial.println("After the network test is complete, please enter the  ");
    Serial.println("AT command in the serial terminal.");
    Serial.println("/**********************************************************/\n\n");

    while (1) {
        while (SerialAT.available()) {
            SerialMon.write(SerialAT.read());
        }
        while (SerialMon.available()) {
            SerialAT.write(SerialMon.read());
        }
    }
}







// /*
//   Rui Santos
//   Complete project details at https://RandomNerdTutorials.com/lilygo-t-sim7000g-esp32-gps-data/
  
//   Permission is hereby granted, free of charge, to any person obtaining a copy
//   of this software and associated documentation files.
  
//   The above copyright notice and this permission notice shall be included in all
//   copies or substantial portions of the Software.
// */

// #define TINY_GSM_MODEM_SIM7000
// #define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb

// #include <TinyGsmClient.h>

// // LilyGO T-SIM7000G Pinout
// #define UART_BAUD   115200
// #define PIN_DTR     25
// #define PIN_TX      27
// #define PIN_RX      26
// #define PWR_PIN     4

// #define LED_PIN     12

// // Set serial for debug console (to Serial Monitor, default speed 115200)
// #define SerialMon Serial
// // Set serial for AT commands
// #define SerialAT  Serial1

// TinyGsm modem(SerialAT);

// void setup(){
//   SerialMon.begin(115200);
//   SerialMon.println("Place your board outside to catch satelite signal");

//   // Set LED OFF
//   pinMode(LED_PIN, OUTPUT);
//   digitalWrite(LED_PIN, HIGH);

//   //Turn on the modem
//   pinMode(PWR_PIN, OUTPUT);
//   digitalWrite(PWR_PIN, HIGH);
//   delay(300);
//   digitalWrite(PWR_PIN, LOW);

//   delay(1000);
  
//   // Set module baud rate and UART pins
//   SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);

//   // Restart takes quite some time
//   // To skip it, call init() instead of restart()
//   SerialMon.println("Initializing modem...");
//   if (!modem.restart()) {
//     Serial.println("Failed to restart modem, attempting to continue without restarting");
//   }
  
//   // Print modem info
//   String modemName = modem.getModemName();
//   delay(500);
//   SerialMon.println("Modem Name: " + modemName);

//   String modemInfo = modem.getModemInfo();
//   delay(500);
//   SerialMon.println("Modem Info: " + modemInfo);
// }

// void loop(){
//   // Set SIM7000G GPIO4 HIGH ,turn on GPS power
//   // CMD:AT+SGPIO=0,4,1,1
//   // Only in version 20200415 is there a function to control GPS power
//   modem.sendAT("+SGPIO=0,4,1,1");
//   if (modem.waitResponse(10000L) != 1) {
//     SerialMon.println(" SGPIO=0,4,1,1 false ");
//   }

//   modem.enableGPS();
  
//   delay(15000);
//   float lat      = 0;
//   float lon      = 0;
//   float speed    = 0;
//   float alt     = 0;
//   int   vsat     = 0;
//   int   usat     = 0;
//   float accuracy = 0;
//   int   year     = 0;
//   int   month    = 0;
//   int   day      = 0;
//   int   hour     = 0;
//   int   min      = 0;
//   int   sec      = 0;
  
//   for (int8_t i = 15; i; i--) {
//     SerialMon.println("Requesting current GPS/GNSS/GLONASS location");
//     if (modem.getGPS(&lat, &lon, &speed, &alt, &vsat, &usat, &accuracy,
//                      &year, &month, &day, &hour, &min, &sec)) {
//       SerialMon.println("Latitude: " + String(lat, 8) + "\tLongitude: " + String(lon, 8));
//       SerialMon.println("Speed: " + String(speed) + "\tAltitude: " + String(alt));
//       SerialMon.println("Visible Satellites: " + String(vsat) + "\tUsed Satellites: " + String(usat));
//       SerialMon.println("Accuracy: " + String(accuracy));
//       SerialMon.println("Year: " + String(year) + "\tMonth: " + String(month) + "\tDay: " + String(day));
//       SerialMon.println("Hour: " + String(hour) + "\tMinute: " + String(min) + "\tSecond: " + String(sec));
//       break;
//     } 
//     else {
//       SerialMon.println("Couldn't get GPS/GNSS/GLONASS location, retrying in 15s.");
//       delay(15000L);
//     }
//   }
//   SerialMon.println("Retrieving GPS/GNSS/GLONASS location again as a string");
//   String gps_raw = modem.getGPSraw();
//   SerialMon.println("GPS/GNSS Based Location String: " + gps_raw);
//   SerialMon.println("Disabling GPS");
//   modem.disableGPS();

//   // Set SIM7000G GPIO4 LOW ,turn off GPS power
//   // CMD:AT+SGPIO=0,4,1,0
//   // Only in version 20200415 is there a function to control GPS power
//   modem.sendAT("+SGPIO=0,4,1,0");
//   if (modem.waitResponse(10000L) != 1) {
//     SerialMon.println(" SGPIO=0,4,1,0 false ");
//   }

//   delay(200);
//   // Do nothing forevermore
//   while (true) {
//       modem.maintain();
//   }
// }

