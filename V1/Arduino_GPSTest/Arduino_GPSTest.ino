// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial

// Set serial for AT commands (to the module)
// Use Hardware Serial on Mega, Leonardo, Micro
#define SerialAT Serial1

#define TINY_GSM_MODEM_SIM7000
#define TINY_GSM_RX_BUFFER 1024  // Set RX buffer to 1Kb
#define SerialAT Serial1
#define DebbugFlag 1
#define GPSdelay 1000

// See all AT commands, if wanted
// #define DUMP_AT_COMMANDS

// set GSM PIN, if any
#define GSM_PIN ""

// Your GPRS credentials, if any
const char apn[] = "YOUR-APN";  //SET TO YOUR APN
const char gprsUser[] = "";
const char gprsPass[] = "";

#include <TinyGsmClient.h>
#include <SPI.h>
#include <SD.h>
#include <Ticker.h>
#include <time.h>
#include <WiFi.h>

const char* ssid = "1495_Guest";
const char* password = "14951495";
long timezone = 1;
byte daysavetime = 1;
String dataMessage;
String filename;

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

#define uS_TO_S_FACTOR 1000000ULL  // Conversion factor for micro seconds to seconds
#define TIME_TO_SLEEP 60           // Time ESP32 will go to sleep (in seconds)

#define UART_BAUD 9600
#define PIN_DTR 25
#define PIN_TX 27
#define PIN_RX 26
#define PWR_PIN 4

#define SD_MISO 2
#define SD_MOSI 15
#define SD_SCLK 14
#define SD_CS 13
#define LED_PIN 12


void enableGPS(void) {
  // Set SIM7000G GPIO4 LOW ,turn on GPS power
  // CMD:AT+SGPIO=0,4,1,1
  // Only in version 20200415 is there a function to control GPS power
  modem.sendAT("+SGPIO=0,4,1,1");
  if (modem.waitResponse(10000L) != 1) {
    DBG(" SGPIO=0,4,1,1 false ");
  }
  modem.enableGPS();
}

float readBattery() {
  //#define BAT_ADC     35
  //#define SOLAR_ADC   36
  int vref = 1100;
  uint16_t volt = analogRead(35);
  float battery_voltage = ((float)volt / 4095.0) * 2.0 * 3.3 * (vref);
  return battery_voltage;
}
float readSolar() {
  //#define BAT_ADC     35
  //#define SOLAR_ADC   36
  int vref = 1100;
  uint16_t volt = analogRead(36);
  float battery_voltage = ((float)volt / 4095.0) * 2.0 * 3.3 * (vref);
  return battery_voltage;
}

void appendFile(fs::FS& fs, const char* path, const char* message) {
  if (DebbugFlag == 1) { Serial.printf("Appending to file: %s\n", path); }

  File file = fs.open(path, FILE_APPEND);
  if (DebbugFlag == 1) {
    if (!file) {
      Serial.println("Failed to open file for appending");
      return;
    }
    if (file.print(message)) {
      Serial.println("Message appended");
    } else {
      Serial.println("Append failed");
    }
  }
  file.close();
}

void writeFile(fs::FS& fs, const char* path, const char* message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}
void disableGPS(void) {
  // Set SIM7000G GPIO4 LOW ,turn off GPS power
  // CMD:AT+SGPIO=0,4,1,0
  // Only in version 20200415 is there a function to control GPS power
  modem.sendAT("+SGPIO=0,4,1,0");
  if (modem.waitResponse(10000L) != 1) {
    DBG(" SGPIO=0,4,1,0 false ");
  }
  modem.disableGPS();
}

void modemPowerOn() {
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, LOW);
  delay(1000);  //Datasheet Ton mintues = 1S
  digitalWrite(PWR_PIN, HIGH);
}

void modemPowerOff() {
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, LOW);
  delay(1500);  //Datasheet Ton mintues = 1.2S
  digitalWrite(PWR_PIN, HIGH);
}


void modemRestart() {
  modemPowerOff();
  delay(1000);
  modemPowerOn();
}

void setup() {
  // Set console baud rate
  SerialMon.begin(115200);
  Serial.println("Rowing Monitor Esp32-GPS");
  SPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS)) {
    Serial.println("SDCard MOUNT FAIL");
  } else {
    Serial.println("SDCard MOUNT Success");
    uint32_t cardSize = SD.cardSize() / (1024 * 1024);
    String str = "SDCard Size: " + String(cardSize) + "MB";
    Serial.println(str);
  }

  File file = SD.open("/logfile.txt");
  if (!file) {
    Serial.println("File doens't exist");
    Serial.println("Creating file...");
    writeFile(SD, "/logfile.txt", "--- NEW session ----\r\n");
  } else {
    Serial.println("File already exists");
  }
  file.close();

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
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("Contacting Time Server");


  delay(10000);
}



void loop() {
  if (!modem.testAT()) {
    Serial.println("Failed to restart modem, attempting to continue without restarting");
    modemRestart();
    return;
  }

  Serial.println("Start positioning . Make sure to locate outdoors.");
  Serial.println("The blue indicator light flashes to indicate positioning.");

  enableGPS();

  float lat = 0;
  float lon = 0;
  float speed = 0;
  float alt = 0;
  int vsat = 0;
  int usat = 0;
  float accuracy = 0;
  int year = 0;
  int month = 0;
  int day = 0;
  int hour = 0;
  int min = 0;
  int sec = 0;
  int i = 0;
  float Bat_mv = 0;
  float Sol_mv = 0;

  uint64_t TimeTicker_us = 0;
  uint64_t TimeTickerStart_us = 0;
  uint64_t Act_time_ms = 0;
  uint64_t Last_loop_Act_time_ms = 0;
  uint64_t NextCall_ms = 0;
  int Runtime_ms = 0;
  int Keep_sampling_flag = 1;

  TimeTickerStart_us = esp_timer_get_time();
  NextCall_ms = GPSdelay;
  Bat_mv = readBattery();
  Sol_mv = readSolar();

  while (Keep_sampling_flag == 1) {
    if (modem.getGPS(&lat, &lon, &speed, &alt, &vsat, &usat, &accuracy, &year, &month, &day, &hour, &min, &sec)) {
      Keep_sampling_flag = 0;
    } else {
      SerialMon.println("Requesting current GPS/GNSS/GLONASS location " + String(i, 8));
      digitalWrite(LED_PIN, HIGH);
      delay(3000);
      digitalWrite(LED_PIN, LOW);
      delay(3000);
    }
  }

  delay(3000);

  if (DebbugFlag == 1) {
    Serial.println("The location has been locked, the latitude and longitude are:");
    SerialMon.println("Latitude: " + String(lat, 8) + "\tLongitude: " + String(lon, 8));
    SerialMon.println("Speed: " + String(speed) + "\tAltitude: " + String(alt));
    SerialMon.println("Visible Satellites: " + String(vsat) + "\tUsed Satellites: " + String(usat));
    SerialMon.println("Accuracy: " + String(accuracy));
    SerialMon.println("Year: " + String(year) + "\tMonth: " + String(month) + "\tDay: " + String(day));
    SerialMon.println("Hour: " + String(hour) + "\tMinute: " + String(min) + "\tSecond: " + String(sec));
  }

  dataMessage = "Starting Recording \r\n";
  appendFile(SD, "/logfile.txt", dataMessage.c_str());
  dataMessage = String(day) + "/" + String(month) + "/" + String(year) + "-" + String(hour) + ":" + String(min) + ":" + String(sec) + "\r\n";
  appendFile(SD, "/logfile.txt", dataMessage.c_str());
  filename = "/Data_" + String(day) + "_" + String(month) + "_" + String(year) + "_" + String(hour) + "_" + String(min) + "_" + String(sec) + ".txt";
  dataMessage = "Filename: " + filename;
  appendFile(SD, "/logfile.txt", dataMessage.c_str());

  File file = SD.open(filename.c_str());
  if (!file) {
    if (DebbugFlag == 1) {
      Serial.println("File doens't exist");
      Serial.println("Creating file...");
    }
    writeFile(SD, filename.c_str(), "--- NEW session ----\r\n");
  } else {
    if (DebbugFlag == 1) { Serial.println("File already exists"); }
  }
  file.close();

  dataMessage = String(day) + "/" + String(month) + "/" + String(year) + "-" + String(hour) + ":" + String(min) + ":" + String(sec) + "\r\n";
  appendFile(SD, filename.c_str(), dataMessage.c_str());

  Keep_sampling_flag = 1;
  Act_time_ms = (esp_timer_get_time() - TimeTickerStart_us) / 1000;
  NextCall_ms = Act_time_ms + GPSdelay;

  while (Keep_sampling_flag == 1) {
    Act_time_ms = (esp_timer_get_time() - TimeTickerStart_us) / 1000;
    if (Act_time_ms > NextCall_ms) {
      digitalWrite(LED_PIN, LOW);
      TimeTicker_us = (esp_timer_get_time() - TimeTickerStart_us);
      if (DebbugFlag == 1) { SerialMon.println("Next: " + String(NextCall_ms, 16) + "\tnow: " + String(Act_time_ms, 16) + "\tdt: " + String(Runtime_ms, 8)); }
      //delay(1000);
      //if ((NextCall-Act_time)> 250) {NextCall=Act_time;};
      //if ((NextCall-Act_time)<-250) {NextCall=Act_time;};

      //Serial.println("batter : %f\n", mv);

      if (DebbugFlag == 1) { SerialMon.println("Battery: " + String(Bat_mv, 8) + "\tV:" + "\tSolar: " + String(Sol_mv, 8) + "\tV:"); }
      if (modem.getGPS(&lat, &lon, &speed, &alt, &vsat, &usat, &accuracy,
                       &year, &month, &day, &hour, &min, &sec)) {
        i = i + 1;

        if (DebbugFlag == 1) { SerialMon.println(String(i, 8) + "\tSp: " + String(speed) + "\tAlt: " + String(alt) + "\tLat: " + String(lat, 8) + "\tLon: " + String(lon, 8)); }
        Bat_mv = readBattery();
        Sol_mv = readSolar();

        dataMessage = String(i) + "," + String(sec + min * 60 + hour * 60 * 60) + "," + String(Act_time_ms) + "," + String(NextCall_ms) + "," + String(Runtime_ms) + "," + String(speed, 4) + "," + String(lat, 8) + "," + String(lon, 8) + "," + String(Bat_mv) + "," + String(accuracy) + "\r\n";
        appendFile(SD, filename.c_str(), dataMessage.c_str());
        Last_loop_Act_time_ms = Act_time_ms;
        Act_time_ms = (esp_timer_get_time() - TimeTickerStart_us) / 1000;
        Runtime_ms = Act_time_ms - Last_loop_Act_time_ms;
        digitalWrite(LED_PIN, HIGH);
      }
      NextCall_ms = NextCall_ms + GPSdelay;
    }

    delay(1);
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
