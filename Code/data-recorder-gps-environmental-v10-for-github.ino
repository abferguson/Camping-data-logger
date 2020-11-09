/*
   Written by Tony Ferguson 7/7/2020
   
   This program periodically logs data from a GPS receiver and a BMP280 environmental sensor and stores
   the data onto a micro SD card.

   MCU: ESP32
   Beitian BN-880 GPS module with magnetic compass capability
   132x64 SSD1306 OLED display module
   BMP280 module
   MicroSD card adaptor (modified to be 3.3 V input instead of 5V)
   TP4056 3.7V Lipo battery charging module
   MCP1500-3302 linear voltage regulator (2)
   2n2222 NPN transistors (2)
   Slide switch (2)
   Power port
   5V solar cell (2)
   
   GPS on Serial2
   Compass (part of GPS) on I2C (not used)
   BME280 on I2C
   OLED on I2C
   SD card reader on SPI
   
   v1: general hardware debug with pin/transistor power switching and one RTC hardware wake pin
   v2: addition of SPIFFS(ESP32 variety), Zambretti, Arduino Time and Timezone
   v3: take out some power pins, work on output format and use boot cause data to choose execution path
   v4: add separate pin for sensing bluetooth enable switch
   v5: clean up, add data file erase function in bluetooth section, add more data saved and displayed
   v6: add smarter gps aquisition (timeout,shorten read time, compare to last saved epoch in SPIFFS)
       Serial output clean up, save cycle time to SD
   v7: more cleanup. better gps data acquisition handling. rotating screens on oled
       remove remainder of magnetic compass module code
   v8: add bluetooth BLE, fix voltage divider math, 100x average of analog read value
   v9: add boot into to datafile and output, estimated time when bad GPS time, boot noted in JSON data
   v10: add % battery value calculation
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GPS.h>
#include <BME280I2C.h>
#include <EnvironmentCalculations.h>
#include <U8g2lib.h>
#include <FS.h>
#include <SPI.h>
#include <SD.h>
#include <SPIFFS.h>
#include <TimeLib.h>
#include <Timezone.h>
#include <ArduinoJson.h>
#include "BluetoothSerial.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

int startTime = millis(); // used to calculate cycle time through program

int swVersion = 10;

time_t central; // central and utc uints for Timezone library. these need to be defined before setup()
time_t utc;
TimeChangeRule usCDT = {"CDT", Second, Sun, Mar, 2, -300};  //UTC - 4 hours
TimeChangeRule usCST = {"CST", First, Sun, Nov, 2, -360};   //UTC - 5 hours
Timezone usCentral(usCDT, usCST);
time_t estTime; // used in case GPS does not give a good time stamp during cycle

unsigned long current_timestamp;    // Actual timestamp read from NTPtime_t now;
unsigned long saved_timestamp;      // Timestamp stored in SPIFFS

// FORECAST RESULT
int accuracy;                       // Counter, if enough values for accurate forecasting
String ZambrettisWords;             // Final statement about weather forecast
String trend_in_words;              // Trend in words
float pressure_value[12];           // Array for the historical pressure values (6 hours, all 30 mins)
float pressure_difference[12];      // Array to calculate trend with pressure differences

// pin 4 RTC ext0 wakeup
int blueToothPin = 33;
int GPSPowerPin = 13;
int SDPowerPin = 12;
int BattVRead = 36;
int VoltDivPowerPin = 14;
// Serial 2 RX pin = 16;
// Serial 2 TX Pin = 17;

#define GPSSerial Serial2
Adafruit_GPS GPS(&GPSSerial);
uint32_t timer = millis();
uint32_t timer2 = millis();
#define GPSECHO false // false=turn off GPS data to serial console
bool gpsTimeGood = false;
//int GPSfixQuality = 0;
int GPSwaitTime = 90000;

int uS_TO_S_FACTOR = 1000000;  /* Conversion factor for micro seconds to seconds */
int TIME_TO_SLEEP = 600;        /* Time ESP32 will go to sleep (in seconds) */
RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR int rtcMonth = 0; // used by Zambretti algorithm
unsigned int savedEpoch = 0;

float tempDewpoint;
float tempDrybulb;
float relHumidity;
float tempHeatindex;
float pressure;
float seaLevelPres;
float absHumidity;
int rel_pressure_rounded;
bool bmefail = false;

float battVoltage = 0;
float percentBattery = 0;

BME280I2C::Settings settings(
   BME280::OSR_X1,
   BME280::OSR_X1,
   BME280::OSR_X1,
   BME280::Mode_Forced,
   BME280::StandbyTime_1000ms,
   BME280::Filter_16,
   BME280::SpiEnable_False,
   BME280I2C::I2CAddr_0x76  //76
);

BME280I2C bme(settings);    // Default : forced mode, standby time = 1000 ms

File myFile;
File dataFile;

U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

const char TEXT_RISING_FAST[]       = "rising quickly";
const char TEXT_RISING[]            = "rising";
const char TEXT_RISING_SLOW[]       = "rising slowly";
const char TEXT_STEADY[]            = "steady";
const char TEXT_FALLING_SLOW[]      = "falling slowly";
const char TEXT_FALLING[]           = "falling";
const char TEXT_FALLING_FAST[]      = "falling quickly";

const char TEXT_ZAMBRETTI_A[]       = "Settled, Fine Weather";
const char TEXT_ZAMBRETTI_B[]       = "Fine Weather";
const char TEXT_ZAMBRETTI_C[]       = "Becoming Fine";
const char TEXT_ZAMBRETTI_D[]       = "Fine, Becoming Less Settled";
const char TEXT_ZAMBRETTI_E[]       = "Fine, Possibly showers";
const char TEXT_ZAMBRETTI_F[]       = "Fairly Fine, Improving";
const char TEXT_ZAMBRETTI_G[]       = "Fairly Fine, Possibly showers early";
const char TEXT_ZAMBRETTI_H[]       = "Fairly Fine, Showers Later";
const char TEXT_ZAMBRETTI_I[]       = "Showery Early, Improving";
const char TEXT_ZAMBRETTI_J[]       = "Changeable, Improving";
const char TEXT_ZAMBRETTI_K[]       = "Fairly Fine, Showers likely";
const char TEXT_ZAMBRETTI_L[]       = "Rather Unsettled, Clearing Later";
const char TEXT_ZAMBRETTI_M[]       = "Unsettled, Probably Improving";
const char TEXT_ZAMBRETTI_N[]       = "Showery Bright Intervals";
const char TEXT_ZAMBRETTI_O[]       = "Showery Becoming Unsettled";
const char TEXT_ZAMBRETTI_P[]       = "Changeable, some rain";
const char TEXT_ZAMBRETTI_Q[]       = "Unsettled, short fine Intervals";
const char TEXT_ZAMBRETTI_R[]       = "Unsettled, Rain later";
const char TEXT_ZAMBRETTI_S[]       = "Unsettled, rain at times";
const char TEXT_ZAMBRETTI_T[]       = "Very Unsettled, Finer at times";
const char TEXT_ZAMBRETTI_U[]       = "Rain at times, Worse later";
const char TEXT_ZAMBRETTI_V[]       = "Rain at times, becoming very unsettled";
const char TEXT_ZAMBRETTI_W[]       = "Rain at Frequent Intervals";
const char TEXT_ZAMBRETTI_X[]       = "Very Unsettled, Rain";
const char TEXT_ZAMBRETTI_Y[]       = "Stormy, possibly improving";
const char TEXT_ZAMBRETTI_Z[]       = "Stormy, much rain";
const char TEXT_ZAMBRETTI_DEFAULT[] = "Sorry, no forecast for the moment";

BluetoothSerial SerialBT;
String Stdbtmessage = "";
bool newStdbtmessage = false;
const size_t capacity = JSON_OBJECT_SIZE(30) + 500;
DynamicJsonDocument jsondata(capacity);

BLECharacteristic *pCharacteristic;
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
bool deviceConnected = false;
String BLEmessage = "";
bool newBLEmessage = false;

unsigned long currentOLEDmillis = millis();
unsigned long delayOLEDmillis = 4000;  // interval between OLED screen updated
boolean mounted = false;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("BLE connected");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("BLE disconnected");
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string rxValue = pCharacteristic->getValue();

    if (rxValue.length() > 0) {
      Serial.print("Received new BLErxValue: ");

      String tempBLErxValue;
      for (int i = 0; i < rxValue.length();) {
        tempBLErxValue += rxValue[i];
        if(tempBLErxValue == "env" || tempBLErxValue == "gps" || tempBLErxValue == "now" || tempBLErxValue == "help" || tempBLErxValue == "misc" || tempBLErxValue == "erase") {
          BLEmessage = tempBLErxValue; // ignore any LF or CR  
        }
        i++;
      } // for
      BLEmessage += "BLE";
      Serial.println(BLEmessage);
      newBLEmessage = true;
    } // if
  } // void onWrite
}; // class MyCallbacks:

//--------------------------------------------------------------------
void setup() {
//  WiFi.disconnect(true);
//  WiFi.mode(WIFI_OFF);
  btStop();
  
  pinMode(BattVRead, INPUT);
  pinMode(GPSPowerPin, OUTPUT);
  pinMode(SDPowerPin, OUTPUT);
  pinMode(VoltDivPowerPin, OUTPUT);
  pinMode(blueToothPin, INPUT_PULLDOWN); // defining this as pin4 input causes issues with the wakeup working correctly
  
  digitalWrite(VoltDivPowerPin, HIGH); // connect GND (GPS and voltage divider)
  digitalWrite(SDPowerPin, HIGH); // switch power on to GPS module
  delay(20);
  
  Wire.begin();
  Wire.setClock(400000);  // speed up i2c wire speed. it works
  
  Serial.begin(115200);
  while(!Serial) // wait for Serial to become available
    ;
  Serial.print("");
  Serial.println("--------------------------------------------------------");

  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));
  Serial.println("rtcMonth: " + String(rtcMonth));

  printWakeupReason();
  
  mounted = SPIFFS.begin(); // turn on SPIFFS before bluetooth or regular section

  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR); // enable wake up when a set time has past
  Serial.println("Setup: ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_4,1); //1 = High, 0 = Low // enable wake up when pin 4 goes high


  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  switch(wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0: 
      if (!SD.begin()) {
        Serial.println("SD module initialization failed!");
        return;
      }
      Serial.println("######### external wake #########");


      SerialBT.begin(" Std Datalogger"); //Bluetooth device name
      Serial.println("Ready for Standard Bluetooth");

 
      // Create the BLE Device
      BLEDevice::init(" BLE Datalogger"); // Give it a name

      // Create the BLE Server
      BLEServer *pServer = BLEDevice::createServer();
      pServer->setCallbacks(new MyServerCallbacks());

      // Create the BLE Service
      BLEService *pService = pServer->createService(SERVICE_UUID);

      // Create a BLE Characteristic
      pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_TX,
        BLECharacteristic::PROPERTY_NOTIFY
      );
                      
      pCharacteristic->addDescriptor(new BLE2902());

      BLECharacteristic *pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_RX,
        BLECharacteristic::PROPERTY_WRITE
      );

      pCharacteristic->setCallbacks(new MyCallbacks());

      // Start the service
      pService->start();

      // Start advertising
      pServer->getAdvertising()->start();
      Serial.println("Ready for Bluetooth BLE");

     
      blueTooth();
      
      oled.noDisplay();  // turn off OLED display

      btStop();
      SerialBT.flush();
      SerialBT.disconnect();
      SerialBT.end();
    break;

  } // switch

// Turn on GPS module after bluetooth section
  digitalWrite(GPSPowerPin, HIGH); // switch on ground connection to GPS module
  delay(500); // let power settle

  
  Serial.println();
  checkSPIFFS();

// Read battery voltage
  readBattVoltage();
  
// GPS
  readSavedEpoch();
  Serial.print("savedEpoch: ");
  Serial.println(savedEpoch);
  readGPS();
  GPS.standby();

// Timezone adjustment
  // set the Time to the latest GPS reading
  Serial.println();
  Serial.println("Date and time...");
  timezoneAdjust();

// BME280
  BME280Read();
  
// Zambretti depends on GPS time and BME pressure
  Spiffs_Zambretti_update();
  
// SD card write
  saveDataToSD();
  
  writeSavedEpoch(); // saves local time to SD
  
  digitalWrite(SDPowerPin, LOW); // switch power off to SD module
  digitalWrite(GPSPowerPin, LOW); // switch power off to GPS module
  digitalWrite(VoltDivPowerPin, LOW); // disconnect GND to buck converter (power for GPS, voltage divider)
  
  Serial.println();
  Serial.println("end of setup()");
  Serial.println();
  Serial.println("Setup: ESP32 going to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");

  Serial.println();
  Serial.print("Elapsed time through setup(): ");
  Serial.println(millis() - startTime);
  Serial.flush(); 

  esp_deep_sleep_start();
} // setup

//==================================================
void loop() {
} // loop()
//==================================================

void blueTooth() {
  bool blueToothMode = true;

  startOLED();
  int screenNumber = 1;
  currentOLEDmillis = millis() - delayOLEDmillis;
  
  while(blueToothMode) { // exit when bluetooth switch goes off
    if((unsigned long)(millis() - currentOLEDmillis) > delayOLEDmillis) {
      BME280Read();
      OLEDScreen(screenNumber);
      screenNumber++;
      if(screenNumber > 3) {
        screenNumber = 1;
      }
      currentOLEDmillis = millis();
    }
   
    if(SerialBT.available()) { // assemble String btmessage from standard bluetooth input
      char incomingChar = SerialBT.read();
      if (incomingChar != '\n'){
        Stdbtmessage += String(incomingChar);
      }
      else{
        Stdbtmessage = "";
      }
      Serial.write(incomingChar);  
    } // BT.available
  
  // Check SD related btmessage and control output accordingly
    if(Stdbtmessage == "env" || Stdbtmessage == "gps" || Stdbtmessage == "now"  || Stdbtmessage == "help" || Stdbtmessage == "misc" || Stdbtmessage == "erase"){
      Stdbtmessage += "STD"; // add STD to message to delineate
      newStdbtmessage = true;
      Serial.println("calling bluetoothReply()");
      bluetoothReply(Stdbtmessage);
      Serial.println("returned from bluetoothReply()");
      newStdbtmessage = false;
      Stdbtmessage = "";
    }

    if(deviceConnected && newBLEmessage) {
      Serial.println("calling bluetoothReply()");
      bluetoothReply(BLEmessage);
      Serial.println("returned from bluetoothReply()");
      newBLEmessage = false;
      BLEmessage = "";
    }

    blueToothMode = digitalRead(blueToothPin); // read blueToothPin
  } // while blueToothMode
} // blueTooth()

void checkSPIFFS() {
  Serial.println("SPIFFS FS check...");
  if (!mounted) {
    Serial.println("  FS not formatted. Doing that now... (can last up to 30 sec).");
    SPIFFS.format();
    Serial.println("  FS formatted...");
    SPIFFS.begin();
  }
  else {
    Serial.println("  SPIFFS FS is already formatted.");
  }
  Serial.println("finish SPIFFS FS check"); 
} // void checkSPIFFS()

void bluetoothReply(String btmessage) {
  String replyStr = "";
  if(btmessage == "nowSTD" || btmessage == "nowBLE") {
    BME280Read();
    readBattVoltage();
    ReadFromSPIFFS();
    ZambrettisWords = ZambrettiSays(char(ZambrettiLetter()));
  
    replyStr = "Temperature: " + String(tempDrybulb,1) + " F\n";
    replyStr += "%Rel Humidity: " + String(relHumidity,1) + "\n";
    replyStr += "Dewpoint: " + String(tempDewpoint,1) + " F\n";
    replyStr += "Pressure: " + String(seaLevelPres,1) + " hPa\n";
    replyStr += "Absolute Humidity: " + String(absHumidity,1) + " g/m^3\n";
    replyStr += "Zambretti: " + ZambrettisWords + "\n";
    replyStr += "Trend: " + trend_in_words + "\n";
    replyStr += "Battery voltage: " + String(battVoltage,1) + " V\n\n";
    sendToBluetooth(replyStr);
  } // if now call

  Serial.println("Opening file for read only");
  myFile = SD.open("/data.txt", FILE_READ);
// for values that are time related only...  
  if (myFile && (btmessage == "envSTD" || btmessage == "envBLE" || btmessage == "gpsSTD" || btmessage == "gpsBLE" || btmessage == "miscSTD" || btmessage == "miscBLE")) {
    String inputString = "";
    while (myFile.available()) {
      inputString = myFile.readStringUntil('\n');
      deserializeJson(jsondata,inputString); // convert String to JSON
//      serializeJson(jsondata,Serial); // print JSON to serial

      int bootcnt = jsondata["bootcnt"];
      if(bootcnt == 1) {
        replyStr += "\nBoot\n";
      }

// Time output
      float hr = jsondata["hr"]; // preface GPS and Envir data with time and date
      float minu = jsondata["min"];
      float sec = jsondata["sec"];
      float month1 = jsondata["month"];
      float day1 = jsondata["day"];
      float yr = jsondata["yr"];
      bool timeGood = jsondata["timegood"];
      // start new line string below
      replyStr = String(month1,0) + "/" + String(day1,0) + "/" + String(yr,0) +" " + String(hr,0) + ":";
      if(minu < 10.0)
        replyStr += "0";
      int tempminu = int(minu);
      replyStr += String(tempminu) + ":";
      if(sec < 10.0)
        replyStr += "0";
      int tempsec = int(sec);
      replyStr += String(tempsec);

      if(!timeGood) { // denote estimated time with asterisk after time stamp
        replyStr += "*";
      }

// Environmental output
      if(btmessage == "envSTD" || btmessage == "envBLE") {
        float drybulb = jsondata["drybulb"];
        float relhumid = jsondata["relhumid"];
        float dewpt = jsondata["dewpt"];
        float seapress = jsondata["seapress"];
        float abshumid = jsondata["abshumid"];
        const char* zambretti = jsondata["zwords"];
        const char* trend = jsondata["trend"];
        replyStr += " Temp " + String(drybulb,0) + "F";
        replyStr += ", %RH " + String(relhumid,0);
        replyStr += ", Dewpt " + String(dewpt,0) +"F";
        replyStr += ", Ahum " + String(abshumid,1) + "g/m^3";
        replyStr += ", Press " + String(seapress,1) + "hPa\n";
        replyStr +=  String(zambretti) + ", " + String(trend) + "\n";
      } // if env call

// GPS output
      if(btmessage == "gpsSTD" || btmessage == "gpsBLE" ) {
        float lat1 = jsondata["lat"];
        lat1 = abs(lat1);
        float lon1 = jsondata["lon"];
        lon1 = abs(lon1);
        char ns = jsondata["NS"];
        char ew = jsondata["EW"];
        int fixqual = jsondata["quality"];
        int numsats = jsondata["sats"];
//        Serial.print("jsondata sats: ");
//        Serial.println(numsats);
        replyStr += "  LAT " + String(lat1,6) +  ns;
        replyStr += ",  LON " + String(lon1,6) + ew;
        replyStr += ", FixQ " + String(fixqual);
        replyStr += ", #Sat " + String(numsats) + "\n";
      } // if gps call

// Misc output
      if(btmessage == "miscSTD" || btmessage == "miscBLE") {
        float acqtime = jsondata["acqtime"];
        float volt = jsondata["volt"];
        float alt = jsondata["alt"];
        float fixed = jsondata["fix"];
        float bootcnt = jsondata["bootcnt"];
//        Serial.println("jsondata misc");
        replyStr += "  Altitude " + String(alt,1) + "m";
        replyStr += ", Acq Time " + String(acqtime,1) + "sec";
        replyStr += ", Fix " + String(fixed,0);
        replyStr += ", Voltage " + String(volt,3) + "V";
        replyStr += ", BootCNT " + String(bootcnt,0) + "\n";
      } // if misc

      sendToBluetooth(replyStr);
      inputString = ""; // reset inputString to blank
    } // while (myFile.available()...

    btmessage = "";
    myFile.close(); 
  } // if myFile exists and time related output requested...

  else {
    Serial.println("error opening file for read only");
  }

  if(btmessage == "eraseSTD" || btmessage == "eraseBLE") {
    Serial.print("Erasing data file on SD card...");
    myFile = SD.open("/data.txt", FILE_WRITE);
    myFile.close();  
    Serial.print("data file erased");
    replyStr = "Data file erased\n\n";
    sendToBluetooth(replyStr);
  }

  if(btmessage == "helpSTD" || btmessage == "helpBLE") {
    replyStr = "Commands: variables\n";
    replyStr += "env: temp, humidity, pressure, forecast, battery\n";
    replyStr += "gps: latitude, longitude\n";
    replyStr += "now: current environmentals, battery\n";
    replyStr += "help: this data\n";
    replyStr += "misc: altitude, acquisition time, GPS fix, battery voltage, boot count\n";
    replyStr += "erase: clear data file\n\n";
    sendToBluetooth(replyStr);
  }
  
} // void bluetoothReply()

void sendToBluetooth(String replyStr) {
// sends a string to either standard bluetooth or BLE
  if(newStdbtmessage) {
    SerialBT.print(replyStr);
  }

  if(newBLEmessage) {
    int btmsgLength = replyStr.length();
    char replyChar[btmsgLength+1];
    String msgFragStr = "";
    char msgFragChar[20];
    replyStr.toCharArray(replyChar,btmsgLength+1);
      
    for(int i = 0; i < btmsgLength; i++) {
      msgFragStr += replyChar[i];
      if(msgFragStr.length() == 19) {
        msgFragStr.toCharArray(msgFragChar,20);
        pCharacteristic->setValue(msgFragChar);
        pCharacteristic->notify();
        msgFragStr = "";
      }
    } // for

    if(msgFragStr.length() > 0) { // capture last fragment < 
      msgFragStr.toCharArray(msgFragChar,20);
      pCharacteristic->setValue(msgFragChar);
      pCharacteristic->notify();
      msgFragStr = "";
    }   
  }// if newBLEmessage
} // void sendToBluetooth

void timezoneAdjust() {
  Serial.println("  Time prior to adjustment");
  printTime();  // print time prior to adjusting with GPS time information

  int Year = int(GPS.year);
  byte Month = byte(GPS.month);
  byte Day = byte(GPS.day);
  byte Hour = byte(GPS.hour);
  byte Minute = byte(GPS.minute);
  byte Second = byte(GPS.seconds);
  setTime(Hour, Minute, Second, Day, Month, Year); // sets ESP32 RTC time to GPS provided UTC
  
  utc = now();  //current time from the Time Library
  central = usCentral.toLocal(utc);
  Serial.print("  Central time epoch with CDT/CST adjust: ");
  Serial.println(central);
  
  int offset; // use Timezone library to determine DST or CST offset hours from UTC
  if(usCentral.locIsDST(central) > 0) {
    offset = -5;
  }
  else {
    offset = -6;
  }
  adjustTime(offset * SECS_PER_HOUR); // adjust ESP32 RTC time using DST/CST offset above

  current_timestamp = now(); // provides local time since now() is timezone adjusted original UTC
  Serial.println("  After ESP32 RTC time adjustment");
  printTime(); // print time after adjusting with GPS time information

  rtcMonth = int(month()); // update local time rctMonth in RTC memory for Zambretti
  Serial.println("  in timezoneAdjust, rtcMonth: " + String(rtcMonth));
  Serial.println("finish date and time");
} // timezoneAdjust

void startOLED() {
//  Serial.println();
  oled.begin();
//  Serial.println("OLED started");
  oled.enableUTF8Print(); 
} // startOLED() 

void saveDataToSD() {
  Serial.println();
  Serial.println("saveDataToSD...");

  if (!SD.begin(SS)) {
    Serial.println("  SD module initialization failed!");
    return;
  }
  
  Serial.println("  SD module initialization done.");

  StaticJsonDocument<600> datalogJSON;
/*
  String tempMin = String(minute());
  String tempSec = String(second());
  if(minute() < 10)
    tempMin = "0" + String(minute());
  if(second() < 10)
    tempSec = "0" + String(second());

  Serial.print("tempMin: ");
  Serial.println(tempMin);
  Serial.print("minute(): ");
  Serial.println(minute());
  Serial.print("tempSec: ");
  Serial.println(tempSec);
  Serial.print("second(): ");
  Serial.println(second());
  */

  if(now() < savedEpoch) { // if the GPS time is not good then estimate
    estTime = savedEpoch + (unsigned int)(millis() - startTime)/1000;
    setTime(estTime); // sets ESP32 RTC time to GPS provided UTC
  }
  
  datalogJSON["epoch"] = current_timestamp;
  datalogJSON["hr"] = hour();
  datalogJSON["min"] = minute();
  datalogJSON["sec"] = second();
  datalogJSON["month"] = month();
  datalogJSON["day"] = day();
  datalogJSON["yr"] = year();
  datalogJSON["drybulb"] = tempDrybulb;
  datalogJSON["dewpt"] = tempDewpoint;
  datalogJSON["relhumid"] = relHumidity;
  datalogJSON["seapress"] = seaLevelPres;
  datalogJSON["abshumid"] = absHumidity;
  datalogJSON["zwords"] = ZambrettisWords;
  datalogJSON["trend"] = trend_in_words;
  if(gpsTimeGood) { // if valid GPS data not aqcuired in time
    datalogJSON["lat"] = (float)GPS.latitude_fixed/10000000.0;
    datalogJSON["NS"] = (char)GPS.lat;
    datalogJSON["lon"] = (float)GPS.longitude_fixed/10000000.0;
    datalogJSON["EW"] = (char)GPS.lon;
  }
  else {
    datalogJSON["lat"] = 0;
    datalogJSON["NS"] = char('-');
    datalogJSON["lon"] = 0;
    datalogJSON["EW"] = char('-');
  }
  datalogJSON["alt"] = (float)GPS.altitude;
  datalogJSON["acqtime"] = (float)(millis() - startTime)/1000;
  datalogJSON["fix"] = GPS.fix;
  datalogJSON["quality"] = (int)GPS.fixquality;
  datalogJSON["sats"] = (int)GPS.satellites;
  datalogJSON["volt"] = battVoltage;
  datalogJSON["bootcnt"] = bootCount;
  datalogJSON["timegood"] = gpsTimeGood;
  datalogJSON["percbatt"] = percentBattery;

  Serial.println("  Opening data file for write/append");
  dataFile = SD.open("/data.txt", FILE_APPEND);

  if (!dataFile) {
    SD.open("/data.txt", FILE_WRITE);
  }
  serializeJson(datalogJSON, dataFile);
  dataFile.println();
  serializeJson(datalogJSON, Serial); // print JSON to serial

  dataFile.close();
  delay(500); // wait for SD to close. see if there is an SD function for this
    
  Serial.println("finish saveDataToSD");
} // saveDataToSD()

void readGPS() {
  Serial.println();
  Serial.println("GPS module...");
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  delay(1000);

  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);

  current_timestamp = now();
  unsigned long startAquireTime = millis();

  while(!gpsTimeGood && (unsigned long)(millis() - startAquireTime < GPSwaitTime)) { // limit to GPSwaitTime milliseconds to aquire GPS fix
    printGPSdata();
  } // while 

  Serial.println("finish GPS module");
} // void readGPS()

void printGPSdata() {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    // Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  } // if .newNMEAreceived()

  if (millis() - timer2 > 5000 ) {
    int Year = int(GPS.year);
    byte Month = byte(GPS.month);
    byte Day = byte(GPS.day);
    byte Hour = byte(GPS.hour);
    byte Minute = byte(GPS.minute);
    byte Second = byte(GPS.seconds);
    setTime(Hour, Minute, Second, Day, Month, Year);
  
    Serial.println(GPS.milliseconds);
    Serial.print("  Date: ");
    Serial.print(GPS.month, DEC);
    Serial.print('/');
    Serial.print(GPS.day, DEC);
    Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("  Fix: ");
    Serial.print((int)GPS.fix);
    Serial.print(", fix quality: ");
    Serial.println((int)GPS.fixquality);
    Serial.print("  Date: ");
    Serial.print(GPS.month, DEC);
    Serial.print('/');
    Serial.print(GPS.day, DEC);
    Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("  Fix: ");
    Serial.print((int)GPS.fix);
    Serial.print(", fix quality: ");
    Serial.println((int)GPS.fixquality);
    Serial.print("  Location: ");
    Serial.print(GPS.latitude, 4);
    Serial.print(GPS.lat);
    Serial.print(", ");
    Serial.print(GPS.longitude, 4);
    Serial.println(GPS.lon);
    timer2 = millis();
   }
   
  if (millis() - timer > 1000 && (int)GPS.fix > 0 && (int)GPS.fixquality > 0) {
    int Year = int(GPS.year);
    byte Month = byte(GPS.month);
    byte Day = byte(GPS.day);
    byte Hour = byte(GPS.hour);
    byte Minute = byte(GPS.minute);
    byte Second = byte(GPS.seconds);
    setTime(Hour, Minute, Second, Day, Month, Year);
  
    time_t central, utc; // get utc for Timezone library
    current_timestamp = now();
//    GPSfixQuality = (int)GPS.fixquality;

    if (current_timestamp > savedEpoch) {
      gpsTimeGood = true; // will cause program to move on
    } // if all GPS data looks good
    
    Serial.print("  Time: ");
    if (GPS.hour < 10) { Serial.print('0'); }
    Serial.print(GPS.hour, DEC); Serial.print(':');
    if (GPS.minute < 10) { Serial.print('0'); }
    Serial.print(GPS.minute, DEC); Serial.print(':');
    if (GPS.seconds < 10) { Serial.print('0'); }
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    if (GPS.milliseconds < 10) {
      Serial.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      Serial.print("0");
    } // else
    
    Serial.println(GPS.milliseconds);
    Serial.print("  Date: ");
    Serial.print(GPS.month, DEC);
    Serial.print('/');
    Serial.print(GPS.day, DEC);
    Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("  Fix: ");
    Serial.print((int)GPS.fix);
    Serial.print(", fix quality: ");
    Serial.println((int)GPS.fixquality);
    Serial.print("  Location: ");
    Serial.print(GPS.latitude, 4);
    Serial.print(GPS.lat);
    Serial.print(", ");
    Serial.print(GPS.longitude, 4);
    Serial.println(GPS.lon);
//    Serial.print("  Speed (knots): ");
//    Serial.println(GPS.speed);
//    Serial.print("  Angle: ");
//    Serial.println(GPS.angle);
    Serial.print("  Altitude: ");
    Serial.println(GPS.altitude);
    Serial.print("  Satellites: ");
    Serial.println((int)GPS.satellites); 
       
    timer = millis(); // reset the timer value
  } // if timer
} // printGPSdata()

void readBattVoltage() {
  Serial.println();
  Serial.print("Battery voltage reading: ");
  int battvread = 0;
  for(int i=1; i < 1001; i++) {
    battvread += analogRead(BattVRead);
  }
  battvread = battvread/1000;
  Serial.print(battvread);
  Serial.print(", ");
//  battVoltage = battvread*3.3/4095.0*(10040.0+6750.0)/10040.0; // original
//  battVoltage = battvread*3.3/4095.0*(99800.0+46800.0)/99800.0; //v2
  battVoltage = (battvread+614.43)/925.67;  

  Serial.print(battVoltage);
  Serial.println("v");

  float estInvPowerTime = -8525.37547*battVoltage + 35956.78317;
  if(estInvPowerTime < 0) {
    estInvPowerTime = 0;
  }
  if(estInvPowerTime > 6804.0) {
    estInvPowerTime = 6804.0;
  }
  Serial.print("estimated inverse time on battery: ");
  Serial.print(estInvPowerTime);
  Serial.println(" sec");
  percentBattery = (6804.0-estInvPowerTime)/6804.0*100.0;
  Serial.print("% battery: ");
  Serial.println(percentBattery);
  
} // readBattVoltage()

void printTime(){
  Serial.print("    epoch, now(): ");
  Serial.println(now());
  Serial.print("    ");
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(month());
  Serial.print("/");
  Serial.print(day());
  Serial.print("/");
  Serial.println(year()); 
} // printTime()

void printDigits(int digits) {
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
} // void printDigits()

void printWakeupReason() {
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  switch(wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
} // void printWakeupReason()

void BME280Read() {
  Serial.println();
  Serial.println("BME280 module...  ");
  Serial.println("  Turning on BME280 sensor.");
  int readcount = 0;
  while(!bme.begin() && readcount < 11) {
    Serial.println("  Could not find BME280 sensor!");
    delay(500);
    readcount++;
  }
  if(readcount < 11) {
    Serial.println("  BME280 sensor Begin successful");
    bmefail = false;
  }
  else {
    Serial.println("  BME280 sensor Begin not successful");
    bmefail = true;
  }

  switch(bme.chipModel()) {
    case BME280::ChipModel_BME280:
      Serial.print("  Found BME280 sensor:");
      break;
    case BME280::ChipModel_BMP280:
      Serial.print("  Found BMP280 sensor:");
      break;
    default:
      Serial.print("  Found UNKNOWN sensor. Error!:")
      ;
  }
  Serial.print(" chip model: ");
  Serial.println(bme.chipModel());
  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_hPa);

  bme.read(pressure, tempDrybulb, relHumidity, tempUnit, presUnit);
  if(bootCount == 1) { // read BMP280 again, after delay, at first boot to get around spurious initial readings
    delay(1100);
    bme.read(pressure, tempDrybulb, relHumidity, tempUnit, presUnit);
  }

  float referencePressure = 1013.25;  // hPa local QFF (official meteor-station reading)
  float outdoorTemp = tempDrybulb;
  float barometerAltitude = 283.5;  // meters ... map readings + barometer position
  EnvironmentCalculations::AltitudeUnit envAltUnit  =  EnvironmentCalculations::AltitudeUnit_Meters;
  EnvironmentCalculations::TempUnit     envTempUnit =  EnvironmentCalculations::TempUnit_Celsius;

  /// To get correct local altitude/height (QNE) the reference Pressure
  ///    should be taken from meteorologic messages (QNH or QFF)
  // Lake Elmo altitude is 930 ft / 283.46 m
  float altitude = EnvironmentCalculations::Altitude(pressure, envAltUnit, referencePressure, tempDrybulb, envTempUnit);

  tempDewpoint = EnvironmentCalculations::DewPoint(tempDrybulb, relHumidity, envTempUnit);

  /// To get correct seaLevel pressure (QNH, QFF)
  ///    the altitude value should be independent on measured pressure.
  /// It is necessary to use fixed altitude point e.g. the altitude of barometer read in a map
  
  seaLevelPres = EnvironmentCalculations::EquivalentSeaLevelPressure(barometerAltitude, tempDrybulb, pressure, envAltUnit, envTempUnit);
  absHumidity = EnvironmentCalculations::AbsoluteHumidity(tempDrybulb, relHumidity, envTempUnit);

  tempHeatindex = EnvironmentCalculations::HeatIndex(tempDrybulb, relHumidity, envTempUnit);

  if (isnan(tempDrybulb) || isnan(pressure) || isnan(relHumidity)) {
//    Serial.println("BME280 malfunction");
    bmefail = true;
  }
  else {
    bmefail = false;
    Serial.println("  BME280:");
    Serial.print("    Temperature: ");
    Serial.print(tempDrybulb); Serial.print(String(tempUnit == BME280::TempUnit_Celsius ? " C  " :" F  "));
    tempDrybulb=tempDrybulb*9/5+32;
    Serial.print(tempDrybulb); Serial.println(" F");
    Serial.print("    DP Temperature: ");
    Serial.print(tempDewpoint); Serial.print(String(tempUnit == BME280::TempUnit_Celsius ? " C  " :" F  "));
    tempDewpoint=tempDewpoint*9/5+32;
    Serial.print(tempDewpoint); Serial.println(" F");
    Serial.print("    Humidity: ");
    Serial.print(relHumidity); Serial.println(" %");
    Serial.print("    Heat index: ");
    Serial.print(tempHeatindex); Serial.print(String(tempUnit == BME280::TempUnit_Celsius ? " C  " :" F  "));
    tempHeatindex=tempHeatindex*9/5+32;
    Serial.print(tempHeatindex); Serial.println(" F");
    Serial.print("    Pressure: ");
    Serial.print(pressure); Serial.print(String(presUnit == BME280::PresUnit_hPa ? " hPa  " : " Pa  "));
    Serial.print(pressure*= 0.02953); Serial.println(" inHg");
    rel_pressure_rounded=(int)(seaLevelPres);
    Serial.print("    rel pressure_rounded: ");
    Serial.print(rel_pressure_rounded); Serial.println(" hPa");
    Serial.print("    Altitude: ");
    Serial.print(altitude); Serial.print((envAltUnit == EnvironmentCalculations::AltitudeUnit_Meters ? " m  " : " ft  "));
    altitude=altitude*3.28084;
    Serial.print(altitude); Serial.println(" ft");
    Serial.print("    Sea Level Pressure: ");
    Serial.print(seaLevelPres); Serial.println(String(presUnit == BME280::PresUnit_hPa ? " hPa" : " Pa"));
    Serial.print("    Absolute Humidity: ");
    Serial.print(absHumidity); Serial.println(" g/cm^3");
  }
  
// set BME280 into sleep mode. This is likely redundant since forced
// mode is supposed to do it, but do it anyway.
  BME280I2C::Settings settings( // this section works with bme280 Tyler Glenn via arduino library manager
    BME280::OSR_X1, // 1 times sampling
    BME280::OSR_X1,
    BME280::OSR_X1,
    BME280::Mode_Sleep,
    BME280::StandbyTime_1000ms,
    BME280::Filter_16,
    BME280::SpiEnable_False,
    BME280I2C::I2CAddr_0x76
  );

  BME280I2C bme(settings); // bme280-master version tyler glenn v3.0
  bme.begin();
  Serial.println("  BME280 put to sleep");
  Serial.println("finish BME module");
} // void BME280Read()

void OLEDScreen(int screenNumber) {
  Serial.print("OLEDScreen number: ");
  Serial.println(screenNumber);
  oled.clear();
//  oled.setContrast(10);  // does not seem to do anything
  oled.drawRFrame(0,0,128,16,3);  // frame around yellow area with rounded corners
  oled.setFont(u8g2_font_6x13_tf); // good size for yellow area with border
//  oled.setFont(u8g2_font_7x14_tn);  // office does not print
//  oled.setFont( u8g2_font_9x18_tn); // 14 pixel height
//  oled.setFont(u8g2_font_courB24_tf); //20 pixel height
  oled.setFontRefHeightExtendedText();
  oled.setDrawColor(1);
  oled.setFontPosTop();
  oled.setFontDirection(0);
  oled.drawFrame(0,16,128,48);  // frame around blue area
//  oled.setFont(u8g2_font_6x10_tf);
//  oled.setFont(u8g2_font_6x13_tf);
//  oled.setFont(u8g2_font_7x14_tn);  // text does not print
  oled.setFont(u8g2_font_9x18_tr); // good. 14 pixel height. tr: transparent, ASCII code 32-127
//  oled.setFont(u8g2_font_9x19B_mn); // not supported
//  oled.setFont(u8g2_font_10x20_tr); // too big

  switch(screenNumber) {
    case 1:
      oled.setCursor(3,2);  // slight right offset to get text away from left frame edge
      oled.print("Weather 1");
      oled.setCursor(4,18);
      oled.println(" Temp:" + String(tempDrybulb, 1) + " F");
      oled.setCursor(4,33);
      oled.println("Dewpt:" + String(tempDewpoint, 1)+ " F");
      oled.setCursor(4,48);
      oled.println("  %RH:" + String(relHumidity, 1));
      oled.sendBuffer();
      break;
      
    case 2:
      readBattVoltage();
      oled.setCursor(3,2);  // slight right offset to get text away from left frame edge
      oled.print("Weather 2");
      oled.setCursor(4,18);
      oled.println("AH:" + String(absHumidity, 1) + " g/m^3");
      oled.setCursor(4,33);
      oled.println(" P:" + String(seaLevelPres, 1) + " hPa");
      oled.setCursor(4,48);
      oled.println("Bt:" + String(percentBattery,1) + " %");
      oled.sendBuffer();
      break;
      
    case 3:
      oled.setFont(u8g2_font_6x10_tf);
      ReadFromSPIFFS();              //read stored values and update data if more recent data is available
      ZambrettisWords = ZambrettiSays(char(ZambrettiLetter()));
      char zwidthchar[40];
      ZambrettisWords.toCharArray(zwidthchar,40);
    
      oled.setCursor(3,2);  // slight right offset to get text away from left frame edge
      oled.print("Forecast");
      oled.setCursor(4,18);

      for(int i = 0; i< 20; i++) { // breakup Zambrettis words into two lines
        oled.print(zwidthchar[i]);
      }
      oled.setCursor(4,30);
      for(int i = 20; i< 39; i++) {
        oled.print(zwidthchar[i]);
      }

      oled.setCursor(4,46);
      oled.println("Trend: " + trend_in_words);
      oled.sendBuffer();
      break;
    } // switch

//  Serial.println("Finished OLEDScreen");
} // void OLEDScreen()

void Spiffs_Zambretti_update() {
  Serial.println();
  Serial.println("Spiffs_Zambretti_update()...  ");
  ReadFromSPIFFS();              //read stored values and update data if more recent data is available
  Serial.println();
  Serial.print("Zambretti data timestamp difference: ");
  Serial.println(current_timestamp - saved_timestamp);

  if (current_timestamp - saved_timestamp > 21600){    // last save older than 6 hours -> re-initialize values
    FirstTimeRun();
  }
  else if (current_timestamp - saved_timestamp > 1800){ // it is time for pressure update (1800 sec = 30 min)
    
    for (int i = 11; i >= 1; i = i -1) {
      pressure_value[i] = pressure_value[i-1];          // shifting values one to the right
  }
   
  pressure_value[0] = rel_pressure_rounded;             // updating with acutal rel pressure (newest value)
  
  if (accuracy < 12) {
    accuracy = accuracy + 1;                            // one value more -> accuracy rises (up to 12 = 100%)
    }
    WriteToSPIFFS(current_timestamp);                   // update timestamp on storage
  }
  else {         
    WriteToSPIFFS(saved_timestamp);                     // do not update timestamp on storage
  }

  int accuracy_in_percent = accuracy*94/12;            // 94% is the max predicion accuracy of Zambretti

  ZambrettisWords = ZambrettiSays(char(ZambrettiLetter()));
  
  Serial.print("  Zambretti says: ");
  Serial.print(ZambrettisWords);
  Serial.print(", ");
  Serial.println(trend_in_words);
  Serial.print("  Prediction accuracy: ");
  Serial.print(accuracy_in_percent);
  Serial.println("%");
  if (accuracy < 12){
    Serial.println("  Not enough weather data yet.");
    Serial.print("  We need ");
    Serial.print((12 - accuracy) / 2);
    Serial.println(" hours more to get sufficient data.");
  }
  Serial.println("  SPIFFS data for Zambetti algorithm saved");
  Serial.println("finish Spiffs_Sambretti_update()");
} // void Spiffs_Sambretti_update()

void ReadFromSPIFFS() {
  Serial.println("ReadFromSPIFFS()...  ");
  char filename [] = "/data.txt";
  File myDataFile = SPIFFS.open(filename, "r");
  if (!myDataFile) {
    Serial.println("  readspiffs: Failed to open file");
    FirstTimeRun(); // if no file, initialize
  }
  
  String temp_data;
  temp_data = myDataFile.readStringUntil('\n');  
  saved_timestamp = temp_data.toInt();
  Serial.print("  Timestamp from SPIFFS: ");  Serial.println(saved_timestamp);
  
  temp_data = myDataFile.readStringUntil('\n');  
  accuracy = temp_data.toInt();
  Serial.print("  Accuracy value read from SPIFFS: ");  Serial.println(accuracy);

//  Serial.print("  Last 12 saved SPIFFS pressure values: ");
  for (int i = 0; i <= 11; i++) {
    temp_data = myDataFile.readStringUntil('\n');
    pressure_value[i] = temp_data.toInt();
//    Serial.print(pressure_value[i]);
//    Serial.print("; ");
  }
  myDataFile.close();
  Serial.println();
  Serial.println("finish ReadFromSPIFFS()");
} // void ReadFromSPIFFS()

void WriteToSPIFFS(int write_timestamp) {
  Serial.println();
  Serial.println("WriteToSPIFFS()...  ");
  char filename [] = "/data.txt";
  File myDataFile = SPIFFS.open(filename, "w"); // Open file for writing (appending)
  if (!myDataFile) {
    Serial.println("  writespiffs: Failed to open file");
  }
  
  Serial.println("  Writing Zambretti data to SPIFFS");
  
  myDataFile.println(write_timestamp); // save timestamp and accuracy to /data.txt
  myDataFile.println(accuracy);
  
  for ( int i = 0; i <= 11; i++) {
    myDataFile.println(pressure_value[i]); // save pressure data with updated values
 }
  myDataFile.close();
  
  Serial.println("  Zambretti data written. Now reading file again.");
  myDataFile = SPIFFS.open(filename, "r");
  Serial.print("  Found in /data.txt = "); 
  while (myDataFile.available()) { 
    Serial.print(myDataFile.readStringUntil('\n'));
    Serial.print("; ");
  }
  myDataFile.close();
  Serial.println();
  Serial.println("finish WriteToSPIFFS()");
} // void WriteToSPIFFS()

void FirstTimeRun() {
  Serial.println("FirstTimeRun(): Starting SPIFFS initializing process.");
  accuracy = 1;
  char filename [] = "/data.txt";
  File myDataFile = SPIFFS.open(filename, "w");
  if (!myDataFile) {
    Serial.println("  firstimerun: Failed to open file");
    Serial.println("  Stopping process - maybe flash size not set (SPIFFS).");
    exit(0);
  }
  myDataFile.println(current_timestamp); // save timestamp to /data.txt
  myDataFile.println(accuracy); // save accuracy value to /data.txt
  for ( int i = 0; i < 12; i++) {
    myDataFile.println(rel_pressure_rounded); // save initial pressure data with current pressure
  }
  Serial.println("  Saved initial pressure data");
  myDataFile.close();
} // void FirstTimeRun()

//-----------------
void readSavedEpoch() {
  Serial.println();
  Serial.println("Reading last epoch value from SPIFFS...  ");
  char filename [] = "/epoch.txt";
  File myDataFile = SPIFFS.open(filename, "r");       // Open file for reading
  if (!myDataFile) {
    Serial.println("  epoch.txt in SPIFFS failed to open");

    Serial.println("  initializing epoch.txt file in SPIFFS");
    char filename [] = "/epoch.txt";
    File myDataFile = SPIFFS.open(filename, "w");            // Open a file for writing
    if (!myDataFile) {
      Serial.println("  failed to create epoch.txt in SPIFFS");
      Serial.println("  Stopping process - maybe flash size not set (SPIFFS).");
      exit(0);
    }
    myDataFile.println(0);
  } // if(!myDataFile) if epoch.txt does not exist
  
  String epochString;
  epochString = myDataFile.readStringUntil('\n');  
  savedEpoch = epochString.toInt();
  Serial.print("  last saved epoch value from SPIFFS: ");
  Serial.println(savedEpoch);
  
  myDataFile.close();
} // readSavedEpoch()

void writeSavedEpoch() {
  Serial.println();
  Serial.println("Write epoch.txt to SPIFFS...  ");
  char filename [] = "/epoch.txt";
  File myDataFile = SPIFFS.open(filename, "w");
  if (!myDataFile) {
    Serial.println("  failed to open epoch.txt file in SPIFFS");
  }
  
  myDataFile.println(now());
  Serial.print("writeSavedEpoch() now(): ");
  Serial.println(now());
  myDataFile.close();
  
  Serial.println("  post-write epoch value read from SPIFFS: ");
  String epochString;
  myDataFile = SPIFFS.open(filename, "r");             // Open file for reading
  epochString = myDataFile.readStringUntil('\n');  
  savedEpoch = epochString.toInt();
  Serial.print("  epoch value in SPIFFS:  "); 
  Serial.println(savedEpoch);
  Serial.println("finish WriteSavedEpoch()");
  myDataFile.close();
} // writeSavedEpoch()
//-------------------------

int CalculateTrend() {
  Serial.println("  CalculateTrend():  Calculating Zambretti trend");
  int trend; // -1 falling; 0 steady; 1 raising
  
  //--> giving the most recent pressure reads more weight
  pressure_difference[0] = (pressure_value[0] - pressure_value[1])   * 1.5;
  pressure_difference[1] = (pressure_value[0] - pressure_value[2]);
  pressure_difference[2] = (pressure_value[0] - pressure_value[3])   / 1.5;
  pressure_difference[3] = (pressure_value[0] - pressure_value[4])   / 2;
  pressure_difference[4] = (pressure_value[0] - pressure_value[5])   / 2.5;
  pressure_difference[5] = (pressure_value[0] - pressure_value[6])   / 3;
  pressure_difference[6] = (pressure_value[0] - pressure_value[7])   / 3.5;
  pressure_difference[7] = (pressure_value[0] - pressure_value[8])   / 4;
  pressure_difference[8] = (pressure_value[0] - pressure_value[9])   / 4.5;
  pressure_difference[9] = (pressure_value[0] - pressure_value[10])  / 5;
  pressure_difference[10] = (pressure_value[0] - pressure_value[11]) / 5.5;
  
  //--> calculating the average and storing it into [11]
  pressure_difference[11] = (  pressure_difference[0]
                             + pressure_difference[1]
                             + pressure_difference[2]
                             + pressure_difference[3]
                             + pressure_difference[4]
                             + pressure_difference[5]
                             + pressure_difference[6]
                             + pressure_difference[7]
                             + pressure_difference[8]
                             + pressure_difference[9]
                             + pressure_difference[10]) / 11;
  
  Serial.print("    Current Zambretti pressure difference average: ");
  Serial.println(pressure_difference[11]);

  if      (pressure_difference[11] > 3.5) {
    trend_in_words = TEXT_RISING_FAST;
    trend = 1;}
  else if (pressure_difference[11] > 1.5   && pressure_difference[11] <= 3.5)  {
    trend_in_words = TEXT_RISING;
    trend = 1;
  }
  else if (pressure_difference[11] > 0.25  && pressure_difference[11] <= 1.5)  {
    trend_in_words = TEXT_RISING_SLOW;
    trend = 1;
  }
  else if (pressure_difference[11] > -0.25 && pressure_difference[11] < 0.25)  {
    trend_in_words = TEXT_STEADY;
    trend = 0;
  }
  else if (pressure_difference[11] >= -1.5 && pressure_difference[11] < -0.25) {
    trend_in_words = TEXT_FALLING_SLOW;
    trend = -1;
  }
  else if (pressure_difference[11] >= -3.5 && pressure_difference[11] < -1.5)  {
    trend_in_words = TEXT_FALLING;
    trend = -1;
  }
  else if (pressure_difference[11] <= -3.5) {
    trend_in_words = TEXT_FALLING_FAST;
    trend = -1;
  }

  Serial.print("    Current trend in words: ");
  Serial.println(trend_in_words);
  Serial.println("  finish CalculateTrend()");

  return trend;
}  // CalculateTrend()

char ZambrettiLetter() {
  Serial.println();
  Serial.println("Calculating Zambretti letter");
  char z_letter;
  int(z_trend) = CalculateTrend();
  // Case trend is falling
  if (z_trend == -1) {
    float zambretti = 0.0009746 * rel_pressure_rounded * rel_pressure_rounded - 2.1068 * rel_pressure_rounded + 1138.7019; 
    if (rtcMonth < 4 || rtcMonth > 9) zambretti = zambretti + 1;
//    Serial.print("  Calculated and rounded Zambretti in numbers: ");
//    Serial.println(round(zambretti));
    switch (int(round(zambretti))) {
      case 0:  z_letter = 'A'; break;       //Settled Fine
      case 1:  z_letter = 'A'; break;       //Settled Fine
      case 2:  z_letter = 'B'; break;       //Fine Weather
      case 3:  z_letter = 'D'; break;       //Fine Becoming Less Settled
      case 4:  z_letter = 'H'; break;       //Fairly Fine Showers Later
      case 5:  z_letter = 'O'; break;       //Showery Becoming unsettled
      case 6:  z_letter = 'R'; break;       //Unsettled, Rain later
      case 7:  z_letter = 'U'; break;       //Rain at times, worse later
      case 8:  z_letter = 'V'; break;       //Rain at times, becoming very unsettled 
      case 9:  z_letter = 'X'; break;       //Very Unsettled, Rain
    }
  }
  // Case trend is steady
  if (z_trend == 0) {
    float zambretti = 138.24 - 0.133 * rel_pressure_rounded;
//    Serial.print("  Calculated and rounded Zambretti in numbers: ");
//    Serial.println(round(zambretti));
    switch (int(round(zambretti))) {
      case 0:  z_letter = 'A'; break;       //Settled Fine
      case 1:  z_letter = 'A'; break;       //Settled Fine
      case 2:  z_letter = 'B'; break;       //Fine Weather
      case 3:  z_letter = 'E'; break;       //Fine, Possibly showers
      case 4:  z_letter = 'K'; break;       //Fairly Fine, Showers likely
      case 5:  z_letter = 'N'; break;       //Showery Bright Intervals
      case 6:  z_letter = 'P'; break;       //Changeable some rain
      case 7:  z_letter = 'S'; break;       //Unsettled, rain at times
      case 8:  z_letter = 'W'; break;       //Rain at Frequent Intervals
      case 9:  z_letter = 'X'; break;       //Very Unsettled, Rain
      case 10: z_letter = 'Z'; break;       //Stormy, much rain
    }
  }
  // Case trend is rising
  if (z_trend == 1) {
    float zambretti = 142.57 - 0.1376 * rel_pressure_rounded;
    //A Summer rising, improves the prospects by 1 unit over a Winter rising
    if (GPS.month < 4 || GPS.month > 9) zambretti = zambretti + 1;
//    Serial.print("  Calculated and rounded Zambretti in numbers: ");
//    Serial.println(round(zambretti));
    switch (int(round(zambretti))) {
      case 0:  z_letter = 'A'; break;       //Settled Fine
      case 1:  z_letter = 'A'; break;       //Settled Fine
      case 2:  z_letter = 'B'; break;       //Fine Weather
      case 3:  z_letter = 'C'; break;       //Becoming Fine
      case 4:  z_letter = 'F'; break;       //Fairly Fine, Improving
      case 5:  z_letter = 'G'; break;       //Fairly Fine, Possibly showers, early
      case 6:  z_letter = 'I'; break;       //Showery Early, Improving
      case 7:  z_letter = 'J'; break;       //Changeable, Improving
      case 8:  z_letter = 'L'; break;       //Rather Unsettled Clearing Later
      case 9:  z_letter = 'M'; break;       //Unsettled, Probably Improving
      case 10: z_letter = 'Q'; break;       //Unsettled, short fine Intervals
      case 11: z_letter = 'T'; break;       //Very Unsettled, Finer at times
      case 12: z_letter = 'Y'; break;       //Stormy, possibly improving
      case 13: z_letter = 'Z'; break;;      //Stormy, much rain
    }
  }
//  Serial.print("  This is Zambretti's famous letter: ");
//  Serial.println(z_letter);
  return z_letter;
} // char ZambrettiLetter()

String ZambrettiSays(char code){
  String zambrettis_words = "";
  switch (code) {
  case 'A': zambrettis_words = TEXT_ZAMBRETTI_A; break;  //see Translation.h
  case 'B': zambrettis_words = TEXT_ZAMBRETTI_B; break;
  case 'C': zambrettis_words = TEXT_ZAMBRETTI_C; break;
  case 'D': zambrettis_words = TEXT_ZAMBRETTI_D; break;
  case 'E': zambrettis_words = TEXT_ZAMBRETTI_E; break;
  case 'F': zambrettis_words = TEXT_ZAMBRETTI_F; break;
  case 'G': zambrettis_words = TEXT_ZAMBRETTI_G; break;
  case 'H': zambrettis_words = TEXT_ZAMBRETTI_H; break;
  case 'I': zambrettis_words = TEXT_ZAMBRETTI_I; break;
  case 'J': zambrettis_words = TEXT_ZAMBRETTI_J; break;
  case 'K': zambrettis_words = TEXT_ZAMBRETTI_K; break;
  case 'L': zambrettis_words = TEXT_ZAMBRETTI_L; break;
  case 'M': zambrettis_words = TEXT_ZAMBRETTI_M; break;
  case 'N': zambrettis_words = TEXT_ZAMBRETTI_N; break;
  case 'O': zambrettis_words = TEXT_ZAMBRETTI_O; break;
  case 'P': zambrettis_words = TEXT_ZAMBRETTI_P; break; 
  case 'Q': zambrettis_words = TEXT_ZAMBRETTI_Q; break;
  case 'R': zambrettis_words = TEXT_ZAMBRETTI_R; break;
  case 'S': zambrettis_words = TEXT_ZAMBRETTI_S; break;
  case 'T': zambrettis_words = TEXT_ZAMBRETTI_T; break;
  case 'U': zambrettis_words = TEXT_ZAMBRETTI_U; break;
  case 'V': zambrettis_words = TEXT_ZAMBRETTI_V; break;
  case 'W': zambrettis_words = TEXT_ZAMBRETTI_W; break;
  case 'X': zambrettis_words = TEXT_ZAMBRETTI_X; break;
  case 'Y': zambrettis_words = TEXT_ZAMBRETTI_Y; break;
  case 'Z': zambrettis_words = TEXT_ZAMBRETTI_Z; break;
   default: zambrettis_words = TEXT_ZAMBRETTI_DEFAULT; break;
  }
  return zambrettis_words;
} // String ZambrettiSays()
