#include <Arduino.h>
#include "wiring_private.h" // pinPeripheral() function
#include <AceRoutine.h>
#include <AceTime.h>
// #include <Adafruit_GPS.h>
#include <TinyGPSPlus.h>
#include <Regexp.h>
extern "C"
{
#include "rtcFunctions.h"
}

// Serial port used by GPS module
#define GPSSerial Serial1

// Pins used by distance sensor on sensorSerial
#define PIN_SENSOR_RX 15  // PA05
#define PAD_SENSOR_RX (SERCOM_RX_PAD_1)
#define PIN_SENSOR_TX 18  // PA04
#define PAD_SENSOR_TX (UART_TX_PAD_0)

// using ace_time::acetime_t;
// using ace_time::TimeZone;
// using ace_time::BasicZoneProcessor;
// using ace_time::ZonedDateTime;
// using ace_time::zonedb::kZoneGMT;

using namespace ace_routine;
using namespace ace_time;

static TinyGPSPlus gps;
static boolean rtcValid = false;
static BasicZoneProcessor losAngelesProcessor;
static BasicZoneProcessor utcProcessor;

static TimeZone losAngelesTz = TimeZone::forZoneInfo(
    &zonedb::kZoneAmerica_Los_Angeles,
    &losAngelesProcessor);
static TimeZone utcTz = TimeZone::forZoneInfo(
    &zonedb::kZoneUTC,
    &utcProcessor);

// Uart sensorSerial(&sercom0, 13, 12, SERCOM_RX_PAD_1, UART_TX_PAD_0);
Uart sensorSerial( &sercom0, PIN_SENSOR_RX, PIN_SENSOR_TX, PAD_SENSOR_RX, PAD_SENSOR_TX );
void SERCOM0_0_Handler()
{
  sensorSerial.IrqHandler();
}
void SERCOM0_1_Handler()
{
  sensorSerial.IrqHandler();
}
void SERCOM0_2_Handler()
{
  sensorSerial.IrqHandler();
}
void SERCOM0_3_Handler()
{
  sensorSerial.IrqHandler();
}

class GpsReadCoroutine : public Coroutine
{
public:
  int runCoroutine() override
  {
    COROUTINE_LOOP()
    {
      gpsCollectTimer = millis();
      while ((millis() - gpsCollectTimer) < (gpsSentenceCollectionSeconds * 1000))
      {
        while (GPSSerial.available() > 0)
        {
          if (gps.encode(GPSSerial.read()))
          {
            // displayInfo();
            // if both gps.time and gps.date are valid, then process RTC
            if (gps.time.isValid() & gps.date.isValid())
            {
              // If RTC has never been updated or it's time for an update
              if ((rtcLastUpdated == 0) | ((rtcGetUnixTime() - rtcLastUpdated) >= rtcUpdateFrequencySeconds))
              {
                Serial.print(F("RTC updated: "));
                utcTime = ZonedDateTime::forComponents(
                    gps.date.year(), gps.date.month(), gps.date.day(), gps.time.hour(), gps.time.minute(), gps.time.second(), utcTz);
                rtcSetUnixTime(utcTime.toUnixSeconds());
                rtcLastUpdated = rtcGetUnixTime();
                rtcValid = true;
                utcTime.printTo(SERIAL_PORT_MONITOR);
                SERIAL_PORT_MONITOR.println();
              }
            }
          }
        }
      }
      COROUTINE_YIELD();
    }
  }

private:
  char c;
  uint32_t gpsCollectTimer;
  uint16_t gpsSentenceCollectionSeconds = 1;
  uint32_t rtcLastUpdated = 0;
  uint32_t rtcUpdateFrequencySeconds = 30 * 60; // update every 30 minutes
  ZonedDateTime utcTime = ZonedDateTime::forComponents(
      1970, 1, 1, 0, 0, 0, utcTz);

  void displayInfo()
  {
    Serial.print(F("Characters processed: "));
    Serial.print(gps.charsProcessed());
    Serial.print(F(", "));

    Serial.print(F("Satellites: "));
    Serial.print(gps.satellites.value());
    Serial.print(F(", "));

    Serial.print(F("Location: "));
    if (gps.location.isValid())
    {
      Serial.print(gps.location.lat(), 6);
      Serial.print(F(","));
      Serial.print(gps.location.lng(), 6);
    }
    else
    {
      Serial.print(F("INVALID"));
    }

    Serial.print(F("  Date/Time: "));
    if (gps.date.isValid())
    {
      Serial.print(gps.date.month());
      Serial.print(F("/"));
      Serial.print(gps.date.day());
      Serial.print(F("/"));
      Serial.print(gps.date.year());
    }
    else
    {
      Serial.print(F("INVALID"));
    }

    Serial.print(F(" "));
    if (gps.time.isValid())
    {
      if (gps.time.hour() < 10)
        Serial.print(F("0"));
      Serial.print(gps.time.hour());
      Serial.print(F(":"));
      if (gps.time.minute() < 10)
        Serial.print(F("0"));
      Serial.print(gps.time.minute());
      Serial.print(F(":"));
      if (gps.time.second() < 10)
        Serial.print(F("0"));
      Serial.print(gps.time.second());
      Serial.print(F("."));
      if (gps.time.centisecond() < 10)
        Serial.print(F("0"));
      Serial.print(gps.time.centisecond());
    }
    else
    {
      Serial.print(F("INVALID"));
    }

    Serial.println();
  }
};

class GetMeasurementCoroutine : public Coroutine
{
public:
  int runCoroutine() override
  {
    COROUTINE_LOOP()
    {
      // Collect measurements if RTC is valid
      if (rtcValid)
      {
        distance = 0;
        // Clear serial buffer prior to getting a reading
        while (sensorSerial.available())
          ch = sensorSerial.read();
        // Toggle pin 11 to get a reading
        digitalWrite(11,HIGH);
        // Measurement is triggered by holding the ranging start/stop pin
        // high for at least 20 microseconds. This may be longer, but must
        // be low before the end of the range cycle or readings will become
        // continuous.
        delay(1);
        digitalWrite(11,LOW);
        // wait for the end of range cycle (~148mS for MB7389)
        delay(150);

        while (sensorSerial.available())
        {
          ch = sensorSerial.read();
          Serial.print(ch);
          if((strIndex < MaxChars) && isAlphaNumeric(ch))
          {
              strValue[strIndex++] = ch; // add alphanumeric character to the string;
          }
          else if (ch == '\r')
          {
              // Serial.print(F("got carriage return, string: "));
              // Serial.println(strValue);
              // RegEx looks for a string that starts with R followed by four digits.
              // The four digits are put into a group, zero, to be later converted to
              // the reading in mm.
              matchState.Target(strValue);
              char result = matchState.Match(regEx.c_str());
              // terminate the string with a 0
              strValue[strIndex] = 0;
              if (result == REGEXP_MATCHED)
              {
                  // Serial.println(F("RegEx Match"));
                  //convert the four digits from group zero to a number
                  distance = atoi(matchState.GetCapture (strValue, 0));
                  #ifdef DEBUG_CAPTURE
                      ostime_t timestamp = os_getTime();
                      printEvent(timestamp, "Distance collected", PrintTarget::Serial);
                      printSpaces(serial, MESSAGE_INDENT);
                      serial.print(F("strValue: "));
                      serial.println(strValue);
                      printSpaces(serial, MESSAGE_INDENT);
                      serial.print(F("Match start: "));
                      serial.println(matchState.MatchStart);
                      printSpaces(serial, MESSAGE_INDENT);
                      serial.print(F("Match length: "));
                      serial.println(matchState.MatchLength);
                      printSpaces(serial, MESSAGE_INDENT);
                      serial.print(F("Match: "));
                      Serial.println(matchState.GetCapture(strValue, 0));
                  #endif 
              }
              else if (result == REGEXP_NOMATCH)
              {
                  // Serial.println(F("RegEx No Match"));
                  #ifdef DEBUG_CAPTURE
                      printEvent(timestamp, "RegEx No Match", PrintTarget::Serial);
                      printSpaces(serial, MESSAGE_INDENT);
                      serial.print(F("strValue with no RegEx match: "));
                      serial.println(strValue);
                  #endif
                  distance = 0;  
              }
              else
              {
                  // Serial.println(F("RegEx Match Error"));
                  #ifdef DEBUG_CAPTURE
                      printEvent(timestamp, "RegEx Match Error", PrintTarget::Serial);
                      printSpaces(serial, MESSAGE_INDENT);
                      serial.print(F("strValue with RegEx error: "));
                      serial.println(strValue);
                  #endif
                  distance = 0;                  
              }

              strIndex = 0;
          }
          else
          {
              // buffer is full or a non-alphanumeric character that is not
              // a carriage return has been found
              // Serial.println(F("non-alphanumeric character that is not a carriage return"));
              #ifdef DEBUG_CAPTURE
                  ostime_t timestamp = os_getTime();
                  printEvent(timestamp, "non-alphanumeric character that is not a carriage return", PrintTarget::Serial);
                  printSpaces(serial, MESSAGE_INDENT);
                  serial.print(F("strValue when this occured: "));
                  for ( int i = 0; i < strIndex; ++i )
                      serial.print(strValue[i]);
                  serial.println();
              #endif 
              strIndex = 0;
          }
        }
        utcTime = ZonedDateTime::forUnixSeconds(rtcGetUnixTime(), utcTz);
        Serial.print(F("Measurement: "));
        utcTime.printTo(SERIAL_PORT_MONITOR);
        Serial.print(F(": "));
        Serial.print(distance);
        SERIAL_PORT_MONITOR.println();
      }
      COROUTINE_DELAY_SECONDS(sensorBetweenReadSeconds);
    }
  }

private:
  char ch;
  MatchState matchState; // match state object for regex
  uint16_t distance;
  const int MaxChars = 20;        // Size of longest string returned by the MB7389
  char strValue[21];              // must be big enough for characters and terminating null
  int strIndex = 0;               // the index into the array storing the received characters
  String regEx = "^R(%d%d%d%d)";  // regular expression pattern to extract from strValue
  uint16_t sensorBetweenReadSeconds = 10;
  ZonedDateTime utcTime;
};

class PrintRtcCoroutine : public Coroutine
{
public:
  int runCoroutine() override
  {
    COROUTINE_LOOP()
    {
      // Get Unix time from RTC and print
      if (rtcValid)
      {
        utcTime = ZonedDateTime::forUnixSeconds(rtcGetUnixTime(), utcTz);
        Serial.print(F("Current RTC: "));
        utcTime.printTo(SERIAL_PORT_MONITOR);
        SERIAL_PORT_MONITOR.println();
      }
      COROUTINE_DELAY_SECONDS(rtcBetweenPrintSeconds);
    }
  }

private:
  uint16_t rtcBetweenPrintSeconds = 60;
  ZonedDateTime utcTime;
};



GpsReadCoroutine readGps;
PrintRtcCoroutine printRtc;
GetMeasurementCoroutine getMeasurement;

void setup()
{

  rtcInit();

  // Assign pin 11 (D11) as output to control distance sensor sampling
  pinMode(11,OUTPUT);
  // Start distance sensor disabled
  digitalWrite(11,LOW);

  Serial.begin(115200);
  Serial.println("Initializing GPS");

  GPSSerial.begin(9600, SERIAL_8N1);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  // GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  // Set the update rate
  // GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1Hz update rate

  // Request updates on antenna status, comment out to keep quiet
  // GPS.sendCommand(PGCMD_ANTENNA);

  sensorSerial.begin(9600, SERIAL_8N1);
  // Assign sensor pins to SERCOM_ALT functionality
  pinPeripheral(PIN_SENSOR_RX, PIO_SERCOM_ALT);
  pinPeripheral(PIN_SENSOR_TX, PIO_SERCOM_ALT);


  delay(1000);

  readGps.setName("readGps");
  printRtc.setName("printGps");
  getMeasurement.setName("getMeasurement");

#if !defined(EPOXY_DUINO)
  delay(1000);
#endif
  Serial.begin(115200);
  while (!Serial); // Leonardo/Micro

  // Create a profiler on the heap for every coroutine.
  // LogBinProfiler::createProfilers();

  // Setup the scheduler.
  CoroutineScheduler::setup();
}

void loop()
{
  // readGps.runCoroutine();
  CoroutineScheduler::loop();
  // CoroutineScheduler::loopWithProfiler();
}
