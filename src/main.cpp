#include <Arduino.h>
#include "wiring_private.h" // pinPeripheral() function
#include <AceRoutine.h>
#include <AceTime.h>
// #include <Adafruit_GPS.h>
#include <TinyGPSPlus.h>
extern "C"
{
  #include "rtcFunctions.h"
}

// Serial port used by GPS module
#define GPSSerial Serial1

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

class GpsReadCoroutine: public Coroutine {
  public:
    int runCoroutine() override {
      COROUTINE_LOOP() {
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
    uint32_t rtcUpdateFrequencySeconds = 30 * 60; //update every 30 minutes
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
        if (gps.time.hour() < 10) Serial.print(F("0"));
        Serial.print(gps.time.hour());
        Serial.print(F(":"));
        if (gps.time.minute() < 10) Serial.print(F("0"));
        Serial.print(gps.time.minute());
        Serial.print(F(":"));
        if (gps.time.second() < 10) Serial.print(F("0"));
        Serial.print(gps.time.second());
        Serial.print(F("."));
        if (gps.time.centisecond() < 10) Serial.print(F("0"));
        Serial.print(gps.time.centisecond());
      }
      else
      {
        Serial.print(F("INVALID"));
      }

      Serial.println();
    }
};

class PrintRtcCoroutine: public Coroutine {
  public:
    int runCoroutine() override {
      COROUTINE_LOOP() {
        // Get Unix time from RTC and print
        if (rtcValid)
        {
          utcTime = ZonedDateTime::forUnixSeconds(rtcGetUnixTime(),utcTz);
          Serial.print(F("Current RTC: "));
          utcTime.printTo(SERIAL_PORT_MONITOR);
          SERIAL_PORT_MONITOR.println();
        }
        COROUTINE_DELAY_SECONDS(rtcBetweenPrintSeconds);
      }
    }
  private:
    uint16_t rtcBetweenPrintSeconds =  60;
    ZonedDateTime utcTime;
};

GpsReadCoroutine readGps;
PrintRtcCoroutine printRtc;

void setup() {
  
  rtcInit();

  Serial.begin(115200);
  Serial.println("Initializing GPS");

  GPSSerial.begin(9600,SERIAL_8N1);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  // GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  // Set the update rate
  // GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1Hz update rate

  // Request updates on antenna status, comment out to keep quiet
  // GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);

  readGps.setName("readGps");
  printRtc.setName("printGps");
#if ! defined(EPOXY_DUINO)
  delay(1000);
#endif
  Serial.begin(115200);
  while (!Serial); // Leonardo/Micro
  
  // Create a profiler on the heap for every coroutine.
  // LogBinProfiler::createProfilers();

  // Setup the scheduler.
  CoroutineScheduler::setup();
}

void loop() {
  // readGps.runCoroutine();
  CoroutineScheduler::loop();
  // CoroutineScheduler::loopWithProfiler();
}
