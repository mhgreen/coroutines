#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SdFat.h>
#include "wiring_private.h" // pinPeripheral() function
#include <AceRoutine.h>
#include <AceTime.h>
// #include <Adafruit_GPS.h>
#include <TinyGPSPlus.h>
#include <Statistic.h>
#include <CircularBuffer.h>
#include <Regexp.h>
#include <SerialTransfer.h>
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

// Pins used to transfer serial data to LoRaWAN on loraSerial
#define PIN_LORA_RX 17  // PB09
#define PAD_LORA_RX (SERCOM_RX_PAD_1)
#define PIN_LORA_TX 16  // PB08
#define PAD_LORA_TX (UART_TX_PAD_0)

// using ace_time::acetime_t;
// using ace_time::TimeZone;
// using ace_time::BasicZoneProcessor;
// using ace_time::ZonedDateTime;
// using ace_time::zonedb::kZoneGMT;

using namespace ace_routine;
using namespace ace_time;

static boolean displayCollectionDetail = false;
static boolean displayRegexDetail = false;
static boolean displayRtcDetail = false;
static boolean displayGpsDetail = false;
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

static SerialTransfer sensorTransfer;

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

Uart loraSerial( &sercom4, PIN_LORA_RX, PIN_LORA_TX, PAD_LORA_RX, PAD_LORA_TX );
void SERCOM4_0_Handler()
{
  loraSerial.IrqHandler();
}
void SERCOM4_1_Handler()
{
  loraSerial.IrqHandler();
}
void SERCOM4_2_Handler()
{
  loraSerial.IrqHandler();
}
void SERCOM4_3_Handler()
{
  loraSerial.IrqHandler();
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
            if (displayGpsDetail)
            {
              displayGpsInfo();
            }
            // if both gps.time and gps.date are valid, then process RTC
            if (gps.time.isValid() & gps.date.isValid())
            {
              // If RTC has never been updated or it's time for an update
              if ((rtcLastUpdated == 0) | ((rtcGetUnixTime() - rtcLastUpdated) >= rtcUpdateFrequencySeconds))
              {
                utcTime = ZonedDateTime::forComponents(
                    gps.date.year(), gps.date.month(), gps.date.day(), gps.time.hour(), gps.time.minute(), gps.time.second(), utcTz);
                rtcSetUnixTime(utcTime.toUnixSeconds());
                rtcLastUpdated = rtcGetUnixTime();
                rtcValid = true;
                if (displayRtcDetail)
                {
                  Serial.print(F("RTC updated: "));
                  utcTime.printTo(SERIAL_PORT_MONITOR);
                  SERIAL_PORT_MONITOR.println();
                }
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
  uint32_t rtcUpdateFrequencySeconds = 120 * 60; // update every two hours
  ZonedDateTime utcTime = ZonedDateTime::forComponents(
      1970, 1, 1, 0, 0, 0, utcTz);

  void displayGpsInfo()
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
        delayMicroseconds(30);
        digitalWrite(11,LOW);
        // wait for the end of range cycle (~148mS for MB7389)
        delay(150);

        while (sensorSerial.available())
        {
          ch = sensorSerial.read();
          // Serial.print(ch);
          if((strIndex < maxChars) && isAlphaNumeric(ch))
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
                  if (displayRegexDetail)
                  {
                      
                      Serial.print(F("Distance collected"));
                      Serial.print(F("strValue: "));
                      Serial.println(strValue);
                      Serial.print(F("Match start: "));
                      Serial.println(matchState.MatchStart);
                      Serial.print(F("Match length: "));
                      Serial.println(matchState.MatchLength);
                      Serial.print(F("Match: "));
                      Serial.println(matchState.GetCapture(strValue, 0));
                  }
              }
              else if (result == REGEXP_NOMATCH)
              {
                  // Serial.println(F("RegEx No Match"));
                  if (displayRegexDetail)
                  {
                      Serial.println(F("RegexNoMatch"));
                      Serial.print(F("strValue with no RegEx match: "));
                      Serial.println(strValue);
                  }
                  distance = 0;  
              }
              else
              {
                  // Serial.println(F("RegEx Match Error"));
                  if (displayRegexDetail)
                  {
                      Serial.println(F("RegEx Match Error"));
                      Serial.print(F("strValue with RegEx error: "));
                      Serial.println(strValue);
                  }
                  distance = 0;                  
              }

              strIndex = 0;
          }
          else
          {
              // buffer is full or a non-alphanumeric character that is not
              // a carriage return has been found
              // Serial.println(F("non-alphanumeric character that is not a carriage return"));
              if (displayCollectionDetail)
              {
                  Serial.println(F("non-alphanumeric character that is not a carriage return"));
                  Serial.print(F("strValue when this occured: "));
                  for ( int i = 0; i < strIndex; ++i )
                      Serial.print(strValue[i]);
                  Serial.println();
              }
              strIndex = 0;
          }
        }
        utcTime = ZonedDateTime::forUnixSeconds(rtcGetUnixTime(), utcTz);
        waterLevelReading.utcTime = rtcGetUnixTime();
        waterLevelReading.distance = distance;

        // If the sensor returned 0, indicating that nothing was collected,
        // or if the sensor returns 5000 or 9999, indicating that no target is
        // visible in the field of view, then don't upload the value.
        if (waterLevelReading.distance == 0 || waterLevelReading.distance == 5000 || waterLevelReading.distance == 9999)
        {
            if (displayCollectionDetail)
            {
              Serial.print(F("Water level reading invalid (0, 5000, or 9999)"));
              Serial.println();
            }
        }
        else
        {
            // Serial.print(F("Raw Measurement: "));
            // utcTime.printTo(SERIAL_PORT_MONITOR);
            // Serial.print(F(", distance "));
            // Serial.print(distance);
            // Serial.print(F(", group count: "));
            // Add the observation to waterLevelGroup to calculate
            // count, average, and standard deviation.
            waterLevelGroup.add(distance);
            // Push the group of collected datapoints to a queue so
            // that the water measurements can be accepted or rejected
            // as a group.
            queue.push(distance);
            // Collect groups of ten water level observations.
            // Serial.print(waterLevelGroup.count());
            // Serial.print(F(", group average: "));
            // Serial.print(waterLevelGroup.average());
            // SERIAL_PORT_MONITOR.println();
        }
        if (waterLevelGroup.count() >= 10)
        {
            if (displayCollectionDetail)
            {
              Serial.print(F("Group count: "));
              Serial.print(waterLevelGroup.count());
              Serial.print(F(", group average: "));
              Serial.print(waterLevelGroup.average());
              Serial.print(F(", group pop sd: "));
              Serial.print(waterLevelGroup.pop_stdev());
              SERIAL_PORT_MONITOR.println();
            }
            // If the standard deviation is less than or equal to ten millimeters then accept the group.
            if (waterLevelGroup.pop_stdev() <= 10)
            {
              if (displayCollectionDetail)
              {
                Serial.print(F("Group standard deviation less than 10mm; adding to waterLevelGroup"));
                SERIAL_PORT_MONITOR.println();
                // Take the measurements set aside in the queue and add them to the main group of
                // water level statistics.
                Serial.print(F("adding distances: "));
              }
              while (!queue.isEmpty())
              {
                // since the group meets the standard deviation limit, add the observations
                // to the general statistics.
                uint16_t distanceToadd = queue.pop();
                waterLevelStats.add(distanceToadd);
                if (displayCollectionDetail)
                {
                  Serial.print(distanceToadd);
                  Serial.print(F(" "));
                }
              }
              if (displayCollectionDetail)
              {
                SERIAL_PORT_MONITOR.println();
                Serial.print(F("waterLevelStats: "));
                utcTime.printTo(SERIAL_PORT_MONITOR);
                Serial.print(F(", count: "));
                Serial.print(waterLevelStats.count());
                Serial.print(F(", average: "));
                Serial.print(waterLevelStats.average());
                Serial.print(F(", pop sd: "));
                Serial.print(waterLevelStats.pop_stdev());
                SERIAL_PORT_MONITOR.println();
              }
              waterLevelGroup.clear();
              queue.clear();
            }
            // If the standard deviation of the sample group is greater than ten millimeters
            // then reject the group.
            else
            {
              if (displayCollectionDetail)
              {
                Serial.print(F("Group standard deviation greater than 10mm, rejecting group"));
                SERIAL_PORT_MONITOR.println();
              }
              waterLevelGroup.clear();
              queue.clear();
            }
        }

        // If it's time to send a message based on the current minute value,
        // and one has not been sent in the current minute, and at least 30
        // water level observations have been collected, then transmit the
        // average of the collected water level values.
        if (((utcTime.minute() % minutesBetweenSend) == 0) & (utcTime.minute() != minuteLastSent) & (waterLevelStats.count() >= 30))
        {
            
            minuteLastSent = utcTime.minute();
            measurementsSinceLastSend = 0;
            if (displayCollectionDetail)
            {
              Serial.print(F("Measurement Ready: "));
              utcTime.printTo(SERIAL_PORT_MONITOR);
              Serial.print(F(", count: "));
              Serial.print(waterLevelStats.count());
              Serial.print(F(", average: "));
              Serial.print(waterLevelStats.average());
              Serial.print(F(", pop sd: "));
              Serial.print(waterLevelStats.pop_stdev());
              Serial.print(F(", variance: "));
              Serial.print(waterLevelStats.variance());
              Serial.print(F(", sending --> "));
              Serial.print(F("time: "));
              Serial.print(waterLevelReading.utcTime);
              Serial.print(F(" ("));
              utcTime.forUnixSeconds(waterLevelReading.utcTime,utcTz).printTo(SERIAL_PORT_MONITOR);
              Serial.print(F(") "));
              Serial.print(F(", distance: "));
              Serial.print(round(waterLevelStats.average()));
              SERIAL_PORT_MONITOR.println();
            }
            waterLevelReading.distance = waterLevelStats.average();
            sensorTransfer.sendDatum(waterLevelReading);
            waterLevelStats.clear();
        }
                
      }
      COROUTINE_DELAY_SECONDS(sensorBetweenReadSeconds);
    }
  }

private:
  char ch;
  MatchState matchState; // match state object for regex
  uint16_t distance;
  const int8_t maxChars = 20;       // size of longest string returned by the MB7389
  char strValue[21];                // string big enough for characters and terminating null (maxChars +1)
  int strIndex = 0;                 // the index into the array storing the received characters
  String regEx = "^R(%d%d%d%d)";    // regular expression pattern to extract from strValue
  uint16_t minutesBetweenSend = 6;  // send measurement every 6 minutes
  uint8_t minuteLastSent = 61;      // Minute in which last measurement is sent (only send once in a minute
                                    // period even if there are multiple readings withing that minute).
                                    // Initially set to 61 so that an initial reading gets sent.
  uint16_t sensorBetweenReadSeconds = 6; //seconds between sensor readings
  uint8_t measurementsSinceLastSend = 0;
  ZonedDateTime utcTime;
  struct STRUCT
  {
    uint32_t utcTime;
    uint16_t distance;
  } waterLevelReading;
  statistic::Statistic<float, uint32_t, true> waterLevelStats;
  statistic::Statistic<float, uint32_t, true> waterLevelGroup;
  CircularBuffer<uint16_t, 20> queue;
  void displayGroupInfo()
  {

  }
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
        if (displayRtcDetail)
        {
          Serial.print(F("Current RTC: "));
          utcTime.printTo(SERIAL_PORT_MONITOR);
          SERIAL_PORT_MONITOR.println();
        }
      }
      COROUTINE_DELAY_SECONDS(rtcBetweenPrintSeconds);
    }
  }

private:
  uint16_t rtcBetweenPrintSeconds = 60;
  ZonedDateTime utcTime;
};

GpsReadCoroutine readGps;
// PrintRtcCoroutine printRtc;
GetMeasurementCoroutine getMeasurement;

void setup()
{

  rtcInit();

  // Assign pin 11 (D11) as output to control distance sensor sampling
  pinMode(11,OUTPUT);
  // Start distance sensor disabled
  digitalWrite(11,LOW);

  Serial.begin(115200);
 
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

  loraSerial.begin(115200, SERIAL_8N1);
  // Assign lora pins to SERCOM_ALT functionality
  pinPeripheral(PIN_LORA_RX, PIO_SERCOM_ALT);
  pinPeripheral(PIN_LORA_TX, PIO_SERCOM_ALT);
  sensorTransfer.begin(loraSerial);

  delay(1000);

  readGps.setName("readGps");
  // printRtc.setName("printGps");
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
