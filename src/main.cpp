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
#include <CmdBuffer.hpp>
#include <CmdCallback.hpp>
#include <CmdParser.hpp>
#include <OneWire.h>
#include <DallasTemperature.h>
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

// Pin used to trigger distance sensor
#define PIN_SENSOR_TRIGGER 11  // PA21

// Pins used to transfer serial data to LoRaWAN on loraSerial
#define PIN_LORA_RX 17  // PB09
#define PAD_LORA_RX (SERCOM_RX_PAD_1)
#define PIN_LORA_TX 16  // PB08
#define PAD_LORA_TX (UART_TX_PAD_0)

// Pins used for water and air temperature
#define ONE_WIRE_BUS_1 9  // PA19

static OneWire oneWireDock(ONE_WIRE_BUS_1);

static DallasTemperature dockWaterTemperature(&oneWireDock);

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

static CmdCallback<3> cmdCallback;
static CmdBuffer<32> cmdBuffer;
static CmdParser     cmdParser;
static char strHelp[] = "HELP";
static char strRtc[] = "RTC";
static char strDisplay[] = "DISPLAY";

static String regEx = "^R(%d%d%d%d)";    // regular expression pattern to extract distance from sensor
static uint16_t popSd = 10;  // maximum allowable population standard deviation 

void displayRtcInfo()
{
  // Get Unix time from RTC and print
  if (rtcValid)
  {
    ZonedDateTime utcTime;
    utcTime = ZonedDateTime::forUnixSeconds(rtcGetUnixTime(), utcTz);
    Serial.print(F("\nCurrent RTC: "));
    utcTime.printTo(Serial);
    Serial.println();
  }
  else
  {
    Serial.println("\nRTC not currently valid");
  }
}

void displayHelp()
{
  Serial.println(F(
    "\n"
    "Available Commands (case insensitive):\n"
    "  RTC - Display current RTC\n"
    "  display collection on|off - Display ongoing waterlevel collection information\n"
    "  display rtc on|off - Display RTC updates\n"
    "  display gps on|off - Display ongoing GPS updates\n"
    "  display regex on|off - Display ongoing regex\n"
    "    To view the current setting of a display command, leave off the on or off parameter\n"
    "    For example: \"display collection\" without the on or off will show the current setting"
  ));
}

void functHelp(CmdParser *cmdParser) 
{
  displayHelp();
}

void functDisplay(CmdParser *cmdParser)
{
    // Serial.print("display ");
    Serial.print(cmdParser->getCmdParam(0));
    Serial.print(" ");

    if (cmdParser->equalCmdParam(1, "COLLECTION")) 
    {
        Serial.println();
        Serial.print(cmdParser->getCmdParam(1));
        Serial.print(" ");
        if (cmdParser->equalCmdParam(2, "ON"))
        {
          Serial.println(cmdParser->getCmdParam(2));
          displayCollectionDetail = true;
        }
        else if (cmdParser->equalCmdParam(2, "OFF"))
        {
          Serial.println(cmdParser->getCmdParam(2));
          displayCollectionDetail = false;
        }
        else
        {
          String cmdArg = cmdParser->getCmdParam(2);
          if (cmdArg == NULL)
          {
            Serial.print(F("is currently set to: "));
            Serial.print(displayCollectionDetail ? "on" : "off");
            Serial.println(F(
              "\n  To change display of ongoing collection: display collection on; display collection off"
            ));
          } else
          {
            Serial.print(cmdParser->getCmdParam(2));
            Serial.print(F(": argument unknown ("));
            Serial.print(cmdParser->getCmdParam(2));
            Serial.println(F(")"));
            displayHelp();
          }
        }
        
    } else if (cmdParser->equalCmdParam(1, "RTC"))
    {
        Serial.println();
        Serial.print(cmdParser->getCmdParam(1));
        Serial.print(" ");
        if (cmdParser->equalCmdParam(2, "ON"))
        {
          Serial.println(cmdParser->getCmdParam(2));
          displayRtcDetail = true;
        }
        else if (cmdParser->equalCmdParam(2, "OFF"))
        {
          Serial.println(cmdParser->getCmdParam(2));
          displayRtcDetail = false;
        }
        else
        {
          String cmdArg = cmdParser->getCmdParam(2);
          if (cmdArg == NULL)
          {
            Serial.print(F("is currently set to: "));
            Serial.print(displayRtcDetail ? "on" : "off");
            Serial.println(F(
              "\n  To change display of ongoing RTC: display rtc on; display rtc off"
            ));
          } else
          {
            Serial.print(cmdParser->getCmdParam(2));
            Serial.print(F(": argument unknown ("));
            Serial.print(cmdParser->getCmdParam(2));
            Serial.println(F(")"));
            displayHelp();
          }
        }
    } else if (cmdParser->equalCmdParam(1, "GPS"))
    {
      Serial.println();
      Serial.print(cmdParser->getCmdParam(1));
      Serial.print(" ");
      if (cmdParser->equalCmdParam(2, "ON"))
      {
        Serial.println(cmdParser->getCmdParam(2));
        displayGpsDetail = true;
      }
      else if (cmdParser->equalCmdParam(2, "OFF"))
      {
        Serial.println(cmdParser->getCmdParam(2));
        displayGpsDetail = false;
      }
      else
      {
          String cmdArg = cmdParser->getCmdParam(2);
          if (cmdArg == NULL)
          {
            Serial.print(F("is currently set to: "));
            Serial.print(displayGpsDetail ? "on" : "off");
            Serial.println(F(
              "\n  To change display of ongoing gps: display gps on; display gps off"
            ));
          } else
          {
            Serial.print(cmdParser->getCmdParam(2));
            Serial.print(F(": argument unknown ("));
            Serial.print(cmdParser->getCmdParam(2));
            Serial.println(F(")"));
            displayHelp();
          }
      }
    } else if (cmdParser->equalCmdParam(1, "REGEX"))
    {
      Serial.println();
      Serial.print(cmdParser->getCmdParam(1));
      Serial.print(" ");
      if (cmdParser->equalCmdParam(2, "ON"))
      {
        Serial.println(cmdParser->getCmdParam(2));
        displayRegexDetail = true;
      }
      else if (cmdParser->equalCmdParam(2, "OFF"))
      {
        Serial.println(cmdParser->getCmdParam(2));
        displayRegexDetail = false;
      }
      else
      {
          String cmdArg = cmdParser->getCmdParam(2);
          if (cmdArg == NULL)
          {
            Serial.print(F("is currently set to: "));
            Serial.println(displayRegexDetail ? "on" : "off");
            Serial.print(F("current regex expression: "));
            Serial.println(regEx);
            Serial.println(F(
              "  To change display of ongoing regex: display regex on; display regex off"
            ));
          } else
          {
            Serial.print(cmdParser->getCmdParam(2));
            Serial.print(F(": argument unknown ("));
            Serial.print(cmdParser->getCmdParam(2));
            Serial.println(F(")"));
            displayHelp();
          }
      }
    }
    // argument unknown
    else 
    {
      Serial.println(cmdParser->getCmdParam(1));
      Serial.print(F("argument unknown ("));
      Serial.print(cmdParser->getCmdParam(1));
      Serial.println(F(")"));
      displayHelp();
    }
}

void functRtc(CmdParser *cmdParser) 
{ 
  displayRtcInfo();
}

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
      // If RTC has never been updated or it's time for an update, then start GPS time collection and update RTC.
      if ((rtcLastUpdated == 0) | ((rtcGetUnixTime() - rtcLastUpdated) >= rtcUpdateFrequencySeconds))
      {
        // Clear serial buffer to remove stale GPS sentences
        while (GPSSerial.available())
          ch = GPSSerial.read();
        gpsCollectTimer = millis();
        while ((millis() - gpsCollectTimer) < gpsSentenceCollectionMs)
        {
          while (GPSSerial.available() > 0)
          {
            if (gps.encode(GPSSerial.read()))
            {
              if (displayGpsDetail)
              {
                displayGpsInfo();
              }
            }
          }
        }
        // If both gps.time and gps.date are valid, then process RTC.
        if (gps.time.isValid() & gps.date.isValid())
        {
          utcTime = ZonedDateTime::forComponents(
              gps.date.year(), gps.date.month(), gps.date.day(), gps.time.hour(), gps.time.minute(), gps.time.second(), utcTz);
          rtcSetUnixTime(utcTime.toUnixSeconds());
          rtcLastUpdated = rtcGetUnixTime();
          rtcValid = true;
          if (displayRtcDetail)
          {
            Serial.print(F("RTC updated: "));
            utcTime.printTo(Serial);
            Serial.println();
          }
        }        
        COROUTINE_YIELD();
      }
      COROUTINE_YIELD();
    }
  }

private:
  char ch;
  uint32_t gpsCollectTimer;
  uint16_t gpsSentenceCollectionMs = 1000;
  uint32_t rtcLastUpdated = 0;
  // uint32_t rtcUpdateFrequencySeconds = 120 * 60; // update RTC every two hours
  uint32_t rtcUpdateFrequencySeconds = 60 * 60; // update RTC every hour
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
        dockWaterTemperature.requestTemperatures();
        distance = 0;
        // Clear serial buffer prior to getting a reading
        while (sensorSerial.available())
          ch = sensorSerial.read();
        // Toggle PIN_SENSOR_TRIGGER to get a reading
        digitalWrite(PIN_SENSOR_TRIGGER,HIGH);
        // Measurement is triggered by holding the ranging start/stop pin
        // high for at least 20 microseconds. This may be longer, but must
        // be low before the end of the range cycle or readings will become
        // continuous.
        delayMicroseconds(30);
        digitalWrite(PIN_SENSOR_TRIGGER,LOW);
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
              if (displayRegexDetail)
              {
                Serial.print(F("got carriage return, string: "));
                Serial.println(strValue);
              }
              // RegEx looks for a string that starts with R followed by four digits.
              // The four digits are put into a group, zero, to be later converted to
              // the reading in mm.
              matchState.Target(strValue);
              char result = matchState.Match(regEx.c_str());
              // terminate the string with a 0
              strValue[strIndex] = 0;
              if (result == REGEXP_MATCHED)
              {
                  if (displayRegexDetail)
                  {
                    Serial.println(F("RegEx Match"));
                  }
                  //convert the four digits from group zero to a number
                  distance = atoi(matchState.GetCapture (strValue, 0));
                  if (displayRegexDetail)
                  {
                      
                      Serial.println(F("Distance collected"));
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
        waterReading.utcTime = rtcGetUnixTime();
        waterReading.distance = distance;

        // If the sensor returned 0, indicating that nothing was collected,
        // or if the sensor returns 5000 or 9999, indicating that no target is
        // visible in the field of view, then don't upload the value.
        if (waterReading.distance == 0 || waterReading.distance == 5000 || waterReading.distance == 9999)
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
            // utcTime.printTo(Serial);
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
            // Serial.println();
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
              Serial.println();
            }
            // If the standard deviation is less than or equal to variable popSd in millimeters then accept the group.
            if (waterLevelGroup.pop_stdev() <= popSd)
            {
              if (displayCollectionDetail)
              {
                Serial.print(F("Group standard deviation less than 10mm; adding to waterLevelGroup"));
                Serial.println();
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
                Serial.println();
                Serial.print(F("waterLevelStats: "));
                utcTime.printTo(Serial);
                Serial.print(F(", count: "));
                Serial.print(waterLevelStats.count());
                Serial.print(F(", average: "));
                Serial.print(waterLevelStats.average());
                Serial.print(F(", pop sd: "));
                Serial.print(waterLevelStats.pop_stdev());
                Serial.println();
                // Serial.println(F("Dock water temperature: "));
                // Serial.print(F("    degrees C from getTempCByIndex: "));
                // waterTempC = dockWaterTemperature.getTempCByIndex(0);
                // Serial.println(waterTempC);
                // waterTempRaw = dockWaterTemperature.celsiusToRaw(waterTempC);
                // Serial.print(F("    1/128 degree C: "));
                // Serial.println(waterTempRaw);
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
                Serial.println();
                Serial.print(F("rejecting distances: "));
                while (!queue.isEmpty())
                {
                  Serial.print(queue.pop());
                  Serial.print(F(" "));
                }
                Serial.println();
              }
              waterLevelGroup.clear();
              queue.clear();
            }
        }

        // If it's time to send a message based on the current minute value,
        // and one has not been sent in the current minute, and at least 30
        // water level observations have been collected, then transmit the
        // average of the collected water level values.
        if (((utcTime.minute() % minutesBetweenSend) == 0) && (utcTime.minute() != minuteLastSent) && (waterLevelStats.count() >= 30))
        {
            
            minuteLastSent = utcTime.minute();
            measurementsSinceLastSend = 0;
            waterTempC = dockWaterTemperature.getTempCByIndex(0);
            waterReading.waterTemp = dockWaterTemperature.celsiusToRaw(waterTempC);
            if (displayCollectionDetail)
            {
              Serial.print(F("Measurement Ready: "));
              utcTime.printTo(Serial);
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
              Serial.print(waterReading.utcTime);
              Serial.print(F(" ("));
              utcTime.forUnixSeconds(waterReading.utcTime,utcTz).printTo(Serial);
              Serial.print(F(") "));
              Serial.print(F(", distance: "));
              Serial.print(round(waterLevelStats.average()));
              Serial.println();
              Serial.println(F("Dock water temperature: "));
              Serial.print(F("    degrees C from getTempCByIndex: "));
              Serial.println(waterTempC);
              Serial.print(F("    1/128 degree C: "));
              Serial.println(waterReading.waterTemp);
            }
            waterReading.distance = waterLevelStats.average();
            sensorTransfer.sendDatum(waterReading);
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
  float waterTempC;
  // int16_t waterTempRaw;
  const int8_t maxChars = 20;       // size of longest string returned by the MB7389
  char strValue[21];                // string big enough for characters and terminating null (maxChars +1)
  int strIndex = 0;                 // the index into the array storing the received characters
  // String regEx = "^R(%d%d%d%d)";    // regular expression pattern to extract from strValue
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
    int16_t waterTemp;
  } waterReading;
  statistic::Statistic<float, uint32_t, true> waterLevelStats;
  statistic::Statistic<float, uint32_t, true> waterLevelGroup;
  CircularBuffer<uint16_t, 20> queue;
};

class PrintRtcCoroutine : public Coroutine
{
public:
  int runCoroutine() override
  {
    COROUTINE_LOOP()
    {
      displayRtcInfo();
      COROUTINE_DELAY_SECONDS(rtcBetweenPrintSeconds);
    }
  }

private:
  uint16_t rtcBetweenPrintSeconds = 60;
  ZonedDateTime utcTime;
};

class ReadCommandLineCoroutine : public Coroutine
{
public:
  int runCoroutine() override
  {
    COROUTINE_LOOP()
    {
      // Check for new char on serial and call function if command was entered
      cmdCallback.updateCmdProcessing(&cmdParser, &cmdBuffer, &Serial);
      COROUTINE_YIELD();
    }
  }
};

GpsReadCoroutine readGps;
// PrintRtcCoroutine printRtc;
GetMeasurementCoroutine getMeasurement;
ReadCommandLineCoroutine readCommandLine;

void setup()
{

  rtcInit();

  // Assign PIN_SENSOR_TRIGGER as output to control distance sensor sampling
  pinMode(PIN_SENSOR_TRIGGER,OUTPUT);
  // Start distance sensor disabled
  digitalWrite(PIN_SENSOR_TRIGGER,LOW);

  GPSSerial.begin(9600, SERIAL_8N1);

  sensorSerial.begin(9600, SERIAL_8N1);
  // Assign sensor pins to SERCOM_ALT functionality
  pinPeripheral(PIN_SENSOR_RX, PIO_SERCOM_ALT);
  pinPeripheral(PIN_SENSOR_TX, PIO_SERCOM_ALT);

  loraSerial.begin(115200, SERIAL_8N1);
  // Assign lora pins to SERCOM_ALT functionality
  pinPeripheral(PIN_LORA_RX, PIO_SERCOM_ALT);
  pinPeripheral(PIN_LORA_TX, PIO_SERCOM_ALT);
  sensorTransfer.begin(loraSerial);

  dockWaterTemperature.setResolution(10);
  dockWaterTemperature.begin();

  delay(1000);

  readGps.setName("readGps");
  // printRtc.setName("printGps");
  getMeasurement.setName("getMeasurement");
  readCommandLine.setName("readCommand");

  cmdCallback.addCmd(strHelp, &functHelp);
  cmdCallback.addCmd(strRtc, &functRtc);
  cmdCallback.addCmd(strDisplay, &functDisplay);

  // Set parser options:
  // enable local echo
  cmdBuffer.setEcho(true);
  // set line termination to carriage return
  cmdBuffer.setEndChar(CMDBUFFER_CHAR_CR);

  // if (displayCollectionDetail || displayGpsDetail || displayRegexDetail || displayRtcDetail)
  // {
  //   Serial.begin(115200);
  //   while (!Serial); // Leonardo/Micro
  // }
  Serial.begin(115200);

  // Create a profiler on the heap for every coroutine.
  // LogBinProfiler::createProfilers();

  // Setup the scheduler.
  CoroutineScheduler::setup();
}

void loop()
{
  CoroutineScheduler::loop();
  // CoroutineScheduler::loopWithProfiler();
}
