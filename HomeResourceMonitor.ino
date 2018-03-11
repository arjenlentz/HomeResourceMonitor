/*
  Home Resource Monitor
  - Solar Hot Water use

  by Arjen Lentz
  - 2014-03-01
  - 2014-10-06  added in default UDP destination IP:port
  - 2015-05-13  added boost relay logic, control and reporting
  - 2015-05-20  changed water pulse counter from byte to unsigned int
  - 2015-05-21  added wH pulse counter, tuned boost logic
  - 2015-07-01  tuned boost logic
  - 2017-03-25  filter for sane temperature readings and jumps
  - 2018-03-11  use OneWire and CRC8 to validate sensor readings

  UDP interface:
  $ echo 'anything' | nc -w 1 -p 8888 -u 192.168.2.162 8888
  $ nc -ulp 8888

  ********************************************************************

  Water flow sensor code based on
  www.practicalarduino.com/projects/water-flow-gauge
  Copyright 2009 Jonathan Oxer <jon@oxer.com.au>
  Copyright 2009 Hugh Blemings <hugh@blemings.org>

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version. http://www.gnu.org/licenses/
*/

#include <string.h>  // for strcmp()


/* Booster relay ruleset
   if (((h >= 14 && h < 20) && temp < 42) ||
       ((h >= 4 || h < 7) && temp < 42))
     {turn on booster relay}
   if (temp > 46)
     {turn off booster relay}
*/
// presuming day time, uses h >= start && h < end
#define BOOST_ON_A_START 14
#define BOOST_ON_A_END   20
#define BOOST_ON_A_TEMP  4200     // 42.00'C

// presuming night time, uses h >= start && h < end
#define BOOST_ON_B_START  4
#define BOOST_ON_B_END    7
#define BOOST_ON_B_TEMP  4200     // 42.00'C

#define BOOST_OFF_TEMP   4600     // 46.00'C



// ----------------------------------------
/*
  Arduino pins and interrupts

  - SD card on SPI bus:
    MOSI - pin 11
    MISO - pin 12
    CLK  - pin 13
    CS   - pin  4

  - Ethernet shield
    CS   - pin 10

  - Real Time Clock via I2C
    SDA	 - pin A4
    SCL  - pin A5

  - Solar collector temp - pin 8
  - Solar vat temp       - pin 9

  - Solar hot water flow - pin 2
  - Solar hot water interrupt = 0 (D2)

  - Solar hot water boost wH pulse - pin 3
  - Solar hot water boost wH pulse interrupt = 1 (D3)

  - Solar hot water booster relay = A0 (near VIN,GND,5V)
*/

#define SPI_SD_CS_PIN 		 4
#define SPI_ETHER_CS_PIN	10
#define SHW_COLLECTOR_TEMP_PIN   8
#define SHW_VAT_TEMP_PIN	 9
#define SHW_HOTWATER_FLOW_PIN	 2
#define SHW_BOOST_RELAY_PIN     A0
#define SHW_BOOST_WH_PULSE_PIN   3

#define SHW_HOTWATER_INTERRUPT	 0
#define SHW_BOOSTPULSE_INTERRUPT 1


// ----------------------------------------
// Freetronics RTC module (DS3232)
#include <Wire.h>

#define DS3232_I2C_ADDRESS 0x68

// Convert binary coded decimal to normal decimal numbers
byte bcdToDec(byte val)
{
  return ( (val / 16 * 10) + (val % 16) );
}


char *Dec2s(byte value)
{
  static char s[3] = "\0";

  s[0] = (char)('0' + (value / 10));
  s[1] = (char)('0' + (value % 10));
  s[2] = '\0';

  return (s);
}



#if 0
// ----------------------------------------
// SD card slot
#include <SPI.h>
#include <SD.h>

File dataFile;
#endif


// ----------------------------------------
// Ether shield
// SPI already included
//#include <SPI.h>
#include <Ethernet.h>

// MAC address and IP address
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xA2
};
IPAddress ip(192, 168, 2, 162);

// Optional DNS (default .1), gateway (default .1), subnet (default 255.255.255.0)

// local port to listen on (UDP)
unsigned int localPort = 8888;

// destination defaults
IPAddress remoteIP(192, 168, 2, 160); // 151=RPi,160=DrSeuss
unsigned int remotePort = 8888;

boolean haveremoteudp = true;  // using above as destination default
boolean firsttime = true;


// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;


// ----------------------------------------
// One wire temp sensors
#include <OneWire.h>

// ringbuf of 10 items for averaging
#define SHW_RINGBUF_NUM_ITEMS 10
byte shw_ringbuf_offset;
// stored in centigrades * 100 (e.g. incl 2 decimals but without decimal point)
int shw_collector_temp_ringbuf[SHW_RINGBUF_NUM_ITEMS],
    shw_vat_temp_ringbuf[SHW_RINGBUF_NUM_ITEMS],
    last_avg_shw_collector_temp = 0,
    last_avg_shw_vat_temp = 0;

boolean shw_boost_state,
        last_shw_boost_state,
        shw_boost_override;


// ----------------------------------------
// Water flow sensor
// The hall-effect flow sensor outputs approximately
// 5.5 pulses per second per litre/minute of flow.
const float shw_hotwater_flowsensor_calibrationFactor = 5.5;

// updated by interrupt handlers
volatile unsigned int shw_hotwater_flowsensor_pulsecount;
volatile unsigned int shw_boost_pulsecount;

float shw_hotwater_flowsensor_flow_rate;
unsigned int shw_hotwater_flowsensor_flow_ml;
unsigned long shw_hotwater_flowsensor_total_ml;

unsigned long shw_hotwater_flowsensor_oldtime;



// ----------------------------------------
// interrupt handler: increment flow pulse count
void shw_hotwater_interrupt_handler()
{
  shw_hotwater_flowsensor_pulsecount++;
}



// ----------------------------------------
// interrupt handler: increment wH pulse count
void shw_boostpulse_interrupt_handler()
{
  shw_boost_pulsecount++;
}


// ----------------------------------------
// try and get sane temperature readings
// returns 1 for success, 0 for fail
// on success, *temp = centigrades * 100 (e.g. incl 2 decimals but without decimal point)
int getSaneCurrentTemp (int pin, int *temp)
{
  uint8_t data[9];
  OneWire ds(pin);

  for (int i = 0; i < 5; i++) {   // take up to 5 readings
    ds.reset();
    ds.write(0xcc);     // ROM command: skip ROM (address all devices on bus)
    ds.write(0x44, 1);  // start conversion, with parasite power on at the end

    delay(1000);        // maybe 750ms is enough, maybe not
    // we might do a ds->depower() here, but the reset will take care of it.
    ds.reset();
    ds.write(0xcc);     // ROM command: skip ROM (address all devices on bus)
    ds.write(0xbe);

    for (int n = 0; n < 9; n++)   // the DS circuits will send us 8 bytes + CRC8
      data[n] = ds.read();

    // calculate CRC8 over bytes 0-7, compare with byte 8 (transmitted CRC8)
    if (OneWire::crc8(data,8) != data[8])
      Serial.println("Temp reading CRC error");
    else {
      // Convert the data to actual temperature
      uint16_t TReading;
      int tmp, sign;

      TReading = (data[1] << 8) + data[0];
      sign = (TReading & 0x8000);             // test most sig bit
      if (sign)     // negative
        TReading = (TReading ^ 0xffff) + 1;   // 2's comp

      tmp = (6 * TReading) + TReading / 4;  // multiply by (100 * 0.0625) or 6.25
      if (sign)
        tmp = -(tmp);

      if (tmp > -500 && tmp < 10000) {      // between -5C and +100C
        *temp = tmp;
        return (1);   // succcess!
      }

      Serial.println("Temp reading out of range");
    }

    delay(500);   // waitawhile (1/2 sec)
  }

  Serial.println("Temp read fail even after 5 tries");
  return (0);   // fail
}


// ----------------------------------------
void setup()
{
  delay( 50 );  // delay to help WizNet Ethernet reset chip wake up

  // initialise serial (debugging)
  Serial.begin(9600);

  Serial.println();
  Serial.println("HomeResourceMonitor - Arjen Lentz (C) 2014-2018");

  // for the RTC
  Wire.begin();


#if 0
  // SD card slot
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);

  // see if the card is present and can be initialized:
  if (!SD.begin(SPI_SD_CS_PIN)) {
    Serial.println("SD card failed, or not present");
  }
#endif


  // initialise Ether shield
  Ethernet.begin(mac, ip);
  Udp.begin(localPort);

  Serial.print("IP addr=");
  Serial.print(Ethernet.localIP());
  Serial.print(":");
  Serial.println(localPort);

  pinMode(SHW_BOOST_RELAY_PIN, OUTPUT);
  digitalWrite(SHW_BOOST_RELAY_PIN, LOW);
  shw_boost_state = last_shw_boost_state = false;
  shw_boost_override = false;

  // initialise water flow sensor
  // (enable internal pullup resistor)
  pinMode(SHW_HOTWATER_FLOW_PIN, INPUT);
  digitalWrite(SHW_HOTWATER_FLOW_PIN, HIGH);

  // initialise running average
  // could check return value, but what would we do about a fail?
  getSaneCurrentTemp(SHW_COLLECTOR_TEMP_PIN,&last_avg_shw_collector_temp);
  getSaneCurrentTemp(SHW_VAT_TEMP_PIN,&last_avg_shw_vat_temp);
  // preload ringbuffer with something hopefully sensible
  for (int i = 0; i < SHW_RINGBUF_NUM_ITEMS; i++) {
    shw_collector_temp_ringbuf[i] = last_avg_shw_collector_temp;
    shw_vat_temp_ringbuf[i] = last_avg_shw_vat_temp;
  }

  shw_ringbuf_offset = 0;
 
  shw_hotwater_flowsensor_pulsecount = 0;
  shw_boost_pulsecount               = 0;
  shw_hotwater_flowsensor_flow_rate  = 0.0;
  shw_hotwater_flowsensor_flow_ml    = 0;
  shw_hotwater_flowsensor_total_ml   = 0;
  shw_hotwater_flowsensor_oldtime    = 0;

  // The Hall-effect sensor is connected to pin 2 which uses interrupt 0.
  // Configured to trigger on a FALLING state change
  // (transition from HIGH state to LOW state)
  attachInterrupt(SHW_HOTWATER_INTERRUPT, shw_hotwater_interrupt_handler, FALLING);

  // The energy meter pulse output is connected to pin 3 which uses interrupt 1.
  // Configured to trigger on a FALLING state change
  // (transition from HIGH state to LOW state)
  attachInterrupt(SHW_BOOSTPULSE_INTERRUPT, shw_boostpulse_interrupt_handler, FALLING);
}/*setup()*/



// ----------------------------------------
void loop()
{
  // from RTC
  byte second, minute, hour, dayofweek, day, month, year;
  // needs to be long to contain N * temp (from ringbuf) during avg calc
  long avg_shw_collector_temp, avg_shw_vat_temp;
  unsigned int shw_boostpulses;

  // ----------------------------------------
  if ((millis() - shw_hotwater_flowsensor_oldtime) > 1000) {   // Only process counters once per second
    Serial.println();
    Serial.println("---");

    // ----------------------------------------
    // RTC

    String date_str, time_str;
    int valid = false;

    do {
      Wire.beginTransmission(DS3232_I2C_ADDRESS);
      Wire.write(0); // set DS3232 register pointer to 00h
      Wire.endTransmission();
      Wire.requestFrom(DS3232_I2C_ADDRESS, 7);

      date_str = "20";

      second     = bcdToDec(Wire.read() & 0x7f);
      minute     = bcdToDec(Wire.read());
      hour       = bcdToDec(Wire.read() & 0x3f);
      dayofweek  = bcdToDec(Wire.read());   // not used
      day        = bcdToDec(Wire.read());
      month      = bcdToDec(Wire.read());
      year       = bcdToDec(Wire.read());

      if (hour <= 23 && minute <= 59 && second <= 59 &&
          year <= 100 && month <= 12 && day <= 31) {
        valid = true;
      }
      else
         Serial.println("Invalid date/time read from RTC");
    } while (!valid);

    date_str += Dec2s(year);
    date_str += '-';
    date_str += Dec2s(month);
    date_str += '-';
    date_str += Dec2s(day);

    time_str = Dec2s(hour);
    time_str += ':';
    time_str += Dec2s(minute);
    time_str += ':';
    time_str += Dec2s(second);

    Serial.print("RTC  ");
    Serial.print(date_str);
    Serial.print(' ');
    Serial.println(time_str);

    // ----------------------------------------
    // SHW temp sensors
    {
      int collector_temp, vat_temp;

      Serial.print("SHW temp");

      // we split the printing here because the getSaneCurrentTemp function
      // might print an error, and we'd like to know which sensor it's about.
      Serial.print("  collector=");
      collector_temp = last_avg_shw_collector_temp;
      if (getSaneCurrentTemp(SHW_COLLECTOR_TEMP_PIN,&collector_temp))
        Serial.print((float) collector_temp / 100.0);

      Serial.print("  vat=");
      vat_temp = last_avg_shw_vat_temp;
      if (getSaneCurrentTemp(SHW_VAT_TEMP_PIN,&vat_temp))
        Serial.print((float) vat_temp / 100.0);

      // try to sanitise wildly fluctuating readings
      // we even things out by not moving more than 1 degree from last avg
      // so if the reading was in fact correct, it'll get there after a while
      // (completely ignoring readings could create deadlocks or other grief)
      if ((collector_temp - last_avg_shw_collector_temp) > 100)
        collector_temp = last_avg_shw_collector_temp + 100;
      else if ((collector_temp - last_avg_shw_collector_temp) < -100)
        collector_temp = last_avg_shw_collector_temp - 100;

      if ((vat_temp - last_avg_shw_vat_temp) > 100)
        vat_temp = last_avg_shw_vat_temp + 100;
      else if ((vat_temp - last_avg_shw_vat_temp) < -100)
        vat_temp = last_avg_shw_vat_temp - 100;

      // we use the average over the last SHW_RINGBUF_NUM_ITEMS measurements
      // the sensor readings fluctuate slightly so otherwise we get too much noise
      shw_collector_temp_ringbuf[shw_ringbuf_offset] = collector_temp;
      shw_vat_temp_ringbuf[shw_ringbuf_offset] = vat_temp;
      shw_ringbuf_offset++;
      shw_ringbuf_offset %= SHW_RINGBUF_NUM_ITEMS;

      avg_shw_collector_temp = avg_shw_vat_temp = 0L;
      for (int i = 0; i < SHW_RINGBUF_NUM_ITEMS; i++) {
        avg_shw_collector_temp += shw_collector_temp_ringbuf[i];
        avg_shw_vat_temp       += shw_vat_temp_ringbuf[i];
      }
      avg_shw_collector_temp /= SHW_RINGBUF_NUM_ITEMS;
      avg_shw_vat_temp       /= SHW_RINGBUF_NUM_ITEMS;


      // SHW boost switch logic
      if (avg_shw_vat_temp >= BOOST_OFF_TEMP) {
        shw_boost_state = shw_boost_override = false;
      }
      else if (shw_boost_override ||
               ((hour >= BOOST_ON_A_START && hour < BOOST_ON_A_END) && avg_shw_vat_temp < BOOST_ON_A_TEMP) ||
               ((hour >= BOOST_ON_B_START && hour < BOOST_ON_B_END) && avg_shw_vat_temp < BOOST_ON_B_TEMP)) {
        shw_boost_state = true;
      }

      if (shw_boost_state != last_shw_boost_state)
        digitalWrite(SHW_BOOST_RELAY_PIN, shw_boost_state ? HIGH : LOW);
      Serial.print("  boost=");
      Serial.print(shw_boost_state);

      Serial.print("  override=");
      Serial.print(shw_boost_override);


      // Disable the interrupt while checking wH pulse count
      detachInterrupt(SHW_BOOSTPULSE_INTERRUPT);

      shw_boostpulses = shw_boost_pulsecount;
      shw_boost_pulsecount = 0;      // Reset the pulse counter so we can start incrementing again

      // Enable the interrupt again now that we've finished sending output
      attachInterrupt(SHW_BOOSTPULSE_INTERRUPT, shw_boostpulse_interrupt_handler, FALLING);

      Serial.print("  whpulses=");
      Serial.println(shw_boostpulses);


      if (firsttime ||
          avg_shw_collector_temp != last_avg_shw_collector_temp ||
          avg_shw_vat_temp != last_avg_shw_vat_temp ||
          shw_boost_state != last_shw_boost_state ||
          shw_boostpulses) {
        String shw_temp_str;
        char str[45];

        shw_temp_str = "\"";
        shw_temp_str += date_str;
        shw_temp_str += "\",\"";
        shw_temp_str += time_str;
        shw_temp_str += "\",";
        shw_temp_str += (float) avg_shw_collector_temp / 100.0;
        shw_temp_str += ',';
        shw_temp_str += (float) avg_shw_vat_temp / 100.0;
        shw_temp_str += ',';
        shw_temp_str += shw_boost_override ? '2' : (shw_boost_state ? '1' : '0');
        shw_temp_str += ',';
        shw_temp_str += shw_boostpulses;

        shw_temp_str.toCharArray(str, 45);

#if 0
        // SD
        dataFile = SD.open("SHW_TEMP.CSV", FILE_WRITE);
        if (dataFile) {
          dataFile.println(str);
          dataFile.close();
        }
        else
          Serial.println("! Can't write to SD file SHW_TEMP.CSV");
#endif

        // UDP
        if (haveremoteudp) {
          Udp.beginPacket(remoteIP, remotePort);
          Udp.write("SHW_TEMP,");
          Udp.write(str);
          Udp.write("\n");
          Udp.endPacket();
        }

        last_avg_shw_collector_temp = (int) avg_shw_collector_temp;
        last_avg_shw_vat_temp = (int) avg_shw_vat_temp;
        last_shw_boost_state = shw_boost_state;
      }
    }

    // ----------------------------------------
    // flow sensor(s)
    Serial.print("SHW hotwater");

    // Disable the interrupt while calculating flow rate
    detachInterrupt(SHW_HOTWATER_INTERRUPT);

    // Because this loop may not complete in exactly 1 second intervals we calculate
    // the number of milliseconds that have passed since the last execution and use
    // that to scale the output. We also apply the calibrationFactor to scale the output
    // based on the number of pulses per second per units of measure (litres/minute in
    // this case) coming from the sensor.
    shw_hotwater_flowsensor_flow_rate = ((1000.0 / (millis() - shw_hotwater_flowsensor_oldtime))
                                         * shw_hotwater_flowsensor_pulsecount)
                                        / shw_hotwater_flowsensor_calibrationFactor;

    // Note the time this processing pass was executed. Note that because we've
    // disabled interrupts the millis() function won't actually be incrementing right
    // at this point, but it will still return the value it was set to just before
    // interrupts went away.
    shw_hotwater_flowsensor_oldtime = millis();

    // Divide the flow rate in litres/minute by 60 to determine how many litres have
    // passed through the sensor in this 1 second interval, then multiply by 1000 to
    // convert to millilitres.
    shw_hotwater_flowsensor_flow_ml = (shw_hotwater_flowsensor_flow_rate / 60) * 1000;

    // Add the millilitres passed in this second to the cumulative total
    shw_hotwater_flowsensor_total_ml += shw_hotwater_flowsensor_flow_ml;

    // During testing it can be useful to output the literal pulse count value so you
    // can compare that and the calculated flow rate against the data sheets for the
    // flow sensor. Uncomment the following two lines to display the count value.
    Serial.print("  pulsecount=");
    Serial.print(shw_hotwater_flowsensor_pulsecount);

    // Reset the pulse counter so we can start incrementing again
    shw_hotwater_flowsensor_pulsecount = 0;

    // Enable the interrupt again now that we've finished sending output
    attachInterrupt(SHW_HOTWATER_INTERRUPT, shw_hotwater_interrupt_handler, FALLING);

    // Print the flow rate for this second in litres / minute
    Serial.print("  flowrate=");
    Serial.print(shw_hotwater_flowsensor_flow_rate, 3);

    // Print the number of millilitres flowed in this second
    Serial.print("  flowml=");             // Output separator
    Serial.print(shw_hotwater_flowsensor_flow_ml);

    // Print the cumulative total of millilitres flowed since starting
    Serial.print("  totalml=");             // Output separator
    Serial.println(shw_hotwater_flowsensor_total_ml);

    if (firsttime || shw_hotwater_flowsensor_flow_rate > 0) {
      // datalogging
      String shw_flow_str;
      char str[40];

      shw_flow_str = "\"";
      shw_flow_str += date_str;
      shw_flow_str += "\",\"";
      shw_flow_str += time_str;
      shw_flow_str += "\",";

      shw_flow_str += shw_hotwater_flowsensor_flow_rate;
      shw_flow_str += ',';
      shw_flow_str += shw_hotwater_flowsensor_flow_ml;

      shw_flow_str.toCharArray(str, 40);

#if 0
      // SD
      dataFile = SD.open("SHW_FLOW.CSV", FILE_WRITE);
      if (dataFile) {
        dataFile.println(str);
        dataFile.close();
      }
      else
        Serial.println("! Can't write to SD file SHW_FLOW.CSV");
#endif

      // UDP
      if (haveremoteudp) {
        Udp.beginPacket(remoteIP, remotePort);
        Udp.write("SHW_FLOW,");
        Udp.write(str);
        Udp.write("\n");
        Udp.endPacket();
      }
    }

    firsttime = false;

#if 0  // DEBUG
    if (haveremoteudp) {
      Udp.beginPacket(remoteIP, remotePort);
      Udp.write('.');
      Udp.endPacket();
    }
#endif

    // ----------------------------------------
  }/*if()*/


  // ----------------------------------------
  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();

  if (packetSize) {
    Serial.print("Rcvd pkt len=");
    Serial.print(packetSize);
    Serial.print(" from ");

    remoteIP = Udp.remoteIP();
    remotePort = Udp.remotePort();
    haveremoteudp = true;

    for (int i = 0; i < 4; i++) {
      Serial.print(remoteIP[i], DEC);
      if (i < 3)
        Serial.print(".");
    }
    Serial.print(":");
    Serial.println(remotePort);

    {
      char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; //buffer to hold incoming packet

      // read the packet into packetBufffer
      Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
      packetBuffer[packetSize] = '\0';
      Serial.print("Pkt=");
      Serial.println(packetBuffer);

      firsttime = true;

      // remote control of boost
      if (!strcmp(packetBuffer, "BOOST_ON")) {
        shw_boost_override = true;
        Serial.println("BOOST_ON");
      }
      else if (!strcmp(packetBuffer, "BOOST_OFF")) {
        shw_boost_override = shw_boost_state = false;
        Serial.println("BOOST_OFF");
      }
    }
  }


  // ----------------------------------------
}/*loop()*/



/* end of file */
