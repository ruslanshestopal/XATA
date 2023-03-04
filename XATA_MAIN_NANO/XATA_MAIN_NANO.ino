#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PZEM004T.h>
#include "Dimmer.h"
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

RF24 radio(5, 6); // CE, CSN

#define ONE_WIRE_BUS 4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

Adafruit_BME280 bme;
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();

static byte ta0[8] = {0x28, 0x8D, 0xA3, 0x07, 0xD6, 0x01, 0x3C, 0x54};// 1
static byte ta1[8] = {0x28, 0x5B, 0xD1, 0x07, 0xD6, 0x01, 0x3C, 0xCC};//2
static byte ta2[8] = {0x28, 0xE9, 0xCB, 0x07, 0xD6, 0x01, 0x3C, 0x64};//3
static byte ta3[8] = {0x28, 0xEA, 0x75, 0x07, 0xD6, 0x01, 0x3C, 0xD0 };//4

static byte ta4[8] = {0x28, 0x82, 0x22, 0x07, 0xD6, 0x01, 0x3C, 0x76}; // зал 2
static byte ta5[8] = {0x28, 0xFF, 0x1A, 0x8E, 0x23, 0x16, 0x04, 0x12}; // ванна
static byte ta6[8] = {0x28, 0xFF, 0x5D, 0x90, 0x23, 0x16, 0x04, 0x87}; //-потолок


Dimmer dimmer(3);
PZEM004T* pzem;
IPAddress ip(192, 168, 1, 1);


struct TX_DATA {
  int16_t t0;
  int16_t t1;
  int16_t t2;
  int16_t t3;
  int16_t t4;
  int16_t t5;
  int16_t t6;
  int16_t t7;
  uint8_t h0;
  int16_t hp;
  int16_t p;
  int32_t e;
};
struct RX_DATA {
  uint8_t c;
  uint8_t s;
  uint8_t r;
};

TX_DATA tx_data;
RX_DATA rx_data;

int latchPin = 12;
int clockPin = 8;
int dataPin = 11;

int relayVal = 0;
int stepCount = 11;

unsigned long prev, interval = 5000;


void setup() {

  Serial.begin(9600);
  //pinMode(latchPin, OUTPUT);
  // pinMode(dataPin, OUTPUT);
  // pinMode(clockPin, OUTPUT);

  sensors.begin();
  sensors.setWaitForConversion(false);
  sensors.setResolution(10);


  dimmer.begin();

  //char testet[3] = {0x31, 0x38, 0x33};
  //int number = atoi(testet);
  //Serial.println(number);

  while (!Serial) { }
  pzem = new PZEM004T(&Serial);
  pzem->setAddress(ip);

  radio.begin();
  radio.setChannel(100);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_HIGH);
  radio.openReadingPipe(1, 0xFEDCBA9876LL);
  radio.openWritingPipe(0xF0F1F2F3F5LL);
  radio.startListening();


  // updateRelay(0);
  Serial.println(F("Slave setup"));
}


void loop() {

  if ( radio.available() ) {

    radio.read(&rx_data, sizeof(rx_data));

    if (rx_data.c == 0xC0) {
      Serial.println("ASK");
    } else if (rx_data.c == 0xC1) {
      Serial.println("SET");
      dimmer.set(rx_data.s);
      updateRelay(rx_data.r);
    }

  }

  unsigned long now = millis();

  if (now - prev > interval ) {
    if (stepCount > 6) {
      stepCount = 0;

      sensors.requestTemperatures();

      int t0 = sensors.getTempC(ta0) * 10;
      int t1 = sensors.getTempC(ta1) * 10;
      int t2 = sensors.getTempC(ta2) * 10;
      int t3 = sensors.getTempC(ta3) * 10;

      int t4 = sensors.getTempC(ta4) * 10;
      int t5 = sensors.getTempC(ta5) * 10;
      int t6 = sensors.getTempC(ta6) * 10;

      tx_data.t0 = t0;
      tx_data.t1 = t1;
      tx_data.t2 = t2;
      tx_data.t3 = t3;
      tx_data.t4 = t4;
      tx_data.t5 = t5;
      tx_data.t6 = t6;


      // BME280

      /*
        sensors_event_t temp_event, pressure_event, humidity_event;
        bme_temp->getEvent(&temp_event);
        bme_pressure->getEvent(&pressure_event);
        bme_humidity->getEvent(&humidity_event);
      */


      tx_data.hp = 100467;//pressure_event.pressure *100;
      tx_data.t7 = 201;//temp_event.temperature * 10;
      tx_data.h0 = 56;//humidity_event.relative_humidity;
    }

    stepCount++;



    int v = pzem->voltage(ip);
    if (v <= 0) {
      tx_data.p = 0;
    } else {
      int p = pzem->power(ip);
      int32_t e = pzem->energy(ip);
      tx_data.p = p;
      tx_data.e = e;
    }



    radio.stopListening();
    radio.write(&tx_data, sizeof(tx_data));
    radio.startListening();

    prev = now;
    char buf[128];
    sprintf(buf, ">>TX2: T0=%d T1=%d T2=%d  T3=%d T4=%d T5=%d T6=%d   |  P=%d E=%d",
            tx_data.t0, tx_data.t1, tx_data.t2,
            tx_data.t3, tx_data.t4, tx_data.t5,
            tx_data.t6, tx_data.p, tx_data.e);
    Serial.println(buf);
  }


}
void updateRelay(int num) {
  if (num == relayVal && num != 0) {
    return;
  }
  relayVal = num;
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, relayVal);
  digitalWrite(latchPin, HIGH);
}
