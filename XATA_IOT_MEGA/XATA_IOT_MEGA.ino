#include "Wire.h"
#include <Nextion.h>
#include "DHT.h"
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"
#include <ArduinoJson.h>
#include <NexVariable.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PZEM004T.h>
#include "Adafruit_SHT31.h"
#include <HTU21D.h>

#define DHTPIN 25     // what digital pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

DHT dht(DHTPIN, DHTTYPE);
DHT dht2(22, DHT21);

#define ONE_WIRE_BUS 23
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

#define ONE_WIRE_BUS2 22
OneWire oneWire2(ONE_WIRE_BUS2);
DallasTemperature sensors2(&oneWire2);




PZEM004T* pzem;
IPAddress ip(192, 168, 1, 1);

Adafruit_SHT31 sht31 = Adafruit_SHT31();

HTU21D myHTU21D(HTU21D_RES_RH11_TEMP11);


#include "ds3231.h" //https://github.com/rodan/ds3231
#define DS3231_I2C_ADDRESS 0x68

unsigned long prev, interval = 5000;
const char double_str[] PROGMEM = "%02d";
const char watt_str[] PROGMEM = "%04dW";
const char double_time_str[] PROGMEM = "%02d:%02d";
const char double_date_str[] PROGMEM = "%02d-%02d-%d";
const char topStr[] PROGMEM = "%i.%01i%c";
const char topHStr[] PROGMEM = "%i%c";

const unsigned long estart = 1766120; //37950
unsigned long eday = 0;

SoftwareSerial mySoftwareSerial(10, 11); // RX, TX
DFRobotDFPlayerMini myDFPlayer;

int currentMinute = 0;
int currentHour = 0;
int currentDay = 0;

int currentPage = 0;

int alarmHour = 7;
int alarmMinute = 0;
int alarmMask = 0; // 62  0 1 1 1 1 1 0

uint32_t red = 255;
uint32_t green = 255;
uint32_t blue = 255;
uint32_t white = 255;

uint32_t aON = 0;
uint32_t bON = 0;

static byte byte1;  // Last byte buffer
static byte byte2;  // Second last byte bufferstatic byte byte1;

#define BUFF_MAX 128

// пол
static byte ta0[8] = {0x28, 0xFF, 0x26, 0x71, 0xC2, 0x16, 0x03, 0xB3}; // ванна
static byte ta1[8] = {0x28, 0xFF, 0x66, 0x20, 0xC4, 0x16, 0x04, 0x5F};// 1
static byte ta2[8] = {0x28, 0xFF, 0x61, 0x2B, 0xC2, 0x16, 0x03, 0xC8}; // 2
static byte ta3[8] = {0x28, 0xFF, 0xB7, 0x28, 0xC2, 0x16, 0x03, 0x82}; // зал 2
// по дому
static byte ta4[8] = {0x28, 0xFF, 0x7A, 0x6C, 0x23, 0x16, 0x04, 0x4D};//- розетка
static byte ta5[8] = {0x28, 0xFF, 0x5D, 0x90, 0x23, 0x16, 0x04, 0x87};//-потолок
static byte ta6[8] = {0x28, 0xFF, 0x88, 0x8D, 0x23, 0x16, 0x04, 0xAE};// на дворе
static byte ta7[8] = {0x28, 0xFF, 0x1A, 0x8E, 0x23, 0x16, 0x04, 0x12};//- в коробке

/* 27
  0x28, 0xFF, 0x88, 0x8D, 0x23, 0x16, 0x04, 0xAE  },   // CRC OK
  0x28, 0xFF, 0x5D, 0x90, 0x23, 0x16, 0x04, 0x87  },    // CRC OK
  };
  pin29
  0x28, 0xFF, 0x7A, 0x6C, 0x23, 0x16, 0x04, 0x4D  },    // CRC OK
  0x28, 0xFF, 0x66, 0x20, 0xC4, 0x16, 0x04, 0x5F  },    // CRC OK
  0x28, 0xFF, 0x61, 0x2B, 0xC2, 0x16, 0x03, 0xC8  },    // CRC OK


*/

NexText aHour = NexText(1, 1, "t0");
NexText aMin = NexText(1, 2, "t1");

NexVariable hh = NexVariable(0, 12, "hh");
NexVariable mm = NexVariable(0, 13, "mm");
NexVariable tmp = NexVariable(0, 16, "tmp");
NexVariable hum = NexVariable(0, 17, "hum");
NexVariable ss = NexVariable(0, 18, "ss");
NexVariable sr = NexVariable(0, 19, "sr");
NexVariable ms = NexVariable(0, 20, "ms");
NexVariable mr = NexVariable(0, 21, "mr");
NexVariable wd = NexVariable(0, 22, "wd");
NexVariable ws = NexVariable(0, 23, "ws");
NexVariable ico = NexVariable(0, 24, "ico");
NexVariable ltxt = NexVariable(0, 25, "ltxt");
NexVariable amsk = NexVariable(0, 28, "amsk");
NexVariable alrm = NexVariable(0, 29, "alrm");
NexVariable uv = NexVariable(0, 30, "uv");
NexVariable date = NexVariable(0, 36, "date");
NexVariable ctmp = NexVariable(0, 37, "ctmp");

NexVariable tmp0 = NexVariable(0, 47, "tp0");
NexVariable tmp1 = NexVariable(0, 48, "tp1");
NexVariable tmp2 = NexVariable(0, 49, "tp2");
NexVariable tmp3 = NexVariable(0, 50, "tp3");
//
NexVariable tmpd = NexVariable(0, 51, "td");
NexVariable hmdd = NexVariable(0, 52, "hd");
//
NexVariable tmp4 = NexVariable(0, 53, "tmp0");
NexVariable tmp5 = NexVariable(0, 54, "tmp1");
NexVariable tmp6 = NexVariable(0, 55, "tmp2");
//электощетчик
NexVariable kwc = NexVariable(0, 56, "watt");
NexVariable pwr = NexVariable(0, 57, "cpw");
NexVariable volt = NexVariable(0, 59, "volt");
NexVariable twatt = NexVariable(0, 68, "twatt");


NexVariable tmplo = NexVariable(0, 65, "tmplo");
NexVariable tmpcu = NexVariable(0, 66, "tmpcu");
NexVariable tmphi = NexVariable(0, 67, "tmphi");


NexCheckbox c0 = NexCheckbox(1, 3, "c0");
NexCheckbox c1 = NexCheckbox(1, 4, "c1");
NexCheckbox c2 = NexCheckbox(1, 5, "c2");
NexCheckbox c3 = NexCheckbox(1, 6, "c3");
NexCheckbox c4 = NexCheckbox(1, 7, "c4");
NexCheckbox c5 = NexCheckbox(1, 8, "c5");
NexCheckbox c6 = NexCheckbox(1, 9, "c6");

NexButton hoursUp = NexButton(1, 10, "b0");
NexButton hoursDown = NexButton(1, 12, "b2");
NexButton minUp = NexButton(1, 11, "b1");
NexButton minDown = NexButton(1, 13, "b3");

NexPage page0 = NexPage(0, 0, "page0");
NexPage page1 = NexPage(1, 0, "page1");
NexPage page2 = NexPage(2, 0, "page2");
NexPage page3 = NexPage(3, 0, "page3");

NexSlider Rslider = NexSlider(3, 4, "h0");
NexSlider Gslider = NexSlider(3, 5, "h1");
NexSlider Bslider = NexSlider(3, 6, "h2");
NexSlider Wslider = NexSlider(3, 7, "h3");

NexDSButton b0 = NexDSButton(3, 1, "bt0");
NexDSButton b1 = NexDSButton(3, 2, "bt1");

NexTouch *nex_listen_list[] = {

  &c0, &c1, &c2, &c3, &c4, &c5, &c6,
  &hoursUp, &hoursDown, &minUp, &minDown,
  &page0,
  &page1,
  &page2,
  &page3,
  &Rslider, &Gslider, &Bslider, &Wslider,
  &b0, &b1,
  NULL
};


void checkMonday(void *ptr)  {
  uint32_t val = 0;
  c0.getValue(&val);
  bitWrite(alarmMask, 1, val);

}
void checkTuesday(void *ptr)  {
  uint32_t val = 0;
  c1.getValue(&val);
  bitWrite(alarmMask, 2, val);
}
void checkWednesday(void *ptr)  {
  uint32_t val = 0;
  c2.getValue(&val);
  bitWrite(alarmMask, 3, val);
}
void checkThursday(void *ptr)  {
  uint32_t val = 0;
  c3.getValue(&val);
  bitWrite(alarmMask, 4, val);
}
void checkFriday(void *ptr)  {
  uint32_t val = 0;
  c4.getValue(&val);
  bitWrite(alarmMask, 5, val);
}
void checkSaturday(void *ptr)  {
  uint32_t val = 0;
  c5.getValue(&val);
  bitWrite(alarmMask, 6, val);
}
void checkSunday(void *ptr)  {
  uint32_t val = 0;
  c6.getValue(&val);
  bitWrite(alarmMask, 0, val);
}
void hoursUpCallback(void *ptr)  {
  alarmHour++;
  if (alarmHour >= 24) {
    alarmHour = 0;
  }
  updateAlarm();
}
void hoursDownCallback(void *ptr)  {
  alarmHour--;
  if (alarmHour < 0) {
    alarmHour = 23;
  }
  updateAlarm();
}
void minUpCallback(void *ptr)  {
  alarmMinute += 5;
  if (alarmMinute >= 60) {
    alarmMinute = 0;
  }
  updateAlarm();
}
void minDownCallback(void *ptr)  {
  alarmMinute -= 1;
  if (alarmMinute < 0) {
    alarmMinute = 59;
  }
  updateAlarm();
}
void updateAlarm()  {
  char buf[5];
  sprintf_P(buf, double_str, alarmHour);
  aHour.setText(buf);
  sprintf_P(buf, double_str, alarmMinute);
  aMin.setText(buf);
  set_alarm();
}
// Page change event:
void page0PushCallback(void *ptr) {

  currentPage = 0;
  myDFPlayer.disableLoop(); //disable loop

  char buf[10];
  sprintf_P(buf, double_time_str, alarmHour, alarmMinute );
  alrm.setText(buf);
  int val = bitRead(alarmMask, currentDay);
  amsk.setValue(val);


  sprintf_P(buf, double_str, currentHour);
  hh.setText(buf);

  sprintf_P(buf, double_str, currentMinute);
  mm.setText(buf);
}
// Page change event:
void page1PushCallback(void *ptr) {

  currentPage = 1;

  c0.setValue(bitRead(alarmMask, 1));
  c1.setValue(bitRead(alarmMask, 2));
  c2.setValue(bitRead(alarmMask, 3));
  c3.setValue(bitRead(alarmMask, 4));
  c4.setValue(bitRead(alarmMask, 5));
  c5.setValue(bitRead(alarmMask, 6));
  c6.setValue(bitRead(alarmMask, 0));

  updateAlarm();

}  // End of press event
// Page change event:
void page2PushCallback(void *ptr)  {
  Serial.println("page2PushCallback");
}
// End of press event
void page3PushCallback(void *ptr)  {

  currentPage = 3;
  Rslider.setValue(red);
  Gslider.setValue(green);
  Bslider.setValue(blue);
  Wslider.setValue(white);

  b0.setValue(aON);
  b1.setValue(bON);

}  // End of press event
void switchACallback(void *ptr) {

  b0.getValue(&aON);

  if (aON == 1) {
    Serial.println("ON");
    analogWrite(7, red);
    analogWrite(6, green);
    analogWrite(5, blue);

  } else {
    Serial.println("OFF");
    analogWrite(7, 0);
    analogWrite(6, 0);
    analogWrite(5, 0);
  }
}
void switchBCallback(void *ptr) {

  b1.getValue(&bON);

  if (bON == 1) {
    analogWrite(4, white);
  } else {
    Serial.println("OFF");
    analogWrite(4, 0);
  }
}
void set_alarm(void) {

  // flags define what calendar component to be checked against the current time in order
  // to trigger the alarm - see datasheet
  // A1M1 (seconds) (0 to enable, 1 to disable)
  // A1M2 (minutes) (0 to enable, 1 to disable)
  // A1M3 (hour)    (0 to enable, 1 to disable)
  // A1M4 (day)     (0 to enable, 1 to disable)
  // DY/DT          (dayofweek == 1/dayofmonth == 0)
  uint8_t flags[5] = { 0, 0, 0, 1, 1 };

  // set Alarm1
  DS3231_set_a1(0, alarmMinute, alarmHour, 0, flags);

  // activate Alarm1
  DS3231_set_creg(DS3231_INTCN | DS3231_A1IE);
}

void setup() {
  mySoftwareSerial.begin(9600);
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial2.begin(9600);
  Serial3.begin(115200);

  pinMode(4, OUTPUT);   // sets the pins as output
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);

  sensors.begin(); // Start up the library for the Temperature Sensors
  sensors.setResolution(10);
  //sensors.setWaitForConversion(false);

  sensors2.begin();
  sensors2.setResolution(10);
  // sensors2.setWaitForConversion(false);

  nexInit();



  if (!myDFPlayer.begin(mySoftwareSerial)) {
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    // while (true);
  }


  Serial.println(F("DFPlayer Mini online."));
  myDFPlayer.setTimeOut(500);
  myDFPlayer.volume(10);  //Set volume value (0~30).


  //  sht31.begin(0x44);

  myHTU21D.begin();
  //myHTU21D.setHeater(HTU21D_ON);

  DS3231_init(DS3231_INTCN);
  DS3231_clear_a1f();
  set_alarm();


  c0.attachPop(checkMonday);
  c1.attachPop(checkTuesday);
  c2.attachPop(checkWednesday);
  c3.attachPop(checkThursday);
  c4.attachPop(checkFriday);
  c5.attachPop(checkSaturday);
  c6.attachPop(checkSunday);

  hoursUp.attachPush(hoursUpCallback, &hoursUp);
  hoursDown.attachPush(hoursDownCallback, &hoursDown);
  minUp.attachPush(minUpCallback, &minUp);
  minDown.attachPush(minDownCallback, &minDown);

  Rslider.attachPop(redCallback);
  Gslider.attachPop(greenCallback);
  Bslider.attachPop(blueCallback);
  Wslider.attachPop(whiteCallback);

  Wslider.attachPop(whiteCallback);
  Wslider.attachPop(whiteCallback);

  b0.attachPop(switchACallback, &b0);
  b1.attachPop(switchBCallback, &b1);

  page0.attachPush(page0PushCallback);  // Page press event
  page1.attachPush(page1PushCallback);  // Page press event
  page2.attachPush(page2PushCallback);  // Page press event
  page3.attachPush(page3PushCallback);  // Page press event

  while (!Serial1) { }
  pzem = new PZEM004T(&Serial1);
  pzem->setAddress(ip);



  nexSerial.print("dim=10");
  nexSerial.write(0xFF);
  nexSerial.write(0xFF);
  nexSerial.write(0xFF);

}

void redCallback(void *ptr) {
  Rslider.getValue(&red);
  if (aON == 1) {
    analogWrite(7, red);
  }
}
void greenCallback(void *ptr) {

  Gslider.getValue(&green);
  if (aON == 1) {
    analogWrite(6, green);
  }
}
void blueCallback(void *ptr) {

  Bslider.getValue(&blue);
  if (aON == 1) {
    analogWrite(5, blue);
  }
}
void whiteCallback(void *ptr) {

  Wslider.getValue(&white);
  if (bON == 1) {
    analogWrite(4, white);
  }
}



void loop() {

  unsigned long now = millis();

  // show time once in a while
  if ((now - prev > interval) && (Serial3.available() <= 0) ) {


    struct ts t;
    char buf[10];

    int humidity = (myHTU21D.readHumidity() );
    int temperature = (myHTU21D.readTemperature() * 10);

    int humidity_out = humidity; //(sht31.readHumidity() * 10);
    int temperature_out = temperature;//(sht31.readTemperature() * 10);

    uint8_t userRegisterData = 0;

    userRegisterData = myHTU21D.read8(HTU21D_USER_REGISTER_READ);
    // Serial.print(humidity);
    // Serial.print("  %  ");
    // Serial.print(temperature);
    // Serial.print("  C  ");
    // Serial.println(userRegisterData, HEX);

    if (humidity <= 15 || humidity >= 99) {
      if (userRegisterData == 0xBB) {
        Serial.println("HEATER ON");
        myHTU21D.setHeater(HTU21D_ON);

      }
      humidity = 100;
      temperature -= 18;

    } else {
      if (userRegisterData == 0xBF) {
        Serial.println("HEATER OFF");
        myHTU21D.setHeater(HTU21D_OFF);
      }

    }

    sensors.requestTemperatures();
    sensors2.requestTemperatures();


    int t0 = round( sensors.getTempC(ta0) * 10);
    int t1 = 0;//round( sensors.getTempC(ta1) * 10);
    int t2 = 0;//round( sensors.getTempC(ta2) * 10);

    int t3 = round( sensors2.getTempC(ta3) * 10);
    int t4 = 0;// round( sensors.getTempC(ta4) * 10);
    int t5 = round( sensors2.getTempC(ta5) * 10);
    int t6 = 0;//round( sensors2.getTempC(ta6) * 10);

    DS3231_get(&t);

    if (currentHour != t.hour) {
      currentHour = t.hour;
      sprintf_P(buf, double_str, t.hour);
      hh.setText(buf);

      if (currentHour % 2 == 0) {
        myDFPlayer.playFolder(2, 1);
      } else {
        myDFPlayer.playFolder(2, 2);
      }

    }
    if (currentDay != t.wday) {
      eday = 0;
      currentDay = t.wday;
      sprintf_P(buf, double_date_str, t.mday, t.mon, t.year);
      date.setText(buf);
      int val = bitRead(alarmMask, currentDay);
      amsk.setValue(val);
    }

    if (currentMinute != t.min) {
      currentMinute = t.min;
      sprintf_P(buf, double_str, t.min);
      mm.setText(buf);
    }

    if (currentPage == 0) {

      sprintf_P(buf, topHStr, humidity ,  char(37) );
      hum.setText(buf);



      sprintf_P(buf, topStr, temperature / 10, abs(temperature % 10), char(176));
      tmp.setText(buf);

      sprintf_P(buf, topStr, t0 / 10, abs(t0 % 10), char(176));
      tmp0.setText(buf);

      sprintf_P(buf, topStr, t1 / 10, abs(t1 % 10), char(176));
      tmp1.setText(buf);

      sprintf_P(buf, topStr, t2 / 10, abs(t2 % 10), char(176));
      tmp2.setText(buf);



      sprintf_P(buf, topStr, t3 / 10, abs(t3 % 10), char(176));
      tmp3.setText(buf);


      sprintf_P(buf, topStr, t4 / 10, abs(t4 % 10), char(176));
      tmp4.setText(buf);


      sprintf_P(buf, topStr, t5 / 10, abs(t5 % 10), char(176));
      tmp5.setText(buf);

      sprintf_P(buf, topStr, t6 / 10, abs(t6 % 10), char(176));
      tmp6.setText(buf);


      sprintf_P(buf, topStr, temperature_out / 10, abs(temperature_out % 10), char(176));
      tmpd.setText(buf);

      sprintf_P(buf, topHStr, humidity_out, char(37));
      hmdd.setText(buf);


      char str[140];

      sprintf(str, "{\"cmd\":\"all\", \"h1\":%d,  \"h2\":%d, \"tin\":%d, \"tout\":%d, \"t0\":%d, \"t1\":%d, \"t2\":%d, \"t3\":%d, \"t4\":%d, \"t5\":%d, \"t6\":%d}\n",
              humidity, humidity_out, temperature, temperature_out, t0, t1, t2, t3, t4, t5, t6);
      Serial3.print(str);


      //  Serial.println(str);

      int v = pzem->voltage(ip);
      int p = pzem->power(ip);
      unsigned long e = pzem->energy(ip);

      if (v < 0.0) {
        v = 0.0;
      }

      if (p < 0) {
        p = 0;
      }

      sprintf(str, "%dV", v );
      volt.setText(str);
      if (p >= 0.0) {
        sprintf(str, "%dW/h", p);
        pwr.setText(str);
      }


      int kw = (estart + e) / 1000;
      int kfp = (estart + e) % 1000;
      int fp = (float)kfp / 10;

      sprintf(str, "%d.%02d kW", kw, fp);
      twatt.setText(str);
      if (eday == 0) {
        eday = e;
      }
      unsigned long uw = (e - eday);
      sprintf(str, "%05luW", uw);

      kwc.setText(str);
      sprintf(str, "{\"cmd\":\"1\", \"v\":%d, \"p\":%d, \"e\":%lu, \"ed\":%lu}", v, p, (estart + e), uw );
      //Serial.println(str);
      Serial3.print(str);

    }

    if (DS3231_triggered_a1()) {
      DS3231_clear_a1f();
      int val = bitRead(alarmMask, currentDay);
      if (val == 1) {
        page2.show();
        myDFPlayer.loopFolder(1);
      }

    }
    prev = now;
  }
  // Run Nextion HMI display
  nexLoop(nex_listen_list);
}



void serialEvent3() {

  while (Serial3.available() ) {

    String espString = Serial3.readStringUntil('\n');
    Serial.print("ESP:>");
    Serial.println(espString);

    if (espString.startsWith("{") ) {

      DynamicJsonBuffer jsonBuffer(360);
      JsonObject& root = jsonBuffer.parseObject(espString);

      if (root != JsonObject::invalid()) {
        String cmd = root["cmd"].as<String>();
        if (cmd == "0") {
          Serial.println("SET WEATHER");
          int i = root["i"]; // 2
          const char* lf = root["fi"]; // ""У неділю ввечері: Сніг та крига до покрив завтовшки до 1 см"
          const char* sunrise = root["sr"]; // "07:40"
          const char* sunset = root["ss"]; // "16:38"
          const char* moonrise = root["mr"]; // "11:31"
          const char* moonset = root["ms"]; // "01:50"
          const char* td = root["td"];
          const char* wdir = root["wd"]; // "Пд"
          const char* wspeed = root["ws"]; // 13
          const char* uval = root["uv"]; // "0"
          const char* tl = root["tl"]; // "0"
          const char* tc = root["tc"]; // "0"
          const char* th = root["th"]; // "0"

          utf8ToByte_CStr(lf);
          utf8ToByte_CStr(wdir);

          sr.setText(sunrise);
          ss.setText(sunset);
          mr.setText(moonrise);
          ms.setText(moonset);
          wd.setText(wdir);
          uv.setText(uval);
          ctmp.setText(td);

          ws.setText(wspeed);
          ltxt.setText(lf);
          ico.setValue(i);
          //
          char buf[10];
          sprintf(buf, "%s%c", tl, char(176));
          tmplo.setText(buf);
          sprintf(buf, "%s%c", tc, char(176));
          tmpcu.setText(buf);
          sprintf(buf, "%s%c", th, char(176));
          tmphi.setText(buf);

          myDFPlayer.playFolder(2, 5);

        } else if (cmd == "set_rgb") {
          red =  root["r"];
          green =  root["g"];
          blue =  root["b"];

          if (aON == 1) {
            analogWrite(7, red);
            analogWrite(6, green);
            analogWrite(5, blue);
          }

        } else if (cmd == "rgb_on") {
          aON = 1;
          analogWrite(7, red);
          analogWrite(6, green);
          analogWrite(5, blue);
        } else if (cmd == "rgb_off") {
          aON = 0;
          analogWrite(7, 0);
          analogWrite(6, 0);
          analogWrite(5, 0);
        } else if (cmd == "cw_on") {
          bON = 1;
          analogWrite(4, white);
        } else if (cmd == "cw_off") {
          bON = 0;
          analogWrite(4, 0);
        } else if (cmd == "ww_on") {
          bON = 1;
          analogWrite(4, white);
        } else if (cmd == "ww_off") {
          bON = 0;
          analogWrite(4, 0);
        } else if (cmd == "ww_set") {
          white =  root["v"];
          if (bON == 1) {
            analogWrite(4, white);
          }
        } else if (cmd == "cw_set") {
          white =  root["v"];
          if (bON == 1) {
            analogWrite(4, white);
          }
        } else if (cmd == "time") {
          const char* now = root["t"];
          struct ts t;
          t.sec = inp2toi(now, 1);
          t.min = inp2toi(now, 3);
          t.hour = inp2toi(now, 5);
          t.wday = now[7] - 48;
          t.mday = inp2toi(now, 8);
          t.mon = inp2toi(now, 10);
          t.year = inp2toi(now, 12) * 100 + inp2toi(now, 14);
          DS3231_set(t);
        }
      } else {
        Serial.println(F("INVALID JSON!"));
      }

    }

  }


}

String utf8ToByte_Str(String s) {
  String r = "";
  char c;
  for (int i = 0; i < s.length(); i++) {
    c = utf8ToByte(s.charAt(i));

    if (c != 0) {
      r += c;
    }
  }
  return r;
}
void utf8ToByte_CStr(char* s) {
  int k = 0;
  char c;
  for (int i = 0; i < strlen(s); i++) {
    c = utf8ToByte(s[i]);
    if (c != 0) {
      s[k++] = c;
    }
  }
  s[k] = '\0';
}
byte utf8ToByte(const byte byteIn) {
  if (byteIn < 128) {
    byte1 = 0;
    byte2 = 0;
    return (byteIn);
  }
  byte lastByte = byte1;
  byte lastByte2 = byte2;

  byte2 = byte1;
  byte1 = byteIn;

  if ((lastByte >= 0xC0) && (lastByte <= 0xDF)) {
    switch (lastByte) {
      case 0xD0:
        byte1 = 0; byte2 = 0;
        return ((byteIn >> 0) & 0xFF) + 0x20; break;
      case 0xD1:
        byte1 = 0; byte2 = 0;
        return ((byteIn >> 0) & 0xFF) + 0x60; break;
    }
  }
  return (0);
}
