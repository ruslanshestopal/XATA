#include <Wire.h>
#include <CircularBuffer.h>
#include <ArduinoJson.h>
#include <WebSocketsClient.h>
#include <HTU21D.h>
//#include "ds3231.h" //https://github.com/rodan/ds3231
#include "DFRobotDFPlayerMini.h"
#include<DigitalIO.h>
#include "SPI.h"      
#include "nRF24L01.h" 
#include "RF24.h"     

#define DS3231_I2C_ADDRESS 0x68


byte mac[] = { 0xDE, 0xAD, 0xBE, 0xFF, 0xBE, 0xED };
IPAddress ip(192, 168, 0, 193);
WebSocketsClient webSocket;

DFRobotDFPlayerMini mp3;


SoftSPI<SOFT_SPI_MISO_PIN, SOFT_SPI_MOSI_PIN, SOFT_SPI_SCK_PIN, SPI_MODE> spi;
RF24 radio(9, 8);

const char nx_0[] PROGMEM = "t32"; // вент1
const char nx_1[] PROGMEM = "t33";// вент2
const char nx_2[] PROGMEM = "t34";// вент3
const char nx_3[] PROGMEM = "t35";// вент4
const char nx_4[] PROGMEM = "t24";// зал2
const char nx_5[] PROGMEM = "t25";// ванна
const char nx_6[] PROGMEM = "t18";// потолок
const char nx_7[] PROGMEM = "vDC";// аккумулятор
// weather forecast
const char nx_8[] PROGMEM = "fi";
const char nx_9[] PROGMEM = "sr";
const char nx_10[] PROGMEM = "ss";
const char nx_11[] PROGMEM = "mr";
const char nx_12[] PROGMEM = "ms";
const char nx_13[] PROGMEM = "uv";
const char nx_14[] PROGMEM = "ws";
const char nx_15[] PROGMEM = "wd";
const char nx_16[] PROGMEM = "tl";
const char nx_17[] PROGMEM = "th";
const char nx_18[] PROGMEM = "tc";
const char nx_19[] PROGMEM = "ico";
// date and time
const char nx_20[] PROGMEM = "page0.mHH";
const char nx_21[] PROGMEM = "page0.mMM";
const char nx_22[] PROGMEM = "dt";
const char nx_31[] PROGMEM = "week_day";
//Nextion variables tails
const char nx_23[] PROGMEM = ".txt=\"";
const char nx_24[] PROGMEM = ".pic=";
const char nx_25[] PROGMEM = ".val=";
//
const char nx_26[] PROGMEM = "t19"; // equipment temperature
// power meter
const char nx_27[] PROGMEM = "v"; // voltage
const char nx_28[] PROGMEM = "p"; // power
const char nx_29[] PROGMEM = "e"; // energy
const char nx_30[] PROGMEM = "ed"; // energy today
const char nx_44[] PROGMEM = "TA"; // total current
const char nx_45[] PROGMEM = "TP"; // total power
// temperature and humidity (part 2)
const char nx_32[] PROGMEM = "t1"; // indoor temperature
const char nx_33[] PROGMEM = "t2"; // outtdoor temperature
const char nx_34[] PROGMEM = "t16"; // outdoor humidity
const char nx_35[] PROGMEM = "t17"; // intdoor humidity
const char nx_36[] PROGMEM = "t20"; // balcony temperature
const char nx_37[] PROGMEM = "hv"; // sensor heater
const char nx_41[] PROGMEM = "bh"; // balcony humidity
//Nextion global defaults
const char nx_38[] PROGMEM = "vent_mask";
const char nx_39[] PROGMEM = "heat_mask";
const char nx_40[] PROGMEM = "alarm_mask";
// UPS
const char nx_42[] PROGMEM = "rT";
const char nx_43[] PROGMEM = "bP";
//BMR280
const char nx_46[] PROGMEM = "hpa";
const char nx_47[] PROGMEM = "mmr";
const char nx_48[] PROGMEM = "bhu";
const char nx_49[] PROGMEM = "bte";
//MHA-Z14A co2 meter
const char nx_50[] PROGMEM = "ppm";


const char* const vars_table[] PROGMEM = {
  nx_0, nx_1, nx_2, nx_3, nx_4, nx_5,
  nx_6, nx_7, nx_8, nx_9, nx_10,
  nx_11,  nx_12, nx_13, nx_14, nx_15,
  nx_16, nx_17,  nx_18, nx_19, nx_20,
  nx_21, nx_22, nx_23,  nx_24, nx_25,
  nx_26, nx_27, nx_28, nx_29, nx_30,
  nx_31, nx_32, nx_33, nx_34, nx_35,
  nx_36, nx_37, nx_38, nx_39, nx_40,
  nx_41, nx_42, nx_43,  nx_44, nx_45,
  nx_46, nx_47, nx_48,  nx_49, nx_50,
};

const char topStr[] PROGMEM = "%i.%01i%c";
const char topHStr[] PROGMEM = "%i%c";
const char dateStr[] PROGMEM = "%02d-%02d-%d";

struct TX_DATA {
  uint8_t c;
  uint8_t s;
  uint8_t r;
};
struct RX1_DATA {
  uint16_t v;
  uint16_t i;
  uint16_t p;

  uint8_t h0;
  uint8_t h1;
  uint8_t h2;

  int16_t t0;
  int16_t t1;
  int16_t t2;

};

struct RX2_DATA {
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

struct RX3_DATA {
  uint8_t c;
  uint8_t s;
  uint16_t r;
};

TX_DATA tx_data;
RX1_DATA rx1_data;
RX2_DATA rx2_data;
RX3_DATA rx3_data;

//define slave i2c address
#define I2C_SLAVE_ADDRESS 9

unsigned long prev, interval = 5000;
uint32_t estart = 22168227; //133 579 77 22.01.2022 21:15 22534

uint32_t eday = 0;

uint8_t current_hh = 0;
uint8_t current_mm = 0;
uint8_t current_weekday = 0;
// Nextion vars
/*
   'vent_mask' variable works as a bitset
   bits 0—23 represents hours from 0 to 23. Read value bitRead(vent_mask, n-hour)
   bits 24—25 (2 bits) fan speed posible valuses are 0,1,2,3 (as on actual knobe). Read value ((1 << 2) - 1) & (vent_mask >> 24)
   bit 26 if set indicates that system is running in auto shceduler mode or manual mode if not set.
   bits 27—29 (3 bits) duration to run, posible valuses 1,2,3,4,5,6 *10 = minutes to run. Read value ((1 << 3) - 1) & (vent_mask >> 27)
*/
uint32_t vent_mask = 0;
uint32_t heat_mask = 0;
uint32_t alarm_mask = 0;

uint8_t currentPage = 0;

struct ts {
  uint8_t sec;         /* seconds */
  uint8_t min;         /* minutes */
  uint8_t hour;        /* hours */
  uint8_t mday;        /* day of the month */
  uint8_t mon;         /* month */
  int16_t year;        /* year */
  uint8_t wday;        /* day of the week */
  uint8_t yday;        /* day in the year */
  uint8_t isdst;       /* daylight saving time */
  uint8_t year_s;      /* year in short notation*/
};

ts t;




int speedArr[4] = {0, 40, 60, 100};

HTU21D myHTU21D(HTU21D_RES_RH11_TEMP11);

class Record {
  public:
    Record(const char *var_name);
    ~Record();
    void print();
    const char *cmd_name;
};
Record::Record(const char *var_name ) {
  cmd_name = (char*) malloc( strlen(var_name) * sizeof(char) + 1 );
  strcpy( cmd_name, var_name);
}
Record::~Record(void) {
  free(cmd_name);
}
void Record::print() {
  Serial2.write(cmd_name);
  Serial2.print("\xFF\xFF\xFF");

}
///
CircularBuffer<Record*, 50> tx_buffer;
CircularBuffer<Record*, 12> forecast_tx_buffer;

#define details(name) (byte*)&name,sizeof(name)

void setup() {

  Serial.begin(9600);
  Serial1.begin(9600);
  Serial2.begin(115200);

  Serial.println(F("Master setup"));


  // start the Ethernet connection:

  if (Ethernet.begin(mac) == 0) {
    Serial.println(F("Failed to configure Ethernet using DHCP"));
    Ethernet.begin(mac, ip);
  }

  webSocket.begin("192.168.0.11", 8081, "/");
  webSocket.onEvent(webSocketEvent);



  if (!mp3.begin(Serial1)) {
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    // while (true);
  }


  Serial.println(F("DFPlayer Mini online."));
  mp3.setTimeOut(500);
  mp3.volume(25);  //Set volume value (0~30).

  //DS3231_init(DS3231_INTCN);
  //DS3231_clear_a1f();
  //set_alarm();


  // myHTU21D.setHeater(HTU21D_OFF);

  // setupDefaults();

  radio.begin();
  radio.setChannel(100);


  radio.setDataRate(RF24_250KBPS); //RF24_250KBPS, RF24_1MBPS или RF24_2MBPS
  radio.setPALevel(RF24_PA_HIGH); // (RF24_PA_MIN=-18dBm, RF24_PA_LOW=-12dBm, RF24_PA_HIGH=-6dBm, RF24_PA_MAX=0dBm).

  radio.openReadingPipe(1, 0xF0F1F2F3F4LL);
  radio.openReadingPipe(2, 0xF0F1F2F3F5LL);
  radio.openReadingPipe(3, 0xF0F1F2F3F6LL);
  radio.openWritingPipe(0xFEDCBA9876LL);
  radio.startListening();
}
void setupDefaults() {
  //Reset Nextion HMI upon Arduino reload
  //72 65 73 74
  char b[4] = {0x72, 0x65, 0x73, 0x74};
  /*
    char buf[15];
    char out_buff[100];
    sprintf(buf, "%d", 0);
    MakeNextionCommand(38, 2, buf, out_buff);
    tx_buffer.push(new Record(out_buff));
    MakeNextionCommand(39, 2, buf, out_buff);
    tx_buffer.push(new Record(out_buff));
    MakeNextionCommand(40, 2, buf, out_buff);
    tx_buffer.push(new Record(out_buff));
  */
  tx_buffer.push(new Record(b));

  while (!tx_buffer.isEmpty()) {
    Record* record = tx_buffer.shift();
    record->print();
    delete record;
  }
  tx_data.c = 0xC1;
  tx_data.s = 0;
  tx_data.r = 0;

  // TX.sendData(I2C_SLAVE_ADDRESS);

}
void MakeNextionCommand(uint8_t index, uint8_t type,  char * payload,  char *out_buff) {

  char buff[strlen(payload) * sizeof(char) + 20 ];
  out_buff[0] = (char)0;
  //char * cmd_buff = (char *) malloc (250);

  strcpy_P(buff, (char*)pgm_read_word(&(vars_table[index])));

  strcat(out_buff, buff);

  if (type == 0) {
    strcpy_P(buff, (char*)pgm_read_word(&(vars_table[23])));
    strcat(out_buff, buff);
    strcat(out_buff,  (char*)payload);

  } else if (type == 1) {
    strcpy_P(buff, (char*)pgm_read_word(&(vars_table[24])));
    strcat(out_buff, buff);
    sprintf(buff,  (char*)payload);
    strcat(out_buff, buff);
  } else {
    strcpy_P(buff, (char*)pgm_read_word(&(vars_table[25])));
    strcat(out_buff, buff);
    sprintf(buff, (char*)payload);
    strcat(out_buff, buff);
  }
  if (type == 0) {
    strcpy(buff, "\"");
    strcat(out_buff, buff);
  }
}

void loop() {

  //webSocket.loop();
  uint8_t pipe;

  if (radio.available(&pipe)) {
    char buf[120];
    switch (pipe) {
      case 1:
        radio.read(&rx1_data, sizeof(rx1_data));
        sprintf(buf, ">>RX1: T0=%d T1=%d T2=%d  H0=%d H1=%d H2=%d  | V=%d I=%d P=%d",
                rx1_data.t0, rx1_data.t1, rx1_data.t2,
                rx1_data.h0, rx1_data.h1, rx1_data.h2,
                rx1_data.v, rx1_data.i, rx1_data.p);
        Serial.println(buf);
        break;
      case 2:
        radio.read(&rx2_data, sizeof(rx2_data));
        sprintf(buf, ">>RX2: T0=%d T1=%d T2=%d  T3=%d T4=%d T5=%d T6=%d   |  P=%d E=%ld",
                rx2_data.t0, rx2_data.t1, rx2_data.t2,
                rx2_data.t3, rx2_data.t4, rx2_data.t5,
                rx2_data.t6, rx2_data.p, rx2_data.e);
        Serial.println(buf);
        break;
      case 3:
        radio.read(&rx3_data, sizeof(rx3_data));
        Serial.println(F("RX3"));
        sprintf(buf, ">>RX3: CMD=%d S=%d PPM=%d", rx3_data.c, rx3_data.s, rx3_data.r);
        Serial.println(buf);

        break;
      default:
        Serial.println(F("unexpected packet received"));
        break;
    }


  }

  unsigned long now = millis();

  if (now - prev > interval ) {
    Serial.println("Request data");
    if (!forecast_tx_buffer.isEmpty()) {
      // forecast quee is pending
      sendForecast();
    }

    tx_data.c = 0xC0;
    radio.stopListening();
    radio.write(&tx_data, sizeof(tx_data));
    radio.startListening();

    makeAndSendToNEXTION();

    prev = now;


  }
  // Serial.println("RX2 check");

}

void collectI2Cdata(uint8_t *rx_data, int rx_size, int i2c_address) {

  byte rx_buffer[rx_size];

  Wire.requestFrom(i2c_address, rx_size, true);
  if (Wire.available() == rx_size) {
    for (int i = 0; i < rx_size; i++) {
      rx_buffer[i] = Wire.read();
      //Serial.print("0x");
      //Serial.print(rx_buffer[i], HEX);
      //Serial.print(",");
    }
    memcpy(rx_data, &rx_buffer, rx_size);
  }


}

void makeAndSendToNEXTION() {

  char buf[120];
  char out_buff[250];

  // DS3231_get(&t);

  if (current_weekday != t.wday) {
    eday = 0;
    current_weekday = t.wday;
  }
  if (current_mm != t.min) {
    current_mm = t.min;
  }

  sprintf(buf, "%d", t.mday);
  MakeNextionCommand(22, 0, buf, out_buff);
  tx_buffer.push(new Record(out_buff));

  sprintf(buf, "%d", t.min);
  MakeNextionCommand(21, 2, buf, out_buff);
  tx_buffer.push(new Record(out_buff));


  sprintf(buf, "%d", t.hour);
  MakeNextionCommand(20, 2, buf, out_buff);
  tx_buffer.push( new Record(out_buff));

  int temperature = 0;
  int humidity = 0;



  int te = 22;//DS3231_get_treg() * 10; // аппаратура
  sprintf_P(buf, topStr, te / 10, abs(te % 10), char(176));
  MakeNextionCommand(26, 0, buf, out_buff);
  tx_buffer.push(new Record(out_buff));

  if (current_hh != t.hour) {
    current_hh = t.hour;
    if (current_hh % 2 == 0) {
      mp3.playFolder(2, 1);
    } else {
      mp3.playFolder(2, 2);
    }
  }


  sprintf(buf, "%d", rx1_data.i);
  MakeNextionCommand(44, 2, buf, out_buff);
  tx_buffer.push(new Record(out_buff));

  sprintf(buf, "%d", rx1_data.p);
  MakeNextionCommand(45, 2, buf, out_buff);
  tx_buffer.push(new Record(out_buff));

  sprintf(buf, "%d", rx1_data.v);
  MakeNextionCommand(27, 2, buf, out_buff);
  tx_buffer.push(new Record(out_buff));

  //
  sprintf_P(buf, topStr, rx1_data.t0 / 10, abs(rx1_data.t0 % 10), char(176));
  MakeNextionCommand(33, 0, buf, out_buff);
  tx_buffer.push(new Record(out_buff));

  sprintf_P(buf, topHStr, rx1_data.h0, char(37));
  MakeNextionCommand(35, 0, buf, out_buff);
  tx_buffer.push(new Record(out_buff));


  sprintf_P(buf, topStr, rx1_data.t1 / 10, abs(rx1_data.t1 % 10), char(176));
  MakeNextionCommand(32, 0, buf, out_buff);
  tx_buffer.push(new Record(out_buff));


  sprintf_P(buf, topHStr, rx1_data.h1,  char(37));
  MakeNextionCommand(34, 0, buf, out_buff);
  tx_buffer.push(new Record(out_buff));

  sprintf_P(buf, topStr, rx1_data.t2 / 10, abs(rx1_data.t2 % 10), char(176));
  MakeNextionCommand(36, 0, buf, out_buff);
  tx_buffer.push(new Record(out_buff));

  sprintf_P(buf, topHStr, rx1_data.h2,  char(37));
  MakeNextionCommand(41, 0, buf, out_buff);
  tx_buffer.push(new Record(out_buff));
  //
  //
  if (currentPage == 0 || currentPage == 5) {
    //collect ventilation ducts temperature
    sprintf_P(buf, topStr, rx2_data.t0 / 10, abs(rx2_data.t0 % 10), char(176));
    MakeNextionCommand(0, 0, buf, out_buff);
    tx_buffer.push(new Record(out_buff));

    sprintf_P(buf, topStr, rx2_data.t1 / 10, abs(rx2_data.t1 % 10), char(176));
    MakeNextionCommand(1, 0, buf, out_buff);
    tx_buffer.push(new Record(out_buff));

    sprintf_P(buf, topStr, rx2_data.t2 / 10, abs(rx2_data.t2 % 10), char(176));
    MakeNextionCommand(2, 0, buf, out_buff);
    tx_buffer.push(new Record(out_buff));

    sprintf_P(buf, topStr, rx2_data.t3 / 10, abs(rx2_data.t3 % 10), char(176));
    MakeNextionCommand(3, 0, buf, out_buff);
    tx_buffer.push(new Record(out_buff));
  }



  sprintf_P(buf, topStr, rx2_data.t4 / 10, abs(rx2_data.t4 % 10), char(176));
  MakeNextionCommand(4, 0, buf, out_buff);
  tx_buffer.push(new Record(out_buff));

  sprintf_P(buf, topStr, rx2_data.t5 / 10, abs(rx2_data.t5 % 10), char(176));
  MakeNextionCommand(5, 0, buf, out_buff);
  tx_buffer.push(new Record(out_buff));

  sprintf_P(buf, topStr, rx2_data.t6 / 10, abs(rx2_data.t6 % 10), char(176));
  MakeNextionCommand(6, 0, buf, out_buff);
  tx_buffer.push(new Record(out_buff));


  if (eday == 0) {
    eday = rx2_data.e;
  }

  sprintf(buf, "%d", rx2_data.p);
  MakeNextionCommand(28, 2, buf, out_buff);
  tx_buffer.push(new Record(out_buff));

  if (rx2_data.e > 0) {

    uint32_t  uw = (rx2_data.e - eday);
    int kw = (estart + rx2_data.e) / 1000;

    sprintf(buf, "%d", kw);
    MakeNextionCommand(29, 2, buf, out_buff);
    tx_buffer.push(new Record(out_buff));

    sprintf(buf, "%ld", uw);
    MakeNextionCommand(30, 2, buf, out_buff);
    tx_buffer.push(new Record(out_buff));
  }

  //BME280
  int hp = rx2_data.hp / 100;
  int mmr = rx2_data.hp / 133.322368421;

  sprintf(buf, "%d", hp);
  MakeNextionCommand(46, 0, buf, out_buff);
  tx_buffer.push(new Record(out_buff));

  sprintf(buf, "%d", mmr );
  MakeNextionCommand(47, 0, buf, out_buff);
  tx_buffer.push(new Record(out_buff));

  sprintf_P(buf, topStr, rx2_data.t7 / 10, abs(rx2_data.t7 % 10), char(176));
  MakeNextionCommand(49, 0, buf, out_buff);
  tx_buffer.push(new Record(out_buff));
  // bathroom hymidity
  sprintf_P(buf, topHStr, rx2_data.h0, char(37));
  MakeNextionCommand(48, 0, buf, out_buff);
  tx_buffer.push(new Record(out_buff));

  //MHA-Z14A co2 meter
  sprintf(buf, "%d", rx3_data.r);
  MakeNextionCommand(50, 0, buf, out_buff);
  tx_buffer.push(new Record(out_buff));

  while (!tx_buffer.isEmpty()) {
    Record* record = tx_buffer.shift();

    char * bufa = record->cmd_name;
    size_t buf_len = strlen(bufa) * sizeof(char);
    webSocket.sendBIN( (uint8_t *)bufa, buf_len);

    record->print();
    delete record;
  }

}

void manageCommand(const char* payload) {

  DynamicJsonDocument doc(440);
  deserializeJson(doc, payload);

  const char* cmd = doc["cmd"]; // "F"
  char buf[10];
  char out_buff[250];

  if (String(cmd) == "F") {
    int i = doc["i"]; // 7
    const char* fi = doc["fi"]; //
    const char* sr = doc["sr"]; // "07:55"
    const char* ss = doc["ss"]; // "16:04"
    const char* mr = doc["mr"]; // "17:12"
    const char* ms = doc["ms"]; // "09:50"
    const char* uv = doc["uv"]; // "0"
    const char* ws = doc["ws"]; // "14.8"
    const char* wd = doc["wd"]; // "Пд-Сх"
    const char* tl = doc["tl"]; // "4"
    const char* th = doc["th"]; // "7.4"
    const char* tc = doc["tc"]; // "6.2"



    utf8ToByte_CStr(fi);
    MakeNextionCommand(8, 0, fi, out_buff);
    forecast_tx_buffer.push(new Record(out_buff));

    MakeNextionCommand(9, 0, sr, out_buff);
    forecast_tx_buffer.push(new Record(out_buff));

    MakeNextionCommand(10, 0, ss, out_buff);
    forecast_tx_buffer.push(new Record(out_buff));

    MakeNextionCommand(11, 0, mr, out_buff);
    forecast_tx_buffer.push(new Record(out_buff));

    MakeNextionCommand(12, 0, ms, out_buff);
    forecast_tx_buffer.push(new Record(out_buff));

    MakeNextionCommand(13, 0, uv, out_buff);
    forecast_tx_buffer.push(new Record(out_buff));

    MakeNextionCommand(14, 0, ws, out_buff);
    forecast_tx_buffer.push(new Record(out_buff));

    utf8ToByte_CStr(wd);
    MakeNextionCommand(15, 0, wd, out_buff);
    forecast_tx_buffer.push(new Record(out_buff));

    MakeNextionCommand(16, 0, tl, out_buff);
    forecast_tx_buffer.push(new Record(out_buff));

    MakeNextionCommand(17, 0, th, out_buff);
    forecast_tx_buffer.push(new Record(out_buff));

    MakeNextionCommand(18, 0, tc, out_buff);
    forecast_tx_buffer.push(new Record(out_buff));

    sprintf(buf, "%d", i);
    MakeNextionCommand(19, 1, buf, out_buff);
    forecast_tx_buffer.push(new Record(out_buff));

    if (tx_buffer.isEmpty()) {
      sendForecast();
    }
  } else if (String(cmd) == "T") {
    //
    const char* now = doc["t"];
    //struct ts t;
    t.sec = inp2toi(now, 1);
    t.min = inp2toi(now, 3);
    t.hour = inp2toi(now, 5);
    t.wday = now[7] - 48;
    t.mday = inp2toi(now, 8);
    t.mon = inp2toi(now, 10);
    t.year = inp2toi(now, 12) * 100 + inp2toi(now, 14);
    //DS3231_set(t);

  } else if (String(cmd) == "U") {

    int s = doc["s"]; // 2
    int av = doc["av"]; // 1738
    int l = doc["l"]; // 0
    int c = doc["c"]; // 1000
    int r = doc["r"]; // 111
    int dc = doc["dc"]; // 135
    int e = doc["e"]; // 65
    long on = doc["on"]; // 1639916175
    long off = doc["off"]; // 1639916286
    int tb = doc["tb"]; // 0
    int ob = doc["ob"]; // 4889


    sprintf(buf, "%d", dc);
    MakeNextionCommand(7, 2, buf, out_buff);
    tx_buffer.push(new Record(out_buff));

    sprintf(buf, "%d", r);
    MakeNextionCommand(42, 2, buf, out_buff);
    tx_buffer.push(new Record(out_buff));

    sprintf(buf, "%d", c);
    MakeNextionCommand(43, 2, buf, out_buff);
    tx_buffer.push(new Record(out_buff));


    while (!tx_buffer.isEmpty()) {
      Record* record = tx_buffer.shift();
      record->print();
      delete record;
    }

  }

}

void sendForecast() {
  while (!forecast_tx_buffer.isEmpty()) {
    Record* record = forecast_tx_buffer.shift();
    record->print();
    delete record;
  }
  mp3.playFolder(2, 5);
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.println("[WSc] Disconnected!\n");
      break;
    case WStype_CONNECTED:
      {
        Serial.print("[WSc] Connected to url: ");
        Serial.println((char *)payload);
      }
      break;
    case WStype_TEXT:
      // Serial.print("[WSc] get text: ");
      // Serial.println((char *)payload);
      manageCommand((const char*) payload);
      break;
    case WStype_BIN:
      break;
  }

}



void serialEvent2() {
  while (Serial2.available() > 0) {
    char c = Serial2.read();
    if (c == 0x65)    {
      delay(10);
      if (Serial2.available() >= 6) {
        static uint8_t buffer[8];
        buffer[0] = c;
        uint8_t i;
        for (i = 1; i < 7; i++)
          buffer[i] = Serial2.read();
        buffer[i] = 0x00;
        if (buffer[4] == 0xFF && buffer[5] == 0xFF && buffer[6] == 0xFF) {
          Serial.print(">>>>buffer: ");
          Serial.println( buffer[2]);
          if (buffer[2] == 0x33) {
            webSocket.sendTXT("{\"cmd\":\"F\"}");
          }

        }
      }
    } else if (c == 0x23) {
      // Variable arrived
      String str = Serial2.readStringUntil(';');
      handleVariable(str);
    }
  }

}
void handleVariable(String str) {
  Serial.print("Master buffer: ");
  Serial.println(str);
  char v = str.charAt(0);

  str.remove(0, 1);
  Serial.println(str);
  uint32_t num = str.toInt();
  Serial.println(num);
  Serial.print("V:");
  Serial.println(v);

  tx_data.c = 0xC1;

  if (v == 0x56) {
    //Ventelation
    vent_mask = num;
    checkVentelationScheduler();

  } else if (v == 0x41) {
    // Alarm

  } else if (v == 0x48) {
    // Heating
    heat_mask = num;
    checkVentelationScheduler();
  }




}
void checkVentelationScheduler() {
  char buf[120];

  int fan_speed = ((1 << 2) - 1) & (vent_mask >> 24);
  int is_auto  = bitRead(vent_mask, 26);

  tx_data.r = heat_mask;
  tx_data.s = 0;
  tx_data.c = 0xC1;

  int duration =  ((1 << 3) - 1) & (vent_mask >> 27);
  int hh_checked = bitRead(vent_mask, t.hour);

  if (is_auto == 1) {

    if (hh_checked == 1) {
      //int duration =  ((1 << 3) - 1) & (vent_mask >> 27);
      if (duration == 6 || t.min < (duration * 10) ) {
        tx_data.s = 100;
        tx_data.r = heat_mask | 16;
        //70 61 67 65 35 2e 74 6d 30 2e 65 6e 3d 31
        /*
          char b[14] = {0x70, 0x61, 0x67, 0x65, 0x35, 0x2E, 0x74, 0x6D, 0x30, 0x2E, 0x65, 0x6E, 0x3D, 0x31};
          tx_buffer.push(new Record(b));

          while (!tx_buffer.isEmpty()) {
          Record* record = tx_buffer.shift();
          record->print();
          delete record;
          }
        */
      }
    }

  } else {
    // Manual mode
    tx_data.s = speedArr[fan_speed];
    if (fan_speed > 0) {
      tx_data.r = heat_mask | 16;
    }
  }

  //TX.sendData(I2C_SLAVE_ADDRESS);

  //sprintf(buf, "Date %d:%d duration: %d speed: %d auto: %d check: %d", t.hour, t.min, duration, fan_speed, is_auto, hh_checked);
  //Serial.println(buf);
}
void selectChannel(uint8_t i) {
  Wire.beginTransmission(0x72);
  Wire.write(1 << i);
  Wire.endTransmission();
}

uint8_t inp2toi(char *cmd, const uint16_t seek) {
  uint8_t rv;
  rv = (cmd[seek] - 48) * 10 + cmd[seek + 1] - 48;
  return rv;
}
// Text encoding ISO-8859-5 for NEXTION HMI
static byte byte1;  // Last byte buffer
static byte byte2;  // Second last byte buffer
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
