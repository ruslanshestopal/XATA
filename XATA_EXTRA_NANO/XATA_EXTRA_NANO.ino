#include <Wire.h>
#include <PZEM004T.h>
#include <HTU21D.h>

#include "SPI.h"      
#include "nRF24L01.h" 
#include "RF24.h"     



PZEM004T* pzem;
IPAddress ip(192, 168, 1, 1);
HTU21D myHTU21D(HTU21D_RES_RH8_TEMP12);
//HTU21D_RES_RH11_TEMP11 HTU21D_RES_RH8_TEMP12



const uint64_t pipe = 0xF0F1F2F3F4LL; 
RF24 radio(7,8); 

struct TX_DATA {
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

struct RX_DATA {
  uint8_t c;
  uint8_t s;
  uint8_t r;
};

RX_DATA rx_data;
TX_DATA tx_data;

unsigned long prev, interval = 5000;
// Arduino library for HTU21D - suggested minimun time between measurements 17sec
float   temperature = 0;
float   humidity    = 0;
uint8_t userRegisterData = 0;
int stepCount = 0;

void setup() {
  while (!Serial) { }
  Serial.begin(9600);
/*
  Wire.begin(22);
  Wire.setWireTimeout(1000, true);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
*/

  while (!Serial) { }
  pzem = new PZEM004T(&Serial);
  pzem->setAddress(ip);

  myHTU21D.begin();


 // bme.begin(0x76);

  radio.begin(); 
  radio.setChannel(100);
  radio.setDataRate(RF24_250KBPS);  // RF24_250KBPS, RF24_1MBPS или RF24_2MBPS
  radio.setPALevel(RF24_PA_HIGH); //RF24_PA_MIN=-18dBm, RF24_PA_LOW=-12dBm, RF24_PA_MED=-6dBM
  radio.openWritingPipe(pipe);
  radio.stopListening();

  Serial.print(F("EXTRA SETUP"));

}

void loop() {
  unsigned long now = millis();


  if (now - prev > interval ) {

    char buf[128];

    if (stepCount > 6) {
      stepCount = 0;
      Serial.println(F("Take mesurements"));


      selectChannel(0);
      temperature = myHTU21D.readTemperature();
      humidity = myHTU21D.readHumidity();


     // correctHumidity(humidity);
      tx_data.t0 = temperature * 10;
      tx_data.h0 = humidity;


      selectChannel(1);
      temperature = myHTU21D.readTemperature();
      tx_data.t1 = temperature * 10;
      tx_data.h1 = myHTU21D.readHumidity();

      selectChannel(2);
      temperature = myHTU21D.readTemperature();
      tx_data.t2 = temperature * 10;
      tx_data.h2 = myHTU21D.readHumidity();

      sprintf(buf, ">>TX: T0=%d T1=%d T2=%d H0=%d H1=%d H2=%d ",
              tx_data.t0, tx_data.t1, tx_data.t2, tx_data.h0, tx_data.h1, tx_data.h2);
      Serial.println(buf);
    }
    //Powert meter
    float v = pzem->voltage(ip);
    if (v < 0.0) {
      tx_data.v = 0;
      tx_data.i = 0;
      tx_data.p = 0;
    } else {
      tx_data.v = v;
      float i = pzem->current(ip);
      tx_data.i = i * 10;
      tx_data.p = v * i;
    }

    sprintf(buf, ">>TX: V=%d I=%d P=%d", tx_data.v, tx_data.i, tx_data.p);
    Serial.println(buf);

    radio.write(&tx_data, sizeof(tx_data));

    stepCount++;
    prev = now;
  }


}

void selectChannel(uint8_t i) {
  Wire.beginTransmission(0x72);
  Wire.write(1 << i);
  Wire.endTransmission();
}
void correctHumidity(float h) {
  if (h < 35 || h > 100) {
    humidity = 100;
    myHTU21D.setHeater(HTU21D_ON);
  } else {
    myHTU21D.setHeater(HTU21D_OFF);
  }
}

/*
void receiveEvent(int numBytes) {
  //Serial.print(F("EXTRA receiveEvent"));
}


void requestEvent() {
  //Serial.println(F("EXTRA requestEvent"));

  uint8_t CS = sizeof(tx_data);

  char *string_ptr = (char *) &tx_data;
  int kk = sizeof(tx_data);
  while (kk--) {
    char c = *string_ptr++;
    Wire.write(c);
  }
}
*/
