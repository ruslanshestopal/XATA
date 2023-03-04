#include <Wire.h>
#include <nRF24L01.h>
#include <RF24.h>

const int pwmPin = 2;
unsigned long th, tl, ppm_pwm = 0;
unsigned long prev, interval = 20000;


RF24 radio(8, 9); // CE, CSN

struct TX_DATA {
  uint8_t c;
  uint8_t s;
  uint16_t r;
};

TX_DATA tx_data;

void setup() {
  Serial.begin(9600);
  pinMode(pwmPin, INPUT_PULLUP);

  radio.begin();
  radio.setChannel(100);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_HIGH);
  radio.openWritingPipe(0xF0F1F2F3F6LL);
  radio.stopListening();


  Serial.println("Start up MH-Z14A !!");
}

void loop() {

  unsigned long now = millis();

  if (now - prev > interval ) {
    //Serial.println(".");

    do {
      Serial.print(".");
      th = pulseIn(pwmPin, HIGH, 1004000) / 1000;
      tl = 1004 - th;
      ppm_pwm = 5000 * (th - 2) / (th + tl - 4);
    } while (th == 0);

    tx_data.c = 0xC1;
    tx_data.s = 0xEE;
    tx_data.r = ppm_pwm;

    radio.write(&tx_data, sizeof(tx_data));

    Serial.println(ppm_pwm);

    prev = now;

  }


}
