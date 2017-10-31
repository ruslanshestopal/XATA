#include "Timer.h"

Timer t;

const int redPin = 11;
const int greenPin = 10;
const int bluePin = 9;

int count = 0;
// Настройка
void setup() {
  // Инициализация портов и выходов
  Serial.begin(115200);
  Serial3.begin(115200);


   setColourRgb(255,255,255);
   t.every(3000, takeReading);
}

// Выполнение
void loop() {
    t.update();
}

void takeReading(){
  count++;
  Serial3.print("{\"t0\":");
  Serial3.print(count);
  Serial3.println(",\"t1\":0,\"t2\":0,\"t3\":0,\"t4\":0,\"t5\":0,\"h0\":0,\"h1\":0,\"h2\":0,\"d\":0}");

}
// Проверка события на порту Serial3
void serialEvent3() {
  while (Serial3.available() ) {
   String espString = Serial3.readStringUntil('\n');
   Serial.print("ESP:");
   Serial.println(espString);
   if(espString[0] == '#') {
          uint32_t rgb = (uint32_t) strtol((const char *) &espString[1], NULL, 16);
              int r = ((rgb >> 16) & 0xFF);
              int g = ((rgb >> 8) & 0xFF);
              int b = ((rgb >> 0) & 0xFF);
              setColourRgb(r,g,b);
    }else if(espString[0] == 'R'){
         Serial.print("RES:");
         Serial.println(espString[1]);
         if(espString[1] == 49){
            setColourRgb(255,255,255);
         }else{
            setColourRgb(0,0,0); 
         }
         
    }
  }
}

void setColourRgb(unsigned int red, unsigned int green, unsigned int blue) {
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);
 }
