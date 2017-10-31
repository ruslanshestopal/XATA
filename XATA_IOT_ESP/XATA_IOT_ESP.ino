#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <WebSocketsServer.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h> 
#include <Hash.h>
#include <FS.h>


#define USE_SERIAL Serial
ESP8266WiFiMulti WiFiMulti;

ESP8266WebServer server = ESP8266WebServer(80);
WebSocketsServer webSocket = WebSocketsServer(81);

String json = "{\"t0\":0,\"t1\":0,\"t2\":0,\"t3\":0,\"t4\":0,\"t5\":0,\"h0\":0,\"h1\":0,\"h2\":0,\"d\":0}";

int st = 1;

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {

    switch(type) {
        case WStype_DISCONNECTED:
            USE_SERIAL.printf("[%u] Disconnected!\n", num);
            break;
        case WStype_CONNECTED: {
            IPAddress ip = webSocket.remoteIP(num);
            USE_SERIAL.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
            webSocket.sendTXT(num, "<connected>");
        }
            break;
        case WStype_TEXT:
            USE_SERIAL.printf("[%u] get Text: %s\n", num, payload);

            if(payload[0] == '#') {
                USE_SERIAL.printf("%s\n", payload);
            }

            break;
    }

}

void setup() {

    USE_SERIAL.begin(115200);
    USE_SERIAL.setDebugOutput(true);

    USE_SERIAL.println();
    USE_SERIAL.println();
    USE_SERIAL.println();

    SPIFFS.begin();
    /*
    Dir dir = SPIFFS.openDir("/");
    while (dir.next()) {    
      String fileName = dir.fileName();
      size_t fileSize = dir.fileSize();
      USE_SERIAL.printf("FS File: %s, \n", fileName.c_str());
    }
    USE_SERIAL.printf("\n");
    */

    for(uint8_t t = 4; t > 0; t--) {
        USE_SERIAL.printf("[SETUP] BOOT WAIT %d...\n", t);
        USE_SERIAL.flush();
        delay(1000);
    }



    WiFiMulti.addAP("IT-MANUFACTURE", "ghbdtnghbdtn");
    WiFiMulti.addAP("SURFSTATION", "");

    while(WiFiMulti.run() != WL_CONNECTED) {
        delay(200);
        Serial.print(".");
    }

    // start webSocket server
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);

    if(MDNS.begin("esp8266")) {
        USE_SERIAL.println("MDNS responder started");
    }
      server.onNotFound([](){
        if(!handleFileRead(server.uri()))
          server.send(404, "text/plain", "FileNotFound");
      });
     
     //server.serveStatic("/", SPIFFS, "/index.htm.gz");
     
     server.on("/all", HTTP_GET, [](){
          server.send(200, "text/json", json);
      });

      server.on("/on", HTTP_GET, [](){
          st = 1;
          server.send(200, "text/plain", "1");
          USE_SERIAL.println("R1");
      });

      server.on("/off", HTTP_GET, [](){
         st = 0;
          server.send(200, "text/plain", "0");
           USE_SERIAL.println("R0");
      });

      server.on("/status", HTTP_GET, [](){
          server.send(200, "text/plain", (st==1 ? "1" : "0"));
      });

    server.begin();

    // Add service to MDNS
    MDNS.addService("http", "tcp", 80);
    MDNS.addService("ws", "tcp", 81);

  ArduinoOTA.setHostname("ESP8266-00001"); // Задаем имя сетевого порта
  ArduinoOTA.begin(); // Инициализируем OTA


}

void loop() {
    webSocket.loop();
    server.handleClient();
    ArduinoOTA.handle(); 
  
  if (Serial.available() > 0) {
   String megaString = Serial.readStringUntil('\n');
   if(megaString[0] == '{') {
         //Serial.print("MEGA:");
         //Serial.println(megaString);
         json = megaString;
   }

  }
   
}

String getContentType(String filename){
  if(server.hasArg("download")) return "application/octet-stream";
  else if(filename.endsWith(".htm")) return "text/html";
  else if(filename.endsWith(".html")) return "text/html";
  else if(filename.endsWith(".css")) return "text/css";
  else if(filename.endsWith(".js")) return "application/javascript";
  else if(filename.endsWith(".png")) return "image/png";
  else if(filename.endsWith(".gif")) return "image/gif";
  else if(filename.endsWith(".jpg")) return "image/jpeg";
  else if(filename.endsWith(".ico")) return "image/x-icon";
  else if(filename.endsWith(".xml")) return "text/xml";
  else if(filename.endsWith(".pdf")) return "application/x-pdf";
  else if(filename.endsWith(".zip")) return "application/x-zip";
  else if(filename.endsWith(".gz")) return "application/x-gzip";
  return "text/plain";
}

bool handleFileRead(String path){
  USE_SERIAL.println("handleFileRead: " + path);
  if(path.endsWith("/")) path += "index.htm";
  String contentType = getContentType(path);
  String pathWithGz = path + ".gz";
  if(SPIFFS.exists(pathWithGz) || SPIFFS.exists(path)){
    if(SPIFFS.exists(pathWithGz))
      path += ".gz";
    File file = SPIFFS.open(path, "r");
    size_t sent = server.streamFile(file, contentType);
    file.close();
    return true;
  }
  return false;
}

