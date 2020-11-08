#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Hash.h>
#include <ArduinoJson.h>
#include <WebSocketsClient.h>


#define USE_SERIAL Serial

ESP8266WiFiMulti WiFiMulti;
ESP8266WebServer server (80);
WebSocketsClient webSocket;

String json = "";//"{\"t0\":0,\"t1\":0,\"t2\":0,\"t3\":0,\"t4\":0,\"t5\":0,\"h0\":0,\"h1\":0,\"h2\":0,\"d\":0}";
String ac_json = "{\"cmd\":\"-1\", \"v\":0, \"p\":0, \"e\":0,\"ed\":0}";

int st = 1;


void setup() {

  USE_SERIAL.begin(115200);
  USE_SERIAL.setDebugOutput(true);



  for (uint8_t t = 4; t > 0; t--) {
    USE_SERIAL.printf("[SETUP] BOOT WAIT %d...\n", t);
    USE_SERIAL.flush();
    delay(1000);
  }

  WiFi.mode(WIFI_STA);
  WiFi.persistent(true);
  WiFiMulti.addAP("SURFSTATION", "");

  while (WiFiMulti.run() != WL_CONNECTED) {
    delay(200);
    Serial.print(".");
  }

  if (MDNS.begin("esp8266")) {
    USE_SERIAL.println("MDNS responder started");
  }


  server.onNotFound([]() {
    server.send(404, "text/plain", "FileNotFound");
  } );

  server.on("/", []() {
    server.send(200, "text/html", "<html><head><script>var connection = new WebSocket('ws://192.168.0.11:8081/', ['ESP8266_INDEX']);connection.onopen = function () {  connection.send('Connect ' + new Date()); }; connection.onerror = function (error) {    console.log('WebSocket Error ', error);};connection.onmessage = function (e) {  console.log('Server: ', e.data);};function sendRGB() {  var r = parseInt(document.getElementById('r').value).toString(16);  var g = parseInt(document.getElementById('g').value).toString(16);  var b = parseInt(document.getElementById('b').value).toString(16);  if(r.length < 2) { r = '0' + r; }   if(g.length < 2) { g = '0' + g; }   if(b.length < 2) { b = '0' + b; }   var rgb = '#'+r+g+b;    console.log('RGB: ' + rgb); connection.send(rgb); }</script></head><body>LED Control:<br/><br/>R: <input id=\"r\" type=\"range\" min=\"0\" max=\"255\" step=\"1\" oninput=\"sendRGB();\" /><br/>G: <input id=\"g\" type=\"range\" min=\"0\" max=\"255\" step=\"1\" oninput=\"sendRGB();\" /><br/>B: <input id=\"b\" type=\"range\" min=\"0\" max=\"255\" step=\"1\" oninput=\"sendRGB();\" /><br/></body></html>");
  });

  server.on("/all", HTTP_GET, []() {
    server.send(200, "text/json", json);
  });

  server.on("/on", HTTP_GET, []() {
    st = 1;
    server.send(200, "text/plain", "1");
    Serial.println("{\"cmd\":\"2\"}");
  });

  server.on("/off", HTTP_GET, []() {
    st = 0;
    server.send(200, "text/plain", "0");
    Serial.println("{\"cmd\":\"3\"}");
  });

  server.on("/status", HTTP_GET, []() {
    server.send(200, "text/plain", (st == 1 ? "1" : "0"));
  });

  server.on("/time", HTTP_GET, []() {
    HTTPClient http;
    http.begin("http://192.168.0.11:3000/time");
    http.addHeader("Content-Type", "application/json");
    int httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK) {
      String response = http.getString();
      server.send(200, "text/json", response);
      Serial.println(response);
    } else {
      server.send(200, "text/json", "{\"cmd\":\"error\"}");
    }
    http.end();
  });
  server.on("/ac", HTTP_GET, []() {
    server.send(200, "text/json", ac_json);
  });

  server.begin();

  // Add service to MDNS
  MDNS.addService("http", "tcp", 80);
  ArduinoOTA.setHostname("ESP8266-00001");
  ArduinoOTA.begin();

  // server address, port and URL
  webSocket.begin("192.168.0.11", 8081, "/", "ESP8266");

  // event handler
  webSocket.onEvent(webSocketEvent);

  // use HTTP Basic Authorization this is optional remove if not needed
  //webSocket.setAuthorization("user", "Password");

  // try ever 5000 again if connection has failed
  webSocket.setReconnectInterval(5000);

}

void loop() {

  webSocket.loop();
  server.handleClient();
  ArduinoOTA.handle();

  if (Serial.available() > 0) {
    String megaString = Serial.readStringUntil('\n');
    webSocket.sendTXT(megaString);
    /*
    if (megaString[0] == '{') {
      DynamicJsonBuffer jsonBuffer(300);
      JsonObject& root = jsonBuffer.parseObject(megaString);

      if (root != JsonObject::invalid()) {
        String cmd = root["cmd"].as<String>();
        if (cmd == "1") {
          ac_json = megaString;
          webSocket.sendTXT(megaString);
        }
      }
    }
*/
  }

}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {

  switch (type) {
    case WStype_DISCONNECTED:
      USE_SERIAL.printf("[WSc] Disconnected!\n");
      break;
    case WStype_CONNECTED: {
        USE_SERIAL.printf("[WSc] Connected to url: %s\n", payload);

        // send message to server when Connected
        webSocket.sendTXT("Connected");
        break;
      }
    case WStype_TEXT:
      USE_SERIAL.printf("%s\n", payload);
      //char str[500];
      //sprintf(str, "%s", payload);
      //Serial.println(str);
      break;
    case WStype_BIN:
      USE_SERIAL.printf("[WSc] get binary length: %u\n", length);
      break;
  }

}
void requestForecast() {

  if ((WiFiMulti.run() == WL_CONNECTED)) {
    HTTPClient http;
    http.begin("http://192.168.0.11:3000/forecast");
    http.addHeader("Content-Type", "application/json");

    int httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK) {
      String response = http.getString();
      Serial.println(response);
    }
    http.end();
  }

}

