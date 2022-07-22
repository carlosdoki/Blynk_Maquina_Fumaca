/****************************************************************************************************************************
  WebServer.ino - Simple Arduino web server sample for SAMD21 running WiFiNINA shield
  For any WiFi shields, such as WiFiNINA W101, W102, W13x, or custom, such as ESP8266/ESP32-AT, Ethernet, etc

  WiFiWebServer is a library for the ESP32-based WiFi shields to run WebServer
  Based on and modified from ESP8266 https://github.com/esp8266/Arduino/releases
  Based on  and modified from Arduino WiFiNINA library https://www.arduino.cc/en/Reference/WiFiNINA
  Built by Khoi Hoang https://github.com/khoih-prog/WiFiWebServer
  Licensed under MIT license

  A simple web server that shows the value of the analog input pins via a web page using an ESP8266 module.
  This sketch will start an access point and print the IP address of your ESP8266 module to the Serial monitor.
  From there, you can open that address in a web browser to display the web page.
  The web page will be automatically refreshed each 20 seconds.

  For more details see: http://yaab-arduino.blogspot.com/p/wifiesp.html
 ***************************************************************************************************************************************/
#define BLYNK_TEMPLATE_ID ""
#define BLYNK_DEVICE_NAME " "
#define BLYNK_AUTH_TOKEN ""

#include "defines.h"
#include <BlynkSimpleEsp32.h>
#include <esp_dmx.h>

#define BLYNK_PRINT Serial

char auth[] = "";

int status = WL_IDLE_STATUS; // the Wifi radio's status
int reqCount = 0;            // number of requests received

WiFiWebServer server(80);

bool fetch_blynk_state = true; // true or false
BlynkTimer timer;

int transmitPin = 17;
int receivePin = 16;
int enablePin = 21;
int num_slots = 100;

dmx_port_t dmxPort = 1;

byte data[DMX_MAX_PACKET_SIZE];

int reqVolume = 0;
int reqFan = 0;

#define RelayPin1 23 // D1

// Relay State
bool toggleState_1 = LOW; // Define integer to remember the toggle state for relay 1

// Switch State
bool SwitchState_1 = LOW;

const String postForms =
    ("<html>\
<head>\
<title>Maquina Fumaca</title>\
<style>\
body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }\
</style>\
</head>\
<body>\
<h1>Tomada</h1><br>\
<form method=\"post\" enctype=\"application/x-www-form-urlencoded\" action=\"/ligar/\">\<input type=\"submit\" value=\"Ligar Maquina Tomada\"></form>\
<form method=\"post\" enctype=\"application/x-www-form-urlencoded\" action=\"/desligar/\">\<input type=\"submit\" value=\"Desligar Maquina Tomada\"></form>\
<h1>Valores Maquina Fumaca</h1><br>\
<form method=\"post\" enctype=\"application/x-www-form-urlencoded\" action=\"/postform/\">\
Volume (Ch1): <input type=\"text\" name=\"volume\" value=\"255\"><br>\
Fan (Ch2): <input type=\"text\" name=\"fan\" value=\"255\"><br>\
<input type=\"submit\" value=\"Liga\">\
</form>\
<h1>Desliga Maquina Fumaca</h1><br>\
<form method=\"post\" enctype=\"application/x-www-form-urlencoded\" action=\"/limpa/\">\
<input type=\"submit\" value=\"Desliga\">\
</form>\
</body>\
</html>");

void handleRoot()
{
  // digitalWrite(led, 1);
  server.send(200, F("text/html"), postForms);
  // digitalWrite(led, 0);
}

void handleLigar()
{
  if (server.method() != HTTP_POST)
  {
    server.send(405, F("text/plain"), F("Method Not Allowed"));
  }
  else
  {
    server.send(200, F("text/plain"), "Maquina desligada\n");
    reqVolume = 0;
    reqFan = 0;
    Blynk.virtualWrite(V0, reqVolume);
    Blynk.virtualWrite(V1, reqFan);
    Blynk.virtualWrite(V2, 1);
    data[1] = reqVolume;
    data[2] = reqFan;
    dmx_write_packet(dmxPort, data, num_slots);
    dmx_send_packet(dmxPort, num_slots);
    dmx_wait_send_done(dmxPort, DMX_PACKET_TIMEOUT_TICK);
    digitalWrite(RelayPin1, LOW);
  }
}

void handleDesligar()
{
  if (server.method() != HTTP_POST)
  {
    server.send(405, F("text/plain"), F("Method Not Allowed"));
  }
  else
  {
    server.send(200, F("text/plain"), "Maquina desligada\n");
    reqVolume = 0;
    reqFan = 0;
    Blynk.virtualWrite(V0, reqVolume);
    Blynk.virtualWrite(V1, reqFan);
    Blynk.virtualWrite(V2, 0);
    data[1] = reqVolume;
    data[2] = reqFan;
    dmx_write_packet(dmxPort, data, num_slots);
    dmx_send_packet(dmxPort, num_slots);
    dmx_wait_send_done(dmxPort, DMX_PACKET_TIMEOUT_TICK);
    digitalWrite(RelayPin1, HIGH);
  }
}
void handlePlain()
{
  if (server.method() != HTTP_POST)
  {
    server.send(405, F("text/plain"), F("Method Not Allowed"));
  }
  else
  {
    server.send(200, F("text/plain"), "Maquina desligada\n");
    reqVolume = 0;
    reqFan = 0;
    Blynk.virtualWrite(V0, reqVolume);
    Blynk.virtualWrite(V1, reqFan);
    data[1] = reqVolume;
    data[2] = reqFan;
    dmx_write_packet(dmxPort, data, num_slots);
    dmx_send_packet(dmxPort, num_slots);
    dmx_wait_send_done(dmxPort, DMX_PACKET_TIMEOUT_TICK);
  }
}

void handleForm()
{
  if (server.method() != HTTP_POST)
  {
    server.send(405, F("text/plain"), F("Method Not Allowed"));
  }
  else
  {
    String message = F("POST form was:\n");

    server.send(200, F("text/plain"), "Maquina Ligada\n");
    reqVolume = server.arg(0).toInt();
    reqFan = server.arg(1).toInt();
    Blynk.virtualWrite(V0, reqVolume);
    Blynk.virtualWrite(V1, reqFan);
    data[1] = reqVolume;
    data[2] = reqFan;
    dmx_write_packet(dmxPort, data, num_slots);
    dmx_send_packet(dmxPort, num_slots);
    dmx_wait_send_done(dmxPort, DMX_PACKET_TIMEOUT_TICK);
  }
}

void handleNotFound()
{
  // digitalWrite(led, 1);

  String message = F("File Not Found\n\n");

  message += F("URI: ");
  message += server.uri();
  message += F("\nMethod: ");
  message += (server.method() == HTTP_GET) ? F("GET") : F("POST");
  message += F("\nArguments: ");
  message += server.args();
  message += F("\n");

  for (uint8_t i = 0; i < server.args(); i++)
  {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }

  server.send(404, F("text/plain"), message);

  reqVolume = 0;
  reqFan = 0;
}

BLYNK_WRITE(V0)
{
  reqVolume = param.asInt();
  data[1] = reqVolume;
  data[2] = reqFan;
  dmx_write_packet(dmxPort, data, num_slots);
  dmx_send_packet(dmxPort, num_slots);
  dmx_wait_send_done(dmxPort, DMX_PACKET_TIMEOUT_TICK);
}

BLYNK_WRITE(V1)
{
  reqFan = param.asInt();
  data[1] = reqVolume;
  data[2] = reqFan;
  dmx_write_packet(dmxPort, data, num_slots);
  dmx_send_packet(dmxPort, num_slots);
  dmx_wait_send_done(dmxPort, DMX_PACKET_TIMEOUT_TICK);
}

BLYNK_WRITE(V2)
{
  toggleState_1 = param.asInt();
  digitalWrite(RelayPin1, !toggleState_1);
}

void printWifiStatus()
{
  // print the SSID of the network you're attached to:
  // you're connected now, so print out the data
  Serial.print(F("You're connected to the network, IP = "));
  Serial.println(WiFi.localIP());

  Serial.print(F("SSID: "));
  Serial.print(WiFi.SSID());

  // print the received signal strength:
  int32_t rssi = WiFi.RSSI();
  Serial.print(F(", Signal strength (RSSI):"));
  Serial.print(rssi);
  Serial.println(F(" dBm"));
}

void checkBlynkStatus()
{ // called every 2 seconds by SimpleTimer

  bool isconnected = Blynk.connected();
  if (isconnected == false)
  {
    // wifiFlag = 1;
    Serial.println("Blynk Not Connected");
    // digitalWrite(wifiLed, HIGH);
  }
  if (isconnected == true)
  {
    // wifiFlag = 0;
    if (!fetch_blynk_state)
    {
      Blynk.virtualWrite(V3, WiFi.localIP().toString().c_str());
      Blynk.virtualWrite(V4, WiFi.RSSI());
    }
    // digitalWrite(wifiLed, LOW);
    // Serial.println("Blynk Connected");
  }
}

BLYNK_CONNECTED()
{
  // Request the latest state from the server
  // if (fetch_blynk_state)
  // {
  //   Blynk.syncVirtual(VPIN_BUTTON_1);
  // }
  Blynk.syncVirtual(V3);
  Blynk.syncVirtual(V4);
}

void sendSensor()
{
  // readSensor();
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  Blynk.virtualWrite(V3, WiFi.localIP().toString().c_str());
  Blynk.virtualWrite(V4, WiFi.RSSI());
}

void setup()
{
  Serial.begin(115200);
  pinMode(RelayPin1, OUTPUT);
  // During Starting all Relays should TURN OFF
  digitalWrite(RelayPin1, !toggleState_1);

  dmx_config_t dmxConfig = DMX_DEFAULT_CONFIG;
  dmx_param_config(dmxPort, &dmxConfig);
  dmx_set_pin(dmxPort, transmitPin, receivePin, enablePin);

  int queueSize = 0;
  int interruptPriority = 1;
  dmx_driver_install(dmxPort, DMX_MAX_PACKET_SIZE, queueSize, NULL,
                     interruptPriority);

  dmx_set_mode(dmxPort, DMX_MODE_WRITE);
  data[0] = 0;

  while (!Serial && millis() < 5000)
    ;

  Serial.print(F("\nStarting WebServer on "));
  Serial.print(BOARD_NAME);
  Serial.print(F(" with "));
  Serial.println(SHIELD_TYPE);
  Serial.println(WIFI_WEBSERVER_VERSION);

#if WIFI_USING_ESP_AT

  // initialize serial for ESP module
  EspSerial.begin(115200);
  // initialize ESP module
  WiFi.init(&EspSerial);

  Serial.println(F("WiFi shield init done"));

#endif

#if !(ESP32 || ESP8266)

// check for the presence of the shield
#if USE_WIFI_NINA
  if (WiFi.status() == WL_NO_MODULE)
#else
  if (WiFi.status() == WL_NO_SHIELD)
#endif
  {
    Serial.println(F("WiFi shield not present"));
    // don't continue
    while (true)
      ;
  }

#if USE_WIFI_NINA
  String fv = WiFi.firmwareVersion();

  if (fv < WIFI_FIRMWARE_LATEST_VERSION)
  {
    Serial.println(F("Please upgrade the firmware"));
  }
#endif

#endif

  Serial.print(F("Connecting to SSID: "));
  Serial.println(ssid);

  status = WiFi.begin(ssid, pass);

  delay(1000);

  // attempt to connect to WiFi network
  while (status != WL_CONNECTED)
  {
    delay(500);

    // Connect to WPA/WPA2 network
    status = WiFi.status();
  }

  printWifiStatus();

  // start the web server on port 80
  server.on(F("/"), handleRoot);
  server.on(F("/limpa/"), handlePlain);
  server.on(F("/postform/"), handleForm);
  server.on(F("/ligar/"), handleLigar);
  server.on(F("/desligar/"), handleDesligar);
  server.onNotFound(handleNotFound);

  server.begin();

  timer.setInterval(2000L, checkBlynkStatus); // check if Blynk server is connected every 2 seconds
  timer.setInterval(1000L, sendSensor);       // Sending Sensor Data to Blynk Cloud every 1 second

  Blynk.config(auth);
  delay(1000);

  if (!fetch_blynk_state)
  {
    Blynk.virtualWrite(V3, WiFi.localIP().toString().c_str());
    Blynk.virtualWrite(V4, WiFi.RSSI());
  }
}

void loop()
{
  // listen for incoming clients
  Blynk.run();
  timer.run();
  server.handleClient();
  dmx_write_packet(dmxPort, data, num_slots);
  dmx_send_packet(dmxPort, num_slots);
  dmx_wait_send_done(dmxPort, DMX_PACKET_TIMEOUT_TICK);

  delay(100);
  status = WiFi.status();
  if (status != WL_CONNECTED)
  {
    reqVolume = 0;
    reqFan = 0;
    data[1] = 0;
    data[2] = 0;
    digitalWrite(RelayPin1, !LOW);
  }
}