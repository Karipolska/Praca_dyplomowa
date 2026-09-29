#include "connection.hpp"

Wifi_plat* Wifi_plat::_instance = nullptr;

Wifi_plat::Wifi_plat() {
    _instance = this; 
}

Wifi_plat::~Wifi_plat() {
    if (_instance != this){
      _instance = nullptr;
    } 
}

void Wifi_plat::connectToWiFi(const char *ssid, const char *pwd) {
  Serial.println("Connecting to WiFi network: " + String(ssid));

  // delete old config
  WiFi.disconnect(true);
  //register event handler

  WiFi.onEvent(WiFiEvent);  // Will call WiFiEvent() from another thread.

  //Initiate connection
  WiFi.begin(ssid, pwd);

  Serial.println("Waiting for WIFI connection...");
}

void Wifi_plat::WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      //When connected set
      Serial.print("WiFi connected! IP address: ");
      Serial.println(WiFi.localIP());
      //initializes the UDP state
      //This initializes the transfer buffer
      _instance->udp.begin(WiFi.localIP(), UDP_PORT);
      _instance->connected = true;
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      _instance->connected = false;
      break;
    default: break;
  }
}
