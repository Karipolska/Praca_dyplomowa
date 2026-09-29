#ifndef CONNECTION_HPP
#define CONNECTION_HPP

#include <WiFi.h>
#include <WiFiUdp.h>
#include <optional>

#define UDP_PORT (8888)

class Wifi_plat{
    
public:
    Wifi_plat();  
    ~Wifi_plat();

public:
    // Network configuration
    WiFiUDP udp;
    boolean connected = false;

public:
    void connectToWiFi(const char *ssid, const char *pwd);
    static void WiFiEvent(WiFiEvent_t event);

private:
    static Wifi_plat* _instance;
};

#endif