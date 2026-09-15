#include <Arduino.h>
#include <string> 
#include <Preferences.h>
#include <vector>
#include <EEPROM.h>

// HX711 circuit wiring
#define DOUT_PIN_1  (15)
#define SCK_PIN_1  (2)

#define DOUT_PIN_2  (26)
#define SCK_PIN_2  (27)

#define DOUT_PIN_3  (5)
#define SCK_PIN_3  (18)

#define DOUT_PIN_4  (32)
#define SCK_PIN_4  (33)

// Defines related to ICM 20948 imu
// SCL - 22, CDA - 21
#define SERIAL_PORT (Serial)
#define DEBUG (1)
#define WIRE_PORT (Wire)
#define AD0_VAL (1)

#define FREQ (80)

void setup();
void loop();
int sendUdpData(WiFiUDP &udp, std::vector<byte*> data);

struct __attribute__((__packed__)) TelemetryData {
    int32_t tens[4];
    int32_t acc[3];
    int32_t gyro[3];
    int32_t counter; // DODAJ TO NA POCZĄTKU    
} payload;