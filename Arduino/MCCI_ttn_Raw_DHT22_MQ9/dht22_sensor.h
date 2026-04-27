#ifndef DHT22_SENSOR_H
#define DHT22_SENSOR_H

#include "Arduino.h"
#include <DHT.h>

#define DHT22_PIN  4
#define DHT22_TYPE DHT22

extern DHT dht22;

void  dht22_Init();
float dht22_getTemperature();   // °C, 0.0 on read failure
float dht22_getHumidity();      // %, 0.0 on read failure

#endif
