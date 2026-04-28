#ifndef DS18B20_SENSOR_H
#define DS18B20_SENSOR_H

#include "Arduino.h"
#include <OneWire.h>
#include <DallasTemperature.h>

#define DS18B20_PIN 3

extern OneWire ds18b20_wire;
extern DallasTemperature ds18b20;

void  ds18b20_Init();
float ds18b20_getTemperature();   // °C, -999.0 on sensor error

#endif
