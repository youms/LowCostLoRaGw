#include "ds18b20_sensor.h"

OneWire ds18b20_wire(DS18B20_PIN);
DallasTemperature ds18b20(&ds18b20_wire);

void ds18b20_Init() {
    ds18b20.begin();
    ds18b20.setResolution(12);
    Serial.print(F("DS18B20: found "));
    Serial.print(ds18b20.getDeviceCount());
    Serial.println(F(" sensor(s)"));
}

float ds18b20_getTemperature() {
    ds18b20.requestTemperatures();
    float t = ds18b20.getTempCByIndex(0);
    if (t == DEVICE_DISCONNECTED_C) return -999.0f;
    return t;
}
