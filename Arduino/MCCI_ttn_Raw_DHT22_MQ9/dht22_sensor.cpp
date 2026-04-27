#include "dht22_sensor.h"

DHT dht22(DHT22_PIN, DHT22_TYPE);

void dht22_Init() {
    dht22.begin();
    Serial.println(F("DHT22 initialized"));
}

float dht22_getTemperature() {
    float t = dht22.readTemperature();
    return isnan(t) ? 0.0 : t;
}

float dht22_getHumidity() {
    float h = dht22.readHumidity();
    return isnan(h) ? 0.0 : h;
}
