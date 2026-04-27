#include "dust_sensor.h"

void dust_Init() {
    pinMode(DUST_LED_PIN, OUTPUT);
    digitalWrite(DUST_LED_PIN, HIGH); // LED off (active LOW)
    pinMode(DUST_ADC_PIN, INPUT);
    Serial.println(F("GP2Y1010AU0F initialized"));
}

int dust_getADC() {
    long sum = 0;
    for (int i = 0; i < DUST_SAMPLES; i++) {
        // Pulse LED, sample at the 280µs sweet spot per Sharp datasheet
        digitalWrite(DUST_LED_PIN, LOW);
        delayMicroseconds(DUST_LED_DELAY_US);
        sum += analogRead(DUST_ADC_PIN);
        digitalWrite(DUST_LED_PIN, HIGH);
        // Rest for remainder of 10ms cycle before next pulse
        delay(DUST_CYCLE_MS);
    }
    return (int)(sum / DUST_SAMPLES);
}
