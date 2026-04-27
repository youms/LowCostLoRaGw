#ifndef DUST_SENSOR_H
#define DUST_SENSOR_H

#include "Arduino.h"

// GP2Y1010AU0F — Sharp optical dust sensor
// D2 is reserved by LMIC DIO0, so LED control is on D5
#define DUST_LED_PIN        5
#define DUST_ADC_PIN        A1    // A0 is used by the gas sensor
#define DUST_SAMPLES        10    // readings averaged per transmission
#define DUST_LED_DELAY_US   280   // µs from LED on to sampling point (datasheet)
#define DUST_CYCLE_MS       10    // ms per sample cycle (LED pulse + rest)

void  dust_Init();
int   dust_getADC();   // averaged raw ADC (0–1023)

#endif
