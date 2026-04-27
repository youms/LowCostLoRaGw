#ifndef MQ9_SENSOR_H
#define MQ9_SENSOR_H

#include "Arduino.h"

#define MQ9_PIN              A0
#define MQ9_RL_VALUE         5.0    // kΩ
#define MQ9_RO_CLEAN_AIR     9.9   // Rs/R0 in clean air per MQ9 datasheet
#define MQ9_R0_MIN           1.0   // kΩ — sanity bounds
#define MQ9_R0_MAX           50.0  // kΩ
#define MQ9_R0_DEFAULT       10.0  // kΩ — datasheet typical

#define MQ9_WARMUP_S          60
#define MQ9_CAL_SAMPLES        5
#define MQ9_CAL_INTERVAL_MS  500
#define MQ9_READ_SAMPLES       5
#define MQ9_READ_INTERVAL_MS  50

extern float mq9_R0;   // Calibrated baseline resistance (kΩ)

void  mq9_Init();      // Warmup + R0 calibration
int   mq9_getADC();    // Single raw ADC reading (0–1023)
float mq9_getRatio();  // Averaged Rs / R0

#endif
