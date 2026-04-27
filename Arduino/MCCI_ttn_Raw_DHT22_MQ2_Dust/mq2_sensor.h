#ifndef MQ2_SENSOR_H
#define MQ2_SENSOR_H

#include "Arduino.h"

#define MQ2_PIN              A0
#define MQ2_RL_VALUE         10000.0   // Ω
#define MQ2_VCC              5.0

#define MQ2_WARMUP_S         15
#define MQ2_CAL_SAMPLES      3
#define MQ2_CAL_INTERVAL_MS  200
#define MQ2_READ_SAMPLES     3
#define MQ2_READ_INTERVAL_MS 50

extern float mq2_R0;   // Baseline resistance in clean air (Ω)

void  mq2_Init();      // Warmup + R0 calibration
int   mq2_getADC();    // Single raw ADC reading (0–1023)
float mq2_getRatio();  // Averaged Rs / R0

#endif
