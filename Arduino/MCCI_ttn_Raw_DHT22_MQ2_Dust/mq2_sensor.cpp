#include "mq2_sensor.h"

float mq2_R0 = 10000.0;

static float mq2_resistance(int adc) {
    float vout = adc * (MQ2_VCC / 1023.0);
    if (vout <= 0.01) return 0;
    return (MQ2_VCC - vout) * MQ2_RL_VALUE / vout;
}

static float mq2_read() {
    float rs = 0;
    for (int i = 0; i < MQ2_READ_SAMPLES; i++) {
        rs += mq2_resistance(analogRead(MQ2_PIN));
        delay(MQ2_READ_INTERVAL_MS);
    }
    return rs / MQ2_READ_SAMPLES;
}

void mq2_Init() {
    pinMode(MQ2_PIN, INPUT);

    Serial.print(F("MQ2 warmup "));
    Serial.print(MQ2_WARMUP_S);
    Serial.println(F("s..."));
    for (int i = MQ2_WARMUP_S; i > 0; i--) {
        if (i % 5 == 0) { Serial.print(i); Serial.print(F("s ")); }
        delay(1000);
    }
    Serial.println();

    float rs = 0;
    for (int i = 0; i < MQ2_CAL_SAMPLES; i++) {
        rs += mq2_resistance(analogRead(MQ2_PIN));
        delay(MQ2_CAL_INTERVAL_MS);
    }
    mq2_R0 = rs / MQ2_CAL_SAMPLES;

    Serial.print(F("MQ2 R0: "));
    Serial.print((int)mq2_R0);
    Serial.println(F(" Ohm"));
}

int mq2_getADC() {
    return analogRead(MQ2_PIN);
}

float mq2_getRatio() {
    if (mq2_R0 <= 0) return 0;
    return mq2_read() / mq2_R0;
}
