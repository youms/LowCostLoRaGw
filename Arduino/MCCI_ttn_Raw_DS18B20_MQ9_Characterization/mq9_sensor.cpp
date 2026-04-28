#include "mq9_sensor.h"

float mq9_R0 = MQ9_R0_DEFAULT;

static float mq9_resistance(int adc) {
    if (adc <= 0) return 0;
    return MQ9_RL_VALUE * (1023.0 - adc) / adc;
}

static float mq9_read() {
    float rs = 0;
    for (int i = 0; i < MQ9_READ_SAMPLES; i++) {
        rs += mq9_resistance(analogRead(MQ9_PIN));
        delay(MQ9_READ_INTERVAL_MS);
    }
    return rs / MQ9_READ_SAMPLES;
}

void mq9_Init() {
    pinMode(MQ9_PIN, INPUT);

    Serial.print(F("MQ9 warmup "));
    Serial.print(MQ9_WARMUP_S);
    Serial.println(F("s..."));
    for (int i = MQ9_WARMUP_S; i > 0; i--) {
        if (i % 10 == 0) { Serial.print(i); Serial.print(F("s ")); }
        delay(1000);
    }
    Serial.println();

    float rs = 0;
    for (int i = 0; i < MQ9_CAL_SAMPLES; i++) {
        rs += mq9_resistance(analogRead(MQ9_PIN));
        delay(MQ9_CAL_INTERVAL_MS);
    }
    mq9_R0 = (rs / MQ9_CAL_SAMPLES) / MQ9_RO_CLEAN_AIR;

    Serial.print(F("MQ9 R0: "));
    Serial.print(mq9_R0, 2);
    Serial.println(F(" kOhm"));

    if (mq9_R0 < MQ9_R0_MIN || mq9_R0 > MQ9_R0_MAX) {
        Serial.println(F("WARNING: R0 out of range — check sensor connection"));
        mq9_R0 = MQ9_R0_DEFAULT;
    }
}

int mq9_getADC() {
    return analogRead(MQ9_PIN);
}

float mq9_getRatio() {
    if (mq9_R0 <= 0) return 0;
    return mq9_read() / mq9_R0;
}
