#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#include "dht22_sensor.h"
#include "mq2_sensor.h"

#define ARDUINO_LMIC_PROJECT_CONFIG_H_SUPPRESS_WARNING

#define SCG_RX2_DR  DR_SF9
#include <single_channel_lmic.h>

///////////////////////////////////////////////////////////////////
// OPTIONAL FEATURES — uncomment to enable
//#define WITH_EEPROM
//#define FORCE_DEFAULT_VALUE
//#define LOW_POWER
//#define SHOW_LOW_POWER_CYCLE
///////////////////////////////////////////////////////////////////

#ifdef WITH_EEPROM
#include <EEPROM.h>
#endif
#ifdef LOW_POWER
#if defined(ARDUINO_AVR_PRO) || defined(ARDUINO_AVR_NANO) || defined(ARDUINO_AVR_UNO) || \
    defined(ARDUINO_AVR_MINI) || defined(ARDUINO_AVR_MEGA2560) || defined(__AVR_ATmega32U4__)
#include "LowPower.h"
#endif
#endif

// ABP session credentials
static const PROGMEM u1_t NWKSKEY[16] = {
    0x55, 0xF7, 0x14, 0xA6, 0x58, 0x11, 0xDE, 0xAF,
    0xF4, 0xAC, 0x6B, 0x65, 0x3F, 0xA5, 0xAB, 0xAD
};
static const u1_t PROGMEM APPSKEY[16] = {
    0xBC, 0xA5, 0x0B, 0x27, 0x85, 0x31, 0xF0, 0x28,
    0xBE, 0x75, 0x9A, 0x66, 0x84, 0x15, 0x76, 0xED
};
static const u4_t DEVADDR = 0x260BE0A2;

void os_getArtEui(u1_t* buf) {}
void os_getDevEui(u1_t* buf) {}
void os_getDevKey(u1_t* buf) {}

// 8-byte positional payload: temp | hum | ADC | Rs/R0
static uint8_t mydata[8];
static osjob_t sendjob;

unsigned TX_INTERVAL = 60;

const lmic_pinmap lmic_pins = {
    .nss  = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst  = 9,
    .dio  = {2, 6, 7},
};

#ifdef WITH_EEPROM
struct lmic_eeprom_t {
    uint8_t  flag1;
    uint8_t  flag2;
    uint32_t seqnoUp;
    unsigned tx_interval;
    uint8_t  overwrite;
};
static lmic_eeprom_t my_eeprom;
#endif

#ifdef LOW_POWER
void lowPower(unsigned long ms) {
    unsigned long remaining = ms;
    Serial.flush();
    delay(5);
#if defined(ARDUINO_AVR_PRO) || defined(ARDUINO_AVR_NANO) || defined(ARDUINO_AVR_UNO) || \
    defined(ARDUINO_AVR_MINI) || defined(ARDUINO_AVR_MEGA2560) || defined(__AVR_ATmega32U4__)
    while (remaining > 0) {
        if (remaining > 8158) {
            LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
            remaining -= 8158;
#ifdef SHOW_LOW_POWER_CYCLE
            Serial.print(F("8"));
#endif
        } else if (remaining > 4158) {
            LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF);
            remaining -= 4158;
#ifdef SHOW_LOW_POWER_CYCLE
            Serial.print(F("4"));
#endif
        } else if (remaining > 2158) {
            LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
            remaining -= 2158;
#ifdef SHOW_LOW_POWER_CYCLE
            Serial.print(F("2"));
#endif
        } else if (remaining > 1158) {
            LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
            remaining -= 1158;
#ifdef SHOW_LOW_POWER_CYCLE
            Serial.print(F("1"));
#endif
        } else {
            delay(remaining);
#ifdef SHOW_LOW_POWER_CYCLE
            Serial.print(F("D"));
#endif
            remaining = 0;
        }
#ifdef SHOW_LOW_POWER_CYCLE
        Serial.flush();
        delay(1);
#endif
    }
#else
    delay(ms);
#endif
}
#endif

void onEvent(ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(F(": "));
    switch (ev) {
        case EV_SCAN_TIMEOUT:    Serial.println(F("EV_SCAN_TIMEOUT"));    break;
        case EV_BEACON_FOUND:    Serial.println(F("EV_BEACON_FOUND"));    break;
        case EV_BEACON_MISSED:   Serial.println(F("EV_BEACON_MISSED"));   break;
        case EV_BEACON_TRACKED:  Serial.println(F("EV_BEACON_TRACKED"));  break;
        case EV_JOINING:         Serial.println(F("EV_JOINING"));         break;
        case EV_JOINED:          Serial.println(F("EV_JOINED"));          break;
        case EV_JOIN_FAILED:     Serial.println(F("EV_JOIN_FAILED"));     break;
        case EV_REJOIN_FAILED:   Serial.println(F("EV_REJOIN_FAILED"));   break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE"));
            if (LMIC.txrxFlags & TXRX_ACK)
                Serial.println(F("Received ACK"));
            if (LMIC.dataLen) {
                Serial.print(F("Downlink: "));
                Serial.print(LMIC.dataLen);
                Serial.println(F(" bytes"));
            }
#ifdef WITH_EEPROM
            my_eeprom.seqnoUp = LMIC.seqnoUp;
            EEPROM.put(0, my_eeprom);
            Serial.print(F("EEPROM saved seqnoUp=")); Serial.println(LMIC.seqnoUp);
#endif
#ifdef LOW_POWER
            Serial.print(F("Sleeping ")); Serial.print(TX_INTERVAL); Serial.println(F("s"));
            Serial.flush();
            lowPower((unsigned long)TX_INTERVAL * 1000);
            os_setCallback(&sendjob, do_send);
#else
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
#endif
            break;
        case EV_LOST_TSYNC:      Serial.println(F("EV_LOST_TSYNC"));     break;
        case EV_RESET:           Serial.println(F("EV_RESET"));           break;
        case EV_RXCOMPLETE:      Serial.println(F("EV_RXCOMPLETE"));      break;
        case EV_LINK_DEAD:       Serial.println(F("EV_LINK_DEAD"));       break;
        case EV_LINK_ALIVE:      Serial.println(F("EV_LINK_ALIVE"));      break;
        case EV_TXSTART:         Serial.println(F("EV_TXSTART"));         break;
        case EV_TXCANCELED:      Serial.println(F("EV_TXCANCELED"));      break;
        case EV_RXSTART:         break;
        case EV_JOIN_TXCOMPLETE: Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept")); break;
        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned)ev);
            break;
    }
}

void do_send(osjob_t* j) {
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
        return;
    }

    float temp  = dht22_getTemperature();
    float hum   = dht22_getHumidity();
    int   adc   = mq2_getADC();
    float ratio = mq2_getRatio();

    int16_t  temp_s  = (int16_t)(temp  * 100);
    uint16_t hum_s   = (uint16_t)(hum  * 100);
    uint16_t adc_s   = (uint16_t)adc;
    uint16_t ratio_s = (uint16_t)(ratio * 1000);

    mydata[0] = highByte(temp_s);
    mydata[1] = lowByte(temp_s);
    mydata[2] = highByte(hum_s);
    mydata[3] = lowByte(hum_s);
    mydata[4] = highByte(adc_s);
    mydata[5] = lowByte(adc_s);
    mydata[6] = highByte(ratio_s);
    mydata[7] = lowByte(ratio_s);

    Serial.print(F("T:")); Serial.print(temp, 1);
    Serial.print(F("C H:")); Serial.print(hum, 1);
    Serial.print(F("% ADC:")); Serial.print(adc);
    Serial.print(F(" Rs/R0:")); Serial.print(ratio, 3);
    Serial.print(F(" ["));
    for (int i = 0; i < 8; i++) {
        if (mydata[i] < 0x10) Serial.print("0");
        Serial.print(mydata[i], HEX);
        if (i < 7) Serial.print(" ");
    }
    Serial.println(F("]"));

    LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
}

void setup() {
    Serial.begin(9600);
    Serial.println(F("DHT22 + MQ2 Raw | ABP | Single-CH 868.1 MHz"));

    dht22_Init();
    mq2_Init();

    os_init();
    LMIC_reset();

    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession(0x13, DEVADDR, nwkskey, appskey);

#ifdef WITH_EEPROM
    EEPROM.get(0, my_eeprom);
    if (my_eeprom.flag1 == 0x12 && my_eeprom.flag2 == 0x35) {
        Serial.println(F("EEPROM: restoring config"));
        Serial.print(F("  seqnoUp : ")); Serial.println(my_eeprom.seqnoUp);
#ifdef FORCE_DEFAULT_VALUE
        Serial.println(F("  FORCE_DEFAULT_VALUE: resetting"));
        my_eeprom.seqnoUp    = 0;
        my_eeprom.tx_interval = TX_INTERVAL;
        my_eeprom.overwrite  = 0;
        EEPROM.put(0, my_eeprom);
#else
        LMIC.seqnoUp = my_eeprom.seqnoUp;
        if (my_eeprom.overwrite == 1 && my_eeprom.tx_interval != 0) {
            TX_INTERVAL = my_eeprom.tx_interval;
            Serial.print(F("  TX_INTERVAL: ")); Serial.print(TX_INTERVAL); Serial.println(F("s"));
        }
#endif
    } else {
        my_eeprom.flag1       = 0x12;
        my_eeprom.flag2       = 0x35;
        my_eeprom.seqnoUp     = 0;
        my_eeprom.tx_interval = TX_INTERVAL;
        my_eeprom.overwrite   = 0;
        EEPROM.put(0, my_eeprom);
        Serial.println(F("EEPROM: initialized"));
    }
#endif

#if defined(CFG_eu868)
    // DR_SF7B on ch0 allows BW250 (DR6) to stay on 868.1 MHz
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);
#elif defined(CFG_us915) || defined(CFG_au915)
    LMIC_selectSubBand(1);
#elif defined(CFG_as923)
#elif defined(CFG_kr920)
#elif defined(CFG_in866)
#else
# error Region not supported
#endif

    // Prevent MAC DlChannelReq from silently redirecting RX1 to another frequency
#if !defined(DISABLE_MCMD_DlChannelReq)
    for (uint8_t i = 0; i < 9; i++) LMIC.channelDlFreq[i] = 0;
#endif

    // Single-channel enforcement: disables ch 1-8, locks shuffle map,
    // disables ADR and link check, asserts DR_SF7 / 14 dBm, sets RX2.
    SCG_init();

    do_send(&sendjob);
}

void loop() {
    SCG_enforce();
    os_runloop_once();
}
