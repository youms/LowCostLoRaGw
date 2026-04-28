/*******************************************************************************
 * LoRaWAN DS18B20 + MQ9 Raw Sensor Sketch
 *
 * No on-device gas calculations. Sends a compact 6-byte binary payload:
 *   [0-1]  int16_t  temperature × 100  (e.g. 23.45 °C → 0x0929)
 *   [2-3]  uint16_t MQ9 raw ADC 0–1023
 *   [4-5]  uint16_t MQ9 Rs/R0 × 1000
 *
 * Hardware (Dragino LoRa Shield v1.4 on Arduino Mega):
 *   NSS=10  RST=9  DIO0=2  DIO1=6  DIO2=7
 *   DS18B20 data pin = 3
 *   MQ9 analog pin  = A0
 *******************************************************************************/

#define ARDUINO_LMIC_PROJECT_CONFIG_H_SUPPRESS_WARNING

#define SCG_RX2_DR  DR_SF9
#include <single_channel_lmic.h>

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#include "ds18b20_sensor.h"
#include "mq9_sensor.h"

// ----------------------------------------------------------------------------
// ABP session credentials — TTN device (MSB first)
// ----------------------------------------------------------------------------
static const PROGMEM u1_t NWKSKEY[16] = {
    0xCA, 0xD2, 0x33, 0xEF, 0x40, 0x24, 0xBB, 0xAF,
    0x80, 0x74, 0xAA, 0x6A, 0xEA, 0x3D, 0x0A, 0x68
};
static const u1_t PROGMEM APPSKEY[16] = {
    0xCA, 0xD2, 0x33, 0xEF, 0x40, 0x24, 0xBB, 0xAF,
    0x80, 0x74, 0xAA, 0x6A, 0xEA, 0x3D, 0x0A, 0x68
};
static const u4_t DEVADDR = 0x260B0BA0;

void os_getArtEui(u1_t* buf) {}
void os_getDevEui(u1_t* buf) {}
void os_getDevKey(u1_t* buf) {}

// ----------------------------------------------------------------------------
// Pin mapping — Dragino LoRa Shield v1.4
// ----------------------------------------------------------------------------
const lmic_pinmap lmic_pins = {
    .nss  = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst  = 9,
    .dio  = {2, 6, 7},
};

// ----------------------------------------------------------------------------
// Global state
// ----------------------------------------------------------------------------
static uint8_t mydata[6];
static osjob_t sendjob;

const unsigned TX_INTERVAL = 60;

// ----------------------------------------------------------------------------
// do_send
// ----------------------------------------------------------------------------
void do_send(osjob_t* j) {
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
        return;
    }

    float temp  = ds18b20_getTemperature();
    int   adc   = mq9_getADC();
    float ratio = mq9_getRatio();

    int16_t  temp_s  = (int16_t)(temp  * 100);
    uint16_t adc_s   = (uint16_t)adc;
    uint16_t ratio_s = (uint16_t)(ratio * 1000);

    mydata[0] = highByte(temp_s);
    mydata[1] = lowByte(temp_s);
    mydata[2] = highByte(adc_s);
    mydata[3] = lowByte(adc_s);
    mydata[4] = highByte(ratio_s);
    mydata[5] = lowByte(ratio_s);

    Serial.print(F("T:"));    Serial.print(temp, 2);
    Serial.print(F("C ADC:")); Serial.print(adc);
    Serial.print(F(" Rs/R0:")); Serial.print(ratio, 3);
    Serial.print(F(" ["));
    for (int i = 0; i < 6; i++) {
        if (mydata[i] < 0x10) Serial.print('0');
        Serial.print(mydata[i], HEX);
        if (i < 5) Serial.print(' ');
    }
    Serial.println(']');

    LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
}

// ----------------------------------------------------------------------------
// onEvent
// ----------------------------------------------------------------------------
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
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:      Serial.println(F("EV_LOST_TSYNC"));      break;
        case EV_RESET:           Serial.println(F("EV_RESET"));            break;
        case EV_RXCOMPLETE:      Serial.println(F("EV_RXCOMPLETE"));       break;
        case EV_LINK_DEAD:       Serial.println(F("EV_LINK_DEAD"));        break;
        case EV_LINK_ALIVE:      Serial.println(F("EV_LINK_ALIVE"));       break;
        case EV_TXSTART:         Serial.println(F("EV_TXSTART"));          break;
        case EV_TXCANCELED:      Serial.println(F("EV_TXCANCELED"));       break;
        case EV_RXSTART:         break;
        case EV_JOIN_TXCOMPLETE: Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept")); break;
        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned)ev);
            break;
    }
}

// ----------------------------------------------------------------------------
// setup
// ----------------------------------------------------------------------------
void setup() {
    Serial.begin(9600);
    Serial.println(F("DS18B20 + MQ9 Raw | ABP | Single-CH 868.1 MHz"));

    ds18b20_Init();
    mq9_Init();

    os_init();
    LMIC_reset();

    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession(0x13, DEVADDR, nwkskey, appskey);

#if defined(CFG_eu868)
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

// ----------------------------------------------------------------------------
// loop
// ----------------------------------------------------------------------------
void loop() {
    SCG_enforce();
    os_runloop_once();
}
