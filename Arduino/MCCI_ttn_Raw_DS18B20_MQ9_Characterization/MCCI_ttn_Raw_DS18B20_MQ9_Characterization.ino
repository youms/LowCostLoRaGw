/*******************************************************************************
 * LoRaWAN DS18B20 + MQ9 Raw Network Characterization
 *
 * Same characterization framework as MCCI_ttn_abp_DS18B20_MQ9_Characterization
 * (16 configs, downlink /@C<index># and /@I<secs>#) but no gas calculations
 * on device — only raw sensor values are sent.
 *
 * Binary payload (padded with 0x00 to targetSize):
 *   [0-1]  int16_t  temperature × 100   (e.g. 23.45 °C → 0x09 0x29)
 *   [2-3]  uint16_t MQ9 raw ADC 0–1023
 *   [4-5]  uint16_t MQ9 Rs/R0 × 1000
 *   [6+]   0x00 padding to targetSize
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

#include "my_temp_sensor_code.h"
#include "mq9_sensor.h"
#include "NetworkParams.h"
#include "DownlinkParser.h"

// ----------------------------------------------------------------------------
// ABP session credentials (MSB first)
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
static uint8_t message[100];
static osjob_t sendjob;
uint32_t     TXPacketCount    = 0;
uint8_t      currentParamIndex = 0;
unsigned int idlePeriodInSec  = 8;

// ----------------------------------------------------------------------------
// Build binary payload: 6 data bytes + zero-padding to targetSize
// ----------------------------------------------------------------------------
void buildRawPayload(uint8_t* buf, float temp, int adc, float ratio, uint8_t targetSize)
{
    memset(buf, 0, targetSize);

    int16_t  temp_s  = (int16_t)(temp  * 100);
    uint16_t adc_s   = (uint16_t)adc;
    uint16_t ratio_s = (uint16_t)(ratio * 1000);

    if (targetSize >= 2) { buf[0] = highByte(temp_s);  buf[1] = lowByte(temp_s); }
    if (targetSize >= 4) { buf[2] = highByte(adc_s);   buf[3] = lowByte(adc_s); }
    if (targetSize >= 6) { buf[4] = highByte(ratio_s); buf[5] = lowByte(ratio_s); }
}

// ----------------------------------------------------------------------------
// do_send
// ----------------------------------------------------------------------------
void do_send(osjob_t* j)
{
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
        return;
    }

    float tempC = (float)sensor_getValue();
    if (tempC == -999.0f) {
        Serial.println(F("DS18B20 error - using random value"));
        tempC = random_value();
    }

    int   adc   = mq9_getADC();
    float ratio = mq9_getRatio();

    uint8_t targetSize = testParams[currentParamIndex].payloadSize;
    buildRawPayload(message, tempC, adc, ratio, targetSize);

    Serial.println(F("=== Sending ==="));
    Serial.print(F("Config : ")); Serial.println(testParams[currentParamIndex].name);
    Serial.print(F("Temp   : ")); Serial.print(tempC, 2);   Serial.println(F(" C"));
    Serial.print(F("ADC    : ")); Serial.println(adc);
    Serial.print(F("Rs/R0  : ")); Serial.println(ratio, 3);
    Serial.print(F("Size   : ")); Serial.print(targetSize); Serial.println(F(" bytes"));
    Serial.print(F("Payload: "));
    for (int i = 0; i < targetSize; i++) {
        if (message[i] < 0x10) Serial.print('0');
        Serial.print(message[i], HEX);
        Serial.print(' ');
    }
    Serial.println();
    Serial.println(F("==============="));

    LMIC_setTxData2(1, message, targetSize, 0);
    Serial.println(F("Packet queued"));
    TXPacketCount++;
}

// ----------------------------------------------------------------------------
// onEvent
// ----------------------------------------------------------------------------
void onEvent(ev_t ev)
{
    Serial.print(os_getTime());
    Serial.print(F(": "));
    bool configChanged = false;

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
                uint8_t len = (LMIC.dataLen < sizeof(message) - 1) ? LMIC.dataLen : sizeof(message) - 1;
                memcpy(message, LMIC.frame + LMIC.dataBeg, len);
                message[len] = '\0';

                Serial.print(F("Downlink: ")); Serial.print(len); Serial.println(F(" bytes"));
                configChanged = parseDownlinkCommand(message, len, currentParamIndex);
                if (configChanged)
                    updateLoRaParams(testParams[currentParamIndex]);
            }

            {
                uint16_t nextDelay = configChanged ? 30 : idlePeriodInSec;
                Serial.print(F("Next TX in ")); Serial.print(nextDelay); Serial.println(F(" s"));
                os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(nextDelay), do_send);
            }
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
void setup()
{
    while (!Serial);
    Serial.begin(38400);
    delay(100);
    Serial.println(F("DS18B20 + MQ9 Raw Network Characterization"));
    Serial.println(F("Downlink /@C<0-15># to change config, /@I<secs># for interval"));

    sensor_Init();
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

    // Override DR with the current test config (intentional — characterization varies DR per config).
    updateLoRaParams(testParams[currentParamIndex]);
    do_send(&sendjob);
}

// ----------------------------------------------------------------------------
// loop
// ----------------------------------------------------------------------------
void loop()
{
    unsigned long now = millis();
    digitalWrite(13, (now & 512) ? HIGH : LOW);
    // Channel-only guard: DR is intentionally varied per test config, so SCG_enforce() is not used here.
    if (LMIC.txChnl != 0) {
        LMIC.txChnl            = 0;
        LMIC.channelShuffleMap = 0x0001;
    }
    os_runloop_once();
}
