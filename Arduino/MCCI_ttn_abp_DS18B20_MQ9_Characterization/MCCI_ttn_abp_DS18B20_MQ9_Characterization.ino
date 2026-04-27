/*******************************************************************************
 * LoRaWAN DS18B20 + MQ9 Network Characterization
 * Based on MCCI_ttn_abp (LoRaWAN/LMIC layer) and
 *       Enhanced_Lora_DS18B20_MQ9_Characterization_Downlink_Controlled (sensor/logic layer)
 *
 * ABP activation, single channel 868.1 MHz
 * 16 network characterization configs switchable via downlink /@C<index>#
 * Transmission interval adjustable via downlink /@I<secs>#
 *
 * Hardware (Dragino LoRa Shield v1.4 on Arduino Mega):
 *   NSS  = 10, RST = 9, DIO0 = 2, DIO1 = 6, DIO2 = 7
 *   DS18B20 data pin = 3
 *   MQ9 analog pin  = A0
 *******************************************************************************/

#define ARDUINO_LMIC_PROJECT_CONFIG_H_SUPPRESS_WARNING

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#include "my_temp_sensor_code.h"
#include "my_gas_sensor_code.h"
#include "NetworkParams.h"    // must come before DownlinkParser
#include "DownlinkParser.h"

// ----------------------------------------------------------------------------
// ABP session credentials — change these to match your TTN device
// ----------------------------------------------------------------------------
static const PROGMEM u1_t NWKSKEY[16] = {
    0x55, 0xF7, 0x14, 0xA6, 0x58, 0x11, 0xDE, 0xAF,
    0xF4, 0xAC, 0x6B, 0x65, 0x3F, 0xA5, 0xAB, 0xAD
};
static const u1_t PROGMEM APPSKEY[16] = {
    0xBC, 0xA5, 0x0B, 0x27, 0x85, 0x31, 0xF0, 0x28,
    0xBE, 0x75, 0x9A, 0x66, 0x84, 0x15, 0x76, 0xED
};
static const u4_t DEVADDR = 0x260BE0A2;

// Required by LMIC even for ABP — left empty
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
uint8_t      message[100];
static osjob_t sendjob;
uint32_t     TXPacketCount   = 0;
uint8_t      currentParamIndex = 8;   // Default: MAX-SF12-BW125-T20
uint8_t      node_addr       = 10;
unsigned int idlePeriodInSec = 8;     // Seconds between transmissions

// ----------------------------------------------------------------------------
// Helpers — kept from original characterization sketch
// ----------------------------------------------------------------------------
char *ftoa(char *a, double f, int precision)
{
    long p[] = {0, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000};
    char *ret = a;
    long heiltal = (long)f;
    itoa(heiltal, a, 10);
    while (*a != '\0') a++;
    *a++ = '.';
    long desimal = abs((long)((f - heiltal) * p[precision]));
    if (desimal < p[precision - 1]) *a++ = '0';
    itoa(desimal, a, 10);
    return ret;
}

// Temperature-only payload (small configs)
void createPaddedPayload(char* dest, float temperature, uint8_t targetSize)
{
    char float_str[10];
    ftoa(float_str, temperature, 2);
    int baseSize = sprintf(dest, "\\!TC/%s", float_str);
    if (targetSize > baseSize) {
        for (int i = baseSize; i < targetSize; i++) dest[i] = 0x00;
    }
}

// Combined temperature + gas payload
void createPaddedPayload(char* dest, float temperature,
                         int gasCO, int gasLPG, int gasMethane,
                         int gasPropane, int gasHydrogen, int gasSmoke,
                         uint8_t targetSize)
{
    char float_str[10];
    ftoa(float_str, temperature, 2);
    int baseSize;

    if (targetSize >= 50) {
        baseSize = sprintf(dest,
            "\\!TC/%s/CO/%d/LPG/%d/CH4/%d/C3H8/%d/H2/%d/SMK/%d",
            float_str, gasCO, gasLPG, gasMethane, gasPropane, gasHydrogen, gasSmoke);
    } else if (targetSize >= 20) {
        baseSize = sprintf(dest,
            "\\!TC/%s/CO/%d/LPG/%d/CH4/%d",
            float_str, gasCO, gasLPG, gasMethane);
    } else {
        baseSize = sprintf(dest, "\\!TC/%s", float_str);
    }

    if (targetSize > baseSize) {
        for (int i = baseSize; i < targetSize; i++) dest[i] = 0x00;
    }
}

// ----------------------------------------------------------------------------
// do_send — read sensors, build payload, queue LoRaWAN uplink
// Called by LMIC scheduler; next call is scheduled from onEvent(EV_TXCOMPLETE)
// ----------------------------------------------------------------------------
void do_send(osjob_t* j)
{
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
        return;
    }

    // Read DS18B20
    float tempC = (float)sensor_getValue();
    if (tempC == -999.0f) {
        Serial.println(F("DS18B20 error - using random value"));
        tempC = random_value();
    }

    // Read MQ9
    int gasCO      = gas_sensor_getValue();
    int gasLPG     = gas_sensor_getLPG();
    int gasMethane = gas_sensor_getMethane();
    int gasPropane = gas_sensor_getPropane();
    int gasHydrogen= gas_sensor_getHydrogen();
    int gasSmoke   = gas_sensor_getSmoke();

    // Build payload to the size dictated by current config
    uint8_t targetSize = testParams[currentParamIndex].payloadSize;
    char payloadStr[100];
    createPaddedPayload(payloadStr, tempC, gasCO, gasLPG, gasMethane, gasPropane, gasHydrogen, gasSmoke, targetSize);
    memcpy(message, payloadStr, targetSize);

    // Debug output
    Serial.println(F("=== Sending ==="));
    Serial.print(F("Config : ")); Serial.println(testParams[currentParamIndex].name);
    Serial.print(F("Temp   : ")); Serial.print(tempC);   Serial.println(F(" C"));
    Serial.print(F("CO/LPG/CH4: ")); Serial.print(gasCO);
    Serial.print('/'); Serial.print(gasLPG);
    Serial.print('/'); Serial.println(gasMethane);
    Serial.print(F("Size   : ")); Serial.print(targetSize); Serial.println(F(" bytes"));
    Serial.print(F("Payload: "));
    for (int i = 0; i < targetSize; i++) {
        Serial.print((message[i] >= 32 && message[i] <= 126) ? (char)message[i] : '.');
    }
    Serial.println();
    Serial.println(F("==============="));

    LMIC_setTxData2(1, message, targetSize, 0);
    Serial.println(F("Packet queued"));
    TXPacketCount++;
}

// ----------------------------------------------------------------------------
// onEvent — LMIC event handler
// ----------------------------------------------------------------------------
void onEvent(ev_t ev)
{
    Serial.print(os_getTime());
    Serial.print(F(": "));
    bool configChanged = false;

    switch (ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;

        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE"));

            if (LMIC.txrxFlags & TXRX_ACK)
                Serial.println(F("Received ACK"));

            // Process downlink payload if present
            if (LMIC.dataLen) {
                uint8_t len = (LMIC.dataLen < sizeof(message) - 1) ? LMIC.dataLen : sizeof(message) - 1;
                memcpy(message, LMIC.frame + LMIC.dataBeg, len);
                message[len] = '\0';

                Serial.print(F("Downlink: ")); Serial.print(len); Serial.println(F(" bytes"));
                configChanged = parseDownlinkCommand(message, len, currentParamIndex, node_addr);
                if (configChanged)
                    updateLoRaParams(testParams[currentParamIndex]);
            }

            // Schedule next uplink — wait longer after a config change
            {
                uint16_t nextDelay = configChanged ? 30 : idlePeriodInSec;
                Serial.print(F("Next TX in ")); Serial.print(nextDelay); Serial.println(F(" s"));
                os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(nextDelay), do_send);
            }
            break;

        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            break; // silence — printing here wrecks RX timing
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;
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
    Serial.println(F("LoRaWAN DS18B20+MQ9 Network Characterization"));
    Serial.println(F("Downlink /@C<0-15># to change config, /@I<secs># for interval"));

    sensor_Init();
    gas_sensor_Init();

    // LMIC init
    os_init();
    LMIC_reset();

    // Load ABP session keys from flash
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession(0x13, DEVADDR, nwkskey, appskey);

#if defined(CFG_eu868)
    // Define all TTN EU868 channels; then disable 1-9 for single-channel operation.
    // Channel 0 DR range extended to DR_SF7B so DR6 configs can transmit on 868.1 MHz.
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
    // configure channels 0..n for your region
#elif defined(CFG_kr920)
    // configure channels 0..n for your region
#elif defined(CFG_in866)
    // configure channels 0..n for your region
#else
# error Region not supported
#endif

    // Force single-channel on 868.1 MHz
    for (int i = 1; i <= 9; i++) LMIC_disableChannel(i);
    Serial.println(F("Single channel: 868.1 MHz"));

    // Prevent MAC DlChannelReq from redirecting downlinks to a different frequency
#if !defined(DISABLE_MCMD_DlChannelReq)
    for (uint8_t i = 0; i < 9; i++) LMIC.channelDlFreq[i] = 0;
#endif

    LMIC_setLinkCheckMode(0);
    LMIC_setAdrMode(0);     // ADR off — we control DR manually per characterization config
    LMIC.dn2Dr = DR_SF9;    // TTN EU868 RX2 window uses SF9

    // Apply initial characterization config
    updateLoRaParams(testParams[currentParamIndex]);

    // Queue first transmission immediately
    do_send(&sendjob);
}

// ----------------------------------------------------------------------------
// loop
// ----------------------------------------------------------------------------
void loop()
{
    // LED heartbeat
    unsigned long now = millis();
    digitalWrite(13, (now & 512) ? HIGH : LOW);

    os_runloop_once();
}
