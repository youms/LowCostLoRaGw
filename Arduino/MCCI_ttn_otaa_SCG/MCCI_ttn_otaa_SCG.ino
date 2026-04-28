/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!
 *
 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/

/*******************************************************************************
 *
 * Modified by C. Pham for support of single-channel gateway and 433MHz band
 * Last update Feb 4th, 2020
 *
 *******************************************************************************/

/*******************************************************************************
 *
 * Refactored for single_channel_lmic.h enforcement layer
 * All single-channel enforcement is now centralised in single_channel_lmic.h.
 * No MCCI library modifications required — library-update-safe.
 *
 * To adapt to a different band or SF, edit the #defines below before the
 * #include <single_channel_lmic.h> line.
 *
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

// ── Single-channel configuration ─────────────────────────────────────────────
// Override defaults here before the include. Examples:
//
//   #define SCG_DATARATE     DR_SF9       // change spreading factor
//   #define SCG_NUM_CHANNELS 72           // for US915
//   #define SCG_RX2_FREQ     434665000    // for EU433 RX2
//
// Defaults (EU868, SF7, 14dBm) are used if nothing is overridden.
#include <single_channel_lmic.h>

// ── LoRaWAN credentials ───────────────────────────────────────────────────────
// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3, 0x70.
static const u1_t PROGMEM APPEUI[8] = { 0x2C, 0xA5, 0x24, 0xFF, 0xFF, 0xEB, 0x27, 0xB8 };
void os_getArtEui(u1_t* buf) { memcpy_P(buf, APPEUI, 8); }

// This should also be in little-endian format, see above.
static const u1_t PROGMEM DEVEUI[8] = { 0xC6, 0x05, 0x07, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getDevEui(u1_t* buf) { memcpy_P(buf, DEVEUI, 8); }

// This key should be in big-endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { 0xDD, 0x35, 0xF2, 0xBA, 0x9F, 0xDA, 0x7D, 0x12, 0x86, 0xAA, 0xE0, 0xE5, 0xE2, 0x80, 0xA8, 0x1C };
void os_getDevKey(u1_t* buf) { memcpy_P(buf, APPKEY, 16); }

// ── Payload and job ───────────────────────────────────────────────────────────
static uint8_t mydata[] = "Hello, world!";
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 30;

// ── Pin mapping ───────────────────────────────────────────────────────────────
/*
 *  C. Pham's ProMini PCB

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 4,
  .dio = {2, 3, LMIC_UNUSED_PIN},
};
*/

/* Fabien Ferrero UCA breakout
 *
 */

// Pin mapping for Dragino LoRa Shield v1.4 on Arduino Uno/Mega
// Adjust these pins if your hardware is different!
const lmic_pinmap lmic_pins = {
    .nss  = 10,               // NSS (CS) pin
    .rxtx = LMIC_UNUSED_PIN,  // Not used on this shield
    .rst  = 9,                // Reset pin
    .dio  = {2, 6, 7},        // DIO0, DIO1, DIO2
};

// ─────────────────────────────────────────────────────────────────────────────

void do_send(osjob_t* j) {
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata) - 1, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after EV_TXCOMPLETE.
}

void onEvent(ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(F("["));
    Serial.print(osticks2ms(os_getTime()));
    Serial.print(F("]: "));

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

            // Print session keys for verification
            Serial.print(F("netid: "));
            Serial.println(LMIC.netid, DEC);
            Serial.print(F("devaddr: "));
            Serial.println(LMIC.devaddr, HEX);

            Serial.print(F("artKey: "));
            for (int i = 0; i < (int)sizeof(LMIC.artKey); i++) {
                if (i != 0) Serial.print(F("-"));
                if (LMIC.artKey[i] < 16) Serial.print(F("0"));
                Serial.print(LMIC.artKey[i], HEX);
            }
            Serial.println();

            Serial.print(F("nwkKey: "));
            for (int i = 0; i < (int)sizeof(LMIC.nwkKey); i++) {
                if (i != 0) Serial.print(F("-"));
                if (LMIC.nwkKey[i] < 16) Serial.print(F("0"));
                Serial.print(LMIC.nwkKey[i], HEX);
            }
            Serial.println();

            // Re-enforce single-channel state after join.
            // Handles: CFList channel injection, DR reset, link check mode.
            // See single_channel_lmic.h — SCG_on_joined() for details.
            SCG_on_joined();
            break;

        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;

        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
                Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
                Serial.print(F("Received "));
                Serial.print(LMIC.dataLen);
                Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
            break;

        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;

        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            // The library's nextJoinState() has already rotated to the next
            // join channel internally. Force it back to channel 0.
            // See single_channel_lmic.h — SCG_on_join_txcomplete() for details.
            SCG_on_join_txcomplete();
            break;

        default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void setup() {
    Serial.begin(38400);
    Serial.println(F("Starting"));

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Let LMIC compensate for +/- 20% clock error to better handle
    // downlink messages on imprecise oscillators (e.g. Arduino ceramic resonator)
    LMIC_setClockError(MAX_CLOCK_ERROR * 20 / 100);

    // ── Band-specific channel setup ───────────────────────────────────────────
    // Configure all channels for the band so the channelMap is fully populated
    // before SCG_init() selectively disables all but channel 0.
    // Note: SCG_init() MUST be called after this block.

#if defined(CFG_eu433)
    // Experimental — RAK 433 band plan
    // https://github.com/RAKWireless/rak_common_for_gateway/blob/master/lora/rak2245/global_conf/global_conf.eu_433.json
    LMIC_setupChannel(0, 433175000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);  // g-band
    LMIC_setupChannel(1, 433375000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);  // g-band
    LMIC_setupChannel(2, 433575000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);  // g-band
    LMIC_setupChannel(3, 433975000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);  // g-band
    LMIC_setupChannel(4, 434175000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);  // g-band
    LMIC_setupChannel(5, 434375000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);  // g-band
    LMIC_setupChannel(6, 434575000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);  // g-band
    LMIC_setupChannel(7, 434775000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);  // g-band
    LMIC_setupChannel(8, 434675000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);  // g2-band

#elif defined(CFG_eu868)
    // Set up the channels used by The Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, but can overload those frequencies — configure the full
    // frequency range of your network here.
    // NA-US channels 0-71 are configured automatically.
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);  // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);  // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);  // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);  // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);  // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);  // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);  // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);  // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);  // g2-band
    // TTN defines an additional channel at 869.525 MHz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.

#elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 (a subband) should be active.
    // TTN recommends the second sub band, 1 in a zero-based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
#endif

    // ── Single-channel enforcement ────────────────────────────────────────────
    // Disables channels 1-8, locks channelShuffleMap, sets DR/power/RX2,
    // and disables ADR. Must run after LMIC_setupChannel() calls above.
    // See single_channel_lmic.h — SCG_init() for full details.
    SCG_init();

    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}

void loop() {
    // Heartbeat LED — blinks at ~1 Hz using millis() bit 9 (toggles every 512 ms)
    unsigned long now = millis();
    if ((now & 512) != 0) {
        digitalWrite(13, HIGH);
    } else {
        digitalWrite(13, LOW);
    }

    // Continuously enforce single-channel compliance before each LMIC tick.
    // Catches runtime drift from LinkAdrReq MAC commands, join DR backoff,
    // or any other internal library state change that affects txChnl or datarate.
    // See single_channel_lmic.h — SCG_enforce() for full details.
    SCG_enforce();

    os_runloop_once();
}
