#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define ARDUINO_LMIC_PROJECT_CONFIG_H_SUPPRESS_WARNING

// Sensor pins
#define MQ2_PIN A0
#define DS18B20_PIN 4

// OneWire setup
OneWire oneWire(DS18B20_PIN);
DallasTemperature sensors(&oneWire);

// MQ2 constants
#define RL_VALUE 10000
#define VCC 5.0
float R0 = 10000.0;

// LoRaWAN credentials
static const PROGMEM u1_t NWKSKEY[16] = { 
    0x55, 0xF7, 0x14, 0xA6, 0x58, 0x11, 0xDE, 0xAF, 
    0xF4, 0xAC, 0x6B, 0x65, 0x3F, 0xA5, 0xAB, 0xAD 
};

static const u1_t PROGMEM APPSKEY[16] = { 
    0xBC, 0xA5, 0x0B, 0x27, 0x85, 0x31, 0xF0, 0x28, 
    0xBE, 0x75, 0x9A, 0x66, 0x84, 0x15, 0x76, 0xED 
};

static const u4_t DEVADDR = 0x260BE0A2;

void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

// Combined payload (8 bytes: 6 for MQ2 + 2 for temp)
static uint8_t mydata[8];
static osjob_t sendjob;

const unsigned TX_INTERVAL = 60;

const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7},
};

float getResistance(int adc) {
    float vout = adc * (VCC / 1023.0);
    if (vout <= 0.01) return 0;
    return (VCC - vout) * RL_VALUE / vout;
}

void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
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
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("AppSKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  Serial.print("-");
                printHex2(artKey[i]);
              }
              Serial.println("");
              Serial.print("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              Serial.print("-");
                      printHex2(nwkKey[i]);
              }
              Serial.println();
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
            // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
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
              Serial.print(F("Downlink data: "));
              for (int i = 0; i < LMIC.dataLen; i++) {
                if (LMIC.frame[LMIC.dataBeg + i] < 0x10) {
                  Serial.print(F("0"));
                }
                Serial.print(LMIC.frame[LMIC.dataBeg + i], HEX);
                Serial.print(" ");
              }
              Serial.println();
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
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
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;
        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void do_send(osjob_t* j){
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
        return;
    }
    
    // Read MQ2 (simplified - 3 readings average)
    long sum = 0;
    for (int i = 0; i < 3; i++) {
        sum += analogRead(MQ2_PIN);
        delay(50);
    }
    int adc = sum / 3;
    
    float Rs = getResistance(adc);
    float ratio = Rs / R0;
    
    // Read temperature
    sensors.requestTemperatures();
    float temp = sensors.getTempCByIndex(0);
    if (temp == DEVICE_DISCONNECTED_C) temp = 0.0;
    
    // Prepare 8-byte payload
    // Bytes 0-1: MQ2 ADC
    mydata[0] = highByte(adc);
    mydata[1] = lowByte(adc);
    
    // Bytes 2-3: MQ2 resistance (scaled)
    uint16_t rs_scaled = (uint16_t)(Rs / 10);
    mydata[2] = highByte(rs_scaled);
    mydata[3] = lowByte(rs_scaled);
    
    // Bytes 4-5: Rs/R0 ratio (scaled)
    uint16_t ratio_scaled = (uint16_t)(ratio * 1000);
    mydata[4] = highByte(ratio_scaled);
    mydata[5] = lowByte(ratio_scaled);
    
    // Bytes 6-7: Temperature (x100)
    int16_t temp_scaled = (int16_t)(temp * 100);
    mydata[6] = highByte(temp_scaled);
    mydata[7] = lowByte(temp_scaled);
    
    // Compact output
    Serial.print(F("T:"));
    Serial.print(temp, 1);
    Serial.print(F("C MQ2:"));
    Serial.print(adc);
    Serial.print(F("/"));
    Serial.print(ratio, 2);
    
    // Simple alerts
    if (ratio < 0.5) Serial.print(F(" GAS!"));
    if (temp > 30) Serial.print(F(" HOT!"));
    if (temp < 0) Serial.print(F(" COLD!"));
    
    Serial.print(F(" ["));
    for (int i = 0; i < 8; i++) {
        if (mydata[i] < 0x10) Serial.print("0");
        Serial.print(mydata[i], HEX);
        if (i < 7) Serial.print(" ");
    }
    Serial.println(F("]"));
    
    LMIC_setTxData2(1, mydata, 8, 0);
}

void setup() {
    Serial.begin(9600);
    Serial.println(F("MQ2+DS18B20 Raw with Complete LoRa Event Handling"));
    
    // Initialize sensors
    sensors.begin();
    pinMode(MQ2_PIN, INPUT);
    
    Serial.print(F("Temp sensors: "));
    Serial.println(sensors.getDeviceCount());
    
    // MQ2 warmup (shorter for memory reasons)
    Serial.println(F("Warmup 15s..."));
    for (int i = 15; i > 0; i--) {
        if (i % 5 == 0) {
            Serial.print(i);
            Serial.print(F("s "));
        }
        delay(1000);
    }
    Serial.println();
    
    // Quick calibration
    int cal_adc = analogRead(MQ2_PIN);
    R0 = getResistance(cal_adc);
    Serial.print(F("R0:"));
    Serial.println((int)R0);

    // LMIC minimal setup
    os_init();
    LMIC_reset();

    #ifdef PROGMEM
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession(0x13, DEVADDR, nwkskey, appskey);
    #else
    LMIC_setSession(0x13, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set. The LMIC doesn't let you change
    // the three basic settings, but we show them here.
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
    #elif defined(CFG_us915) || defined(CFG_au915)
    // NA-US and AU channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
    #elif defined(CFG_as923)
    // Set up the channels used in your country. Only two are defined by default,
    // and they cannot be changed.  Use BAND_CENTI to indicate 1% duty cycle.
    // LMIC_setupChannel(0, 923200000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
    // LMIC_setupChannel(1, 923400000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);

    // ... extra definitions for channels 2..n here
    #elif defined(CFG_kr920)
    // Set up the channels used in your country. Three are defined by default,
    // and they cannot be changed. Duty cycle doesn't matter, but is conventionally
    // BAND_MILLI.
    // LMIC_setupChannel(0, 922100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
    // LMIC_setupChannel(1, 922300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
    // LMIC_setupChannel(2, 922500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);

    // ... extra definitions for channels 3..n here.
    #elif defined(CFG_in866)
    // Set up the channels used in your country. Three are defined by default,
    // and they cannot be changed. Duty cycle doesn't matter, but is conventionally
    // BAND_MILLI.
    // LMIC_setupChannel(0, 865062500, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
    // LMIC_setupChannel(1, 865402500, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
    // LMIC_setupChannel(2, 865985000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);

    // ... extra definitions for channels 3..n here.
    #else
    # error Region not supported
    #endif

    // Disable link check validation (automatically enabled
    // during join, but because slow data rates change max TX
    // size, we don't use it in this example.
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note this may be overridden
    // by the radio's default values) 
    LMIC_setDrTxpow(DR_SF7, 14);

    Serial.println(F("Starting..."));
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}