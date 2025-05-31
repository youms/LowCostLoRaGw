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

void onEvent (ev_t ev) {
    switch(ev) {
        case EV_TXCOMPLETE:
            Serial.println(F("TX_OK"));
            if (LMIC.dataLen) {
                Serial.print(F("RX:"));
                Serial.println(LMIC.dataLen);
            }
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_TXSTART:
            Serial.println(F("TX_START"));
            break;
        default:
            break; // Minimize serial output to save memory
    }
}

void do_send(osjob_t* j){
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("Busy"));
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
    Serial.println(F("MQ2+DS18B20 Raw"));
    
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
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
    #elif defined(CFG_us915) || defined(CFG_au915)
    LMIC_selectSubBand(1);
    #endif

    LMIC_setLinkCheckMode(0);
    LMIC.dn2Dr = DR_SF9;
    LMIC_setDrTxpow(DR_SF7, 14);

    Serial.println(F("Starting..."));
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}