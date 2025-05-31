#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#define ARDUINO_LMIC_PROJECT_CONFIG_H_SUPPRESS_WARNING

// MQ2 Gas Sensor Setup
#define MQ2_PIN A0          // Arduino's pin connected to AO pin of the MQ2 sensor
#define WARMUP_DELAY 20000  // 20 seconds for MQ2 sensor to warm up
#define RL_VALUE 10000      // Load resistance in ohms (10kΩ)
#define VCC 5.0             // Circuit voltage

// Calibration values (should be measured in clean air after 24-48h preheating)
float R0_LPG = 10000.0;     // Baseline resistance for LPG in clean air
float R0_METHANE = 10000.0; // Baseline resistance for Methane in clean air
float R0_CO = 10000.0;      // Baseline resistance for CO in clean air

// LoRaWAN Network Session Key (MSB format)
static const PROGMEM u1_t NWKSKEY[16] = { 
    0x55, 0xF7, 0x14, 0xA6, 0x58, 0x11, 0xDE, 0xAF, 
    0xF4, 0xAC, 0x6B, 0x65, 0x3F, 0xA5, 0xAB, 0xAD 
};

// LoRaWAN Application Session Key (MSB format)
static const u1_t PROGMEM APPSKEY[16] = { 
    0xBC, 0xA5, 0x0B, 0x27, 0x85, 0x31, 0xF0, 0x28, 
    0xBE, 0x75, 0x9A, 0x66, 0x84, 0x15, 0x76, 0xED 
};

// LoRaWAN end-device address (DevAddr) - Change this for every node!
static const u4_t DEVADDR = 0x260BE0A2;

// These callbacks are only used in over-the-air activation (OTAA)
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

// Enhanced payload buffer (10 bytes for multiple gas concentrations)
static uint8_t mydata[10];
static osjob_t sendjob;

// Schedule TX every this many seconds
const unsigned TX_INTERVAL = 60; // Increased to 60s for more detailed analysis

// Pin mapping for Dragino LoRa Shield v1.4
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7},
};

// Gas concentration calculation functions
float calculateResistance(int adc_value) {
    float Vout = adc_value * (VCC / 1023.0);
    if (Vout == 0) return 0; // Prevent division by zero
    float Rs = (VCC - Vout) * RL_VALUE / Vout;
    return Rs;
}

float calculateLPG(float rs_r0_ratio) {
    if (rs_r0_ratio <= 0) return 0;
    return 987.26 * pow(rs_r0_ratio, -2.162);
}

float calculateMethane(float rs_r0_ratio) {
    if (rs_r0_ratio <= 0) return 0;
    return 2217.8 * pow(rs_r0_ratio, -2.827);
}

float calculateHydrogen(float rs_r0_ratio) {
    if (rs_r0_ratio <= 0) return 0;
    return 988.05 * pow(rs_r0_ratio, -1.767);
}

float calculateCO(float rs_r0_ratio) {
    if (rs_r0_ratio <= 0) return 0;
    return 605.18 * pow(rs_r0_ratio, -3.937);
}

float calculateAlcohol(float rs_r0_ratio) {
    if (rs_r0_ratio <= 0) return 0;
    return 75.906 * pow(rs_r0_ratio, -1.691);
}

float calculatePropane(float rs_r0_ratio) {
    if (rs_r0_ratio <= 0) return 0;
    return 614.56 * pow(rs_r0_ratio, -2.564);
}

float calculateSmoke(float rs_r0_ratio) {
    if (rs_r0_ratio <= 0) return 0;
    return 143.01 * pow(rs_r0_ratio, -2.186);
}

String getGasAlert(float lpg, float methane, float co, float alcohol) {
    String alerts = "";
    
    // Safety thresholds (ppm)
    if (co > 200) alerts += "CO-DANGER ";
    else if (co > 50) alerts += "CO-HIGH ";
    
    if (lpg > 5000) alerts += "LPG-DANGER ";
    else if (lpg > 1000) alerts += "LPG-HIGH ";
    
    if (methane > 5000) alerts += "CH4-DANGER ";
    else if (methane > 1000) alerts += "CH4-HIGH ";
    
    if (alcohol > 1000) alerts += "ALCOHOL-HIGH ";
    
    if (alerts == "") alerts = "NORMAL";
    
    return alerts;
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

// Alternative: Send ONLY raw values for TTN-side calculation
// This saves Arduino processing and allows flexible formula updates

void do_send(osjob_t* j){
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Read MQ2 sensor (averaged)
        long sum = 0;
        for (int i = 0; i < 10; i++) {
            sum += analogRead(MQ2_PIN);
            delay(50);
        }
        int sensorValue = sum / 10;
        
        // Calculate basic parameters
        float Rs = calculateResistance(sensorValue);
        float voltage = sensorValue * (VCC / 1023.0);
        float R0_avg = (R0_LPG + R0_METHANE + R0_CO) / 3.0;
        float rs_r0_ratio = Rs / R0_avg;
        
        // RAW-ONLY PAYLOAD (6 bytes instead of 10)
        // Byte 0-1: Raw ADC value
        mydata[0] = highByte(sensorValue);
        mydata[1] = lowByte(sensorValue);
        
        // Byte 2-3: Sensor resistance Rs (divided by 10 for scaling)
        uint16_t rs_scaled = (uint16_t)(Rs / 10);
        mydata[2] = highByte(rs_scaled);
        mydata[3] = lowByte(rs_scaled);
        
        // Byte 4-5: Rs/R0 ratio (scaled by 1000)
        uint16_t ratio_scaled = (uint16_t)(rs_r0_ratio * 1000);
        mydata[4] = highByte(ratio_scaled);
        mydata[5] = lowByte(ratio_scaled);
        
        Serial.println(F("=== Raw Sensor Data ==="));
        Serial.print(F("ADC: ")); Serial.println(sensorValue);
        Serial.print(F("Voltage: ")); Serial.print(voltage, 3); Serial.println(F("V"));
        Serial.print(F("Rs: ")); Serial.print(Rs, 0); Serial.println(F("Ω"));
        Serial.print(F("Rs/R0: ")); Serial.println(rs_r0_ratio, 3);
        
        // Show payload
        Serial.print(F("Payload (hex): "));
        for (int i = 0; i < 6; i++) { // Only 6 bytes now
            if (mydata[i] < 0x10) Serial.print("0");
            Serial.print(mydata[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
        
        LMIC_setTxData2(1, mydata, 6, 0); // 6 bytes instead of 10
        Serial.println(F("📡 Raw sensor data packet queued"));
        Serial.println(F("All gas calculations will be done on TTN"));
        Serial.println(F("======================="));
    }
}

void setup() {
    delay(5000);
    Serial.begin(115200);
    Serial.println(F("🔬 MQ2 Multi-Gas Analyzer with MCCI LMIC & TTN"));
    Serial.println(F("================================================"));
    
    // Initialize MQ2 sensor
    pinMode(MQ2_PIN, INPUT);
    Serial.println(F("Warming up MQ2 sensor for gas detection..."));
    Serial.println(F("Please wait 20 seconds for sensor stabilization..."));
    
    // Warmup countdown
    for (int i = 20; i > 0; i--) {
        Serial.print(F("Warmup: "));
        Serial.print(i);
        Serial.println(F(" seconds remaining"));
        delay(1000);
    }
    
    Serial.println(F("MQ2 sensor warmup complete!"));
    
    // Calibration reminder
    Serial.println(F("CALIBRATION NOTICE:"));
    Serial.println(F("For accurate readings, calibrate in clean air"));
    Serial.println(F("after 24-48 hours of preheating."));
    
    // Initial reading for R0 estimation
    delay(1000);
    int initial_adc = analogRead(MQ2_PIN);
    float initial_Rs = calculateResistance(initial_adc);
    
    Serial.print(F("Initial resistance (R0 estimate): "));
    Serial.print(initial_Rs, 0);
    Serial.println(F("Ω"));
    
    // Update R0 values with initial reading (rough calibration)
    R0_LPG = R0_METHANE = R0_CO = initial_Rs;
    
    Serial.println(F("================================================"));

    // LMIC init
    os_init();
    LMIC_reset();

    // Set static session parameters for ABP
    #ifdef PROGMEM
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x13, DEVADDR, nwkskey, appskey);
    #else
    LMIC_setSession (0x13, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    #if defined(CFG_eu868)
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
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
    #endif

    LMIC_setLinkCheckMode(0);
    LMIC.dn2Dr = DR_SF9;
    LMIC_setDrTxpow(DR_SF7, 14);

    // Start job
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}