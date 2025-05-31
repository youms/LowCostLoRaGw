#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define ARDUINO_LMIC_PROJECT_CONFIG_H_SUPPRESS_WARNING

// OneWire DS18B20 Temperature Sensor Setup
#define ONE_WIRE_BUS 4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// MQ2 Gas Sensor Setup
#define MQ2_PIN A0          // Arduino's pin connected to AO pin of the MQ2 sensor
#define WARMUP_DELAY 20000  // 20 seconds for MQ2 sensor to warm up

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
// For ABP, we leave them empty but they must be present
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

// Combined payload buffer (4 bytes: 2 for temperature, 2 for gas sensor)
static uint8_t mydata[4];
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty cycle limitations)
const unsigned TX_INTERVAL = 30;

// Pin mapping for Dragino LoRa Shield v1.4 on Arduino Uno/Mega
// Adjust these pins if your hardware is different!
const lmic_pinmap lmic_pins = {
    .nss = 10,                      // NSS (CS) pin
    .rxtx = LMIC_UNUSED_PIN,        // Not used on this shield
    .rst = 9,                       // Reset pin
    .dio = {2, 6, 7},               // DIO0, DIO1, DIO2
};

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
              Serial.print(F("Data: "));
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
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Get temperature reading
        sensors.requestTemperatures(); 
        float tempC = sensors.getTempCByIndex(0);
        
        // Check if temperature reading is valid
        if (tempC == DEVICE_DISCONNECTED_C) {
            Serial.println(F("Error: Could not read temperature data"));
            tempC = 0.0; // Use 0 as fallback
        }
        
        // Convert temperature to 2 bytes (16-bit integer)
        // Multiply by 100 to preserve 2 decimal places
        int16_t tempInt = (int16_t)(tempC * 100); 
        mydata[0] = highByte(tempInt);
        mydata[1] = lowByte(tempInt);
        
        // Read MQ2 sensor value (0-1023 for 10-bit ADC)
        int sensorValue = analogRead(MQ2_PIN);
        
        // Convert to voltage for better understanding (assuming 5V reference)
        float voltage = sensorValue * (5.0 / 1023.0);
        
        // Store MQ2 value in the last 2 bytes
        mydata[2] = highByte(sensorValue);
        mydata[3] = lowByte(sensorValue);
        
        // Display readings
        Serial.println(F("=== Sensor Readings ==="));
        Serial.print(F("Temperature: "));
        Serial.print(tempC);
        Serial.print(F(" °C ("));
        Serial.print(tempInt);
        Serial.println(F(")"));
        
        Serial.print(F("MQ2 Raw Value: "));
        Serial.print(sensorValue);
        Serial.print(F(" | Voltage: "));
        Serial.print(voltage);
        Serial.println(F("V"));
        
        // Alert conditions
        if (tempC > 30.0) {
            Serial.println(F("*** WARNING: High temperature detected! ***"));
        }
        if (tempC < 0.0) {
            Serial.println(F("*** WARNING: Freezing temperature detected! ***"));
        }
        if (sensorValue > 300) {
            Serial.println(F("*** WARNING: High gas level detected! ***"));
        }
        
        // Show payload being sent
        Serial.print(F("Payload (hex): "));
        for (int i = 0; i < sizeof(mydata); i++) {
            if (mydata[i] < 0x10) Serial.print("0");
            Serial.print(mydata[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
        
        // Prepare upstream data transmission at the next possible time
        LMIC_setTxData2(1, mydata, sizeof(mydata), 0); // port 1, 4 bytes, no ack
        Serial.println(F("Packet queued - Combined sensor data"));
        Serial.println(F("======================="));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    delay(5000); // Give time to open serial monitor
    Serial.begin(115200);
    Serial.println(F("Starting Combined Temperature & Gas Sensor with MCCI LMIC & TTN"));

    // Initialize temperature sensor
    sensors.begin();
    Serial.print(F("Temperature sensor devices found: "));
    Serial.println(sensors.getDeviceCount());
    
    if (sensors.getDeviceCount() == 0) {
        Serial.println(F("No DS18B20 temperature sensors found!"));
        Serial.println(F("Check wiring: VCC, GND, Data to pin 4 with 4.7k pullup"));
    }

    // Initialize MQ2 sensor
    pinMode(MQ2_PIN, INPUT);
    Serial.println(F("Warming up MQ2 sensor..."));
    Serial.println(F("Please wait 20 seconds for sensor stabilization..."));
    
    // Show countdown for warmup
    for (int i = 20; i > 0; i--) {
        Serial.print(F("Warmup: "));
        Serial.print(i);
        Serial.println(F(" seconds remaining"));
        delay(1000);
    }
    
    Serial.println(F("MQ2 sensor warmup complete!"));
    
    // Read initial sensor values
    int initialMQ2 = analogRead(MQ2_PIN);
    sensors.requestTemperatures();
    float initialTemp = sensors.getTempCByIndex(0);
    
    Serial.println(F("=== Initial Readings ==="));
    Serial.print(F("Initial temperature: "));
    if (initialTemp != DEVICE_DISCONNECTED_C) {
        Serial.print(initialTemp);
        Serial.println(F(" °C"));
    } else {
        Serial.println(F("No temperature sensor detected"));
    }
    Serial.print(F("Initial MQ2 reading: "));
    Serial.println(initialMQ2);
    Serial.println(F("========================"));

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters for ABP
    // Instead of dynamically establishing a session by joining the network, 
    // precomputed session parameters are provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x13, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x13, DEVADDR, NWKSKEY, APPSKEY);
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

    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}