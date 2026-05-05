#include <SPI.h>
#include <avr/wdt.h>
#include <hal/hal.h>
#include <lmic.h>

#define ARDUINO_LMIC_PROJECT_CONFIG_H_SUPPRESS_WARNING

#define SCG_RX2_DR DR_SF9
#include <single_channel_lmic.h>

#include "DownlinkParser.h"

///////////////////////////////////////////////////////////////////
// OPTIONAL FEATURES — uncomment to enable
#define WITH_EEPROM
// #define FORCE_DEFAULT_VALUE
// #define LOW_POWER
// #define SHOW_LOW_POWER_CYCLE
///////////////////////////////////////////////////////////////////

#ifdef WITH_EEPROM
#include <EEPROM.h>
#endif
#ifdef LOW_POWER
#if defined(ARDUINO_AVR_PRO) || defined(ARDUINO_AVR_NANO) ||                   \
    defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MINI) ||                   \
    defined(ARDUINO_AVR_MEGA2560) || defined(__AVR_ATmega32U4__)
#include "LowPower.h"
#endif
#endif

// ABP session credentials (same device as MCCI_ttn_abp_SCG)
static const PROGMEM u1_t NWKSKEY[16] = {
    0x55, 0xF7, 0x14, 0xA6, 0x58, 0x11, 0xDE, 0xAF, 0xF4,
    0xAC, 0x6B, 0x65, 0x3F, 0xA5, 0xAB, 0xAD
    // 0xDB, 0xAC, 0xE3, 0x0A, 0xBC, 0x8D, 0x0A, 0xEC,
    // 0x70, 0xE2, 0xE5, 0xB8, 0x0E, 0xB4, 0x1D, 0x81
};
static const u1_t PROGMEM APPSKEY[16] = {
    0xBC, 0xA5, 0x0B, 0x27, 0x85, 0x31, 0xF0, 0x28, 0xBE,
    0x75, 0x9A, 0x66, 0x84, 0x15, 0x76, 0xED
    // 0xA9, 0xCB, 0xDB, 0x31, 0x32, 0x40, 0x2D, 0x17,
    // 0xEE, 0xEE, 0xAC, 0x12, 0x73, 0x39, 0x24, 0x0A
};
static const u4_t DEVADDR = 0x260BE0A2;
// 0x260B9642;

void os_getArtEui(u1_t *buf) {}
void os_getDevEui(u1_t *buf) {}
void os_getDevKey(u1_t *buf) {}

// 10-byte positional payload: temp | hum | MQ9 ADC | MQ9 Rs/R0 | dust ADC
static uint8_t mydata[10];
static osjob_t sendjob;

#define DEFAULT_TX_INTERVAL 30
unsigned TX_INTERVAL = DEFAULT_TX_INTERVAL;

#ifdef WITH_EEPROM
struct lmic_eeprom_t {
  uint8_t flag1;
  uint8_t flag2;
  uint32_t seqnoUp;
  unsigned tx_interval;
  uint8_t overwrite;
};
static lmic_eeprom_t my_eeprom;
#endif

#ifdef LOW_POWER
void lowPower(unsigned long ms) {
  unsigned long remaining = ms;
  Serial.flush();
  delay(5);
#if defined(ARDUINO_AVR_PRO) || defined(ARDUINO_AVR_NANO) ||                   \
    defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MINI) ||                   \
    defined(ARDUINO_AVR_MEGA2560) || defined(__AVR_ATmega32U4__)
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

const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7},
};

// ── Fake sensor helpers
// ─────────────────────────────────────────────────────── Returns a random
// float in [lo, hi] with one decimal of precision.
static float fakeFloat(float lo, float hi) {
  return lo + (random(0, (long)((hi - lo) * 10 + 1))) / 10.0f;
}

// Returns a random int in [lo, hi].
static int fakeInt(int lo, int hi) { return random(lo, hi + 1); }

// ── Software restart / factory reset
// ──────────────────────────────────────────
static void softwareReset() {
  Serial.println(F("Restarting..."));
  Serial.flush();
  delay(100);
  wdt_enable(WDTO_15MS);
  while (1) {
  } // watchdog fires → hardware reset
}

static void factoryReset() {
  Serial.println(F("Factory reset..."));
#ifdef WITH_EEPROM
  my_eeprom.flag1 = 0x12;
  my_eeprom.flag2 = 0x35;
  my_eeprom.seqnoUp = 0;
  my_eeprom.tx_interval = DEFAULT_TX_INTERVAL;
  my_eeprom.overwrite = 0;
  EEPROM.put(0, my_eeprom);
  Serial.println(F("EEPROM cleared"));
#endif
  softwareReset(); // restart with clean state
}

void onEvent(ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(F(": "));
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
    SCG_on_joined();
    break;
  case EV_JOIN_FAILED:
    Serial.println(F("EV_JOIN_FAILED"));
    break;
  case EV_REJOIN_FAILED:
    Serial.println(F("EV_REJOIN_FAILED"));
    break;
  case EV_TXCOMPLETE:
    Serial.println(F("EV_TXCOMPLETE"));
    Serial.print(F("txrxFlags: 0x"));
    Serial.println(LMIC.txrxFlags, HEX);
    if (LMIC.txrxFlags & TXRX_DNW1)
      Serial.println(F("Received in RX1"));
    else if (LMIC.txrxFlags & TXRX_DNW2)
      Serial.println(F("Received in RX2"));
    else if (LMIC.dataLen == 0)
      Serial.println(F("No downlink received"));
    if (LMIC.txrxFlags & TXRX_ACK)
      Serial.println(F("Received ACK"));
    {
      // ── Parse downlink command if present ──
      DownlinkResult dlResult = {DL_NONE, 0};
      if (LMIC.dataLen) {
        Serial.print(F("Downlink "));
        Serial.print(LMIC.dataLen);
        Serial.println(F(" bytes"));
        uint8_t dlBuf[20];
        uint8_t dlLen =
            min((uint8_t)(sizeof(dlBuf) - 1), (uint8_t)LMIC.dataLen);
        memcpy(dlBuf, LMIC.frame + LMIC.dataBeg, dlLen);
        dlBuf[dlLen] = '\0';
        dlResult = parseDownlinkCommand(dlBuf, dlLen);
      }

      // Update TX_INTERVAL if changed
      if (dlResult.action == DL_INTERVAL_CHANGED) {
        TX_INTERVAL = (unsigned)dlResult.value;
#ifdef WITH_EEPROM
        my_eeprom.tx_interval = TX_INTERVAL;
        my_eeprom.overwrite = 1;
#endif
      }

      // Persist seqnoUp
#ifdef WITH_EEPROM
      my_eeprom.seqnoUp = LMIC.seqnoUp;
      EEPROM.put(0, my_eeprom);
      Serial.print(F("EEPROM saved seqnoUp="));
      Serial.println(LMIC.seqnoUp);
#endif

      // Handle restart / factory reset (these never return)
      if (dlResult.action == DL_FACTORY_RESET)
        factoryReset();
      if (dlResult.action == DL_RESTART)
        softwareReset();

      // Calculate next TX delay (sleep overrides normal interval)
      unsigned long nextSec = (dlResult.action == DL_SLEEP)
                                  ? dlResult.value * 60UL
                                  : (unsigned long)TX_INTERVAL;

#ifdef LOW_POWER
      long jitter = random(-(long)(nextSec * 100), (long)(nextSec * 100));
      unsigned long sleep_ms = nextSec * 1000UL + jitter;
      Serial.print(F("Sleeping "));
      Serial.print(sleep_ms / 1000.0, 1);
      Serial.println(F("s"));
      Serial.flush();
      lowPower(sleep_ms);
      os_setCallback(&sendjob, do_send);
#else
      long jitter_ticks = sec2osticks(nextSec) / 10;
      long rand_jitter = random(-jitter_ticks, jitter_ticks);
      ostime_t next_tx = os_getTime() + sec2osticks(nextSec) + rand_jitter;
      Serial.print(F("Next TX in "));
      Serial.print(nextSec);
      Serial.println(F("s"));
      os_setTimedCallback(&sendjob, next_tx, do_send);
#endif
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
    break;
  case EV_JOIN_TXCOMPLETE:
    Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
    SCG_on_join_txcomplete();
    break;
  default:
    Serial.print(F("Unknown event: "));
    Serial.println((unsigned)ev);
    break;
  }
}

void do_send(osjob_t *j) {
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
    return;
  }

  // Generate fake sensor readings in realistic ranges (Yaoundé climate)
  float temp = fakeFloat(18.0, 38.0); // °C
  float hum = fakeFloat(40.0, 95.0);  // %
  int mq9 = fakeInt(80, 700);         // 10-bit ADC count
  float ratio = fakeFloat(0.5, 8.0);  // Rs/R0
  int dust = fakeInt(30, 500);        // 10-bit ADC count

  int16_t temp_s = (int16_t)(temp * 100);
  uint16_t hum_s = (uint16_t)(hum * 100);
  uint16_t mq9_s = (uint16_t)mq9;
  uint16_t ratio_s = (uint16_t)(ratio * 1000);
  uint16_t dust_s = (uint16_t)dust;

  mydata[0] = highByte(temp_s);
  mydata[1] = lowByte(temp_s);
  mydata[2] = highByte(hum_s);
  mydata[3] = lowByte(hum_s);
  mydata[4] = highByte(mq9_s);
  mydata[5] = lowByte(mq9_s);
  mydata[6] = highByte(ratio_s);
  mydata[7] = lowByte(ratio_s);
  mydata[8] = highByte(dust_s);
  mydata[9] = lowByte(dust_s);

  // Dust density for Serial monitor only (not transmitted)
  float dust_v = dust * (5.0 / 1023.0);
  float dust_d = max(0.0f, 170.0f * dust_v - 100.0f);

  Serial.print(F("T:"));
  Serial.print(temp, 1);
  Serial.print(F("C H:"));
  Serial.print(hum, 1);
  Serial.print(F("% MQ9:"));
  Serial.print(mq9);
  Serial.print(F(" Rs/R0:"));
  Serial.print(ratio, 3);
  Serial.print(F(" Dust:"));
  Serial.print(dust_d, 1);
  Serial.print(F("ug/m3"));
  Serial.print(F(" ["));
  for (int i = 0; i < 10; i++) {
    if (mydata[i] < 0x10)
      Serial.print("0");
    Serial.print(mydata[i], HEX);
    if (i < 9)
      Serial.print(" ");
  }
  Serial.println(F("]"));

  LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
}

void setup() {
  wdt_disable(); // in case we came from a watchdog reset
  Serial.begin(38400);
  Serial.println(F("Fake DHT22+MQ9+Dust | ABP | Single-CH 868.1 MHz"));
  Serial.println(F(
      "Downlink: /@I<s># interval, /@S<min># sleep, /@R# restart, /@Z# reset"));

  randomSeed(analogRead(A0)); // seed from floating pin noise

  os_init();
  LMIC_reset();
  LMIC_setClockError(MAX_CLOCK_ERROR * 20 / 100);

  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession(0x13, DEVADDR, nwkskey, appskey);

#ifdef WITH_EEPROM
  EEPROM.get(0, my_eeprom);
  if (my_eeprom.flag1 == 0x12 && my_eeprom.flag2 == 0x35) {
    Serial.println(F("EEPROM: restoring config"));
    Serial.print(F("  seqnoUp : "));
    Serial.println(my_eeprom.seqnoUp);
#ifdef FORCE_DEFAULT_VALUE
    Serial.println(F("  FORCE_DEFAULT_VALUE: resetting"));
    my_eeprom.seqnoUp = 0;
    my_eeprom.tx_interval = TX_INTERVAL;
    my_eeprom.overwrite = 0;
    EEPROM.put(0, my_eeprom);
#else
    LMIC.seqnoUp = my_eeprom.seqnoUp;
    if (my_eeprom.overwrite == 1 && my_eeprom.tx_interval != 0) {
      TX_INTERVAL = my_eeprom.tx_interval;
      Serial.print(F("  TX_INTERVAL: "));
      Serial.print(TX_INTERVAL);
      Serial.println(F("s"));
    }
#endif
  } else {
    my_eeprom.flag1 = 0x12;
    my_eeprom.flag2 = 0x35;
    my_eeprom.seqnoUp = 0;
    my_eeprom.tx_interval = TX_INTERVAL;
    my_eeprom.overwrite = 0;
    EEPROM.put(0, my_eeprom);
    Serial.println(F("EEPROM: initialized"));
  }
#endif

#if defined(CFG_eu868)
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK, DR_FSK), BAND_MILLI);
#elif defined(CFG_us915) || defined(CFG_au915)
  LMIC_selectSubBand(1);
#elif defined(CFG_as923)
#elif defined(CFG_kr920)
#elif defined(CFG_in866)
#else
#error Region not supported
#endif

#if !defined(DISABLE_MCMD_DlChannelReq)
  for (uint8_t i = 0; i < 9; i++)
    LMIC.channelDlFreq[i] = 0;
#endif

  SCG_init();

  do_send(&sendjob);
}

void loop() {
  SCG_enforce();
  os_runloop_once();
}
