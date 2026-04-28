# SingleChannelLMIC

**User-space single-channel enforcement layer for the MCCI LoRaWAN LMIC library.**  
Library-update-safe — zero modifications to MCCI source files required.

*Project 3 — MCCI LoRaWAN Library Modifications Catalog for Single-Channel Operation*  
*University of Yaoundé 1, Department of Computer Sciences — Academic Year 2025-2026*  
*Author: KOUEGUA YOUMBI*

---

## Why this library exists

The MCCI LoRaWAN LMIC library is built around multi-channel gateways. Its internal scheduler (`LMIC_findNextChannel`), join procedure (`LMICeulike_initJoinLoop`), and MAC command handlers (`LinkAdrReq`, `DlChannelReq`, `CFList`) all assume that multiple channels are available and will actively rotate, add, or renegotiate them.

Low-cost single-channel gateways (e.g. Raspberry Pi + SX1276) listen on exactly one frequency. Any frame sent on a second channel is lost.

The naive fix — patching the MCCI library source — works until the next library update overwrites those patches. This library provides the same protection entirely in user space, as a single header that any sketch includes. The MCCI library is never touched.

---

## Installation

Copy the `SingleChannelLMIC` folder into your Arduino `libraries` directory:

```
Arduino/
  libraries/
    SingleChannelLMIC/
      single_channel_lmic.h
      library.properties
      README.md
```

Restart the Arduino IDE. The library will appear under **Sketch → Include Library → SingleChannelLMIC**.

**Dependency:** [MCCI LoRaWAN LMIC library](https://github.com/mcci-catena/arduino-lmic) must be installed.

---

## Quick start

### ABP sketch

```cpp
#define ARDUINO_LMIC_PROJECT_CONFIG_H_SUPPRESS_WARNING

// Override RX2 data rate before including the header (optional — default is DR_SF12)
#define SCG_RX2_DR  DR_SF9
#include <single_channel_lmic.h>

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

// ... credentials and pin map ...

void setup() {
    // ... LMIC_reset(), LMIC_setupChannel(), LMIC_setSession() ...

    // Keep the DlChannelReq guard in the sketch (compile-time conditional)
#if !defined(DISABLE_MCMD_DlChannelReq)
    for (uint8_t i = 0; i < 9; i++) LMIC.channelDlFreq[i] = 0;
#endif

    SCG_init();   // replaces all manual channel/DR/RX2 setup
    do_send(&sendjob);
}

void loop() {
    SCG_enforce();        // must come before os_runloop_once()
    os_runloop_once();
}
```

### OTAA sketch

```cpp
#define SCG_RX2_DR  DR_SF9
#include <single_channel_lmic.h>

// ...

void onEvent(ev_t ev) {
    switch (ev) {
        case EV_JOINED:
            SCG_on_joined();          // counters CFList channel injection
            break;
        case EV_JOIN_TXCOMPLETE:
            SCG_on_join_txcomplete(); // counters join channel rotation
            break;
        // ...
    }
}

void setup() {
    // ... channel setup block, LMIC_setSession() ...
#if !defined(DISABLE_MCMD_DlChannelReq)
    for (uint8_t i = 0; i < 9; i++) LMIC.channelDlFreq[i] = 0;
#endif
    SCG_init();
    do_send(&sendjob);
}

void loop() {
    SCG_enforce();
    os_runloop_once();
}
```

---

## Configuration

Override any of these **before** `#include <single_channel_lmic.h>`:

| Define | Default | Description |
|--------|---------|-------------|
| `SCG_CHANNEL` | `0` | Channel index to enforce (0 = ch0, EU868: 868.1 MHz) |
| `SCG_DATARATE` | `DR_SF7` | Spreading factor to enforce |
| `SCG_TXPOWER` | `14` | TX power in dBm (EU868 regulatory maximum) |
| `SCG_NUM_CHANNELS` | `9` | Total number of channels in the band (EU868: 9, US915: 72) |
| `SCG_RX2_DR` | `DR_SF12` | RX2 window data rate |
| `SCG_RX2_FREQ` | `869525000` | RX2 window frequency in Hz (TTN EU868 default) |

**Example — enforce SF9 instead of SF7:**
```cpp
#define SCG_DATARATE  DR_SF9
#include <single_channel_lmic.h>
```

---

## API reference

| Function | Where to call | Purpose |
|----------|---------------|---------|
| `SCG_init()` | `setup()` — after `LMIC_reset()` and all `LMIC_setupChannel()` calls | Disables all channels except `SCG_CHANNEL`, locks `channelShuffleMap`, disables ADR, sets DR/TXpow, sets RX2, disables link check |
| `SCG_on_joined()` | `onEvent()` — `EV_JOINED` case | Re-disables channels added by CFList in Join Accept, re-locks shuffle map, re-asserts DR, disables link check |
| `SCG_on_join_txcomplete()` | `onEvent()` — `EV_JOIN_TXCOMPLETE` case | Corrects `txChnl` after join channel rotation by `nextJoinState()` |
| `SCG_enforce()` | `loop()` — **before** `os_runloop_once()` | Corrects `txChnl` and `datarate` if altered by `LinkAdrReq`, ADR, or DR backoff |

> **ABP note:** `EV_JOINED` and `EV_JOIN_TXCOMPLETE` do not fire in normal ABP operation. Call only `SCG_init()` in `setup()` and `SCG_enforce()` in `loop()`.

---

## Special case: characterization / variable-DR sketches

If your sketch intentionally changes the data rate per transmission (e.g. a network characterization experiment), using `SCG_enforce()` in `loop()` will override your DR. Use channel-only enforcement instead:

```cpp
void loop() {
    // Channel-only: DR is under test control, not enforced here
    if (LMIC.txChnl != 0) {
        LMIC.txChnl            = 0;
        LMIC.channelShuffleMap = 0x0001;
    }
    os_runloop_once();
}
```

Call `SCG_init()` in `setup()` as usual to set the initial state, then let your parameter-update function override the DR immediately after.

---

## Known limitation

`LinkAdrReq` and `CFList` MAC commands are processed **inside** `os_runloop_once()`. `SCG_enforce()` and `SCG_on_joined()` correct the resulting state on the **next tick or event**, not the same tick. No wrong-channel transmission occurs because LoRaWAN TX is always scheduled after the corrective call has already run — but there is a one-tick window where the internal state is incorrect.

Eliminating this window entirely requires a library-side patch (see the Technical Manual for the formal `#define LMIC_SINGLE_CHANNEL_GATEWAY` proposal).

---

## Threats addressed

| Threat | MCCI mechanism | Addressed by |
|--------|---------------|-------------|
| Channel rotation | `LMIC_findNextChannel()` + `channelShuffleMap` | `SCG_init()`, `SCG_enforce()` |
| ADR channel renegotiation | `LMICeulike_mapChannels()` via `LinkAdrReq` | `SCG_init()` (ADR off), `SCG_enforce()` |
| CFList channel injection | `LMICeulike_processJoinAcceptCFList()` | `SCG_on_joined()` |
| Join channel rotation | `LMICeulike_nextJoinState()` | `SCG_on_join_txcomplete()` |
| DR reset by join | `nextJoinState()` DR backoff | `SCG_on_joined()`, `SCG_enforce()` |
| RX2 renegotiation | `RXParamSetupReq` MAC command | `SCG_init()`, `SCG_on_joined()` |
| DL channel redirect | `DlChannelReq` MAC command | Sketch-level `channelDlFreq` zeroing |
| Link check re-join | `LINK_CHECK_DEAD` → re-join | `SCG_init()`, `SCG_on_joined()` |
