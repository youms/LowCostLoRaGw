/*******************************************************************************
 * single_channel_lmic.h
 *
 * User-space single-channel enforcement layer for the MCCI LoRaWAN LMIC library.
 * Library-update-safe: zero modifications to MCCI source files.
 *
 * Include this header in any MCCI LMIC sketch targeting a single-channel gateway,
 * then call the four SCG_* functions at the points listed below:
 *
 *   setup()            → SCG_init()               after LMIC_reset() + channel setup
 *   EV_JOINED          → SCG_on_joined()
 *   EV_JOIN_TXCOMPLETE → SCG_on_join_txcomplete()
 *   loop()             → SCG_enforce()             before os_runloop_once()
 *
 * Project 3 — MCCI LoRaWAN Library Modifications Catalog
 * University of Yaoundé 1, Department of Computer Sciences
 ******************************************************************************/

#ifndef SINGLE_CHANNEL_LMIC_H
#define SINGLE_CHANNEL_LMIC_H

// ─────────────────────────────────────────────────────────────────────────────
// CONFIGURATION
// Override any of these defines before #include "single_channel_lmic.h"
// ─────────────────────────────────────────────────────────────────────────────

#ifndef SCG_CHANNEL
  #define SCG_CHANNEL      0            // Channel index to lock to
                                        //   EU868: 0 = 868.1 MHz
                                        //   EU433: 0 = 433.175 MHz
#endif

#ifndef SCG_DATARATE
  #define SCG_DATARATE     DR_SF7       // Spreading factor / data rate
#endif

#ifndef SCG_TXPOWER
  #define SCG_TXPOWER      14           // TX power in dBm
#endif

#ifndef SCG_NUM_CHANNELS
  #define SCG_NUM_CHANNELS 9            // Total channels in the plan
                                        //   EU868: 9  (0-8)
                                        //   EU433: 9  (0-8)
                                        //   US915: 72 (0-71)
#endif

#ifndef SCG_RX2_DR
  #define SCG_RX2_DR       DR_SF12      // RX2 window data rate
#endif

#ifndef SCG_RX2_FREQ
  #define SCG_RX2_FREQ     869525000    // RX2 window frequency (Hz)
                                        //   TTN EU868 default: 869.525 MHz
                                        //   TTN EU433 default: 434.665 MHz
#endif

// ─────────────────────────────────────────────────────────────────────────────
// SCG_init()
//
// Call once in setup(), after LMIC_reset() and after all LMIC_setupChannel()
// calls for your band.
//
// Threats addressed:
//   - Channel map: disables all channels except SCG_CHANNEL
//   - Shuffle map: locks channelShuffleMap so LMIC_findNextChannel() can only
//                  return SCG_CHANNEL
//   - ADR:         disables Adaptive Data Rate (ADR expects multi-channel)
//   - Data rate:   asserts SCG_DATARATE and SCG_TXPOWER
//   - RX2:         sets RX2 window frequency and data rate
// ─────────────────────────────────────────────────────────────────────────────
static inline void SCG_init(void) {
    for (int i = 0; i < SCG_NUM_CHANNELS; i++) {
        if (i != SCG_CHANNEL)
            LMIC_disableChannel(i);
    }

    LMIC.channelShuffleMap = (1 << SCG_CHANNEL);

    LMIC_setAdrMode(0);
    LMIC_setDrTxpow(SCG_DATARATE, SCG_TXPOWER);

    LMIC.dn2Dr   = SCG_RX2_DR;
    LMIC.dn2Freq = SCG_RX2_FREQ;

    Serial.print(F("[SCG] Locked: ch="));
    Serial.print(SCG_CHANNEL);
    Serial.print(F(" DR=SF"));
    Serial.print(12 - SCG_DATARATE);
    Serial.print(F(" PWR="));
    Serial.print(SCG_TXPOWER);
    Serial.println(F("dBm"));
}

// ─────────────────────────────────────────────────────────────────────────────
// SCG_on_joined()
//
// Call inside the EV_JOINED handler, before break.
//
// Threats addressed:
//   - CFList injection: the Join Accept can carry a CFList that re-enables up
//                       to 5 additional channels; this re-disables them
//   - DR reset:         the join process can reset the data rate; re-asserts it
//   - Link check:       auto-enabled during join; disabled here (not supported
//                       by TTN and triggers LINK_CHECK_DEAD behaviour)
// ─────────────────────────────────────────────────────────────────────────────
static inline void SCG_on_joined(void) {
    for (int i = 0; i < SCG_NUM_CHANNELS; i++) {
        if (i != SCG_CHANNEL)
            LMIC_disableChannel(i);
    }

    LMIC.channelShuffleMap = (1 << SCG_CHANNEL);
    LMIC_setDrTxpow(SCG_DATARATE, SCG_TXPOWER);
    LMIC_setLinkCheckMode(0);

    Serial.println(F("[SCG] Post-join enforcement applied"));
}

// ─────────────────────────────────────────────────────────────────────────────
// SCG_on_join_txcomplete()
//
// Call inside the EV_JOIN_TXCOMPLETE handler, before break.
//
// Threats addressed:
//   - Join channel rotation: LMICeulike_nextJoinState() selects the next join
//                            channel internally before this event fires.
//                            This corrects txChnl and the shuffle map after
//                            each failed join attempt.
// ─────────────────────────────────────────────────────────────────────────────
static inline void SCG_on_join_txcomplete(void) {
    LMIC.txChnl            = SCG_CHANNEL;
    LMIC.channelShuffleMap = (1 << SCG_CHANNEL);

    Serial.print(F("[SCG] Join retry -> ch="));
    Serial.println(SCG_CHANNEL);
}

// ─────────────────────────────────────────────────────────────────────────────
// SCG_enforce()
//
// Call in loop(), before os_runloop_once().
//
// Threats addressed:
//   - Runtime channel drift: catches any txChnl change made inside the library
//                            state machine (including aftermath of LinkAdrReq
//                            MAC commands processed on the previous tick)
//   - Runtime DR drift:      catches datarate changes from ADR commands or
//                            join retry backoff that slipped through
//
// Known limitation: LinkAdrReq and CFList are processed inside os_runloop_once()
// on the tick they arrive. SCG_enforce() corrects the state on the NEXT tick.
// The channel map may be temporarily incorrect for one tick, but no actual
// wrong-channel transmission occurs because the TX is scheduled after the
// corrective enforce() call has already run.
// ─────────────────────────────────────────────────────────────────────────────
static inline void SCG_enforce(void) {
    if (LMIC.txChnl != SCG_CHANNEL) {
        LMIC.txChnl            = SCG_CHANNEL;
        LMIC.channelShuffleMap = (1 << SCG_CHANNEL);
    }

    if (LMIC.datarate != SCG_DATARATE) {
        LMIC.datarate = SCG_DATARATE;
    }
}

#endif // SINGLE_CHANNEL_LMIC_H
