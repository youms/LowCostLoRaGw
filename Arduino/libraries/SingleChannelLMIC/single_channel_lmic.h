/*******************************************************************************
 * single_channel_lmic.h
 *
 * User-space single-channel enforcement layer for the MCCI LoRaWAN LMIC library.
 * Library-update-safe: zero modifications to MCCI source files required.
 *
 * USAGE — include this header in your sketch and call the four functions:
 *
 *   setup()            → SCG_init()               after LMIC_reset() + channel setup
 *   EV_JOINED          → SCG_on_joined()
 *   EV_JOIN_TXCOMPLETE → SCG_on_join_txcomplete()
 *   loop()             → SCG_enforce()             before os_runloop_once()
 *
 * CONFIGURATION — override any define before #include <single_channel_lmic.h>:
 *
 *   #define SCG_CHANNEL       0          // channel index (0 = first channel)
 *   #define SCG_DATARATE      DR_SF7     // spreading factor
 *   #define SCG_TXPOWER       14         // dBm
 *   #define SCG_NUM_CHANNELS  9          // EU868/EU433: 9 ; US915: 72
 *   #define SCG_RX2_DR        DR_SF12    // RX2 window data rate
 *   #define SCG_RX2_FREQ      869525000  // RX2 window frequency (Hz)
 *
 * KNOWN LIMITATION:
 *   LinkAdrReq and CFList MAC commands are processed inside os_runloop_once().
 *   SCG_enforce() and SCG_on_joined() correct state on the following tick or
 *   event. No actual wrong-channel transmission occurs because TX is always
 *   scheduled after the corrective call has run.
 *
 * Project 3 — MCCI LoRaWAN Library Modifications Catalog
 * University of Yaoundé 1, Department of Computer Sciences
 * Academic Year 2025-2026
 ******************************************************************************/

#ifndef SINGLE_CHANNEL_LMIC_H
#define SINGLE_CHANNEL_LMIC_H

// ─────────────────────────────────────────────────────────────────────────────
// CONFIGURATION DEFAULTS
// ─────────────────────────────────────────────────────────────────────────────

#ifndef SCG_CHANNEL
  #define SCG_CHANNEL      0            // Channel 0:
                                        //   EU868 → 868.1 MHz
                                        //   EU433 → 433.175 MHz
#endif

#ifndef SCG_DATARATE
  #define SCG_DATARATE     DR_SF7       // SF7 = highest data rate, shortest ToA
#endif

#ifndef SCG_TXPOWER
  #define SCG_TXPOWER      14           // 14 dBm — EU868 regulatory maximum
#endif

#ifndef SCG_NUM_CHANNELS
  #define SCG_NUM_CHANNELS 9            // EU868 / EU433: channels 0-8
                                        // US915: set to 72 (channels 0-71)
#endif

#ifndef SCG_RX2_DR
  #define SCG_RX2_DR       DR_SF12     // TTN EU868 RX2 default
#endif

#ifndef SCG_RX2_FREQ
  #define SCG_RX2_FREQ     869525000   // TTN EU868 RX2 default: 869.525 MHz
                                       // TTN EU433 RX2 default: 434.665 MHz → 434665000
#endif

// ─────────────────────────────────────────────────────────────────────────────
// SCG_init()
//
// Call once in setup(), AFTER LMIC_reset() and AFTER all LMIC_setupChannel()
// calls for your band. LMIC_setupChannel() enables channels in channelMap;
// this function must run after to selectively disable all but SCG_CHANNEL.
//
// Threats addressed:
//   [§2.2] Channel map       — disables channels 1 to SCG_NUM_CHANNELS-1
//   [§2.1] Shuffle map       — locks channelShuffleMap to (1 << SCG_CHANNEL)
//                              so LMIC_findNextChannel() can only return 0
//   [§3.6] ADR mode          — disables Adaptive Data Rate (multi-channel only)
//   [§3.1] Data rate         — asserts SCG_DATARATE and SCG_TXPOWER
//   [§5.1] RX2 window        — sets RX2 frequency and data rate
// ─────────────────────────────────────────────────────────────────────────────
static inline void SCG_init(void) {
    // Disable all channels except the target channel.
    // Must run after LMIC_setupChannel() calls, which enable channels in channelMap.
    for (int i = 0; i < SCG_NUM_CHANNELS; i++) {
        if (i != SCG_CHANNEL)
            LMIC_disableChannel(i);
    }

    // Lock the shuffle map so LMIC_findNextChannel() cannot select other channels.
    // This guards against LMICeulike_initJoinLoop() resetting the map to 0x0007
    // (the 3 default EU868 channels) at the start of OTAA.
    LMIC.channelShuffleMap = (1 << SCG_CHANNEL);

    // Disable ADR — the ADR algorithm assumes multiple channels and will
    // attempt to adjust DR and channel mask based on network feedback.
    LMIC_setAdrMode(0);

    // Assert desired data rate and TX power.
    // Also called in SCG_on_joined() since join can reset these.
    LMIC_setDrTxpow(SCG_DATARATE, SCG_TXPOWER);

    // Set RX2 window parameters explicitly.
    // These are overwritten by Join Accept and RXParamSetupReq MAC commands;
    // SCG_on_joined() re-asserts them after join if needed.
    LMIC.dn2Dr   = SCG_RX2_DR;
    LMIC.dn2Freq = SCG_RX2_FREQ;

    // Disable link check validation.
    // For ABP this must be done here (no EV_JOINED fires after setSession).
    // For OTAA this is called again in SCG_on_joined() after join resets it.
    LMIC_setLinkCheckMode(0);

    Serial.print(F("[SCG] init — ch="));
    Serial.print(SCG_CHANNEL);
    Serial.print(F(" SF"));
    Serial.print(12 - SCG_DATARATE);
    Serial.print(F(" "));
    Serial.print(SCG_TXPOWER);
    Serial.println(F("dBm"));
}

// ─────────────────────────────────────────────────────────────────────────────
// SCG_on_joined()
//
// Call inside the EV_JOINED case of onEvent(), before break.
//
// Threats addressed:
//   [§2.8] CFList injection  — Join Accept can carry up to 5 new channels via
//                              CFList; LMICeulike_processJoinAcceptCFList()
//                              adds them to channelMap before EV_JOINED fires.
//                              This re-disables them.
//   [§3.4] DR reset          — join process can reset datarate; re-asserts it.
//   [§3.1] Link check        — auto-enabled during join; disabled here.
//                              LINK_CHECK_DEAD triggers re-join behaviour which
//                              would rotate join channels again.
// ─────────────────────────────────────────────────────────────────────────────
static inline void SCG_on_joined(void) {
    // Re-disable channels that the CFList in the Join Accept may have added.
    // TTN v3 sends a CFList for EU868, adding channels 3-7.
    for (int i = 0; i < SCG_NUM_CHANNELS; i++) {
        if (i != SCG_CHANNEL)
            LMIC_disableChannel(i);
    }

    // Re-lock shuffle map after join resets it.
    LMIC.channelShuffleMap = (1 << SCG_CHANNEL);

    // Re-assert DR and TX power — join may have altered these.
    LMIC_setDrTxpow(SCG_DATARATE, SCG_TXPOWER);

    // Disable link check validation.
    // Auto-enabled during join but not supported by TTN at this time.
    // If left enabled, LINK_CHECK_DEAD fires after ~32 unacknowledged uplinks,
    // triggering a re-join that rotates channels again.
    LMIC_setLinkCheckMode(0);

    Serial.println(F("[SCG] post-join enforcement applied"));
}

// ─────────────────────────────────────────────────────────────────────────────
// SCG_on_join_txcomplete()
//
// Call inside the EV_JOIN_TXCOMPLETE case of onEvent(), before break.
// EV_JOIN_TXCOMPLETE fires when a join request TX completes with no Join Accept
// received (i.e. each failed join attempt).
//
// Threats addressed:
//   [§6.2] Join channel rotation — LMICeulike_nextJoinState() calls
//                                  LMIC_findNextChannel() to advance to the
//                                  next join channel BEFORE this event fires.
//                                  txChnl may therefore be 1 or 2 at this point.
//                                  This corrects it for the next attempt.
// ─────────────────────────────────────────────────────────────────────────────
static inline void SCG_on_join_txcomplete(void) {
    // Force channel back to SCG_CHANNEL for the next join attempt.
    // nextJoinState() has already selected the next channel internally;
    // we override it here before the scheduler queues the retry.
    LMIC.txChnl            = SCG_CHANNEL;
    LMIC.channelShuffleMap = (1 << SCG_CHANNEL);

    Serial.print(F("[SCG] join retry forced to ch="));
    Serial.println(SCG_CHANNEL);
}

// ─────────────────────────────────────────────────────────────────────────────
// SCG_enforce()
//
// Call in loop(), BEFORE os_runloop_once().
//
// Threats addressed:
//   [§2.5] LinkAdrReq aftermath — the network can send a LinkAdrReq MAC command
//                                 embedded in a downlink, which calls
//                                 LMICeulike_mapChannels() inside os_runloop_once()
//                                 and may re-enable channels. This corrects txChnl
//                                 and channelShuffleMap on the next tick.
//   [§3.2] ADR DR change        — applyAdrRequests() can reset datarate even
//                                 with LMIC_setAdrMode(0). Caught here.
//   [§3.4] Join DR backoff      — nextJoinState() drops DR after each shuffle
//                                 cycle. Caught here on the following tick.
//
// Note: this function deliberately uses direct struct access (LMIC.txChnl,
// LMIC.datarate) rather than the public API. The public LMIC_setDrTxpow()
// sets OP_NEXTCHNL which would trigger another channel selection cycle;
// direct assignment avoids that overhead for the continuous enforcement case.
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
