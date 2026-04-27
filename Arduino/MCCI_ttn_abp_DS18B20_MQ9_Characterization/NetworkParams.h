////////////////////////////////////////////////////////////////
// Network Parameters Module - LoRaWAN / MCCI LMIC version
// Replaces sf/bw/cr triplet with a single LMIC DataRate (dr)
////////////////////////////////////////////////////////////////

#ifndef NETWORK_PARAMS_H
#define NETWORK_PARAMS_H

// EU868 LoRaWAN max FRMPayload per DR (without FHDR):
//   DR0 SF12/BW125 : 51 bytes
//   DR1 SF11/BW125 : 51 bytes
//   DR2 SF10/BW125 : 51 bytes
//   DR3  SF9/BW125 : 115 bytes
//   DR4  SF8/BW125 : 222 bytes
//   DR5  SF7/BW125 : 222 bytes
//   DR6  SF7/BW250 : 222 bytes  (DR_SF7B - non-standard on 868.1 MHz but used here for characterization)

struct NetworkTestParams {
    uint8_t  dr;          // LMIC DataRate: DR_SF7 .. DR_SF12, DR_SF7B
    uint8_t  payloadSize; // Target payload size in bytes (must respect DR limit above)
    const char* name;     // Human-readable description
};

// 16 characterization configurations
// BW500 entries from the SX12XX version are remapped to DR_SF7B (SF7/BW250),
// the closest standard LoRaWAN DR. Payload sizes capped at DR limit where needed.
const NetworkTestParams testParams[] = {
    // MIN configurations (SF7)
    {DR_SF7,  20,  "MIN-SF7-BW125-T20"},
    {DR_SF7B, 20,  "MIN-SF7B-BW250-T20"},   // was SF7/BW500
    {DR_SF7,  50,  "MIN-SF7-BW125-T50"},
    {DR_SF7,  80,  "MIN-SF7-BW125-T80"},

    // MEAN configurations (SF9)
    {DR_SF9,  20,  "MEAN-SF9-BW125-T20"},
    {DR_SF7B, 20,  "MEAN-SF7B-BW250-T20"},  // was SF9/BW500 (no SF9/BW500 in LoRaWAN)
    {DR_SF9,  50,  "MEAN-SF9-BW125-T50"},
    {DR_SF9,  80,  "MEAN-SF9-BW125-T80"},

    // MAX configurations (SF12) — payload capped at 51 bytes (EU868 DR0 limit)
    {DR_SF12, 20,  "MAX-SF12-BW125-T20"},
    {DR_SF7B, 20,  "MAX-SF7B-BW250-T20"},   // was SF12/BW500
    {DR_SF12, 50,  "MAX-SF12-BW125-T50"},
    {DR_SF12, 51,  "MAX-SF12-BW125-T51"},   // was T80, capped at 51 (DR0 limit)

    // EXTRA configurations
    {DR_SF8,  30,  "EXTRA-SF8-BW125-T30"},
    {DR_SF10, 40,  "EXTRA-SF10-BW125-T40"},
    {DR_SF11, 51,  "EXTRA-SF11-BW125-T51"}, // was T60, capped at 51 (DR1 limit)
    {DR_SF7B, 100, "EXTRA-SF7B-BW250-T100"} // was SF7/BW500/T100
};

const uint8_t NUM_TEST_PARAMS = sizeof(testParams) / sizeof(testParams[0]);

void updateLoRaParams(const NetworkTestParams& params) {
    Serial.print(F("LoRa params -> "));
    Serial.println(params.name);
    // ADR must be disabled (LMIC_setAdrMode(0) in setup) or the network will override this
    LMIC_setDrTxpow(params.dr, 14);
}

#endif // NETWORK_PARAMS_H
