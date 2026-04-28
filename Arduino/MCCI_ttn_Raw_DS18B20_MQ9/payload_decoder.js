// TTN payload decoder for MCCI_ttn_Raw_DS18B20_MQ9
// Paste into TTN Console → Application → Payload Formatters → Uplink
//
// Byte layout (6 bytes total):
//   [0-1]  int16_t  temperature × 100
//   [2-3]  uint16_t MQ9 raw ADC (0–1023)
//   [4-5]  uint16_t MQ9 Rs/R0 × 1000

function decodeUplink(input) {
    var b = input.bytes;
    if (b.length < 6) return { errors: ["payload too short"] };

    var temp_raw = (b[0] << 8) | b[1];
    if (temp_raw & 0x8000) temp_raw -= 0x10000;   // sign-extend int16
    var adc   = (b[2] << 8) | b[3];
    var ratio = (b[4] << 8) | b[5];

    return {
        data: {
            temperature_C: temp_raw / 100.0,
            mq9_adc:       adc,
            mq9_rs_r0:     ratio / 1000.0
        }
    };
}
