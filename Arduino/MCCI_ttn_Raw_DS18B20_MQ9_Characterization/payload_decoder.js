// TTN payload decoder for MCCI_ttn_Raw_DS18B20_MQ9_Characterization
//
// Byte layout (first 6 bytes carry data; remainder is 0x00 padding):
//   [0-1]  int16_t  temperature × 100
//   [2-3]  uint16_t MQ9 raw ADC (0–1023)
//   [4-5]  uint16_t MQ9 Rs/R0 × 1000

function decodeUplink(input) {
    var b = input.bytes;
    if (b.length < 6) return { errors: ["payload too short (need 6 bytes)"] };

    var temp_raw = (b[0] << 8) | b[1];
    if (temp_raw & 0x8000) temp_raw -= 0x10000;   // sign-extend int16

    return {
        data: {
            temperature_C: temp_raw / 100.0,
            mq9_adc:       (b[2] << 8) | b[3],
            mq9_rs_r0:     ((b[4] << 8) | b[5]) / 1000.0
        }
    };
}
