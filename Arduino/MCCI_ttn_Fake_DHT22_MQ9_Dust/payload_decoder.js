// TTN payload decoder for MCCI_ttn_Fake_DHT22_MQ9_Dust (and MCCI_ttn_Raw_DHT22_MQ9_Dust)
// Paste into TTN Console → Application → Payload Formatters → Uplink
//
// Byte layout (10 bytes total):
//   [0-1]  int16_t  temperature × 100          → °C
//   [2-3]  uint16_t humidity × 100             → %
//   [4-5]  uint16_t MQ9 raw ADC (0–1023)       → raw
//   [6-7]  uint16_t MQ9 Rs/R0 × 1000           → CO ppm via power law
//   [8-9]  uint16_t dust raw ADC (0–1023)       → µg/m³ via linear formula
//
// CO concentration (MQ9 datasheet power law for CO, clean-air R0 baseline):
//   CO_ppm = 100 × (Rs/R0)^(−1.912)
//   Derived from two characteristic points: (Rs/R0=1.0 → 100 ppm), (Rs/R0=0.3 → 1000 ppm)
//
// Dust density (Sharp GP2Y1010AU0F linear model):
//   V       = adc × (5.0 / 1023)
//   dust    = max(0,  170 × V − 100)            [µg/m³]
//
// Link quality (from input.settings — available in formatter):
//   spreading_factor, bandwidth_khz, frequency_mhz
//
// NOTE — RSSI and SNR are gateway metadata, NOT available in the formatter function.
//   Read them from: TTN Console live data, MQTT topic v3/.../up (rx_metadata[].rssi / .snr),
//   or any webhook/integration that receives the full uplink JSON.

function decodeUplink(input) {
    var b = input.bytes;
    if (b.length < 10) return { errors: ["payload too short, expected 10 bytes"] };

    // Temperature (signed)
    var temp_raw = (b[0] << 8) | b[1];
    if (temp_raw & 0x8000) temp_raw -= 0x10000;
    var temperature_C = temp_raw / 100.0;

    // Humidity
    var humidity_pct = ((b[2] << 8) | b[3]) / 100.0;

    // MQ9: raw ADC and Rs/R0 ratio
    var mq9_adc = (b[4] << 8) | b[5];
    var rs_r0   = ((b[6] << 8) | b[7]) / 1000.0;

    // CO concentration from Rs/R0 power law (valid range ~0.1 – 10)
    var co_ppm = 0;
    if (rs_r0 > 0) {
        co_ppm = Math.round(100.0 * Math.pow(rs_r0, -1.912) * 10) / 10;
    }

    // Dust density
    var dust_adc  = (b[8] << 8) | b[9];
    var dust_v    = dust_adc * (5.0 / 1023.0);
    var dust_ugm3 = Math.round(Math.max(0.0, 170.0 * dust_v - 100.0) * 10) / 10;

    // Link quality — transmission settings provided by TTN to the formatter
    var sf      = null;
    var bw_khz  = null;
    var freq_mhz = null;
    if (input.settings) {
        var s = input.settings;
        if (s.data_rate && s.data_rate.lora) {
            sf     = s.data_rate.lora.spreading_factor;
            bw_khz = s.data_rate.lora.bandwidth / 1000;
        }
        if (s.frequency) {
            freq_mhz = Math.round(parseInt(s.frequency) / 1000) / 1000;  // Hz → MHz, 3 dp
        }
    }

    return {
        data: {
            // Sensor fields
            temperature_C:  temperature_C,
            humidity_pct:   humidity_pct,
            mq9_adc:        mq9_adc,
            mq9_rs_r0:      rs_r0,
            co_ppm:         co_ppm,
            dust_adc:       dust_adc,
            dust_ugm3:      dust_ugm3,
            // Link quality (from transmission settings)
            spreading_factor: sf,
            bandwidth_khz:    bw_khz,
            frequency_mhz:    freq_mhz
            // rssi and snr: not available here — read from rx_metadata in MQTT/webhook
        }
    };
}
