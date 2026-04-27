# EUI Generator

A command-line tool to generate locally-administered EUI-64 and EUI-48 addresses for devices without hardware-assigned identifiers (e.g. LoRaWAN/TTN prototype devices).

## Background

EUI (Extended Unique Identifier) addresses are normally assigned by hardware manufacturers via an IEEE-registered OUI prefix. When building prototype or software-defined devices, no such prefix is available.

The solution defined by IEEE 802 §8.2 is to generate a random address and flag it as **locally administered** — this guarantees it will never collide with any hardware-assigned address, and with 62 random bits the probability of two generated addresses colliding is negligible (~1 in 4.6 × 10¹⁸).

## Algorithm

1. Generate 8 random bytes via `os.urandom` (cryptographically secure, backed by `/dev/urandom`)
2. **Set bit 1** of byte 0 → U/L flag = 1 (locally administered)
3. **Clear bit 0** of byte 0 → I/G flag = 0 (individual / unicast)

```
byte[0] = (byte[0] | 0x02) & 0xFE
```

For EUI-48, the same logic applies over 6 bytes instead of 8.

## Requirements

Python 3.6+ — no third-party dependencies.

## Usage

```bash
# Generate one EUI-64 (default)
python3 eui_generator.py
# FA-B0-24-5C-33-7C-AF-98

# Generate one EUI-48
python3 eui_generator.py --eui48
# 66:A6:3A:6C:69:76

# Use colons instead of dashes
python3 eui_generator.py --colon
# D2:33:6B:72:69:1D:68:AB

# Generate multiple addresses at once
python3 eui_generator.py -n 5
# FA-B0-24-5C-33-7C-AF-98
# D2-05-28-62-24-3A-B1-76
# CA-9C-53-E4-D0-BE-D8-C8
# 52-7C-3C-3E-F4-22-FE-98
# 02-6B-48-AE-EB-12-33-A9

# Combine flags
python3 eui_generator.py --eui48 --colon -n 3
```

## Options

| Option | Description |
|--------|-------------|
| `--eui48` | Generate a 48-bit EUI-48 address (default is 64-bit EUI-64) |
| `--colon` | Use `:` as byte separator instead of `-` |
| `-n N`, `--count N` | Number of addresses to generate (default: 1) |

## References

- [IEEE 802 §8.2 — Local and Administered Addresses](https://standards.ieee.org/wp-content/uploads/import/documents/tutorials/eui.pdf)
- [things-nyc/random-eui64](https://github.com/things-nyc/random-eui64) — original inspiration
- [TTN forum — DevEUI for non-hardware assigned values](https://www.thethingsnetwork.org/forum/t/deveui-for-non-hardware-assigned-values/2093)
