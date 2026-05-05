#-------------------------------------------------------------------------------
# CloudLBS.py - LoRa Basics Station uplink forwarder (queue side)
#
# Called per-packet by post_processing_gw.py via lorawan_encrypted_clouds.
# Parses the received LoRaWAN packet, builds a LBS updf JSON message,
# and appends it to the queue file for lbs_daemon.py to pick up and send.
#
# Usage (as cloud script):
#   python CloudLBS.py <ldata> <pdata> <rdata> <tdata> <gwid>
#
# Arguments follow the same convention as CloudTTN.py:
#   ldata  - base64-encoded full LoRaWAN PHYPayload
#   pdata  - "dst,ptype,src,seq,datalen,SNR,RSSI"
#   rdata  - "bw,cr,sf,freq_kHz"
#   tdata  - ISO 8601 timestamp (optionally suffixed with *tmst for SX1301)
#   gwid   - 16-char hex gateway ID (e.g. 0000B827EBD1B236)
#-------------------------------------------------------------------------------

import base64
import binascii
import calendar
import json
import os
import struct
import sys
import time
from dateutil import parser as dtparser

sys.dont_write_bytecode = True
import key_LBS

# EU868 data-rate table: (sf, bw_kHz) -> LBS DR index.
# Extend this dict for other regions (AS923, AU915, US915, ...) if needed.
DR_TABLE = {
    (12, 125): 0,
    (11, 125): 1,
    (10, 125): 2,
    ( 9, 125): 3,
    ( 8, 125): 4,
    ( 7, 125): 5,
    ( 7, 250): 6,
}


def parse_phy(ldata):
    """
    Decode a base64 LoRaWAN PHYPayload and split it into the individual fields
    that the LBS updf message requires.

    LoRaWAN uplink frame layout (bytes):
      [0]       MHDR
      [1..4]    DevAddr   (little-endian)
      [5]       FCtrl
      [6..7]    FCnt      (little-endian)
      [8..8+N]  FOpts     (N = FCtrl & 0x0F, may be 0)
      [8+N]     FPort     (absent when no application payload follows)
      [9+N..-4] FRMPayload
      [-4..]    MIC

    Returns (mhdr, dev_addr, fctrl, fcnt, fopts_hex, fport, frm_hex, mic)
    where dev_addr and mic are signed int32 as required by the LBS spec.
    """
    phy = base64.b64decode(ldata)

    mhdr     = struct.unpack('B',  phy[0:1])[0]
    dev_addr = struct.unpack('<i', phy[1:5])[0]   # signed int32 for LBS
    fctrl    = struct.unpack('B',  phy[5:6])[0]
    fcnt     = struct.unpack('<H', phy[6:8])[0]   # unsigned int16

    fopts_len = fctrl & 0x0F
    fopts_hex = binascii.hexlify(phy[8:8 + fopts_len]).upper()

    # Byte index where FPort would start (right after FOpts)
    idx = 8 + fopts_len

    # FPort is present only when there are bytes left beyond the 4-byte MIC
    if len(phy) - idx > 4:
        fport   = struct.unpack('B', phy[idx:idx + 1])[0]
        frm_hex = binascii.hexlify(phy[idx + 1:len(phy) - 4]).upper()
    else:
        fport   = -1   # LBS convention: -1 means FPort absent
        frm_hex = ''

    mic = struct.unpack('<i', phy[-4:])[0]         # signed int32 for LBS

    return mhdr, dev_addr, fctrl, fcnt, fopts_hex, fport, frm_hex, mic


def main(ldata, pdata, rdata, tdata, gwid):

    # --- Parse pdata ---
    # Format: dst,ptype,src,seq,datalen,SNR,RSSI
    arr  = map(int, pdata.split(','))
    dst  = arr[0]
    src  = arr[2]
    SNR  = arr[5]
    RSSI = arr[6]

    # Only LoRaWAN packets have dst==256; everything else is skipped
    if dst != 256:
        print "CloudLBS: not a LoRaWAN packet (dst=%d), skipping" % dst
        return

    src_str = "0x%0.8X" % src
    if key_LBS.source_list and src_str not in key_LBS.source_list:
        print "CloudLBS: %s not in source_list, skipping" % src_str
        return

    # --- Parse rdata ---
    # Format: bw,cr,sf,freq_kHz
    arr = map(int, rdata.split(','))
    rbw = arr[0]
    rsf = arr[2]
    rfq = arr[3]   # frequency in kHz (e.g. 868100 = 868.1 MHz)

    # Map (SF, BW) to a LBS DR index; fail loudly if the combination is unknown
    dr = DR_TABLE.get((rsf, rbw), -1)
    if dr == -1:
        print "CloudLBS: no DR mapping for SF%d BW%d, skipping" % (rsf, rbw)
        return

    # --- Parse tdata ---
    # post_processing_gw.py appends a timestamp counter after '*'
    # e.g. "2019-03-25T18:46:00.528+01:00*29641444"
    # This is xtime in LBS: the timer value at reception, used by the LNS
    # to schedule downlinks (RX1 opens exactly 1s after the uplink xtime).
    # On SX1301 this is a hardware concentrator counter; on SX1276 it is a
    # software timer -- either way it is what CloudTTN.py already sends as tmst.
    # When absent, xtime stays 0 and TTN will not attempt timed downlinks.
    xtime = 0
    if '*' in tdata:
        parts = tdata.split('*')
        tdata = parts[0]
        try:
            xtime = int(parts[1])
        except ValueError:
            pass

    dt = dtparser.parse(tdata)
    # calendar.timegm treats the struct as UTC; utctimetuple() converts tz-aware
    # datetimes to UTC first, so the result is a correct Unix timestamp
    rxtime = float(calendar.timegm(dt.utctimetuple()))
    if dt.microsecond:
        rxtime += dt.microsecond / 1e6

    # --- Parse PHYPayload ---
    try:
        mhdr, dev_addr, fctrl, fcnt, fopts_hex, fport, frm_hex, mic = parse_phy(ldata)
    except Exception as ex:
        print "CloudLBS: PHYPayload parse failed: %s" % ex
        return

    # --- Build the LBS updf message ---
    # Full spec: https://doc.sm.tc/station/tcproto.html#upstream-messages
    #
    # xtime MUST be non-zero: TTN's LBS front-end silently drops uplinks whose
    # xtime==0 (gateway status stays "connected" but no uplink reaches the NS).
    # xtime=0 does NOT suppress downlinks - it just breaks uplink delivery.
    # The SX1276 tmst counter is an adequate substitute for the SX1301 hardware
    # counter; TTN uses it only for RX1 scheduling, not for message validation.
    #
    # FOpts / FRMPayload: per LBS spec these fields must be ABSENT (not "")
    # when there is no content; include them only when non-empty.
    updf = {
        "msgtype":    "updf",
        "MHdr":       mhdr,
        "DevAddr":    dev_addr,       # signed int32
        "FCtrl":      fctrl,
        "FCnt":       fcnt,
        "FPort":      fport,          # -1 if absent
        "MIC":        mic,            # signed int32 (negative values are normal)
        "RefTime":    0.0,            # 0.0 = no GPS reference available
        "DR":         dr,
        "Freq":       rfq * 1000,     # kHz -> Hz (e.g. 868100 kHz -> 868100000 Hz)
        "upInfo": {
            "rssi":   float(RSSI),
            "snr":    float(SNR),
            "rxtime": rxtime,         # Unix timestamp (float)
            "xtime":  xtime,          # SX1276 tmst counter; must be non-zero for TTN
            "rctx":   0              # radio context, 0 for single-channel gateway
        }
    }
    # LBS spec: FOpts and FRMPayload must be absent when empty, not ""
    if fopts_hex:
        updf["FOpts"] = fopts_hex
    if frm_hex:
        updf["FRMPayload"] = frm_hex

    # --- Append to queue file ---
    # lbs_daemon.py polls this file and forwards each line to TTN over WebSocket.
    # Using append mode: atomic for small writes on Linux, safe for concurrent access.
    try:
        with open(key_LBS.queue_file, 'a') as f:
            f.write(json.dumps(updf) + '\n')
        print "CloudLBS: queued updf | DevAddr=%s DR=DR%d Freq=%.3fMHz RSSI=%d SNR=%d" % (
            src_str, dr, rfq / 1000.0, RSSI, SNR)
    except Exception as ex:
        print "CloudLBS: failed to write queue: %s" % ex


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5])
