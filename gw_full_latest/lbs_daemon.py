#!/usr/bin/env python2
#-------------------------------------------------------------------------------
# lbs_daemon.py - LoRa Basics Station uplink daemon (WebSocket side)
#
# Runs as a persistent background process alongside the gateway.
# Maintains a single authenticated WebSocket connection to the TTN LNS,
# performs the LBS version/router_config handshake at startup, then loops:
#   - reads updf messages written by CloudLBS.py from the queue file
#   - forwards them to TTN over the open WebSocket
#   - reads incoming messages from TTN (downlinks, router_config refreshes)
#   - reconnects automatically if the WebSocket drops
#
# Start manually (from /home/pi/lora_gateway/):
#   python lbs_daemon.py
# Or pass the gateway ID explicitly:
#   python lbs_daemon.py 0000B827EBD1B236
#
# Must be run from the gateway directory so that key_LBS.py is importable.
#-------------------------------------------------------------------------------

import base64
import binascii
import json
import os
import ssl
import sys
import time

import websocket   # python-websocket 0.53.0 (websocket-client)

sys.dont_write_bytecode = True
import key_LBS

# How long (seconds) to wait between reconnection attempts after a failure
RECONNECT_DELAY = 30

# Total seconds allowed for the version + router_config handshake at startup
HANDSHAKE_TIMEOUT = 15

# WebSocket recv timeout (seconds). Keep this short: every ms saved here
# directly reduces the total LBS->TTN->dnmsg round-trip latency and gives
# more time for the C++ gateway binary to read downlink/downlink.txt before
# the RX1 check window at t_reception + 900ms (DELAY_DNWFILE in the binary).
WS_RECV_TIMEOUT = 0.1

# Seconds between queue-file checks in the main loop.
QUEUE_POLL_INTERVAL = 0.1

# EU868 LBS DR index -> Semtech txpk datr string.
# Used when converting a dnmsg to txpk format for the gateway binary.
DR_DATR = {
    0: "SF12BW125",
    1: "SF11BW125",
    2: "SF10BW125",
    3: "SF9BW125",
    4: "SF8BW125",
    5: "SF7BW125",
    6: "SF7BW250",
}


#////////////////////////////////////////////////////////////
# CHANGE HERE THE VARIOUS PATHS FOR YOUR LOG FILES
#////////////////////////////////////////////////////////////
LOG_PATH = "/home/pi/Dropbox/LoRa-test/"
_gwid = "N/A"

try:
    with open("gateway_conf.json", "r") as f:
        conf = json.load(f)
        _gwid = conf["gateway_conf"]["gateway_ID"]
except:
    pass

_daemonlog_filename = LOG_PATH + "lbs_daemon_" + str(_gwid) + ".log"
# END
#////////////////////////////////////////////////////////////

def log(msg):
    from datetime import datetime
    timestamp = datetime.now().strftime("%Y-%m-%dT%H:%M:%S.%f")
    log_str = "[%s] lbs_daemon: %s" % (timestamp, msg)

    # Print to console
    print log_str
    sys.stdout.flush()

    # Also write to log file if the directory exists
    if os.path.exists(LOG_PATH):
        try:
            with open(_daemonlog_filename, "a") as f:
                f.write(log_str + "\n")
        except:
            pass


class LBSDaemon(object):

    def __init__(self, gwid):
        # Convert the raw gateway ID to the TTN V3 LBS URL format.
        # TTN V3 requires the "eui-" prefix in the /traffic/ path.
        # e.g. "0000B827EBD1B236" -> "eui-b827ebffffd1b236"
        self.eui = "eui-" + (gwid[4:10] + "FFFF" + gwid[10:]).lower()
        self.ws  = None

    # ------------------------------------------------------------------
    # Connection helpers
    # ------------------------------------------------------------------

    def _url(self):
        return "wss://{}:{}/traffic/{}".format(
            key_LBS.lorawan_server,
            key_LBS.lorawan_port,
            self.eui
        )

    def _connect(self):
        log("connecting to " + self._url())

        # ssl.CERT_REQUIRED verifies the TTN server certificate against the
        # system CA bundle. Set to ssl.CERT_NONE only for local testing.
        ws = websocket.WebSocket(sslopt={"cert_reqs": ssl.CERT_REQUIRED})

        # Authorization header must be a list of strings in websocket-client 0.53.0
        ws.connect(
            self._url(),
            header=["Authorization: Bearer " + key_LBS.api_key]
        )
        ws.settimeout(WS_RECV_TIMEOUT)
        self.ws = ws
        log("TCP/TLS connected, starting handshake")
        self._handshake()

    def _handshake(self):
        """
        LBS startup sequence (BasicStation protocol):
          1. Client sends   'version'       -> identifies itself to the LNS
          2. Server sends   'version'       -> LNS acknowledges
          3. Server sends   'router_config' -> channel plan and policies
        If the server does not push router_config spontaneously, we request it.
        The gateway is ready to send updf only after router_config is received.
        """

        # Step 1: send our version message
        version_msg = json.dumps({
            "msgtype":  "version",
            "station":  "lbs-gw",
            "firmware": "1.0.0",
            "package":  "",
            "model":    "raspberry-pi",
            "protocol": 2,          # BasicStation protocol version
            "features": ""
        })
        self.ws.send(version_msg)
        log("sent version")

        # Step 2 + 3: wait for the server's version and router_config
        got_version = False
        got_config  = False
        deadline    = time.time() + HANDSHAKE_TIMEOUT

        while time.time() < deadline:
            try:
                raw = self.ws.recv()
                if not raw:
                    continue
                msg   = json.loads(raw)
                mtype = msg.get("msgtype", "")

                if mtype == "version":
                    log("server version: station=%s firmware=%s" % (
                        msg.get("station", "?"), msg.get("firmware", "?")))
                    got_version = True

                elif mtype == "router_config":
                    log("got router_config (NetID count: %d)" % len(
                        msg.get("NetID") or []))
                    got_config = True

            except websocket.WebSocketTimeoutException:
                pass   # normal: no message yet, keep waiting

            if got_config:   # router_config alone is sufficient; version is optional
                break

        # If the server did not push router_config, request it explicitly.
        # Some LNS implementations wait for the client to ask.
        if not got_config:
            log("requesting router_config")
            self.ws.send(json.dumps({"msgtype": "router_config"}))

            deadline = time.time() + HANDSHAKE_TIMEOUT
            while time.time() < deadline:
                try:
                    raw = self.ws.recv()
                    if raw:
                        msg = json.loads(raw)
                        if msg.get("msgtype") == "router_config":
                            log("got router_config")
                            got_config = True
                            break
                except websocket.WebSocketTimeoutException:
                    pass

        if not got_config:
            raise Exception("handshake failed: router_config not received within %ds" % HANDSHAKE_TIMEOUT)

        log("handshake complete, ready to forward uplinks")

    # ------------------------------------------------------------------
    # Main loop helpers
    # ------------------------------------------------------------------

    def _process_queue(self):
        """
        Atomically claim the queue file by renaming it, then read and send
        every updf line it contains.

        os.rename() is atomic on Linux: CloudLBS.py always appends to
        key_LBS.queue_file, so renaming it away gives us an exclusive snapshot
        with no risk of losing a line that arrives mid-read.
        """
        proc_file = key_LBS.queue_file + ".proc"

        try:
            os.rename(key_LBS.queue_file, proc_file)
        except OSError:
            return   # queue file does not exist yet, nothing to do

        try:
            with open(proc_file, 'r') as f:
                lines = f.readlines()
        except Exception as ex:
            log("queue read error: %s" % ex)
            return
        finally:
            try:
                os.remove(proc_file)
            except OSError:
                pass

        for line in lines:
            line = line.strip()
            if not line:
                continue
            try:
                log("updf json: " + line)
                self.ws.send(line)
                # Parse just enough to produce a useful log line
                updf = json.loads(line)
                log("sent updf | DevAddr=0x%08X DR=DR%s Freq=%dHz" % (
                    updf.get("DevAddr", 0) & 0xFFFFFFFF,
                    updf.get("DR", "?"),
                    updf.get("Freq", 0)))
            except Exception as ex:
                log("send failed: %s" % ex)
                raise   # propagate to the outer loop to trigger reconnection

    def _poll_recv(self):
        """
        Non-blocking read of one incoming WebSocket message.
        Handles:
          - dnmsg       : downlink request from TTN
          - router_config: channel plan refresh (logged, no action needed)
          - timeout     : no message available (normal, ignored)
        """
        try:
            raw = self.ws.recv()
            if not raw:
                return
            msg   = json.loads(raw)
            mtype = msg.get("msgtype", "")

            if mtype == "timesync":
                # LNS measures round-trip latency and aligns its clock to the
                # gateway. Echo txtime back; gpstime=0 means no GPS reference.
                resp = json.dumps({
                    "msgtype": "timesync",
                    "txtime":  msg.get("txtime", 0),
                    "gpstime": 0,
                    "xtime":   int(time.time() * 1e6)  # microseconds, best we have
                })
                self.ws.send(resp)
                # log("timesync response sent")
                pass

            elif mtype == "dnmsg":
                log("downlink received: diid=%s RX1Freq=%s RX1DR=%s" % (
                    msg.get("diid", "?"),
                    msg.get("RX1Freq", "?"),
                    msg.get("RX1DR", "?")))
                self._write_downlink(msg)
                self._send_txconf(msg)

            elif mtype == "router_config":
                log("router_config refreshed by server")

            elif mtype:
                log("recv msgtype=%s" % mtype)

        except websocket.WebSocketTimeoutException:
            pass   # expected when no downlink is pending

        except Exception as ex:
            log("recv error: %s" % ex)
            raise   # propagate to outer loop to trigger reconnection

    def _send_txconf(self, dnmsg):
        # Report the scheduled TX time so TTN marks the downlink as transmitted
        # and waits for the device MAC response, rather than immediately
        # re-queuing the command for the next uplink.
        try:
            txconf = json.dumps({
                "msgtype": "txconf",
                "diid":    dnmsg.get("diid", 0),
                "rctx":    dnmsg.get("rctx", 0),
                "xtime":   dnmsg.get("xtime", 0),
                "txtime":  time.time()
            })
            self.ws.send(txconf)
            log("sent txconf diid=%s xtime=%s" % (
                dnmsg.get("diid", "?"), dnmsg.get("xtime", "?")))
        except Exception as ex:
            log("failed to send txconf: %s" % ex)

    def _write_downlink(self, dnmsg):
        """
        Convert a LBS dnmsg to Semtech txpk format and write it to
        downlink/downlink.txt so the gateway binary (SX12XX_lora_gateway)
        can read it and schedule the actual transmission on the SX127X radio.

        Timing: the binary checks the file at t_reception + 900ms (DELAY_DNWFILE)
        for RX1 and at t_reception + 1900ms for RX2.  With WS_RECV_TIMEOUT and
        QUEUE_POLL_INTERVAL both at 0.1s the total path latency is ~500ms,
        leaving ~400ms of margin to hit the RX1 check window.

        The xtime field from the dnmsg is the same SX1276 counter value that was
        sent as updf.upInfo.xtime, offset by RxDelay*1e6 us.  The binary calls
        this tmst and uses it directly for transmission scheduling.
        """
        try:
            pdu_hex  = dnmsg.get("pdu", "")
            pdu_bin  = binascii.unhexlify(pdu_hex)
            data_b64 = base64.b64encode(pdu_bin)
            if isinstance(data_b64, bytes):
                data_b64 = data_b64.decode('ascii')

            rx1_freq = dnmsg.get("RX1Freq", 868100000)
            rx1_dr   = dnmsg.get("RX1DR", 5)
            datr     = DR_DATR.get(rx1_dr, "SF7BW125")

            # Extract RX2 parameters from TTN dnmsg for the second receive window
            rx2_freq = dnmsg.get("RX2Freq", 869525000)
            rx2_dr   = dnmsg.get("RX2DR", 0)
            rx2_datr = DR_DATR.get(rx2_dr, "SF12BW125")

            # TTN echoes the uplink xtime back unchanged in dnmsg.xtime.
            # The gateway must add RxDelay*1e6 to get the RX1 TX timestamp.
            # Use modulo 2^32 to match the SX1276's 32-bit counter wraparound.
            rx_delay = dnmsg.get("RxDelay", 1)
            tx_tmst  = (dnmsg.get("xtime", 0) + rx_delay * 1000000) % (2**32)

            txpk = {
                "txpk": {
                    "imme": False,
                    "rfch": 0,
                    "powe": 14,
                    "ant":  0,
                    "brd":  0,
                    "tmst": tx_tmst,
                    "freq": round(rx1_freq / 1e6, 3),
                    "modu": "LORA",
                    "datr": datr,
                    "codr": "4/5",
                    "ipol": True,
                    "size": len(pdu_bin),
                    "data": data_b64,
                    "rx2freq": round(rx2_freq / 1e6, 3),
                    "rx2datr": rx2_datr
                }
            }

            with open("downlink/downlink.txt", 'w') as f:
                f.write(json.dumps(txpk) + '\n')
            rx_delay = dnmsg.get("RxDelay", 1)
            log("downlink txpk written | RX1: freq=%.3fMHz datr=%s | RX2: freq=%.3fMHz datr=%s | RxDelay=%ds tmst=%u size=%d" % (
                rx1_freq / 1e6, datr, rx2_freq / 1e6, rx2_datr, rx_delay, tx_tmst, len(pdu_bin)))

        except Exception as ex:
            log("failed to write downlink txpk: %s" % ex)

        # Also save the raw dnmsg for inspection / debugging
        try:
            with open(key_LBS.downlink_file, 'w') as f:
                f.write(json.dumps(dnmsg) + '\n')
        except Exception as ex:
            log("failed to write raw dnmsg: %s" % ex)

    # ------------------------------------------------------------------
    # Entry point
    # ------------------------------------------------------------------

    def run(self):
        while True:
            try:
                self._connect()

                # Inner loop: forward uplinks and receive downlinks
                while True:
                    self._process_queue()
                    self._poll_recv()
                    time.sleep(QUEUE_POLL_INTERVAL)

            except KeyboardInterrupt:
                log("interrupted, stopping")
                if self.ws:
                    try:
                        self.ws.close()
                    except Exception:
                        pass
                break

            except Exception as ex:
                log("connection lost (%s), retrying in %ds" % (ex, RECONNECT_DELAY))
                if self.ws:
                    try:
                        self.ws.close()
                    except Exception:
                        pass
                    self.ws = None
                time.sleep(RECONNECT_DELAY)


def main(gwid):
    log("starting | EUI=eui-%s server=%s:%d" % (
        (gwid[4:10] + "FFFF" + gwid[10:]).lower(),
        key_LBS.lorawan_server,
        key_LBS.lorawan_port))
    daemon = LBSDaemon(gwid)
    daemon.run()


if __name__ == "__main__":
    if len(sys.argv) > 1:
        # Gateway ID passed as command-line argument
        main(sys.argv[1])
    else:
        # Read the gateway ID from gateway_conf.json (same as downlink_lorawan.py)
        gwconf_path = "/home/pi/lora_gateway/gateway_conf.json"
        try:
            with open(gwconf_path, 'r') as f:
                conf = json.load(f)
            gwid = conf["gateway_conf"]["gateway_ID"]
        except Exception as ex:
            print "lbs_daemon: cannot read gateway ID from %s: %s" % (gwconf_path, ex)
            sys.exit(1)
        main(gwid)
