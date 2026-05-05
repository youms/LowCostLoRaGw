#!/usr/bin/env python2
# Diagnostic script: shows the full TTN LBS handshake including error body.
# Run from the gateway directory: python lbs_debug.py 0000B827EBDCE0CD

import socket
import ssl
import sys

sys.dont_write_bytecode = True
import key_LBS

def raw_http_probe(eui):
    """
    Open a raw TLS socket and send a WebSocket upgrade request manually.
    This lets us read the full HTTP response body that the websocket library
    discards when it gets a non-101 status code.
    """
    host = key_LBS.lorawan_server
    port = key_LBS.lorawan_port
    path = "/traffic/" + eui

    request = (
        "GET {path} HTTP/1.1\r\n"
        "Host: {host}:{port}\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n"
        "Sec-WebSocket-Version: 13\r\n"
        "Authorization: Bearer {key}\r\n"
        "\r\n"
    ).format(path=path, host=host, port=port, key=key_LBS.api_key)

    print "--- raw HTTP request ---"
    print request.strip()
    print "---"

    ctx = ssl.create_default_context()
    sock = socket.create_connection((host, port), timeout=10)
    ssl_sock = ctx.wrap_socket(sock, server_hostname=host)
    ssl_sock.sendall(request.encode('ascii'))

    # Read until we have the HTTP response headers (ends with \r\n\r\n).
    # Stop there: if the server replied 101 it is now in WebSocket mode
    # and will not send more HTTP bytes until we send a WebSocket frame.
    # In Python 2.7 an SSL socket timeout raises ssl.SSLError, not socket.timeout.
    response = b""
    try:
        while True:
            chunk = ssl_sock.recv(4096)
            if not chunk:
                break
            response += chunk
            if b"\r\n\r\n" in response:
                break   # full HTTP response headers received
    except (socket.timeout, ssl.SSLError):
        pass   # timeout after headers means server is waiting for WS frames
    finally:
        ssl_sock.close()

    print "--- raw HTTP response ---"
    print response.decode('utf-8', errors='replace')
    print "---"

def main(gwid):
    eui = "eui-" + (gwid[4:10] + "FFFF" + gwid[10:]).lower()

    print "gateway ID (raw)  : " + gwid
    print "EUI in URL        : " + eui
    print "server            : " + key_LBS.lorawan_server + ":" + str(key_LBS.lorawan_port)
    print "API key prefix    : " + key_LBS.api_key[:20] + "..."
    print ""

    raw_http_probe(eui)

if __name__ == "__main__":
    if len(sys.argv) > 1:
        main(sys.argv[1])
    else:
        print "usage: python lbs_debug.py <gwid>"
        sys.exit(1)
