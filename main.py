#!/usr/bin/python3

# from vcuui.server import run_server

# run_server(port=80)


import socket
import sys
import os
import vcuui.gnss as gnss




# UBX-CFG-PRT

def test1():
    # msg = bytearray.fromhex('06 04 04 00 00 00 08 00')    # stop
    # msg = bytearray.fromhex('06 00 01 00 01')  # UBX-CFG-PRT poll 
    # msg = bytearray.fromhex('06 04 04 00 FF FF 01 00')  # Cold Start
    msg = bytearray.fromhex('06 31 01 00 00')  # Poll TP5
    # msg = bytearray.fromhex('0A 04 00 00')  # MO-VER

    ubx_msg = gnss._create_ubx_message(msg)
    msg = ''
    for x in ubx_msg:
        msg = msg + f'{x:02x}'

    print(msg)
    # return

    server_address = '/var/run/gpsd.sock'

    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        sock.connect(server_address)

        sock.sendall('?devices'.encode())

        data = sock.recv(512)
        print(data.decode())

        # stop_msg_str = '&/dev/ttyS3=b56206040400000008001674'
        cmd = '&/dev/ttyS3=' + msg
        print(cmd)
        sock.sendall(cmd.encode())

        data = sock.recv(512)
        print(data.decode())

        sock.close()

    except socket.error as msg:
        print(msg)
        sys.exit(1)


def test2():
    """
                    # ask for raw, not rare, data
                    data_out = b'?WATCH={'
                    if opts['target']['device'] is not None:
                        # add in the requested device
                        data_out += (b'"device":"' + opts['target']['device'] +
                                    b'",')
                    data_out += b'"enable":true,"raw":2}\r\n'
    """

    gpsd_client_raw = '?WATCH={"device":"/dev/ttyS3","enable":true,"raw":2}'

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect(('127.0.0.1', 2947))

        sock.send(gpsd_client_raw.encode())

        while True:
            data = sock.recv(512)
            # print(data.decode())
            msg = ''
            for d in data:
                msg = msg + f'{d:02x}'

            print(msg)


    except socket.error as msg:
        print(msg)
        sys.exit(1)


test2()
