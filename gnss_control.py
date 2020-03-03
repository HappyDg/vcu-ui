#!/usr/bin/python3

import binascii
import socket
import sys

# import os
import vcuui.gnss as gnss

msg_stop = bytearray.fromhex('06 04 04 00 00 00 08 00')    # stop
msg_cold_start = bytearray.fromhex('06 04 04 00 FF FF 01 00')  # Cold Start
msg_mon_ver = bytearray.fromhex('0A 04 00 00')  # MON-VER
msg_cfg_port_poll = bytearray.fromhex('06 00 01 00 01')  # UBX-CFG-PRT poll 
msg_nav_status_poll = bytearray.fromhex('01 03 00 00')

# msg_cfg_port_uart_9600 = bytearray.fromhex('06 00 14 00 01 00 00 00 c0 08 00 00 80 25 00 00 07 00 01 00 00 00 00 00')
msg_cfg_port_uart_115200 = bytearray.fromhex('06 00 14 00 01 00 00 00 c0 08 00 00 00 c2 01 00 07 00 01 00 00 00 00 00')

msg_upd_sos_poll = bytearray.fromhex('09 14 00 00')
msg_upd_sos_save = bytearray.fromhex('09 14 04 00 00 00 00 00')
msg_upd_sos_clear = bytearray.fromhex('09 14 04 00 01 00 00 00')


def send(control_msg):
    ubx_msg = gnss._create_ubx_message(control_msg)

    msg = ''
    for x in ubx_msg:
        msg = msg + f'{x:02x}'

    print(msg)

    msg2 = binascii.hexlify(ubx_msg)
    print(msg2)

    # return

    server_address = '/var/run/gpsd.sock'

    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        sock.connect(server_address)

        # sock.sendall('?devices'.encode())
        # data = sock.recv(512)
        # print(data.decode())

        # cmd = '&/dev/ttyS3=' + msg
        # print(cmd)
        # sock.send(cmd.encode())

        cmd = '&/dev/ttyS3='.encode() + msg2
        print(cmd)
        sock.send(cmd)

        data = sock.recv(512)
        print(data.decode())

        sock.close()

    except socket.error as msg:
        print(msg)
        sys.exit(1)


# send(msg_nav_status_poll)
# send(msg_cfg_port_poll)
# send(msg_cfg_port_uart_115200)
send(msg_upd_sos_poll)
send(msg_upd_sos_clear)
