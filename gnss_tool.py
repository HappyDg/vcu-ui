#!/usr/bin/python3

import binascii
import logging
import queue
import socket
import struct
import sys
import threading
import time
from enum import Enum


FORMAT = '%(asctime)-15s %(levelname)-8s %(message)s'
logging.basicConfig(format=FORMAT)
logger = logging.getLogger('gnss_tool')
logger.setLevel(logging.DEBUG)


class UbxFrame(object):
    SYNC_1 = 0xb5
    SYNC_2 = 0x62

    # @classmethod
    # def _create(cls, id, msg):
    #     frame = UbxFrame(cls, id, msg)

    def __init__(self, cls, id, length, msg):
        super().__init__()
        self.cls = cls
        self.id = id
        self.length = length
        # print(self.length)
        self.msg = msg
        # print(self.msg)
        # print(len(self.msg))
        assert self.length == len(self.msg)
        # TODO: checksum as 16 bit integer?
        self._calc_checksum()
        # print(self.cka, self.ckb)

    def is_class_id(self, cls, id):
        return cls == self.cls and id == self.id

    def to_bytes(self):
        assert self.cka != -1 and self.ckb != -1
        # TODO: use SYNC_1, SYNC_2
        msg = bytearray(b'\xb5\x62')
        msg.append(self.cls)
        msg.append(self.id)
        msg.append((self.length >> 0) % 0xFF)
        msg.append((self.length >> 8) % 0xFF)
        msg += self.msg
        msg.append(self.cka)
        msg.append(self.ckb)

        return msg

    def _calc_checksum(self):
        self._init_checksum()

        self._add_checksum(self.cls)
        self._add_checksum(self.id)
        self._add_checksum((self.length >> 0) % 0xFF)
        self._add_checksum((self.length >> 8) % 0xFF)
        for d in self.msg:
            self._add_checksum(d)

    def _init_checksum(self):
        self.cka = 0
        self.ckb = 0

    def _add_checksum(self, byte):
        self.cka += byte
        self.cka &= 0xFF
        self.ckb += self.cka
        self.ckb &= 0xFF

    def __str__(self):
        return f'UBX: cls:{self.cls:02x} id:{self.id:02x} len:{self.length}'


class UbxUpdSos(UbxFrame):
    def __init__(self, ubx_frame):
        super().__init__(ubx_frame.cls, ubx_frame.id, ubx_frame.length, ubx_frame.msg)
        if ubx_frame.length == 8 and ubx_frame.msg[0] == 3:
            self.cmd = ubx_frame.msg[0]
            self.response = ubx_frame.msg[4]
        else:
            self.cmd = -1
            self.response = -1

    def __str__(self):
        return f'UBX-UPD-SOS: cmd:{self.cmd}, response:{self.response}'


# TODO: Inner class of UbxParser
class State(Enum):
    init = 1
    sync = 2
    cls = 3
    id = 4
    len1 = 5
    len2 = 6
    data = 7
    crc1 = 8
    crc2 = 9


class UbxParser(object):
    def __init__(self, queue):
        super().__init__()

        self.queue = queue
        self.state = State.init

        self.msg_class = 0
        self.msg_id = 0
        self.msg_len = 0
        self.msg_data = bytearray()
        self.ofs = 0
        self.cka = 0
        self.ckb = 0

    def process(self, data):
        # print(data)
        for d in data:
            # x = f's: {state}: {d:02x}\n'
            # self.f.write(x)

            if self.state == State.init:
                if d == UbxFrame.SYNC_1:
                    self.state = State.sync

            elif self.state == State.sync:
                if d == UbxFrame.SYNC_2:
                    self._reset()
#                    self.msg_class = 0
#                    self.msg_id = 0
#                    self.msg_len = 0
#                    self.msg_data = bytearray()
#                    self.cka = 0
#                    self.ckb = 0

                    self.state = State.cls
                else:
                    self.state = State.init

            elif self.state == State.cls:
                self.msg_class = d
                self.state = State.id

            elif self.state == State.id:
                self.msg_id = d
                self.state = State.len1

            elif self.state == State.len1:
                self.msg_len = d
                self.state = State.len2

            elif self.state == State.len2:
                self.msg_len = self.msg_len + (d * 256)
                # TODO: Handle case with len = 0 -> goto CRC directly
                self.ofs = 0
                self.state = State.data

            elif self.state == State.data:
                self.msg_data.append(d)
                self.ofs += 1
                if self.ofs == self.msg_len:
                    self.state = State.crc1

            elif self.state == State.crc1:
                self.cka = d
                self.state = State.crc2

            elif self.state == State.crc2:
                self.ckb = d

                # Build frame from received data. This will compute the checksum
                #print(self.cka, self.ckb)
                frame = UbxFrame(self.msg_class, self.msg_id, self.msg_len, self.msg_data)
                #print(binascii.hexlify(frame.to_bytes()))
                #print(frame.cka, frame.ckb)
#                self.parse_frame(frame)

                if frame.cka == self.cka and frame.ckb == self.ckb:
                    self.parse_frame(frame)
                else:
                    logger.warning(f'checksum error in frame')

                self._reset()
                self.state = State.init

    def _reset(self):
        self.msg_class = 0
        self.msg_id = 0
        self.msg_len = 0
        self.msg_data = bytearray()
        self.cka = 0
        self.ckb = 0
        self.ofs = 0

    def parse_frame(self, ubx_frame):
        # logger.debug(f'frame {cls:02x} {id:02x} {len(msg)} {binascii.hexlify(msg)}')
        # self.f.write(f'frame {cls:02x} {id:02x} {len(msg)}\n')

        if ubx_frame.is_class_id(0x09, 0x14):
            logger.debug(f'UBX-UPD-SOS: {binascii.hexlify(ubx_frame.to_bytes())}')
            frame = UbxUpdSos(ubx_frame)
            self.queue.put(frame)
        else:
            self.queue.put(ubx_frame)

        """
        if cls == 0x06 and id == 0x00:
            # print(binascii.hexlify(msg))
            decoded = self.decoder.cfg_prt(msg)
            print(decoded)
            # self.f.write(decoded + '\n')
        elif cls == 0x01 and id == 0x03:
            decoded = self.decoder.nav_status(msg)
            print(decoded)
        elif cls == 0x09 and id == 0x14:
            logger.debug(f'UBX-UPD-SOS: {binascii.hexlify(msg)}')
            frame = UbxUpdSos(cls, id, msg)
            self.queue.put(frame)
        """


class GnssUBlox(threading.Thread):
    gpsd_control_socket = '/var/run/gpsd.sock'
    gpsd_data_socket = ('127.0.0.1', 2947)

    def __init__(self, device_name):
        super().__init__()

        self.device_name = device_name
        self.cmd_header = f'&{self.device_name}='.encode()
        self.connect_msg = f'?WATCH={{"device":"{self.device_name}","enable":true,"raw":2}}'.encode()

        self.listen_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.response_queue = queue.Queue()
        self.parser = UbxParser(self.response_queue)
        self.thread_ready_event = threading.Event()
        self.thread_stop_event = threading.Event()

        self.wait_msg_class = -1
        self.wait_msg_id = -1
        # self.f = open('log.txt', 'w')

    def setup(self):
        # Start worker thread in daemon mode, will invoke run() method
        self.daemon = True
        self.start()

        # Wait for worker thread to become ready.
        # Without this wait we might send the command before the thread can
        # handle the response.
        logger.info('waiting for receive thread to become active')
        self.thread_ready_event.wait()

    def cleanup(self):
        logger.info('requesting thread to stop')
        self.thread_stop_event.set()

        # Wait until thread ended
        self.join(timeout=1.0)
        logger.info('thread stopped')

    def wait_for(self, msg_class, msg_id):
        """
        Define message message to wait for
        """
        self.wait_msg_class = msg_class
        self.wait_msg_id = msg_id

    def send(self, ubx_message):
        try:
            self.control_sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            self.control_sock.connect(GnssUBlox.gpsd_control_socket)

            # sock.sendall('?devices'.encode())
            # data = sock.recv(512)
            # print(data.decode())

            msg_in_binary = ubx_message.to_bytes()
            msg_in_ascii = binascii.hexlify(msg_in_binary)
            # logger.debug(f'sending {msg_in_ascii}')

            cmd = self.cmd_header + msg_in_ascii
            logger.debug(f'sending control message {cmd}')
            self.control_sock.sendall(cmd)

            # checking for response (OK or ERROR)
            data = self.control_sock.recv(512)
            response = data.decode().strip()
            logger.debug(f'response: {response}')

            # TODO: check why we need to close socket here...
            self.control_sock.close()

        except socket.error as msg_in_ascii:
            logger.error(msg_in_ascii)

    def wait(self, timeout=3.0):
        logger.info(f'waiting {timeout}s for reponse from listener thread')

        time_end = time.time() + timeout
        while time.time() < time_end:
            try:
                res = self.response_queue.get(True, timeout)
                logger.info(f'got response {res}')
                if isinstance(res, UbxFrame):
                    if res.cls == self.wait_msg_class and res.id == self.wait_msg_id:
                        return res
            except queue.Empty:
                logger.warning('timeout...')

    def run(self):
        """
        Thread running method

        - receives raw data from gpsd
        - parses ubx frames, decodes them
        - if a frame is received it is put in the receive queue
        """
        try:
            logger.info('connecting to gpsd')
            self.listen_sock.connect(('127.0.0.1', 2947))
        except socket.error as msg:
            logger.error(msg)
            # TODO: Error handling

        try:
            logger.debug('starting raw listener on gpsd')
            self.listen_sock.send(self.connect_msg)
            self.listen_sock.settimeout(0.25)

            logger.debug('receiver ready')
            self.thread_ready_event.set()

            while not self.thread_stop_event.is_set():
                try:
                    data = self.listen_sock.recv(8192)
                    if data:
                        self.parser.process(data)
                except socket.timeout:
                    pass

        except socket.error as msg:
            print("error")
            logger.error(msg)

        logger.debug('receiver done')


msg_stop = bytearray.fromhex('06 04 04 00 00 00 08 00')    # stop
msg_cold_start = bytearray.fromhex('06 04 04 00 FF FF 01 00')  # Cold Start
msg_mon_ver = bytearray.fromhex('0A 04 00 00')  # MON-VER
msg_cfg_port_poll = bytearray.fromhex('06 00 01 00 01')  # UBX-CFG-PRT poll
msg_nav_status_poll = bytearray.fromhex('01 03 00 00')

# msg_cfg_port_uart_9600 = bytearray.fromhex('06 00 14 00 01 00 00 00 c0 08 00 00 80 25 00 00 07 00 01 00 00 00 00 00')
msg_cfg_port_uart_115200 = bytearray.fromhex('06 00 14 00 01 00 00 00 c0 08 00 00 00 c2 01 00 07 00 01 00 00 00 00 00')

msg_upd_sos_save = bytearray.fromhex('09 14 04 00 00 00 00 00')
msg_upd_sos_clear = bytearray.fromhex('09 14 04 00 01 00 00 00')

# b'b562091400001d60'
#   b562091400001d26
msg_upd_sos_poll = UbxFrame(0x09, 0x14, 0, bytearray())
print(msg_upd_sos_poll)
print(msg_upd_sos_poll.cka)
print(msg_upd_sos_poll.ckb)
print(binascii.hexlify(msg_upd_sos_poll.to_bytes()))

# 01 06 3400 10ae510e50ecffff2f0803df154c921990f594033c2cd01bcd000000feffffff01000000feffffff160000009f00020c7e923d02 c8 e0
x = UbxFrame(0x01, 0x06, 0x34, bytearray.fromhex('10ae510e50ecffff2f0803df154c921990f594033c2cd01bcd000000feffffff01000000feffffff160000009f00020c7e923d02'))
print(x)
print(f'{x.cka:02x}')
print(f'{x.ckb:02x}')
#quit()

r = GnssUBlox('/dev/ttyS3')
r.setup()

for i in range(0, 2):
    # TODO: combine method, because response frame has same class/id as polling message
    r.wait_for(0x09, 0x14)
    r.send(msg_upd_sos_poll)

    res = r.wait()
    assert(res)
    if res:
        print(f'SOS state is {res.response}')

    time.sleep(0.87)

r.cleanup()
