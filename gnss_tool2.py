#!/usr/bin/python3

import binascii
import queue
import socket
import struct
import sys
import threading

import vcuui.gnss as gnss

"""
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
"""

def flag_s(flag, descs):
    """Decode flag using descs, return a string.  Ignores unknown bits."""

    s = ''
    for key, value in sorted(descs.items()):
        if key == (key & flag):
            s += value
            s += ' '

    return s.strip()


class ubx_decoder:
    cfg_prt_flags = {
        0x2: 'extendedTxTimeout',
    }

    cfg_prt_proto = {
        0x1: 'UBX',
        0x2: 'NMEA',
        0x4: 'RTCM2',
        0x20: 'RTCM3',
    }

    # Names for portID values in UBX-CFG-PRT, UBX-MON-IO, etc.
    port_ids = {0: 'DDC',  # The inappropriate name for i2c used in the spec
                1: 'UART1',
                2: 'UART2',
                3: 'USB',
                4: 'SPI',
                }

    def cfg_prt(self, buf):
        """UBX-CFG-PRT decode"""

        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        portid = buf[0]
        idstr = '%u (%s)' % (portid, self.port_ids.get(portid, '?'))

        if 1 == m_len:
            return " Poll request PortID %s" % idstr

        # Note that this message can contain multiple 20-byte submessages, but
        # only in the send direction, which we don't currently do.
        if 20 > m_len:
            return "Bad Length %s" % m_len

        u = struct.unpack_from('<BBHLLHHHH', buf, 0)

        s = [' PortID %s reserved1 %u txReady %#x' % (idstr, u[1], u[2])]
        s.append({1: '  mode %#x baudRate %u',
                  2: '  mode %#x baudRate %u',
                  3: '  reserved2 [%u %u]',
                  4: '  mode %#x reserved2 %u',
                  0: '  mode %#x reserved2 %u',
                  }.get(portid, '  ???: %u,%u') % tuple(u[3:5]))
        s.append('  inProtoMask %#x outProtoMask %#x' % tuple(u[5:7]))
        s.append({1: '  flags %#x reserved2 %u',
                  2: '  flags %#x reserved2 %u',
                  3: '  reserved3 %u reserved4 %u',
                  4: '  flags %#x reserved3 %u',
                  0: '  flags %#x reserved3 %u',
                  }.get(portid, '  ??? %u,%u') % tuple(u[7:]))

        if portid == 0:
            s.append('    slaveAddr %#x' % (u[3] >> 1 & 0x7F))

        s.append('    inProtoMask (%s)\n'
                 '    outProtoMask (%s)' %
                 (flag_s(u[5], self.cfg_prt_proto),
                  flag_s(u[6], self.cfg_prt_proto)))

        if portid in set([1, 2, 4, 0]):
            s.append('    flags (%s)' % flag_s(u[7], self.cfg_prt_flags))

        return '\n'.join(s)

    def nav_status(self, buf):
        """UBX-NAV-STATUS decode"""
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 16 > m_len:
            return " Bad Length %s" % m_len

        u = struct.unpack_from('<LBBBBLL', buf, 0)
        return ('  iTOW:%d ms, fix:%d flags:%#x fixstat:%#x flags2:%#x\n'
                '  ttff:%d, msss:%d' % u)

    def sos_poll(self, buf):
        m_len = len(buf)
        print(f'UPD-SPS: {m_len}, {buf[4]}')


class Parser():
    def __init__(self, queue):
        super().__init__()

        self.queue = queue
        self.decoder = ubx_decoder()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # self.f = open('log.txt', 'w')

    def listen(self):
        try:
            self.sock.connect(('127.0.0.1', 2947))

            gpsd_client_raw = '?WATCH={"device":"/dev/ttyS3","enable":true,"raw":2}'
            self.sock.send(gpsd_client_raw.encode())

            state = 'init'

            while True:
                data = self.sock.recv(32768)
                # print(binascii.hexlify(data))

                for d in data:
                    # x = f's: {state}: {d:02x}\n'
                    # self.f.write(x)

                    if state == 'init':
                        if d == 0xb5:
                            state = 'sync'
                    elif state == 'sync':
                        if d == 0x62:
                            msg_class = 0
                            msg_id = 0
                            msg_len = 0
                            msg_data = bytearray()
                            ofs = 0
                            state = 'class'
                        else:
                            state = 'init'
                    elif state == 'class':
                        msg_class = d
                        # print(f'class {msg_class}')
                        state = 'id'
                    elif state == 'id':
                        msg_id = d
                        # print(f'id {msg_id}')
                        state = 'len1'
                    elif state == 'len1':
                        msg_len = d
                        state = 'len2'
                    elif state == 'len2':
                        msg_len = msg_len + (d * 256)
                        # print(f'msg len is {msg_len}')
                        state = 'data'
                    elif state == 'data':
                        msg_data.append(d)
                        ofs += 1
                        if ofs == msg_len:
                            state = 'crc1'
                    elif state == 'crc1':
                        state = 'crc2'
                    elif state == 'crc2':
                        self.process(msg_class, msg_id, msg_data)

                        msg_class = 0
                        msg_id = 0
                        msg_len = 0
                        msg_data = bytearray()

                        state = 'init'

        except socket.error as msg:
            print(msg)

    # TODO: rename cls, class UbxFrame ?
    def process(self, cls, id, msg):
        # print(f'frame {cls:02x} {id:02x} {len(msg)}')
        # self.f.write(f'frame {cls:02x} {id:02x} {len(msg)}\n')

        if cls == 0x06 and id == 0x00:
            # print(binascii.hexlify(msg))
            decoded = self.decoder.cfg_prt(msg)
            print(decoded)
            # self.f.write(decoded + '\n')
        elif cls == 0x01 and id == 0x03:
            decoded = self.decoder.nav_status(msg)
            print(decoded)
        elif cls == 0x09 and id == 0x14:
            print(binascii.hexlify(msg))
            # decoded = self.decoder.nav_status(msg)
            # print(decoded)
            self.decoder.sos_poll(msg)
            self.queue.put((cls, id))
            # self.queue.put(1)


class GnssUBlox(threading.Thread):
    server_address = '/var/run/gpsd.sock'

    def __init__(self):
        super().__init__()

        self.daemon = True

        self.queue = queue.Queue()

        self.parser = Parser(self.queue)
        self.sock = None
        # self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)

        self.wait_msg_class = -1
        self.wait_msg_id = -1

    def setup(self):
        #try:
        #    self.sock.connect(GnssUBlox.server_address)
        #    self.sock.sendall('?devices'.encode())
        #    data = self.sock.recv(512)
        #    print(data.decode())
        #except socket.error as msg:
        #    print(msg)

        self.start()

    def wait_for(self, msg_class, msg_id):
        self.wait_msg_class = msg_class
        self.wait_msg_id = msg_id

    def send(self, control_msg):
        ubx_msg = self._create_ubx_message(control_msg)
        msg = binascii.hexlify(ubx_msg)
        print(msg)

        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            self.sock.connect(GnssUBlox.server_address)

            # sock.sendall('?devices'.encode())
            # data = sock.recv(512)
            # print(data.decode())

            # cmd = '&/dev/ttyS3=' + msg
            # print(cmd)
            # sock.send(cmd.encode())

            cmd = '&/dev/ttyS3='.encode() + msg
            # print(cmd)
            self.sock.send(cmd)

            # data = self.sock.recv(512)
            # print(data.decode())

            self.sock.close()

        except socket.error as msg:
            print(msg)

    def wait(self, timeout=5.0):
        try:
            res = self.queue.get(True, timeout)
            print(f'got response {res}')
        except queue.Empty:
            pass

    def run(self):
        self.parser.listen()

    def _create_ubx_message(self, data):
        cka, ckb = self._calc_checksum(data)
        msg = bytearray(b'\xb5\x62') + data
        msg.append(cka)
        msg.append(ckb)
        return msg

    def _calc_checksum(self, data):
        cka, ckb = 0, 0
        # ckb = 0

        for byte in data:
            cka += byte
            cka &= 0xFF
            ckb += cka
            ckb &= 0xFF

        return cka, ckb


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

r = GnssUBlox()
r.setup()

r.wait_for(0x09, 0x14)
r.send(msg_upd_sos_poll)

r.wait(2.0)
