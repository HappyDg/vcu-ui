#!/usr/bin/python3

import binascii
import logging
import queue
import socket
import struct
import threading
import time
from enum import Enum


FORMAT = '%(asctime)-15s %(levelname)-8s %(message)s'
logging.basicConfig(format=FORMAT)
logger = logging.getLogger('gnss_tool')
logger.setLevel(logging.DEBUG)


class ClsMsgId(object):
    """
    Class and Msg Id

    All UBX messages are identified by their message class and id. 
    This class models the message identifiers.
    """
    def __init__(self, clsId, msgId):
        self.clsId = clsId
        self.msgId = msgId

    def __eq__(self, other):
        return self.clsId == other.clsId and self.msgId == other.msgId

    def __str__(self):
        return f'clsId:{self.clsId:02x} msgId:{self.msgId:02x}'


class UbxMessage(Enum):
    """
    Messages (class, id) known by this module
    """
    ACK_ACK = ClsMsgId(0x05, 0x01)
    ACK_NAK = ClsMsgId(0x05, 0x00)

    CFG_TP5 = ClsMsgId(0x06, 0x31)

    UPD_SOS = ClsMsgId(0x09, 0x14)
    MON_VER = ClsMsgId(0x0a, 0x04)


class UbxFrame(object):
    SYNC_1 = 0xb5
    SYNC_2 = 0x62

    def __init__(self, cls_msg_id, length=0, data=bytearray()):
        super().__init__()
        self.cls_msg_id = cls_msg_id
        self.length = length
        self.data = data
        assert self.length == len(self.data)
        self._calc_checksum()

    def matches(self, msg_type):
        assert isinstance(msg_type, UbxMessage)

        # msg_type.value is the ClsMsgId object behind the enum value
        #   e.g. UbxMessage.ACK_ACK becomes ClsMsgId(0x05, 0x01)
        return self.cls_msg_id == msg_type.value

    def to_bytes(self):
        """
        Returns binary represention of frame in wire format

        Returned bytearray can be sent to modem
        """
        msg = bytearray([UbxFrame.SYNC_1, UbxFrame.SYNC_2])
        msg.append(self.cls_msg_id.clsId)
        msg.append(self.cls_msg_id.msgId)
        msg.append((self.length >> 0) % 0xFF)
        msg.append((self.length >> 8) % 0xFF)
        msg += self.data
        msg.append(self.cka)
        msg.append(self.ckb)

        return msg

    def _calc_checksum(self):
        """
        Computes checksum over frame and stores in object
        """
        self._init_checksum()
        self._add_checksum(self.cls_msg_id.clsId)
        self._add_checksum(self.cls_msg_id.msgId)
        self._add_checksum((self.length >> 0) & 0xFF)
        self._add_checksum((self.length >> 8) & 0xFF)
        for d in self.data:
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
        return f'UBX: {self.cls_msg_id} len:{self.length}'


class UbxPoll(UbxFrame):
    """
    Base class for a polling frame.

    Create by specifying u-blox message class and id.
    e.g. UbxPoll(UbxMessage.UPD_SOS)
    """
    def __init__(self, msg_type):
        assert isinstance(msg_type, UbxMessage)

        # msg_type.value is the ClsMsgId object behind the enum value
        #   e.g. UbxMessage.ACK_ACK becomes ClsMsgId(0x05, 0x01)
        super().__init__(msg_type.value)


class UbxAckAck(UbxFrame):
    def __init__(self, msg):
        super().__init__(msg.cls_msg_id, msg.length, msg.data)
        if msg.length == 2:
            self.ack_clsID = msg.data[0]
            self.ack_msgID = msg.data[1]
        else:
            self.ack_clsID = -1
            self.ack_msgID = -1

    def __str__(self):
        return f'UBX-ACK-ACK: clsId:{self.ack_clsID}, msgId:{self.ack_msgID}'


class UbxMonVer(UbxFrame):
    """
    UBX-MON-VER response
    """
    EXTENSION_SIZE = 30

    def __init__(self, msg):
        super().__init__(msg.cls_msg_id, msg.length, msg.data)
        if msg.length >= 40:
            self.swVersion = msg.data[0:29].decode()
            self.hwVersion = msg.data[30:39].decode()
            self.extension = []

            remain = msg.length - 40
            offset = 40
            if remain % UbxMonVer.EXTENSION_SIZE == 0:
                while remain > 0:
                    ext = msg.data[offset:offset+UbxMonVer.EXTENSION_SIZE].decode()
                    self.extension.append(ext)
                    remain -= UbxMonVer.EXTENSION_SIZE
                    offset += UbxMonVer.EXTENSION_SIZE

            for t in self.extension:
                print(t)

    def __str__(self):
        res = f'UBX-UPD-SOS:\nSW: {self.swVersion}\nHW: {self.hwVersion}'
        for ext in self.extension:
            res += '\n' + ext
        return res


class UbxUpdSos(UbxFrame):
    """
    UBX-UPD-SOS response
    """
    def __init__(self, msg):
        super().__init__(msg.cls_msg_id, msg.length, msg.data)
        if msg.length == 8 and msg.data[0] == 3:
            self.cmd = msg.data[0]
            self.response = msg.data[4]
            # TODO: Validity check (0..3)
        else:
            self.cmd = -1
            self.response = -1

    def __str__(self):
        resp_str = self._response_str()
        return f'UBX-UPD-SOS: cmd:{self.cmd}, response:{self.response} ({resp_str})'

    def _response_str(self):
        resp_str = ['unknown', 'failed', 'restored', 'not restored (no backup)']
        # TODO: Error check
        return resp_str[self.response]


class UbxUpdSosAction(UbxFrame):
    """
    UBX-UPD-SOS action

    action: 0 = create
            1 = clear
    """
    def __init__(self, action):
        # TODO: build message depending on action parameter
        msg = bytearray.fromhex('01 00 00 00')
        super().__init__(UbxMessage.UPD_SOS.value, 4, msg)


class UbxCfgTp5(UbxFrame):
    """
    UBX-CFG-TP5 response
    """
    def __init__(self, msg):
        super().__init__(msg.cls_msg_id, msg.length, msg.data)
        if msg.length == 32:
            tpIdx, version, res1, antCableDelay, rfGroupDelay = struct.unpack('BBhhh', msg.data[0:8])
            print(tpIdx, version, antCableDelay, rfGroupDelay)

            freqPeriod, freqPeriodLock, pulseLenRatio, pulseLenRatioLock = struct.unpack('<IIII', msg.data[8:24])
            print(freqPeriod, freqPeriodLock, pulseLenRatio, pulseLenRatioLock)

            userConfigDelay = struct.unpack('<I', msg.data[24:28])
            print(userConfigDelay)

            flags = struct.unpack('<I', msg.data[24:28])
            print(flags)
        else:
            pass

    def __str__(self):
        return f'UBX-CFG-TP5:'


class UbxParser(object):
    """
    Parser that tries to extract UBX frames from arbitrary byte streams

    Byte streams can also be NMEA or other frames. Unfortunately,
    u-blox frame header also appears in NMEA frames (e.g. version
    information). Such data will be detected by a checksum error

    TODO: Do more elaborate parsing to filter out such data in advance
    """

    class State(Enum):
        """
        Parser states
        """
        init = 1
        sync = 2
        cls = 3
        id = 4
        len1 = 5
        len2 = 6
        data = 7
        crc1 = 8
        crc2 = 9

    def __init__(self, queue):
        super().__init__()

        self.queue = queue
        self.state = __class__.State.init
        self._reset()

    def process(self, data):
        for d in data:
            if self.state == __class__.State.init:
                if d == UbxFrame.SYNC_1:
                    self.state = __class__.State.sync

            elif self.state == __class__.State.sync:
                if d == UbxFrame.SYNC_2:
                    self._reset()
                    self.state = __class__.State.cls
                else:
                    self.state = __class__.State.init

            elif self.state == __class__.State.cls:
                self.msg_class = d
                self.state = __class__.State.id

            elif self.state == __class__.State.id:
                self.msg_id = d
                self.state = __class__.State.len1

            elif self.state == __class__.State.len1:
                self.msg_len = d
                self.state = __class__.State.len2

            elif self.state == __class__.State.len2:
                self.msg_len = self.msg_len + (d * 256)
                # TODO: Handle case with len = 0 -> goto CRC directly
                # TODO: Handle case with unreasonable size
                self.ofs = 0
                self.state = __class__.State.data

            elif self.state == __class__.State.data:
                self.msg_data.append(d)
                self.ofs += 1
                if self.ofs == self.msg_len:
                    self.state = __class__.State.crc1

            elif self.state == __class__.State.crc1:
                self.cka = d
                self.state = __class__.State.crc2

            elif self.state == __class__.State.crc2:
                self.ckb = d

                # Build frame from received data. This computes the checksum
                # if checksum matches received checksum forward message to
                # parser for further dissection
                cls_id = ClsMsgId(self.msg_class, self.msg_id)
                frame = UbxFrame(cls_id, self.msg_len, self.msg_data)

                if frame.cka == self.cka and frame.ckb == self.ckb:
                    self.parse_message(frame)
                else:
                    logger.warning(f'checksum error in frame')

                self._reset()
                self.state = __class__.State.init

    def parse_message(self, frame):
        if frame.matches(UbxMessage.ACK_ACK):
            frame = UbxAckAck(frame)
        elif frame.matches(UbxMessage.MON_VER):
            frame = UbxMonVer(frame)
        elif frame.matches(UbxMessage.UPD_SOS):
            frame = UbxUpdSos(frame)
        elif frame.matches(UbxMessage.CFG_TP5):
            frame = UbxCfgTp5(frame)
        else:
            # If we can't parse the frame, return as is
            frame = frame
            # frame = None

        if frame:
            self.queue.put(frame)

    def _reset(self):
        """
        Resets parser state for next frame
        """
        self.msg_class = 0
        self.msg_id = 0
        self.msg_len = 0
        self.msg_data = bytearray()
        self.cka = 0
        self.ckb = 0
        self.ofs = 0


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

        self.wait_cls_msg_id = None

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

    # High Level API

    def sos_state(self):
        logger.debug('checking SOS restore state')
        msg_upd_sos_poll = UbxPoll(UbxMessage.UPD_SOS)
        res = self.poll(msg_upd_sos_poll)
        if res:
            return res.response

    def sos_create_backup(self):
        logger.debug('creating state backup file')

        msg = UbxUpdSosAction(0)
        print(msg.to_bytes())
        self.expect(UbxMessage.ACK_ACK)
        self.send(msg)
        res = self.wait()
        print(res)

    def sos_clear_backup(self):
        logger.debug('clearing state backup file')

        msg = UbxUpdSosAction(1)
        print(msg.to_bytes())
        self.expect(UbxMessage.ACK_ACK)
        self.send(msg)
        res = self.wait()
        print(res)

    # Low Level API

    def poll(self, message):
        """
        Polls a receiver status

        - sends the specified poll message
        - waits for receiver message with same class/id as poll message
        """
        assert isinstance(message, UbxFrame)
        assert message.length == 0      # poll message have no payload

        self.expect(message.cls_msg_id)
        self.send(message)
        res = self.wait()
        return res

    def expect(self, cls_msg_id):
        """
        Defines message message to wait for
        """
        logger.debug(f'preparing to wait for {cls_msg_id}')
        self.wait_cls_msg_id = cls_msg_id

    def send(self, message):
        """
        Sends binary message to GNSS modem

        - opens connection to gpsd control socket (not data socket)
        - creates control message in gpsd format (ASCII format)
            "&/dev/ttyS3=b56201011234..."
        - sends message and checks gpsd control daemon response
        """
        try:
            self.control_sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            self.control_sock.connect(GnssUBlox.gpsd_control_socket)

            # sock.sendall('?devices'.encode())
            # data = sock.recv(512)
            # print(data.decode())

            msg_in_binary = message.to_bytes()
            msg_in_ascii = binascii.hexlify(msg_in_binary)
            # logger.debug(f'sending {msg_in_ascii}')

            cmd = self.cmd_header + msg_in_ascii
            logger.debug(f'sending control message {cmd}')
            self.control_sock.sendall(cmd)

            # checking for response (OK or ERROR)
            data = self.control_sock.recv(512)
            response = data.decode().strip()
            logger.debug(f'gpsd response: {response}')

            # TODO: check why we need to close socket here...
            self.control_sock.close()

        except socket.error as msg_in_ascii:
            logger.error(msg_in_ascii)

    def wait(self, timeout=3.0):
        """
        waits for message defined in expect() to arrive and returns it
        """
        logger.debug(f'waiting {timeout}s for reponse from listener thread')

        time_end = time.time() + timeout
        while time.time() < time_end:
            try:
                res = self.response_queue.get(True, timeout)
                logger.debug(f'got response {res}')
                if isinstance(res, UbxFrame):
                    if res.cls_msg_id == self.wait_cls_msg_id:
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
            # ready = False

            while not self.thread_stop_event.is_set():
                try:
                    data = self.listen_sock.recv(8192)
                    if data:
                        # if not ready:
                        #     self.thread_ready_event.set()
                        #     ready = True
                        self.parser.process(data)

                except socket.timeout:
                    pass

        except socket.error as msg:
            logger.error(msg)

        logger.debug('receiver done')


def main():
    r = GnssUBlox('/dev/ttyS3')
    r.setup()
    # time.sleep(0.5)

    # r.sos_clear_backup()
    # response = r.sos_state()
    # if response:
    #     print(f'SOS state is {response}')

    msg = UbxPoll(UbxMessage.MON_VER)
    r.poll(msg)
    # print(res)

    msg_upd_tp5_poll = UbxPoll(UbxMessage.CFG_TP5)
    res = r.poll(msg_upd_tp5_poll)
    # print(res)
    # quit()

    for i in range(0, 1):
        print(f'***** {i} ***********************')
        response = r.sos_state()
        if response:
            print(f'SOS state is {response}')
        else:
            print('no reponse')

        # msg_upd_tp5_poll = UbxFrame(*UbxCfgTp5.CLASS_ID())
        # res = r.poll(msg_upd_tp5_poll)
        # print(res)

        time.sleep(0.87)

    r.cleanup()


if __name__ == "__main__":
    main()
