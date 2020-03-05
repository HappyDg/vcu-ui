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
        return f'clsId: 0x{self.clsId:02x}, msgId: 0x{self.msgId:02x}'


class UbxMessage(Enum):
    """
    Messages (class, id) known by this module
    """
    ACK_ACK = ClsMsgId(0x05, 0x01)
    ACK_NAK = ClsMsgId(0x05, 0x00)

    UPD_SOS = ClsMsgId(0x09, 0x14)
    MON_VER = ClsMsgId(0x0a, 0x04)


class UbxChecksum(object):
    def __init__(self):
        super().__init__()
        self.cka = 0
        self.ckb = 0

    def ok(self, cka, ckb):
        return cka == self.cka and ckb == self.ckb

    def reset(self):
        self.cka = 0
        self.ckb = 0

    def add(self, byte):
        self.cka += byte
        self.cka &= 0xFF
        self.ckb += self.cka
        self.ckb &= 0xFF


class UbxFrame(object):
    SYNC_1 = 0xb5
    SYNC_2 = 0x62

    # TODO: add constructor for empty frame and ctor to initialize with
    # data from buffer
    @classmethod
    def empty(cls, cls_msg_id):
        self = cls(cls_msg_id)
        return self

    @classmethod
    def from_data(cls, cls_msg_id, data, cka, ckb):
        """
        Construct frame from received data

        Typically used in receiver parser to create frame
        """
        obj = cls(cls_msg_id)
        obj.data = data
        obj.cka = cka
        obj.ckb = ckb
        return obj

    def __init__(self, cls_msg_id, data=bytearray()):
        super().__init__()
        self.cls_msg_id = cls_msg_id
        self.data = data

    @property
    def length(self):
        return len(self.data)

    def add_u8(self, value):
        assert 0 <= value <= 255
        self.data.append(value)

    def add_u32(self, value):
        data = struct.pack('<I', value)
        self.data += data

    def checksum_ok(self, cka, ckb):
        return cka == self.cka and ckb == self.ckb

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
        self._calc_checksum()

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


class UbxResponse(object):
    """
    Base class for responses from modem

    Typically constructed from the parser
    """
    def __init__(self, cls_msg_id, data):
        super().__init__()
        self.cls_msg_id = cls_msg_id
        self.data = data

    # TODO: add methods for easy parsing of messages


# TODO: Does this need to derive from UbxFrame.
# Actually the frame data is not required anymore
# If a configuration request shall ever be define
# using this class it can return a UbxFrame object
# (or bytestream).
# Agregation or inheritance seems unnecessary
class UbxAckAck(UbxResponse):
    def __init__(self, data):
        super().__init__(UbxMessage.ACK_ACK.value, data)

        if len(self.data) == 2:
            self.ack_clsID = self.data[0]
            self.ack_msgID = self.data[1]
        else:
            self.ack_clsID = -1
            self.ack_msgID = -1

    def __str__(self):
        return f'UBX-ACK-ACK: clsId:{self.ack_clsID}, msgId:{self.ack_msgID}'


class UbxMonVer(UbxResponse):
    """
    UBX-MON-VER response
    """
    EXTENSION_SIZE = 30

    def __init__(self, data):
        # super().__init__(msg)
        super().__init__(UbxMessage.MON_VER.value, data)

        if len(self.data) >= 40:
            self.swVersion = self.data[0:29].decode()
            self.hwVersion = self.data[30:39].decode()
            self.extension = []

            remain = len(self.data) - 40
            offset = 40
            if remain % UbxMonVer.EXTENSION_SIZE == 0:
                while remain > 0:
                    ext = self.data[offset:offset+UbxMonVer.EXTENSION_SIZE].decode()
                    self.extension.append(ext)
                    remain -= UbxMonVer.EXTENSION_SIZE
                    offset += UbxMonVer.EXTENSION_SIZE

    def __str__(self):
        res = f'UBX-UPD-SOS:\nSW: {self.swVersion}\nHW: {self.hwVersion}'
        for ext in self.extension:
            res += '\n' + ext
        return res


class UbxUpdSos(UbxResponse):
    """
    UBX-UPD-SOS response
    """
    RESPONSE_STR = ['unknown', 'failed', 'restored', 'not restored (no backup)']

    def __init__(self, data):
        # super().__init__(msg)
        super().__init__(UbxMessage.UPD_SOS.value, data)

        if len(self.data) == 8 and self.data[0] == 3:
            self.cmd = self.data[0]
            self.response = self.data[4]
            # TODO: Validity check (0..3)
        else:
            self.cmd = -1
            self.response = -1

    def __str__(self):
        resp = self._response_str()
        return f'UBX-UPD-SOS: cmd:{self.cmd}, response:{self.response} ({resp})'

    def _response_str(self):
        # TODO: range check
        return UbxUpdSos.RESPONSE_STR[self.response]


class UbxUpdSosAction(UbxFrame):
    """
    UBX-UPD-SOS action

    action: 0 = create
            1 = clear
    """
    def __init__(self, action):
        super().__init__(UbxMessage.UPD_SOS.value)

        if action == 0 or action == 1:
            self.add_u32(action)
        else:
            raise ValueError


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
        # TODO: Uppercase, CamelCase ?
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
        self.cksum = UbxChecksum()
        self.state = __class__.State.init

    def process(self, data):
        for d in data:
            if self.state == __class__.State.init:
                if d == UbxFrame.SYNC_1:
                    self.state = __class__.State.sync

            elif self.state == __class__.State.sync:
                if d == UbxFrame.SYNC_2:
                    self.msg_reset()
                    self.cksum.reset()
                    self.state = __class__.State.cls
                else:
                    self.state = __class__.State.init

            elif self.state == __class__.State.cls:
                self.msg_class = d
                self.cksum.add(d)
                self.state = __class__.State.id

            elif self.state == __class__.State.id:
                self.msg_id = d
                self.cksum.add(d)
                self.state = __class__.State.len1

            elif self.state == __class__.State.len1:
                self.msg_len = d
                self.cksum.add(d)
                self.state = __class__.State.len2

            elif self.state == __class__.State.len2:
                self.msg_len = self.msg_len + (d * 256)
                self.cksum.add(d)
                # TODO: Handle case with unreasonable size
                if self.msg_len != 0:
                    self.state = __class__.State.data
                else:
                    self.state = __class__.State.crc1

            elif self.state == __class__.State.data:
                self.msg_data.append(d)
                self.cksum.add(d)
                if len(self.msg_data) == self.msg_len:
                    self.state = __class__.State.crc1

            elif self.state == __class__.State.crc1:
                self.msg_cka = d
                self.state = __class__.State.crc2

            elif self.state == __class__.State.crc2:
                self.msg_ckb = d

                self.process_message()

                self.msg_reset()
                self.state = __class__.State.init

    def process_message(self):
        # Build frame from received data.
        # if checksum is ok forward message to
        # parser for further dissection

        if self.cksum.ok(self.msg_cka, self.msg_ckb):
            cls_id = ClsMsgId(self.msg_class, self.msg_id)
            # frame = UbxFrame.from_data(cls_id, self.msg_data, self.msg_cka, self.msg_ckb)

            # TODO: Create message factory
            # if frame.matches(UbxMessage.ACK_ACK):
            if cls_id == UbxMessage.ACK_ACK.value:
                response = UbxAckAck(self.msg_data)
            # elif frame.matches(UbxMessage.MON_VER):
            elif cls_id == UbxMessage.MON_VER.value:
                response = UbxMonVer(self.msg_data)
            # elif frame.matches(UbxMessage.UPD_SOS):
            elif cls_id == UbxMessage.UPD_SOS.value:
                response = UbxUpdSos(self.msg_data)
            else:
                # If we can't parse the frame, return as is
                # response = frame
                response = None

            if response:
                self.queue.put(response)

        else:
            logger.warning(f'checksum error in frame')

    def msg_reset(self):
        """
        Resets message buffer information
        """
        self.msg_class = 0
        self.msg_id = 0
        self.msg_len = 0
        self.msg_data = bytearray()
        self.msg_cka = 0
        self.msg_ckb = 0


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
        self.timeout = 2.0              # default timeout

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

    def version(self):
        request = UbxPoll(UbxMessage.MON_VER)
        res = self.poll(request)
        return res

    def sos_state(self):
        logger.debug('checking SOS restore state')
        msg_upd_sos_poll = UbxPoll(UbxMessage.UPD_SOS)
        res = self.poll(msg_upd_sos_poll)
        if res:
            return res.response

    def sos_create_backup(self):
        logger.debug('creating state backup file')

        msg = UbxUpdSosAction(0)
        # TODO: Encapsulation -> command()
        # retry mechanism
        self.expect(UbxMessage.ACK_ACK.value)
        self.send(msg)
        res = self.wait()
        return res

    def sos_clear_backup(self):
        logger.debug('clearing state backup file')

        msg = UbxUpdSosAction(1)
        # print(msg.to_bytes())
        self.expect(UbxMessage.ACK_ACK.value)
        self.send(msg)
        res = self.wait()
        return res

    # Low Level API

#    def register_response(cls_msg_id):
#        pass

    def poll(self, message):
        """
        Polls a receiver status

        - sends the specified poll message
        - waits for receiver message with same class/id as poll message
        """
        assert isinstance(message, UbxFrame)
        assert message.length == 0      # poll message have no payload

        self.expect(message.cls_msg_id)
        # TODO: retry mechanism ?
        for i in range(3):
            self.send(message)
            res = self.wait(timeout=self.timeout)
            if res:
                return res

            logger.warning('no response, retrying')

    def expect(self, cls_msg_id):
        """
        Defines message message to wait for
        """
        logger.debug(f'preparing to wait for {cls_msg_id}')
        self.wait_cls_msg_id = cls_msg_id

    def list_devices(self):
        """
        """
        try:
            self.control_sock = socket.socket(socket.AF_UNIX,
                                              socket.SOCK_STREAM)
            self.control_sock.connect(GnssUBlox.gpsd_control_socket)

            self.control_sock.sendall('?devices'.encode())
            data = self.control_sock.recv(512)

            # TODO: don't print, but return list of devices
            print(data.decode())

            self.control_sock.close()

        except socket.error as msg_in_ascii:
            logger.error(msg_in_ascii)

    def send(self, message):
        """
        Sends binary message to GNSS modem

        - opens connection to gpsd control socket (not data socket)
        - creates control message in gpsd format (ASCII format)
            "&/dev/ttyS3=b56201011234..."
        - sends message and checks gpsd control daemon response
        """
        try:
            self.control_sock = socket.socket(socket.AF_UNIX,
                                              socket.SOCK_STREAM)
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
                if isinstance(res, UbxResponse):
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

            # Operate blocking socket with timeout so we can stop
            # thread by signalling thread_stop_event
            self.listen_sock.settimeout(0.25)

            logger.debug('receiver ready')
            self.thread_ready_event.set()

            while not self.thread_stop_event.is_set():
                try:
                    data = self.listen_sock.recv(8192*4)
                    if data:
                        self.parser.process(data)

                except socket.timeout:
                    pass

        except socket.error as msg:
            logger.error(msg)

        logger.debug('receiver done')


def main():
    r = GnssUBlox('/dev/ttyS3')
    r.setup()
    r.list_devices()

    r.timeout = 0.2
    version = r.version()
    print(version)
    print()

    r.timeout = 2.0
    version = r.version()
    print(version)
    print()

    # r.sos_create_backup()
    # res = r.sos_clear_backup()
    # print(res)

    # r.sos_clear_backup()

    response = r.sos_state()
    if response:
        print(f'>> SOS state is {response}')

    for i in range(0, 10000):
        print(f'***** {i} ***********************')
        response = r.sos_state()
        if response:
            print(f'>> SOS state is {response}')
        else:
            print('>> no reponse')
            assert False

        time.sleep(0.87)

    r.cleanup()


if __name__ == "__main__":
    main()
