import logging
import threading
import time

from ping3 import ping
from vcuui.led import LED_BiColor
from vcuui.mm import MM
from vcuui.sysinfo import SysInfo

logger = logging.getLogger('vcu-ui')


class Model(object):
    # Singleton accessor
    instance = None

    def __init__(self):
        super().__init__()

        assert Model.instance is None
        Model.instance = self

        self.worker = ModelWorker(self)
        self.gsm_connection = GsmWorker(self)
        self.lock = threading.Lock()
        self.data = dict()

        self.led_ind = LED_BiColor('/sys/class/leds/ind')
        self.led_stat = LED_BiColor('/sys/class/leds/status')
        self.cnt = 0

    def setup(self):
        self.led_stat.green()
        self.led_ind.green()

        self.worker.setup()
        self.gsm_connection.setup()

    def get_all(self):
        with self.lock:
            return self.data

    def get(self, origin):
        with self.lock:
            if origin in self.data:
                return self.data[origin]

    def publish(self, origin, value):
        """
        Report event (with data) to data model

        Safe to be called from any thread
        """

        # logger.debug(f'get data from {origin}')
        # logger.debug(f'values {value}')
        with self.lock:
            self.data[origin] = value

            if origin == 'things':
                if value['state'] == 'sending':
                    self.led_ind.yellow()
                else:
                    self.led_ind.green()


class ModelWorker(threading.Thread):
    def __init__(self, model):
        super().__init__()

        self.model = model

    def setup(self):
        self.lock = threading.Lock()
        self.daemon = True
        self.name = 'model-worker'
        self.start()

    def run(self):
        self._sysinfo()
        self._network()
        self._modem()

        cnt = 0
        while True:
            self._sysinfo()

            if cnt % 4 == 2:
                self._network()

            if cnt % 10 == 5:
                self._modem()

            cnt += 1
            time.sleep(1.0)

    def _sysinfo(self):
        si = SysInfo()

        ver = dict()
        ver['serial'] = si.serial()
        ver['sys'] = si.version()
        ver['bl'] = si.bootloader_version()
        ver['hw'] = si.hw_version()
        self.model.publish('sys-version', ver)

        dt = dict()
        dt['date'] = si.date()
        dt['uptime'] = si.uptime()
        self.model.publish('sys-datetime', dt)

        info = dict()
        info['mem'] = si.meminfo()
        info['load'] = si.load()
        info['temp'] = si.temperature()
        info['v_in'] = si.input_voltage()
        info['v_rtc'] = si.rtc_voltage()
        self.model.publish('sys-misc', info)

    def _network(self):
        si = SysInfo()

        info = dict()
        info['bytes'] = si.ifinfo('wwan0')
        self.model.publish('net-wwan0', info)

    def _modem(self):
        info = dict()
        m = MM.modem()
        if m:
            info['modem-id'] = str(m.id)

            state = m.state()
            access_tech = m.access_tech()
            info['state'] = state
            info['access-tech'] = access_tech

            loc_info = m.location()
            if loc_info['mcc']:
                info['location'] = loc_info

            sq = m.signal()
            info['signal-quality'] = sq

            if access_tech == 'lte':
                sig = m.signal_lte()
                info['signal-lte'] = sig
            elif access_tech == 'umts':
                sig = m.signal_umts()
                info['signal-umts'] = sig

            b = m.bearer()
            if b:
                info['bearer-id'] = str(b.id)
                ut = b.uptime()
                if ut:
                    info['bearer-uptime'] = ut
                    ip = b.ip()
                    info['bearer-ip'] = ip

        self.model.publish('modem', info)


class GsmWorker(threading.Thread):
    # Singleton accessor
    # TODO: required?
    instance = None

    def __init__(self, model):
        super().__init__()

        assert GsmWorker.instance is None
        GsmWorker.instance = self

        self.model = model
        self.state = 'init'
        self.counter = 0

    def setup(self):
        self.daemon = True
        self.name = 'gsm-worker'
        self.start()

    def run(self):
        logger.info("running gsm thread")
        self.state = 'init'
        self.counter = 0
        link_data = dict()

        while True:
            info = self.model.get('modem')
            if self.state == 'init':
                # check if we have a valid bearer
                try:
                    if info and 'bearer-ip' in info:
                        logger.info('bearer found')
                        self.state = 'connected'
                except KeyError:
                    pass

            elif self.state == 'connected':
                try:
                    if info and 'bearer-ip' not in info:
                        logger.warning('lost IP connection')

                        link_data['delay'] = 0.0
                        self.model.publish('link', link_data)
                        self.state = 'init'
                    else:
                        if self.counter % 5 == 2:
                            try:
                                delay = ping('1.1.1.1', timeout=1.0)
                                if delay:
                                    link_data['delay'] = round(float(delay), 3)
                                else:
                                    link_data['delay'] = 0.0

                                self.model.publish('link', link_data)

                            except OSError as e:
                                logger.warning('Captured ping error')
                                logger.warning(e)

                                link_data['delay'] = 0.0
                                self.model.publish('link', link_data)
                                self.state = 'init'
                except KeyError:
                    pass

            self.counter += 1
            time.sleep(1.0)
