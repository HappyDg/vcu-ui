"""
Main Page (Single Page)
"""
import logging

import tornado.web

from vcuui._version import __version__ as version
from vcuui.data_model import Model
from vcuui.tools import secs_to_hhmm

logger = logging.getLogger('vcu-ui')


class TE(object):
    """
    Table Element to be displayed in info.tpl
    """
    def __init__(self, header, text):
        self.header = header
        self.text = text


def nice(items, data, linebreak=False):
    res = ''
    for i in items:
        key, header, unit = i
        val = data[key]
        if res != '':
            if linebreak:
                res += '</br>'
            else:
                res += ', '

        res += f'{header}: {val}'
        if unit != '':
            res += f' {unit}'

    return res


class MainHandler(tornado.web.RequestHandler):
    def get(self):
        self.render_page()

    def render_page(self, message=None, console=None):
        logger.info('rendering page')

        try:
            tes = list()
            data = dict()

            # General System Information
            m = Model.instance
            md = m.get_all()

            cloud_log_state = md['cloud']

            serial = md['sys-version']['serial']

            tes.append(TE('System', ''))
            text = nice([('sys', 'System', ''),
                        ('bl', 'Bootloader', ''),
                        ('hw', 'Hardware', '')],
                        md['sys-version'], True)
            tes.append(TE('Version', text))

            dt = md['sys-datetime']['date']
            tes.append(TE('Date', dt))

            ut = md['sys-datetime']['uptime']
            tes.append(TE('Uptime', ut))

            total, free = md['sys-misc']['mem']
            tes.append(TE('Memory', f'Total: {total} kB<br>Free: {free} kB'))

            a, b, c = md['sys-misc']['load']
            tes.append(TE('Load', f'{a}, {b}, {c}'))

            temp = md['sys-misc']['temp']
            tes.append(TE('Temperature', f'{temp:.0f} °C'))

            v_in = md['sys-misc']['v_in']
            v_rtc = md['sys-misc']['v_rtc']
            tes.append(TE('Voltages', f'Input: {v_in:.1f} V, RTC: {v_rtc:.2f} V'))

            # Network Information
            tes.append(TE('', ''))
            tes.append(TE('Network', ''))

            rx, tx = md['net-wwan0']['bytes']
            if rx and tx:
                rx = int(rx) / 1000000
                tx = int(tx) / 1000000
                tes.append(TE('wwan0', f'Rx: {rx:.1f} MB<br>Tx: {tx:.1f} MB'))

            # Modem Information
            mi = md['modem']
            if 'modem-id' in mi:
                tes.append(TE('', ''))
                tes.append(TE('Mobile', ''))

                tes.append(TE('Modem Id', mi['modem-id']))

                state = mi['state']
                access_tech = mi['access-tech']
                tes.append(TE('State', f'{state}, {access_tech}'))

                if 'location' in mi:
                    loc_info = mi['location']
                    if loc_info['mcc']:
                        text = nice([('mcc', 'MCC', ''),
                                    ('mnc', 'MNC', ''),
                                    ('lac', 'LAC', ''),
                                    ('cid', 'CID', '')],
                                    loc_info)
                        tes.append(TE('Cell', text))
                        data.update(loc_info)

                sq = mi['signal-quality']
                tes.append(TE('Signal', f'{sq} %'))

                if access_tech == 'lte':
                    sig = mi['signal-lte']
                    text = nice([('rsrp', 'RSRP', 'dBm'),
                                ('rsrq', 'RSRQ', 'dB')],
                                sig, True)
                    tes.append(TE('Signal LTE', text))
                elif access_tech == 'umts':
                    sig = mi['signal-umts']
                    text = nice([('rscp', 'RSRP', 'dBm'),
                                ('ecio', 'ECIO', 'dB')],
                                sig, True)
                    tes.append(TE('Signal UMTS', text))

                if 'bearer-id' in mi:
                    tes.append(TE('', ''))
                    tes.append(TE('Bearer Id', mi['bearer-id']))

                    if 'bearer-uptime' in mi:
                        ut = mi['bearer-uptime']
                        if ut:
                            uth, utm = secs_to_hhmm(ut)
                            tes.append(TE('Uptime', f'{uth}:{utm:02} h'))
                            ip = mi['bearer-ip']
                            tes.append(TE('IP', ip))

                    if 'link' in md:
                        if 'delay' in md['link']:
                            delay_in_ms = md['link']['delay'] * 1000.0
                            tes.append(TE('Ping', f'{delay_in_ms:.0f} ms'))

            else:
                tes.append(TE('', ''))
                tes.append(TE('Modem Id', 'No Modem'))

            # GNSS
            if 'gnss-pos' in md:
                tes.append(TE('', ''))
                tes.append(TE('GNSS', ''))

                pos = md['gnss-pos']
                tes.append(TE('Fix', pos['fix']))
                text = f'Longitude: {pos["lon"]:.9f}, Latitude: {pos["lat"]:.9f}'
                tes.append(TE('Position', text))
                text = nice([('speed', '', 'km/h')], pos)
                tes.append(TE('Speed', f'{pos["speed"]:.0f} m/s, {pos["speed"]*3.60:.0f} km/h'))

            self.render('main.html',
                        title=f'VCU Pro ({serial})',
                        table=tes,
                        data=data,
                        message=message,
                        console=console,
                        version=version,
                        cloud_log=cloud_log_state)

        except KeyError as e:
            logger.warning(f'lookup error {e}')
            self.render('main.html',
                        title='VCU Pro',
                        message=f'Data lookup error: {e} not found',
                        table=None,
                        data=None,
                        console=None,
                        version='n/a',
                        cloud_log=False)
