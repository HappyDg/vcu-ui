"""
GNSS Page
"""
import logging

import tornado.web

from vcuui._version import __version__ as version
from vcuui.data_model import Model
from vcuui.gnss_model import Gnss

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


class GnssHandler(tornado.web.RequestHandler):
    def get(self):
        self.build_gnss()

    def build_gnss(self):
        logger.info('rendering page')

        try:
            tes = list()
            data = dict()

            # General System Information
            m = Model.instance
            md = m.get_all()

            serial = md['sys-version']['serial']

            gnss = Gnss.instance
            gnss.invalidate()

            # GNSS Version

            ver = gnss.version()
            tes.append(TE('Version', ''))
            tes.append(TE('HW', ver['hwVersion']))
            tes.append(TE('SW', ver['swVersion']))
            tes.append(TE('FW', ver['fwVersion']))
            tes.append(TE('Protocol', ver['protocol']))
            tes.append(TE('', ''))

            # GNSS Status (live)
            if 'gnss-pos' in md:
                # logger.debug("build_gnss 2")
                tes.append(TE('Status', ''))

                pos = md['gnss-pos']
                tes.append(TE('Fix', pos['fix']))
                text = f'Longitude: {pos["lon"]:.9f}, Latitude: {pos["lat"]:.9f}'
                tes.append(TE('Position', text))
                text = nice([('speed', '', 'km/h')], pos)
                tes.append(TE('Speed', f'{pos["speed"]:.0f} m/s, {pos["speed"]*3.60:.0f} km/h'))
                # tes.append(TE('', ''))

            # Config
            # logger.debug("build_gnss 3")
            data['dyn_model'] = str(gnss.dynamic_model())

            # logger.debug("build_gnss 4")
            data['nmea_protocol'] = gnss.nmea_protocol()

            # logger.debug("build_gnss 5")
            uart_cfg = gnss.uart_settings()
            data['uart_settings'] = f'{uart_cfg["bitrate"]} bps, {uart_cfg["mode"]}'

            # logger.debug("build_gnss 6")
            align = gnss.auto_align()
            data['imu_auto_align'] = 'On' if align else 'Off'

            logger.debug("build_gnss 7")
            align_state = gnss.auto_align_state()
            data['imu_auto_align_state'] = align_state

            logger.debug("build_gnss 8")
            cfg_angles = gnss.imu_cfg_angles()
            data['imu_cfg_roll'] = str(round(cfg_angles['roll']))
            data['imu_cfg_pitch'] = str(round(cfg_angles['pitch']))
            data['imu_cfg_yaw'] = str(round(cfg_angles['yaw']))

            vrp_ant = gnss.vrp_ant()
            data['vrp_ant_x'] = str(vrp_ant['x'])
            data['vrp_ant_y'] = str(vrp_ant['y'])
            data['vrp_ant_z'] = str(vrp_ant['z'])

            vrp_imu = gnss.vrp_imu()
            data['vrp_imu_x'] = str(vrp_imu['x'])
            data['vrp_imu_y'] = str(vrp_imu['y'])
            data['vrp_imu_z'] = str(vrp_imu['z'])

            # logger.debug("build_gnss 9")
            roll, pitch, yaw = gnss.auto_align_angles()
            angles_str = f'roll: {roll}°, pitch: {pitch}°, yaw: {yaw}°'
            data['imu_angles'] = angles_str

            # logger.debug("build_gnss 10")
            esf_status = gnss.esf_status()
            text = nice([('fusion', 'Fusion', ''),
                        ('ins', 'INS', ''),
                        ('imu', 'IMU', ''),
                        ('imu-align', 'IMU Alignment', '')],
                        esf_status)
            data['esf_status'] = text
            logger.debug(data)

            self.render('gnss.html',
                        title=f'{serial}',
                        table=tes,
                        data=data,
                        message='',
                        version=version)

        except KeyError as e:
            self.render('gnss.html',
                        title='NG800/VCU Pro',
                        message=f'Data lookup error: {e} not found',
                        table=None,
                        data=None,
                        version='n/a')
