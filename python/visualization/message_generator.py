# -*- coding: utf-8 -*-
#
# Copyright 2025 egogoboy.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

import numpy as np
import time
from gnuradio import gr
from datetime import date, datetime
import math
import threading
import pmt

class message_generator(gr.sync_block):
    """
    Message generator блок
    Выход: поток байт
    """
    def __init__(self, msg_frequency=10.0):
        gr.sync_block.__init__(
            self,
            name="Message generator",
            in_sig=[],
            out_sig=[]
        )

        self.message_port_register_out(pmt.intern("out"))

        self.period = 0.01 / msg_frequency

        self.lat0 = 58.135
        self.lon0 = 39.600
        self.alt0 = 250.0
        self.speed_kmh = 50.0
        self.step_sec = 0.2
        
        self.iterator = 0
        self.t = 0.2
        self.t0 = int(datetime.now().timestamp() * 1000)

        self.past_time = self.t0 - 1000
        self.past_lat = self.lat0
        self.past_lon = self.lon0

        # Флаг для остановки потока
        self._running = True

        # Запускаем поток-генератор
        self.thread = threading.Thread(target=self._run)
        self.thread.daemon = True
        self.thread.start()

    def _run(self):
        while self._running:
            self._tick()
            time.sleep(self.period)


    def stop(self):
        self._running = False
        self.thread.join()
        return super().stop()


    def _tick(self):
        self.iterator += 1

        d = {}

        cur_time = self.t0 + self.iterator*1000

        lat = 58.135 + 0.3 * math.sin(self.t)

        lon = 39.600 + 0.3 * math.sin(self.t) * math.cos(self.t)

        alt = self.alt0 + (20 * math.sin(self.t / 3))

        distance_km = self.haversine(self.past_lat, self.past_lon, lat, lon)
        delta_t = (cur_time - self.past_time) / 1000
        speed_mps = (distance_km * 1000) / delta_t
        speed = speed_mps * 1.94384  # knots

        cog = self.get_cog(self.past_lat, lat, self.past_lon, lon)

        pitch = -10 * math.cos(2 * self.t)
        roll = -20 * math.sin(self.t)

        dx = 0.01 * math.cos(self.t) * math.cos(self.t) - 0.01 * math.sin(self.t) * math.sin(self.t)
        dy = 0.01 * math.cos(self.t)
        yaw = (math.degrees(math.atan2(dx, dy)) + 360) % 360

        vx = speed * math.sin(yaw)
        vy = speed * math.cos(yaw)

        d = {
            'timestamp': cur_time,
            'lat': lat,
            'lon': lon,
            'alt': alt,
            'sog': speed,
            'cog': cog,
            'pitch': pitch,
            'roll': roll,
            'yaw': yaw,
            'vx': vx,
            'vy': vy,
            'valid': 1
        }

        self._publish_dict(d)

        self.past_time = cur_time
        self.past_lat = lat
        self.past_lon = lon

        self.t += 0.0016


    def _publish_dict(self, d):
        pmt_dict = pmt.make_dict()
        for k, v in d.items():
            if v is None:
                continue
            if hasattr(v, 'to_pmt') or isinstance(v, pmt.pmt_python.pmt_base):
                val_pmt = v
            else:
                val_pmt = pmt.to_pmt(v)
            pmt_dict = pmt.dict_add(pmt_dict, pmt.intern(str(k)), val_pmt)
        try:
            self.message_port_pub(pmt.intern('out'), pmt_dict)
        except Exception as e:
            print("[PARSER] message_port_pub error:", e)


    def build_record(self, record: dict) -> bytes:
        if not record:
            return b""

        unix_time = int(time.time() * 1000)

        lat = record.get('lat', 0.0)
        lon = record.get('lon', 0.0)

        ns = "N" if lat >= 0 else "S"
        ew = "E" if lon >= 0 else "W"

        lat_dm = self.deg2dm(lat, "lat")
        lon_dm = self.deg2dm(lon, "lon")

        fields = [
            record.get('timestamp', 0),
            lat_dm,
            ns,
            lon_dm,
            ew,
            record.get('alt', 0),
            record.get('sog', 0),
            record.get('cog', 0),
            record.get('vx', 0),
            record.get('vy', 0),
            record.get('pitch', 0),
            record.get('roll', 0),
            record.get('yaw', 0),
            record.get('valid', 0)
        ]

        record_str = ",".join(map(str, fields)) + "\n"

        return record_str.encode("utf-8")


    def deg2dm(self, deg: float, latlon: str) -> str:
        d = int(abs(deg))
        m = (abs(deg) - d) * 60
        if latlon == "lat":
            return f"{d:02d}{m:07.4f}"
        else:
            return f"{d:03d}{m:07.4f}"


    def haversine(self, lat1, lon1, lat2, lon2):
        R = 6371.0

        phi1 = np.radians(lat1)
        phi2 = np.radians(lat2)
        d_phi = np.radians(lat2 - lat1)
        d_lambda = np.radians(lon2 - lon1)

        a = np.sin(d_phi / 2.0)**2 + np.cos(phi1) * np.cos(phi2) * np.sin(d_lambda / 2.0)**2
        c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))

        return R * c


    def normalize_deg(self, angle):
        """Нормализация в [0,360)."""
        return (angle % 360 + 360) % 360


    def get_cog(self, lat1, lat2, lon1, lon2):
        if lat1 == lat2 and lon1 == lon2:
            return None

        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dlam = math.radians(lon2 - lon1)

        x = math.sin(dlam) * math.cos(phi2)
        y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlam)

        bearing_rad = math.atan2(x, y)
        bearing_deg = math.degrees(bearing_rad)
        return self.normalize_deg(bearing_deg)

