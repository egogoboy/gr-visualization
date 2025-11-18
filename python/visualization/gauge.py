# -*- coding: utf-8 -*-
#
# Copyright 2025 egogoboy.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

import time
import threading
import queue
import pmt
import pynmea2
import numpy as np
import serial
import time
from gnuradio import gr

try:
    import serial as pyserial
except Exception:
    pyserial = None

class gauge(gr.sync_block):
    """
    NMEA Parser block.

    Параметры конструктора:
        serial_port: путь к последовательному устройству, например "/dev/ttyUSB0".
                     Если пустая строка, блок читает байты из stream input (in_sig).
        baud: baudrate для serial_port (int)
        msg_delay: необязательная пауза между публикациями (сек)
    Входы:
        - поток байт (np.uint8) — используется только если serial_port == "".
    Выходы:
        - message port 'msg_out' — pmt dictionary с ключами:
          timestamp, lat, lon, alt, sog, cog, vx, vy, pitch, roll, yaw, valid
    """
    def __init__(self, serial_port: str = "", baud: int = 115200, msg_delay: float = 0.0):
        gr.sync_block.__init__(self,
                               name="NMEA Parser",
                               in_sig=[],
                               out_sig=None)

        self.serial_port = serial_port or ""
        self.baud = int(baud)

        self.send_commang_to_gauge("PNVGINS,DEBUG,1")

        self.msg_delay = float(msg_delay)

        self._buffer = bytearray()
        self._buf_lock = threading.Lock()

        self._line_q = queue.Queue()

        self.message_port_register_out(pmt.intern("msg_out"))

        self._running = True

        self._parser_thread = threading.Thread(target=self._parse_queue_loop, daemon=True)
        self._parser_thread.start()

        self._ser = None
        self._reader_thread = None
        if self.serial_port:
            if pyserial is None:
                print("[Gauge block] pyserial not installed; cannot open serial port")
            else:
                try:
                    self._ser = pyserial.Serial(self.serial_port,
                                                self.baud,
                                                timeout=0.5,
                                                bytesize=pyserial.EIGHTBITS,
                                                parity=pyserial.PARITY_NONE,
                                                stopbits=pyserial.STOPBITS_ONE)
                    self._reader_thread = threading.Thread(target=self._serial_reader_loop, daemon=True)
                    self._reader_thread.start()
                    print(f"[Gauge block] opened serial {self.serial_port} @ {self.baud}")
                except Exception as e:
                    print("[Gauge block] Failed to open serial port:", e)
                    self._ser = None


    # -------------------------
    # serial reader thread
    # -------------------------
    def _serial_reader_loop(self):
        """
        Читает байты из self._ser и добавляет в буфер.
        Вычленение целых строк выполняется тут же.
        """
        while self._running and self._ser:
            try:
                data = self._ser.read(1024)  # read up to 1024 bytes, blocking up to timeout

                if data:
                    with self._buf_lock:
                        self._buffer.extend(data)
                    self._extract_lines_to_queue()
                else:
                    time.sleep(0.01)

            except Exception as e:
                print("[Gauge block] serial read error:", e)
                time.sleep(0.5)


    # -------------------------
    # stream input via work()
    # -------------------------
    def work(self, input_items, output_items):
        """
        Если serial_port не указан, сюда прилетают байты из upstream (file_source/throttle и т.д.).
        Мы аккумулируем и разбираем строки так же, как при чтении из serial.
        """
        if not self.serial_port:
            data = bytes(input_items[0])

            if data:
                with self._buf_lock:
                    self._buffer.extend(data)
                self._extract_lines_to_queue()

            return len(input_items[0])
        else:
            return 0


    # -------------------------
    # helper: извлечь NMEA-предложения из буфера и поместить в очередь
    # -------------------------
    def _extract_lines_to_queue(self):
        """
        Из буфера достаём последовательности от '$' до '\r\n'.
        Каждое полное NMEA-предложение кладём в очередь.
        """
        while True:
            with self._buf_lock:
                start = self._buffer.find(b'$')

                if start == -1:
                    self._buffer.clear()
                    return

                end = self._buffer.find(b'\r\n', start)

                if end == -1:
                    return

                raw = bytes(self._buffer[start:end + 2])
                del self._buffer[:end + 2]

            # decode safe
            try:
                s = raw.decode('ascii', errors='ignore').strip()
            except Exception:
                s = raw.decode('utf-8', errors='ignore').strip()

            if s.startswith('$'):
                self._line_q.put(s)


    # -------------------------
    # parser loop
    # -------------------------
    def _parse_queue_loop(self):
        """
        Получаем строки из очереди и собираем record'ы,
        (объединение GGA/RMC/PNVGIMU по timestamp и публикация при смене timestamp).
        Упрощённая версия: при получении строки формируем dict d и публикуем сразу.
        (Можем расширить логику объединения — см. дальше).
        """
        current_ts = None
        current_record = {}

        while self._running:
            try:
                line = self._line_q.get(timeout=0.1)
            except queue.Empty:
                continue

            # парсинг строки
            try:
                if line.startswith('$PNVGIMU'):
                    fields = line.split(',')
                    d = {}

                    if len(fields) >= 4:
                        d['timestamp'] = fields[1]
                        d['roll'] = self._to_float_safe(fields[2])
                        d['pitch'] = self._to_float_safe(fields[3])

                    if len(fields) >= 5:
                        d['yaw'] = self._to_float_safe(fields[4])

                    if len(fields) >= 8:
                        d['ax'] = self._to_float_safe(fields[5])
                        d['ay'] = self._to_float_safe(fields[6])
                        d['az'] = self._to_float_safe(fields[7].split('*')[0])
                else:
                    msg = pynmea2.parse(line)

                    if isinstance(msg, pynmea2.types.talker.GGA):
                        d = {
                            'timestamp': str(msg.timestamp),
                            'lat': msg.latitude,
                            'lon': msg.longitude,
                            'alt': self._to_float_safe(msg.altitude),
                            'fix': int(msg.gps_qual) if msg.gps_qual is not None else None
                        }
                    elif isinstance(msg, pynmea2.types.talker.RMC):
                        d = {
                            'timestamp': str(msg.timestamp),
                            'lat': msg.latitude,
                            'lon': msg.longitude,
                            'speed': self._to_float_safe(msg.spd_over_grnd),
                            'course': self._to_float_safe(msg.true_course),
                            'valid': 1 if msg.status == 'A' else 0
                        }
                    else:
                        d = {}

                if not d:
                    continue

                ts = d.get('timestamp')
                if ts is None:
                    self._publish_dict(d)
                else:
                    if current_ts is None:
                        current_ts = ts
                        current_record = {}

                    if ts != current_ts:
                        self._publish_record(current_record)
                        current_record = {}
                        current_ts = ts

                    current_record.update(d)

                if self.msg_delay:
                    time.sleep(self.msg_delay)

            except pynmea2.ParseError:
                continue
            except Exception as e:
                print("[Gauge block] parse error:", e)
                continue

        if current_record:
            self._publish_record(current_record)


    # -------------------------
    # publish helpers
    # -------------------------
    def _publish_record(self, record):
        if not record:
            return

        unix_time = int(time.time() * 1000)
        msg = {
            'timestamp': unix_time,
            'lat': record.get('lat'),
            'lon': record.get('lon'),
            'alt': record.get('alt'),
            'sog': record.get('speed'),
            'cog': record.get('course'),
            'vx': record.get('vx'),
            'vy': record.get('vy'),
            'pitch': record.get('pitch'),
            'roll': record.get('roll'),
            'yaw': record.get('yaw'),
            'valid': record.get('valid', 0)
        }

        self._publish_dict(msg)


    def _publish_dict(self, d):
        # build pmt dict
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
            self.message_port_pub(pmt.intern('msg_out'), pmt_dict)
        except Exception as e:
            print("[Gauge block] message_port_pub error:", e)


    # -------------------------
    # utils
    # -------------------------
    @staticmethod
    def _to_float_safe(x):
        try:
            return float(x)
        except Exception:
            return None


    # -------------------------
    # stop/cleanup
    # -------------------------
    def stop(self):
        self._running = False
        try:
            if getattr(self, "_ser", None):
                try:
                    self._ser.cancel_read()
                except Exception:
                    pass

                try:
                    self._ser.close()
                except Exception:
                    pass

        except Exception:
            pass

        try:
            if getattr(self, "_reader_thread", None):
                self._reader_thread.join(timeout=1.0)
        except Exception:
            pass

        try:
            if getattr(self, "_parser_thread", None):
                self._parser_thread.join(timeout=1.0)
        except Exception:
            pass

        self.send_commang_to_gauge("PNVGINS,DEBUG,0")

        return super().stop()


    def send_commang_to_gauge(self, payload):
        cs = self.nmea_checksum(payload)

        command = f"${payload}*{cs}\r\n"

        with serial.Serial(self.serial_port, self.baud, timeout=1) as ser:
            ser.write(command.encode())

        print(f'[Gauge Block] sent command {command} to {self.serial_port}')


    # -------------------------
    # nmea checksum for gauge
    # -------------------------
    def nmea_checksum(self, sentence: str) -> str:
        checksum = 0
        for c in sentence:
            checksum ^= ord(c)
        return f"{checksum:02X}"


