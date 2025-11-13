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

        self.send_commang_to_gauge("PNVGINS,DEBUG,1")

        self.serial_port = serial_port or ""
        self.baud = int(baud)
        self.msg_delay = float(msg_delay)

        # общий буфер байт (bytearray) и защита
        self._buffer = bytearray()
        self._buf_lock = threading.Lock()

        # очередь уже разобранных строк (str)
        self._line_q = queue.Queue()

        # message port
        self.message_port_register_out(pmt.intern("msg_out"))

        # контроль работы
        self._running = True

        # старт парсерного потока
        self._parser_thread = threading.Thread(target=self._parse_queue_loop, daemon=True)
        self._parser_thread.start()

        # если serial_port указан — запустим читатель
        self._ser = None
        self._reader_thread = None
        if self.serial_port:
            if pyserial is None:
                print("[PARSER] pyserial not installed; cannot open serial port")
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
                    print(f"[PARSER] opened serial {self.serial_port} @ {self.baud}")
                except Exception as e:
                    print("[PARSER] Failed to open serial port:", e)
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
                    # ничего не прочитали — даём процессору немножко отдохнуть
                    time.sleep(0.01)
            except Exception as e:
                print("[PARSER] serial read error:", e)
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
            # вернуть число обработанных входных элементов
            return len(input_items[0])
        else:
            # если читаем напрямую из serial — ничего не делаем
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
                    # в буфере нет начала предложения → чистим хвост
                    self._buffer.clear()
                    return

                end = self._buffer.find(b'\r\n', start)
                if end == -1:
                    # нет конца предложения, ждём прихода новых байт
                    return

                # вырезаем полное предложение ($...<CR><LF>)
                raw = bytes(self._buffer[start:end + 2])
                # убираем использованную часть из буфера
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
        # пример: текущий record по timestamp
        current_ts = None
        current_record = {}

        while self._running:
            try:
                line = self._line_q.get(timeout=0.1)
            except queue.Empty:
                continue

            # парсинг строки
            try:
                # специальная строка от IMU (твоя собственная)
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
                    # стандартный NMEA парсинг
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
                        # другие типы — можно расширить
                        d = {}
                # если d пуст — пропускаем
                if not d:
                    continue

                # --- логика объединения записей по timestamp ---
                ts = d.get('timestamp')
                if ts is None:
                    # если нет времени — публикуем немедленно
                    self._publish_dict(d)
                else:
                    # простая логика: если timestamp сменился — публикуем текущий record
                    if current_ts is None:
                        current_ts = ts
                        current_record = {}
                    if ts != current_ts:
                        # публикация собранного
                        self._publish_record(current_record)
                        current_record = {}
                        current_ts = ts
                    current_record.update(d)
                    # в твоей логике можно публиковать сразу при сборе
                    # но здесь мы публикуем при смене timestamp
                # optional delay between publishes
                if self.msg_delay:
                    time.sleep(self.msg_delay)

            except pynmea2.ParseError:
                # игнорируем багнутые NMEA строки
                continue
            except Exception as e:
                print("[PARSER] parse error:", e)
                continue

        # при выходе: публикуем последний record
        if current_record:
            self._publish_record(current_record)


    # -------------------------
    # publish helpers
    # -------------------------
    def _publish_record(self, record):
        if not record:
            return
        # можно преобразовать time -> unix ms, но здесь оставим как есть
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
            # если v уже PMT — не конвертируем
            if hasattr(v, 'to_pmt') or isinstance(v, pmt.pmt_python.pmt_base):
                val_pmt = v
            else:
                val_pmt = pmt.to_pmt(v)
            pmt_dict = pmt.dict_add(pmt_dict, pmt.intern(str(k)), val_pmt)
        try:
            self.message_port_pub(pmt.intern('msg_out'), pmt_dict)
        except Exception as e:
            print("[PARSER] message_port_pub error:", e)


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
        # закрываем serial (если открыт)
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
        # join threads
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

        with serial.Serial(self.port, self.baud, timeout=1) as ser:
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


