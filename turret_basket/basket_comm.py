"""
================================================================================
BASKET CHALLENGE: STM32 SERI PORT SURUCUSU (FCS COMMUNICATOR)
================================================================================
PC ile STM32F401RE arasindaki 100 Hz ikili (binary) iletisim katmani.
- ST-Link Sanal COM Portunu otomatik tanir.
- Pan ve Tilt servo komutlarini (0.1 derece cozunurluk) gonderir.
- 80 ms solenoid firlatma darbesini (STATE_FIRING) tetikler.
- TF-Luna LiDAR mesafe telemetrisini ve guncel servo acilarini okur.
================================================================================
"""

import struct
import threading
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None

from turret_basket.config import CONFIG
# Protokol Sabitleri (turret_basket.protocol - Tek Kaynak SSOT)
from turret_basket.protocol import (
    PC_TO_STM_HEADER,
    STM_TO_PC_HEADER,
    TLM_FRAME_LEN,
    ANGLE_SCALE,
    ANGLE_OFFSET_DEG,
    CMD_FORMAT,
    TLM_FORMAT,
    STATE_TRACKING,
    STATE_FIRING,
)


_ZERO_CMD_EPS = 1e-9


class FCSCommunicator:
    """STM32 mikrodenetleyici ile ikili paket uzerinden konusan iletisim sinifi."""

    def __init__(self, port=None, baudrate=None, reconnect_interval=2.0, stale_timeout=0.3):
        self.port = port
        self.baudrate = CONFIG.hardware.baudrate if baudrate is None else baudrate
        self.reconnect_interval = reconnect_interval
        self.stale_timeout = stale_timeout

        self.last_reconnect_attempt = 0.0
        self.ser = None
        self._lock = threading.Lock()
        self._rx_buffer = bytearray()

        # Telemetri
        self.distance = 0       # cm (TF-Luna)
        self.strength = 0       # TF-Luna sinyal gucu
        self.pan_angle = 0.0    # Merkezden goreli derece (-75 .. +75)
        self.tilt_angle = 0.0   # Merkezden goreli derece (-25 .. +25)
        self.last_rx_time = 0.0

        # Kuantalama biriktiricisi
        self._accum_pan = 0.0
        self._accum_tilt = 0.0

        self.running = True
        self.thread = threading.Thread(target=self._rx_worker, daemon=True)
        self.thread.start()

    def _find_port(self):
        """Windows COM portlarinda ST-Link'i otomatik bulur."""
        if not serial:
            return None
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            # STMicroelectronics VID: 0483 veya STLink
            desc = (p.description or "").lower()
            hwid = (getattr(p, "hwid", "") or "").lower()
            if getattr(p, "vid", None) == 0x0483 or "0483:" in hwid or "stlink" in desc or "stm" in desc:
                return p.device
        if ports:
            return ports[0].device
        return None

    def _connect(self):
        if not serial:
            return False
        now = time.time()
        if now - self.last_reconnect_attempt < self.reconnect_interval:
            return False
        self.last_reconnect_attempt = now

        # Kullanici acikca port verdiyse ve sistemde mevcutsa onu dene; degilse ST-Link'i otonom ara
        target_port = None
        try:
            available = [p.device.upper() for p in serial.tools.list_ports.comports()]
            if self.port and self.port.upper() in available:
                target_port = self.port
        except Exception:
            pass

        if not target_port:
            target_port = self._find_port()
        if not target_port:
            return False

        try:
            with self._lock:
                if self.ser and self.ser.is_open:
                    self.ser.close()
                self.ser = serial.Serial(target_port, self.baudrate, timeout=0.02, write_timeout=0.04)
                self._rx_buffer.clear()
            return True
        except Exception:
            with self._lock:
                self.ser = None
            return False

    def is_connected(self):
        with self._lock:
            return bool(self.ser and self.ser.is_open and (time.time() - self.last_rx_time < self.stale_timeout))

    def send_command(
        self,
        delta_pan_deg: float,
        delta_tilt_deg: float,
        speed_pwm: float = 0.0,
        state: int = STATE_TRACKING,
    ):
        """STM32'ye 0.1 derece cozunurlukte goreli hareket, TB6612FNG PWM devri ve ates komutu yollar."""
        # Artik biriktirici (0.1 derece alti adimlar kaybolmaz, sifir komutta sifirlanir)
        if abs(float(delta_pan_deg)) < _ZERO_CMD_EPS:
            self._accum_pan = 0.0
            pan_ticks = 0
        else:
            self._accum_pan += float(delta_pan_deg)
            pan_ticks = int(round(self._accum_pan * ANGLE_SCALE))
            self._accum_pan -= (pan_ticks / ANGLE_SCALE)

        if abs(float(delta_tilt_deg)) < _ZERO_CMD_EPS:
            self._accum_tilt = 0.0
            tilt_ticks = 0
        else:
            self._accum_tilt += float(delta_tilt_deg)
            tilt_ticks = int(round(self._accum_tilt * ANGLE_SCALE))
            self._accum_tilt -= (tilt_ticks / ANGLE_SCALE)

        if not self.ser or not self.ser.is_open:
            return False

        spd_byte = int(max(0, min(255, round(speed_pwm)))) & 0xFF

        # 8 baytlik komut paketi
        payload = struct.pack(CMD_FORMAT, PC_TO_STM_HEADER, pan_ticks, tilt_ticks, spd_byte, int(state) & 0xFF)
        checksum = sum(payload) & 0xFF
        packet = payload + bytes([checksum])

        try:
            with self._lock:
                if self.ser and self.ser.is_open:
                    self.ser.write(packet)
                    return True
            return False
        except Exception:
            with self._lock:
                try:
                    if self.ser:
                        self.ser.close()
                except Exception:
                    pass
                self.ser = None
            return False

    def trigger_fire(self):
        """80 ms solenoid darbesi ile dart firlatmasini tetikler."""
        return self.send_command(0.0, 0.0, state=STATE_FIRING)

    def _rx_worker(self):
        """Arka planda telemetri paketlerini (10 bayt) ayrıştıran thread."""
        while self.running:
            if not self.ser or not self.ser.is_open:
                self._connect()
                time.sleep(0.1)
                continue

            try:
                raw = self.ser.read(64)
                if not raw:
                    time.sleep(0.005)
                    continue

                self._rx_buffer.extend(raw)
                while len(self._rx_buffer) >= TLM_FRAME_LEN:
                    # Baslik ara
                    if self._rx_buffer[0] != STM_TO_PC_HEADER:
                        self._rx_buffer.pop(0)
                        continue

                    frame = self._rx_buffer[:TLM_FRAME_LEN]
                    payload = frame[:-1]
                    expected_cs = sum(payload) & 0xFF
                    actual_cs = frame[-1]

                    if expected_cs == actual_cs:
                        # Gecerli paket
                        dist_raw, str_raw, p_raw, t_raw = struct.unpack_from(TLM_FORMAT, payload, 1)
                        with self._lock:
                            self.distance = dist_raw
                            self.strength = str_raw
                            # 0..180 servo acisindan 90 derece merkez ofsetini cikar
                            self.pan_angle = (p_raw / ANGLE_SCALE) - ANGLE_OFFSET_DEG
                            self.tilt_angle = (t_raw / ANGLE_SCALE) - ANGLE_OFFSET_DEG
                            self.last_rx_time = time.time()
                        del self._rx_buffer[:TLM_FRAME_LEN]
                    else:
                        self._rx_buffer.pop(0)

            except Exception:
                with self._lock:
                    try:
                        if self.ser:
                            self.ser.close()
                    except Exception:
                        pass
                    self.ser = None
                time.sleep(0.1)

    def close(self):
        self.running = False
        with self._lock:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.close()
                except Exception:
                    pass
            self.ser = None
