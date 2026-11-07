"""
================================================================================
MODUL: DONANIM HABERLESME VE SERI PORT PROTOKOLU (FCS COMMUNICATOR / UART)
================================================================================
Bu modul, PC (Python) ile gomulu sistem (STM32 mikrodenetleyici) arasindaki
tam cift yonlu (Full-Duplex), asenkron ve binary paketli iletisim katmanidir.

PROTOKOL VE PAKET YAPISI (115200 Baud, sabitler: turret_fcs/protocol.py):
1. Komut Paketi (PC -> STM32) [8 Bayt]:
   [PC_TO_STM_HEADER] [Pan Delta * 10: int16 (2B, 0.1 deg)] [Tilt Delta * 10: int16 (2B, 0.1 deg)] [Speed: uint8 (1B)] [State/Fire: uint8 (1B)] [Checksum: uint8 (1B)]
2. Telemetri Paketi (STM32 -> PC) [10 Bayt]:
   [STM_TO_PC_HEADER] [TF-Luna Mesafe: uint16 (2B)] [Sinyal Gucu: uint16 (2B)] [Pan Acisi * 10: int16 (2B, 0.1 deg)] [Tilt Acisi * 10: int16 (2B, 0.1 deg)] [Checksum: uint8 (1B)]

GELISMIS OZELLIKLER:
- Otomatik Port Tespiti (Auto Plug-and-Play): Windows COM portlarini tarayarak
  ST-Link / STM32 USB VID '0483' kimligini otomatik tespit eder ve baglanir.
- Bagimsiz Okuma Thread'i: UART RX okumasini ayri thread'de calistirarak ana kontrol
  dongusunun bloklanmasini engeller.
- Checksum Dogrulama: Gurultulu seri hatta hatali baytlari tespit edip eler.
================================================================================
"""

import struct
import threading
import time

from .protocol import (
    ANGLE_OFFSET_DEG,
    ANGLE_SCALE,
    CMD_FORMAT,
    PC_TO_STM_HEADER,
    STM_TO_PC_HEADER,
    TLM_FORMAT,
    TLM_FRAME_LEN,
)

try:
    import serial
    import serial.tools.list_ports
except ImportError:  # optional hardware dependency; FCS still runs without a serial port
    serial = None


# Seri hat sabitleri (protokol cerceve sabitleri protocol.py'de)
_ZERO_CMD_EPS = 1e-9        # Bu degerin altindaki komut "sifir" sayilir ve artigi sifirlar
_INT16_STEP_LIMIT = 32000   # Paket alani int16; tasma payi birakilarak kirpilir
_RX_BUFFER_MAX = 4096       # Cerceve disi gurultu birikirse tampon bu sinirda kirpilir


class FCSCommunicator:
    """
    STM32 mikrodenetleyicisi ile tek bir seri port uzerinden Full-Duplex
    ve Saf Binary (ikili) paket protokolu ile calisan iletisim sinifi:

      1. PC -> STM32 komut cercevesi (8 bayt):
         [PC_TO_STM_HEADER] [Pan: int16] [Tilt: int16] [Speed: uint8] [State: uint8] [Checksum: uint8]

      2. STM32 -> PC telemetri cercevesi (10 bayt):
         [STM_TO_PC_HEADER] [Mesafe: uint16] [Sinyal: uint16] [Pan*10: int16] [Tilt*10: int16] [Checksum: uint8]

    Cerceve sabitleri `turret_fcs/protocol.py` ve firmware `protocol.h` ile
    ortaktir; iki taraf birlikte guncellenmelidir.
    Baglanti: Once acikca verilen port, sonra ST-Link / STM32 USB VID (0483),
    sonra tek USB seri port otomatik secilir.
    """

    def __init__(self, port=None, baudrate=115200, reconnect_interval=2.0, stale_timeout=0.3):
        self.port = port
        self.baudrate = baudrate
        self.reconnect_interval = reconnect_interval
        self.stale_timeout = stale_timeout

        self.last_reconnect_attempt = 0.0
        self.ser = None
        self._lock = threading.Lock()
        self._rx_buffer = bytearray()

        # TF-Luna ve Taret Aci telemetri verileri
        self.distance = 0
        self.strength = 0
        self.servo_pan_deg = 0.0
        self.servo_tilt_deg = 0.0
        self.has_servo_telemetry = False
        self.last_valid_time = 0.0

        # Kuantalama Artik Biriktiricisi (0.1 deg alti adimlari korur)
        self.residue_pan = 0.0
        self.residue_tilt = 0.0

        self.running = True
        self.connect()
        self.thread = threading.Thread(target=self._binary_reader_worker, daemon=True)
        self.thread.start()

    def _find_target_port(self):
        """Mevcut seri portlari oncelik sirasina gore tarar."""
        if serial is None:
            return None
        try:
            available = list(serial.tools.list_ports.comports())
        except Exception:
            return None
        if not available:
            return None

        # 1. Kullanici acikca port verdiyse onu kullan.
        if self.port:
            for p in available:
                if p.device.upper() == self.port.upper():
                    return p.device

        # 2. ST-Link / STM32 USB VID.
        for p in available:
            desc = (p.description or "").lower()
            hwid = (p.hwid or "").lower()
            if "0483:" in hwid or "stlink" in desc or "stm" in desc or "stmicroelectronics" in desc:
                return p.device

        # 3. Tek COM portu varsa.
        if len(available) == 1:
            return available[0].device

        # 4. Genel USB-Serial donanimlari.
        for p in available:
            desc = (p.description or "").lower()
            if any(x in desc for x in ["ch340", "cp210", "ftdi", "usb serial", "usb-to-serial"]):
                return p.device
        return None

    def connect(self):
        """Seri port baglantisini baslatir."""
        self.last_reconnect_attempt = time.time()
        if serial is None:
            self.ser = None
            return False
        target_port = self._find_target_port()
        if not target_port:
            self.ser = None
            return False

        try:
            with self._lock:
                if self.ser and self.ser.is_open:
                    self.ser.close()
                self.ser = serial.Serial(
                    target_port,
                    self.baudrate,
                    timeout=0.04,
                    write_timeout=0.04,
                )
            self.port = target_port
            self._rx_buffer.clear()
            return True
        except (serial.SerialException, OSError):
            self.ser = None
            return False

    @property
    def is_connected(self):
        return self.ser is not None and self.ser.is_open

    @property
    def is_luna_online(self):
        """TF-Luna LiDAR'in STM32 uzerinden canli telemetri akitip akitmadigini dondurur."""
        return (
            self.is_connected
            and (time.time() - self.last_valid_time <= self.stale_timeout)
            and (self.distance > 0 or self.strength > 0)
        )

    def _disconnect_port(self):
        """Seri portu guvenlice sifirlar, worker thread'i calisir durumda tutar."""
        with self._lock:
            if self.ser:
                try:
                    if self.ser.is_open:
                        self.ser.close()
                except Exception:
                    pass
                finally:
                    self.ser = None

    def send(self, pan_angle: float, tilt_angle: float, fcs_state: int, speed_pwm: int = 0):
        """Taret hedef acilarini, motor hizini ve durumunu binary paket olarak gonderir.
        
        Args:
            pan_angle: Pan acisal duzeltme (derece).
            tilt_angle: Tilt acisal duzeltme (derece).
            fcs_state: Sistem durumu (TRACKING, LOCKED, FIRING, ESTOP).
            speed_pwm: TB6612FNG PWM duty degeri (0..255). 0 ise STM32 varsayilani (217 = ~%85) kullanilir.
        """
        now = time.time()
        if not self.is_connected:
            if (now - self.last_reconnect_attempt) > self.reconnect_interval:
                self.connect()
            if not self.is_connected:
                return

        # Her eksen kendi kuantalama artiklarini tasir; sifir komut o ekseni sifirlar.
        if abs(float(pan_angle)) < _ZERO_CMD_EPS:
            self.residue_pan = 0.0
            p = 0
        else:
            total_p = float(pan_angle) * ANGLE_SCALE + self.residue_pan
            p_step = int(round(total_p))
            self.residue_pan = total_p - p_step
            p = max(-_INT16_STEP_LIMIT, min(_INT16_STEP_LIMIT, p_step))

        if abs(float(tilt_angle)) < _ZERO_CMD_EPS:
            self.residue_tilt = 0.0
            t = 0
        else:
            total_t = float(tilt_angle) * ANGLE_SCALE + self.residue_tilt
            t_step = int(round(total_t))
            self.residue_tilt = total_t - t_step
            t = max(-_INT16_STEP_LIMIT, min(_INT16_STEP_LIMIT, t_step))

        spd = max(0, min(255, int(speed_pwm)))
        state = int(fcs_state) & 0xFF
        try:
            payload = struct.pack(CMD_FORMAT, PC_TO_STM_HEADER, p, t, spd, state)
            checksum = sum(payload) & 0xFF
            with self._lock:
                if self.ser and self.ser.is_open:
                    self.ser.write(payload + bytes([checksum]))
        except (serial.SerialException, serial.SerialTimeoutException, OSError):
            self._disconnect_port()

    def _consume_rx_buffer(self):
        """Biriken baytlardan gecerli 10 baytlik telemetri paketlerini ayiklar."""
        while len(self._rx_buffer) >= TLM_FRAME_LEN:
            header_index = self._rx_buffer.find(bytes([STM_TO_PC_HEADER]))
            if header_index < 0:
                self._rx_buffer.clear()
                return
            if header_index > 0:
                del self._rx_buffer[:header_index]
            if len(self._rx_buffer) < TLM_FRAME_LEN:
                return

            packet = self._rx_buffer[:TLM_FRAME_LEN]
            calc_csum = (STM_TO_PC_HEADER + sum(packet[1:9])) & 0xFF
            if calc_csum != packet[9]:
                del self._rx_buffer[0]
                continue

            dist, strength, p_raw, t_raw = struct.unpack(TLM_FORMAT, bytes(packet[1:9]))
            self.distance = dist
            self.strength = strength
            self.servo_pan_deg = (p_raw / ANGLE_SCALE) - ANGLE_OFFSET_DEG
            self.servo_tilt_deg = (t_raw / ANGLE_SCALE) - ANGLE_OFFSET_DEG
            self.has_servo_telemetry = True
            self.last_valid_time = time.time()
            del self._rx_buffer[:10]

    def _binary_reader_worker(self):
        """STM32'den gelen 10 baytlik ikili telemetri paketlerini okur."""
        while self.running:
            if not self.is_connected:
                now = time.time()
                if (now - self.last_reconnect_attempt) > self.reconnect_interval:
                    self.connect()
                time.sleep(0.1)
                continue

            try:
                with self._lock:
                    if not self.ser or not self.ser.is_open:
                        continue
                    waiting = self.ser.in_waiting
                    raw_bytes = self.ser.read(waiting) if waiting > 0 else b""

                if raw_bytes:
                    self._rx_buffer.extend(raw_bytes)
                    if len(self._rx_buffer) > _RX_BUFFER_MAX:
                        del self._rx_buffer[:-TLM_FRAME_LEN]
                    self._consume_rx_buffer()
                else:
                    time.sleep(0.002)
            except (serial.SerialException, OSError):
                self._disconnect_port()
                time.sleep(0.1)
            except Exception:
                time.sleep(0.01)

    def get_luna_data(self):
        """(mesafe_cm, sinyal_gucu) dondurur. 0.3 sn veri gelmezse (0, 0) dondurur."""
        if time.time() - self.last_valid_time > self.stale_timeout:
            return 0, 0
        return self.distance, self.strength

    def get_servo_angles(self):
        """
        STM32 mikrodenetleyicisinden donen gercek servo acilarini (bagil derece cinsinden) dondurur.
        Telemetri bagli degilse veya bayatsa (None, None) dondurur.
        """
        if not self.has_servo_telemetry or (time.time() - self.last_valid_time > self.stale_timeout):
            return None, None
        return self.servo_pan_deg, self.servo_tilt_deg

    def close(self):
        """Seri portu ve okuma thread'ini guvenli kapatir (Sadece ana uygulama cikisinda)."""
        self.running = False
        self._disconnect_port()
        try:
            if hasattr(self, "thread") and self.thread.is_alive():
                self.thread.join(timeout=0.2)
        except Exception:
            pass
