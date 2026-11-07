"""PC <-> STM32 ikili protokol sozlesmesi (tek kaynak).

Bu modul, firmware tarafindaki `Core/Inc/protocol.h` dosyasinin Python
karsiligidir. Iki taraf ayni anda guncellenmelidir; aksi halde checksum tutsa
bile alanlar kayabilir.

Komut cercevesi (PC -> STM32, 8 bayt):
    [PC_TO_STM_HEADER][pan i16][tilt i16][speed u8][state u8][checksum u8]

Telemetri cercevesi (STM32 -> PC, 10 bayt):
    [STM_TO_PC_HEADER][mesafe u16][sinyal u16][pan i16][tilt i16][checksum u8]

Aci birimi 0.1 derecedir (deger = aci * 10). Telemetrideki servo acilari
0..180 derece araligindadir; PC tarafi 90 derece merkez offsetini cikarir.
Speed birimi 0..255 PWM duty degeridir (varsayilan 217: ~%85 nominal devir).
"""

PC_TO_STM_HEADER = 0xAA
STM_TO_PC_HEADER = 0x55
LUNA_HEADER = 0x59

CMD_FRAME_LEN = 8
TLM_FRAME_LEN = 10
LUNA_FRAME_LEN = 9

ANGLE_SCALE = 10.0
ANGLE_OFFSET_DEG = 90.0

CMD_FORMAT = "<BhhBB"
TLM_FORMAT = "<HHhh"

STATE_TRACKING = 0
STATE_LOCKED = 1
STATE_FIRING = 2
STATE_ESTOP = 3
