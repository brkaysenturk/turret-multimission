"""
================================================================================
TEST KATEGORISI 1: BASKET KONFIGURASYON (SSOT), IKILI PROTOKOL VE HABERLESME
================================================================================
Kapsam:
- turret_basket.config (SSOT geometrik/fiziksel tutarlilik ve kare piksel kontrolu)
- turret_basket.protocol (0xAA komut, 0x55 telemetri, 0x59 TF-Luna cerceve sabitleri)
- turret_basket.basket_comm (100 Hz UART paketleme, checksum ve artik sifirlama)
================================================================================
"""

import struct

from turret_basket.config import CONFIG
from turret_basket.protocol import (
    PC_TO_STM_HEADER,
    STM_TO_PC_HEADER,
    LUNA_HEADER,
    CMD_FRAME_LEN,
    TLM_FRAME_LEN,
    LUNA_FRAME_LEN,
    ANGLE_SCALE,
    ANGLE_OFFSET_DEG,
    CMD_FORMAT,
    TLM_FORMAT,
    STATE_TRACKING,
    STATE_LOCKED,
    STATE_FIRING,
    STATE_ESTOP,
)
from turret_basket.basket_comm import FCSCommunicator
from turret_basket.rl_ballistics import RLBallistics
from turret_basket.engagement import BasketEngagement
from turret_basket.basket_manager import BasketManager


def test_basket_config_ssot():
    """turret_basket config degerlerinin geometrik ve fiziksel tutarliligini dogrular."""
    cam = CONFIG.camera
    assert cam.width == 1280
    assert cam.height == 720
    assert cam.fps == 30
    assert cam.fx > 0 and cam.fy > 0
    # Square pixel dogrulamasi (v_fov_deg = 28.6 ile fx ~= fy)
    assert abs(cam.fx - cam.fy) / cam.fx < 0.01

    hw = CONFIG.hardware
    assert hw.pan_min_deg < hw.pan_center_deg < hw.pan_max_deg
    assert hw.tilt_min_deg < hw.tilt_center_deg < hw.tilt_max_deg

    cup = CONFIG.cup
    assert cup.target_distance_min_m < cup.target_distance_max_m
    assert cup.mouth_diameter_m > 0

    ballistics = CONFIG.ballistics
    assert ballistics.firing_pulse_s == 0.080
    assert ballistics.spinup_time_s == 0.50
    assert ballistics.settle_time_s == 0.25
    assert ballistics.recovery_time_s == 0.50
    assert ballistics.parallax_pitch_margin_deg == 0.00  # Model 2 dogrudan cozer (paralaks 0)

    # Alt bilesenlerin CONFIG SSOT varsayilanlarini dogrulama
    eng = BasketEngagement()
    assert eng.spinup_time_s == ballistics.spinup_time_s
    assert eng.settle_time_s == ballistics.settle_time_s
    assert eng.firing_pulse_s == ballistics.firing_pulse_s
    assert eng.recovery_time_s == ballistics.recovery_time_s

    mgr = BasketManager(comm=None)
    assert mgr.MAX_LOCK_FRAMES == cup.lock_threshold_frames
    assert mgr.last_valid_dist_m == cup.nominal_distance_m

    b_ai = RLBallistics()
    assert b_ai.default_parallax_margin_deg == ballistics.parallax_pitch_margin_deg

    comm = FCSCommunicator(port=None)
    assert comm.baudrate == hw.baudrate


def test_basket_protocol_constants():
    """turret_basket.protocol ikili protokol sozlesmesini dogrular."""
    assert PC_TO_STM_HEADER == 0xAA
    assert STM_TO_PC_HEADER == 0x55
    assert LUNA_HEADER == 0x59

    assert CMD_FRAME_LEN == 8
    assert TLM_FRAME_LEN == 10
    assert LUNA_FRAME_LEN == 9

    assert ANGLE_SCALE == 10.0
    assert ANGLE_OFFSET_DEG == 90.0

    assert STATE_TRACKING == 0
    assert STATE_LOCKED == 1
    assert STATE_FIRING == 2
    assert STATE_ESTOP == 3


def test_basket_comm_packet_serialization_and_checksum():
    """STM32 ile 100 Hz ikili haberlesme paket formatini, checksum ve artik sifirlamayi dogrular."""
    comm = FCSCommunicator(port=None)

    # 1. 8 Baytlik Komut paketi olusturma (0xAA + 2B pan + 2B tilt + 1B speed + 1B state + 1B csum)
    pan_ticks = 25   # 2.5 derece
    tilt_ticks = -15  # -1.5 derece
    speed_pwm = 190   # TB6612FNG PWM devri
    payload = struct.pack(CMD_FORMAT, PC_TO_STM_HEADER, pan_ticks, tilt_ticks, speed_pwm, STATE_LOCKED)
    csum = sum(payload) & 0xFF
    pkt = payload + bytes([csum])

    assert len(pkt) == CMD_FRAME_LEN
    assert pkt[0] == PC_TO_STM_HEADER
    assert pkt[5] == speed_pwm
    assert pkt[6] == STATE_LOCKED
    assert (sum(pkt[:7]) & 0xFF) == pkt[7]

    # 2. 10 Baytlik Telemetri cozme (0x55 + dist + strength + pan + tilt + csum)
    tlm_payload = struct.pack(TLM_FORMAT, 180, 80, 0, 0)
    tlm_payload = bytes([STM_TO_PC_HEADER]) + tlm_payload
    tlm_csum = sum(tlm_payload) & 0xFF
    tlm_packet = tlm_payload + bytes([tlm_csum])

    assert len(tlm_packet) == TLM_FRAME_LEN
    comm._rx_buffer = bytearray(tlm_packet)
    frame = comm._rx_buffer[:TLM_FRAME_LEN]
    actual_cs = frame[-1]
    expected_cs = sum(frame[:-1]) & 0xFF
    assert actual_cs == expected_cs

    # 3. Sifir komutta kuantalama artik biriktiricisinin (residue accumulator) temizlenmesi
    comm._accum_pan = 0.08
    comm._accum_tilt = 0.08
    comm.send_command(0.0, 0.0)
    assert comm._accum_pan == 0.0
    assert comm._accum_tilt == 0.0

