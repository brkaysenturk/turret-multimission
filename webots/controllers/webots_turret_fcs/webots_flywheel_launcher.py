"""
================================================================================
MODUL: WEBOTS CIFTLI VOLAN VE SOLENOID FIRLATICI (FLYWHEEL LAUNCHER - v2.0)
================================================================================
Bu modul, cift fircasiz DC volan motorlarini (23.500 RPM), 80 ms darbe genislikli
JF-0530B solenoid iticiyi ve Webots fizigiyle 10 m/s sunger dart firlatmasini
saf simulasyon zamani (sim_t) uzerinden yonetir.
================================================================================
"""

import math

from turret_fcs.config import CONFIG


def rot_matrix_to_axis_angle(R):
    """3x3 donus matrisini Webots eksen-aci [x, y, z, angle] formatina cevirir."""
    trace = R[0][0] + R[1][1] + R[2][2]
    cos_theta = max(-1.0, min(1.0, (trace - 1.0) / 2.0))
    theta = math.acos(cos_theta)
    if abs(theta) < 1e-4:
        return "0 0 1 0"
    sin_theta = math.sin(theta)
    x = (R[2][1] - R[1][2]) / (2.0 * sin_theta)
    y = (R[0][2] - R[2][0]) / (2.0 * sin_theta)
    z = (R[1][0] - R[0][1]) / (2.0 * sin_theta)
    norm = math.sqrt(x * x + y * y + z * z)
    if norm < 1e-6:
        return "0 0 1 0"
    return f"{x/norm:.4f} {y/norm:.4f} {z/norm:.4f} {theta:.4f}"


class WebotsFlywheelLauncher:
    """Cift volan motoru, solenoid itici ve fiziksel Nerf dart uretim yoneticisi."""

    def __init__(self, robot):
        self.robot = robot
        self.TARGET_RAD_S = CONFIG.flywheel.target_speed_rad_s
        self.MUZZLE_VELOCITY = CONFIG.ballistics.projectile_speed_mps  # 10.0 m/s
        self.SOLENOID_PULSE_S = CONFIG.ballistics.firing_pulse_s      # 0.08 s (80 ms)

        # Volan motorlari (Baslangicta kapali / standby)
        self.flywheel_left = robot.getDevice('flywheel_left')
        self.flywheel_right = robot.getDevice('flywheel_right')
        if self.flywheel_left and self.flywheel_right:
            self.flywheel_left.setPosition(float('inf'))
            self.flywheel_right.setPosition(float('inf'))
            self.flywheel_left.setVelocity(0.0)
            self.flywheel_right.setVelocity(0.0)

        # Solenoid itici ve konum sensoru
        self.solenoid = robot.getDevice('solenoid')
        self.solenoid_sensor = robot.getDevice('solenoid_sensor')
        timestep = int(robot.getBasicTimeStep())
        if self.solenoid_sensor:
            self.solenoid_sensor.enable(timestep)
        if self.solenoid:
            self.solenoid.setPosition(0.0)

        self.root_children = robot.getRoot().getField("children")
        self.kafa_node = robot.getFromDef("KAFA_BLOGU")

        self.total_darts_fired = 0
        self.active_dart_nodes = []
        self.solenoid_state = "idle"
        self.solenoid_hold_start = 0.0
        self.solenoid_trigger_time = 0.0
        self.pending_dart_spawn = False
        self.is_spinning = False

    def start_spinup(self):
        """Volan motorlarini nominal atis devrine (23.500 RPM) cikarir."""
        if self.flywheel_left and self.flywheel_right and not self.is_spinning:
            self.flywheel_left.setVelocity(self.TARGET_RAD_S)
            self.flywheel_right.setVelocity(-self.TARGET_RAD_S)
            self.is_spinning = True

    def stop_flywheels(self):
        """Atis sonrasi veya emniyette volan motorlarini durdurur."""
        if self.is_spinning and self.flywheel_left and self.flywheel_right:
            self.flywheel_left.setVelocity(0.0)
            self.flywheel_right.setVelocity(0.0)
            self.is_spinning = False

    def trigger(self):
        """FIRING evresinde 80 ms solenoid itme darbesini baslatir."""
        self.solenoid_trigger_time = self.robot.getTime()
        if self.solenoid:
            self.solenoid_state = "pushing"
            self.pending_dart_spawn = True
            self.solenoid.setPosition(0.05)
        else:
            self._spawn_dart()

    def update(self, sim_time):
        """Solenoid darbe zamanlamasini (80 ms) ve eski dart temizligini sim_t uzerinden yonetir."""
        if self.solenoid and self.solenoid_sensor:
            if self.solenoid_state == "pushing":
                if self.solenoid_sensor.getValue() >= 0.040:
                    self.solenoid_state = "holding"
                    self.solenoid_hold_start = sim_time
                    if self.pending_dart_spawn:
                        self._spawn_dart()
                        self.pending_dart_spawn = False
                elif (sim_time - self.solenoid_trigger_time) > 0.15:
                    # Sensor/model takildiysa dart'i yine de birak ve solenoidi geri cek.
                    if self.pending_dart_spawn:
                        self._spawn_dart()
                        self.pending_dart_spawn = False
                    self.solenoid.setPosition(0.0)
                    self.solenoid_state = "retracting"
            elif self.solenoid_state == "holding":
                # 80 ms (0.08 s) darbe suresi
                if (sim_time - self.solenoid_hold_start) >= self.SOLENOID_PULSE_S:
                    self.solenoid.setPosition(0.0)
                    self.solenoid_state = "retracting"
            elif self.solenoid_state == "retracting":
                if self.solenoid_sensor.getValue() <= 0.005:
                    self.solenoid_state = "idle"

        # 2.5 saniyeden eski dartlari sahneden kaldir
        while self.active_dart_nodes and (sim_time - self.active_dart_nodes[0][1]) > 2.5:
            old_node, _ = self.active_dart_nodes.pop(0)
            try:
                old_node.remove()
            except Exception:
                pass

    def _spawn_dart(self):
        """Kafa blogunun baktigi yonde fiziksel 10 m/s hizli 12g Nerf darti uretir."""
        self.total_darts_fired += 1

        head_pos = self.kafa_node.getPosition() if self.kafa_node else [0.0, 0.0, 1.0131]
        head_rot = self.kafa_node.getOrientation() if self.kafa_node else [1, 0, 0, 0, 1, 0, 0, 0, 1]

        fx_dir = head_rot[1]
        fy_dir = head_rot[4]
        fz_dir = head_rot[7]

        # Namlu cikis konumu (CAD v10: Namlu ucu +0.075 m onde, kafa Z ekseninde +0.0256 m yukarida)
        spawn_x = head_pos[0] + head_rot[1] * 0.075 + head_rot[2] * 0.0256
        spawn_y = head_pos[1] + head_rot[4] * 0.075 + head_rot[5] * 0.0256
        spawn_z = head_pos[2] + head_rot[7] * 0.075 + head_rot[8] * 0.0256

        # Dart govdesini namlu eksenine hizala
        R_pitch90 = [
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.0, -1.0, 0.0]
        ]
        R_comb = [
            [sum(head_rot[3*r + k] * R_pitch90[k][c] for k in range(3)) for c in range(3)]
            for r in range(3)
        ]
        rot_str = rot_matrix_to_axis_angle(R_comb)

        # Webots PROTO dart dugumu
        dart_vrml = f"""
        Solid {{
          translation {spawn_x:.4f} {spawn_y:.4f} {spawn_z:.4f}
          rotation {rot_str}
          children [
            DEF DART_SHAPE Pose {{
              children [
                Shape {{
                  appearance PBRAppearance {{
                    baseColor 0.1 0.3 0.9
                    roughness 0.5
                    metalness 0.1
                  }}
                  geometry Cylinder {{
                    height 0.072
                    radius 0.0065
                  }}
                }}
              ]
            }}
          ]
          name "foam_dart_{self.total_darts_fired}"
          contactMaterial "dart_foam"
          boundingObject USE DART_SHAPE
          physics Physics {{
            density -1
            mass 0.012
            centerOfMass [0 0 0]
            inertiaMatrix [
              0.000008 0.000002 0.000008
              0 0 0
            ]
            damping Damping {{
              linear 0.05
              angular 0.1
            }}
          }}
        }}
        """

        if self.root_children:
            self.root_children.importMFNodeFromString(-1, dart_vrml)
            dart_node = self.root_children.getMFNode(-1)
            if dart_node:
                # 10 m/s namlu cikis hizi
                vx = fx_dir * self.MUZZLE_VELOCITY
                vy = fy_dir * self.MUZZLE_VELOCITY
                vz = fz_dir * self.MUZZLE_VELOCITY
                dart_node.setVelocity([vx, vy, vz, 0.0, 0.0, 0.0])
                current_time = self.robot.getTime()
                self.active_dart_nodes.append((dart_node, current_time))