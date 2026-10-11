#!/usr/bin/env python3
import time
from collections import deque
from statistics import median
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Float32MultiArray, Int32, String

NO_ECHO_CM = 400.0

MAJU, STOPPING, MUNDUR = 'MAJU', 'STOPPING', 'MUNDUR'
SCAN_KANAN, SCAN_KIRI, BELOK = 'SCAN KANAN', 'SCAN KIRI', 'BELOK'


class AvoiderBrain(Node):
    def __init__(self):
        super().__init__('avoider_brain')

        dp = self.declare_parameter
        dp('obstacle_dist', 30.0)           # cm, <= ini dianggap halangan
        dp('min_forward_speed', 50)         # PWM 0..255
        dp('forward_speed', 255)            # PWM maksimum
        dp('full_speed_dist', 120.0)        # cm, >= ini kecepatan penuh
        dp('pwm_ramp_interval_ms', 25)
        dp('pwm_ramp_up_step', 3)
        dp('pwm_ramp_down_step', 6)
        dp('turn_start_speed', 65)          # PWM awal setelah belok
        dp('reverse_speed', 120)
        dp('turn_speed', 100)
        dp('stop_time_ms', 150)
        dp('reverse_time_ms', 400)
        dp('turn_time_ms', 300)
        dp('front_angle', 90)
        dp('right_angle', 30)
        dp('left_angle', 150)
        dp('median_window', 1)              # 1 = tanpa filter (sama seperti firmware)
        dp('scan_samples', 1)               # 1 pembacaan per sisi (sama seperti firmware)
        dp('scan_timeout', 2.0)             # detik, batas tunggu sampel scan
        dp('data_timeout', 0.5)             # detik, telemetry dianggap hilang
        dp('rate_hz', 50.0)                 # frekuensi loop brain
        dp('start_enabled', True)

        self.enabled = bool(self.P('start_enabled'))

        # ---------------- pub / sub ----------------
        self.cmd_pub = self.create_publisher(Twist, '/avoider/cmd_vel', qos_profile_sensor_data)
        self.servo_pub = self.create_publisher(Int32, '/avoider/servo_cmd', qos_profile_sensor_data)
        self.status_pub = self.create_publisher(String, '/avoider/status', 10)
        self.create_subscription(Float32MultiArray, '/avoider/telemetry',
                                 self.on_telemetry, qos_profile_sensor_data)
        self.create_subscription(Bool, '/avoider/enable', self.on_enable, 10)

        self.tel = None            # dict: angle, ready, dist
        self.tel_t = 0.0
        self.tel_seq = 0
        self.used_seq = 0

        self.state = MAJU
        self.t_state = self.now()
        self.window = deque(maxlen=1)
        self.front_d = None
        self.samples = []
        self.right_d = 0.0
        self.left_d = 0.0
        self.turn_dir = -1
        self.cur_speed = 0         # PWM maju saat ini (untuk ramp)
        self.ramp_t = self.now()
        self.was_active = False

        self.lin_pwm = 0.0         # +maju / -mundur  (PWM 0..255)
        self.ang_pwm = 0.0         # +putar kiri / -kanan (PWM 0..255)
        self.servo_target = int(self.P('front_angle'))

        self.create_timer(1.0 / float(self.P('rate_hz')), self.step)
        self.get_logger().info('avoider_brain siap. Menunggu /avoider/telemetry ...')

    def P(self, name):
        return self.get_parameter(name).value

    def sec(self, name):
        return self.P(name) / 1000.0

    @staticmethod
    def now():
        return time.monotonic()

    def on_enable(self, msg):
        self.enabled = bool(msg.data)
        self.get_logger().info(f'enable = {self.enabled}')

    def on_telemetry(self, msg):
        d = list(msg.data)
        if len(d) < 3:
            return
        self.tel = {'angle': int(round(d[0])), 'ready': d[1] > 0.5, 'dist': float(d[2])}
        self.tel_t = self.now()
        self.tel_seq += 1

    def enter(self, state, start_speed=0):
        self.state = state
        self.t_state = self.now()
        self.samples = []
        if state == MAJU:
            self.window = deque(maxlen=max(1, int(self.P('median_window'))))
            self.front_d = None
            self.cur_speed = start_speed
            self.ramp_t = self.now()

    def elapsed(self):
        return self.now() - self.t_state

    def take_sample(self, angle, need_ready):
        """Jarak (cm) dari telemetry BARU bila servo ada di `angle` (dan sudah diam
        bila need_ready), selain itu None. Tiap telemetry hanya dipakai satu kali."""
        if self.tel_seq == self.used_seq or self.tel is None:
            return None
        self.used_seq = self.tel_seq
        t = self.tel
        if t['angle'] != angle or (need_ready and not t['ready']):
            return None
        return NO_ECHO_CM if t['dist'] < 0 else t['dist']

    # ---- kecepatan maju proporsional jarak (calculateForwardTarget) ----
    def forward_target(self, d):
        obstacle = self.P('obstacle_dist')
        full = self.P('full_speed_dist')
        vmin = self.P('min_forward_speed')
        vmax = self.P('forward_speed')
        if d <= obstacle:
            return 0
        if d >= full:
            return vmax
        ratio = (d - obstacle) / (full - obstacle)
        return vmin + int(ratio * (vmax - vmin))

    # ---- ramp PWM (rampForwardSpeed); tahan terhadap frekuensi loop berbeda ----
    def ramp(self, target):
        interval = self.sec('pwm_ramp_interval_ms')
        n = int((self.now() - self.ramp_t) / interval)
        if n <= 0:
            return self.cur_speed
        self.ramp_t += n * interval
        if self.cur_speed < target:
            self.cur_speed = min(target, self.cur_speed + self.P('pwm_ramp_up_step') * n)
        elif self.cur_speed > target:
            self.cur_speed = max(target, self.cur_speed - self.P('pwm_ramp_down_step') * n)
        return self.cur_speed

    def step(self):
        front = int(self.P('front_angle'))
        right = int(self.P('right_angle'))
        left = int(self.P('left_angle'))
        fresh = self.tel is not None and (self.now() - self.tel_t) < self.P('data_timeout')
        active = self.enabled and fresh

        if not active:
            self.was_active = False
            self.lin_pwm = self.ang_pwm = 0.0
            self.servo_target = front
            self.publish('STANDBY' if not self.enabled else 'NO DATA')
            return
        if not self.was_active:             # baru aktif -> mulai dari MAJU (kecepatan 0)
            self.was_active = True
            self.enter(MAJU, 0)

        self.lin_pwm = self.ang_pwm = 0.0

        if self.state == MAJU:
            self.servo_target = front
            s = self.take_sample(front, need_ready=False)
            if s is not None:
                self.window.append(s)
                self.front_d = median(self.window)
            if self.front_d is not None:
                d = self.front_d
                if d <= self.P('obstacle_dist'):
                    self.get_logger().info(f'Halangan {d:.1f} cm -> berhenti')
                    self.cur_speed = 0
                    self.enter(STOPPING)
                else:
                    self.lin_pwm = self.ramp(self.forward_target(d))
            # belum ada sampel di depan -> diam sebentar

        elif self.state == STOPPING:
            self.servo_target = front
            if self.elapsed() >= self.sec('stop_time_ms'):
                self.enter(MUNDUR)

        elif self.state == MUNDUR:
            self.servo_target = front
            self.lin_pwm = -self.P('reverse_speed')
            if self.elapsed() >= self.sec('reverse_time_ms'):
                self.enter(SCAN_KANAN)

        elif self.state == SCAN_KANAN:
            self.servo_target = right
            s = self.take_sample(right, need_ready=True)
            if s is not None:
                self.samples.append(s)
            if len(self.samples) >= self.P('scan_samples') or self.elapsed() > self.P('scan_timeout'):
                self.right_d = median(self.samples) if self.samples else 0.0
                self.enter(SCAN_KIRI)

        elif self.state == SCAN_KIRI:
            self.servo_target = left
            s = self.take_sample(left, need_ready=True)
            if s is not None:
                self.samples.append(s)
            if len(self.samples) >= self.P('scan_samples') or self.elapsed() > self.P('scan_timeout'):
                self.left_d = median(self.samples) if self.samples else 0.0
                self.turn_dir = 1 if self.left_d > self.right_d else -1   # +1 = kiri
                self.get_logger().info(
                    f'Scan kanan {self.right_d:.1f} cm | kiri {self.left_d:.1f} cm -> '
                    f'belok {"KIRI" if self.turn_dir > 0 else "KANAN"}')
                self.enter(BELOK)

        elif self.state == BELOK:
            self.servo_target = front
            self.ang_pwm = self.turn_dir * self.P('turn_speed')   # angular.z + = putar kiri
            if self.elapsed() >= self.sec('turn_time_ms'):
                self.enter(MAJU, self.P('turn_start_speed'))

        self.publish(self.state)

    def publish(self, status):
        t = Twist()
        t.linear.x = float(self.lin_pwm) / 255.0       # 1.0 = PWM 255 di ESP32
        t.angular.z = float(self.ang_pwm) / 255.0
        self.cmd_pub.publish(t)
        self.servo_pub.publish(Int32(data=int(self.servo_target)))
        self.status_pub.publish(String(data=status))


def main():
    rclpy.init()
    node = AvoiderBrain()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
