#!/usr/bin/env python3
import time
from collections import deque
from typing import Deque, Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import TwistStamped

try:
    from moteus_msgs.msg import PositionCommand, ControllerState
    HAVE_MOTEUS = True
except Exception:
    HAVE_MOTEUS = False
    PositionCommand = None  # type: ignore
    ControllerState = None  # type: ignore

# ---- Mode names (z dokumentacji) ----
MODE_NAMES = {
    0: "stopped (clears faults)",
    1: "fault",
    2: "preparing",
    3: "preparing",
    4: "preparing",
    5: "pwm",
    6: "voltage",
    7: "voltage_foc",
    8: "voltage_dq",
    9: "current",
    10: "position",
    11: "timeout",
    12: "zero_velocity",
    13: "stay_within",
    14: "measure_inductance",
    15: "brake",
}

# ---- Fault codes (z dokumentacji) ----
FAULT_CODES = {
    # Real faults (aktywny przy mode=1)
    32: "calibration fault – encoder nie wykrył magnesu podczas kalibracji",
    33: "motor driver fault – typowo undervoltage lub inny błąd elektryczny",
    34: "over voltage – bus > servo.max_voltage (np. regeneracja bez flux braking)",
    35: "encoder fault – odczyty niezgodne z obecnością magnesu",
    36: "motor not configured – brak kalibracji (moteus_tool --calibrate)",
    37: "pwm cycle overrun – błąd wewnętrzny firmware",
    38: "over temperature – przekroczona maksymalna temperatura",
    39: "outside limit – start pozycji poza servopos.position_min/max",
    40: "under voltage – zbyt niskie napięcie",
    41: "config changed – zmiana konfiguracji wymagająca stopu",
    42: "theta invalid – brak ważnego enkodera komutacji",
    43: "position invalid – brak ważnego enkodera wyjściowego",
    44: "driver enable fault – nie udało się włączyć sterownika MOSFET",
    45: "stop position deprecated – użyto przestarzałej funkcji",
    46: "timing violation – naruszenie ograniczenia czasowego",
    47: "bemf feedforward no accel – włączone bez limitu przyspieszenia",
    48: "invalid limits – position_min/max poza zakresem",
    # Limity (nie muszą oznaczać fault, informują co ogranicza moc)
    96: "limit active: servo.max_velocity",
    97: "limit active: servo.max_power_W",
    98: "limit active: maximum system voltage",
    99: "limit active: servo.max_current_A",
    100: "limit active: servo.fault_temperature",
    101: "limit active: servo.motor_fault_temperature",
    102: "limit active: commanded maximum torque",
    103: "limit active: servopos.position_min/max",
}

def decode_fault_code(code: Optional[int]) -> str:
    """Zwraca opis kodu fault/limitu, albo 'no fault' / 'unknown fault X'."""
    if code is None or code == 0:
        return "no fault"
    return FAULT_CODES.get(code, f"unknown fault {code}")

class RateMeter:
    def __init__(self, alpha: float = 0.2):
        self.alpha = alpha
        self.ema = 0.0
        self.last_t: Optional[float] = None

    def tick(self):
        t = time.monotonic()
        if self.last_t is not None:
            dt = t - self.last_t
            if dt > 0:
                fps = 1.0 / dt
                self.ema = self.alpha * fps + (1 - self.alpha) * self.ema
        self.last_t = t

    def value(self) -> float:
        return self.ema

class StatsWindow:
    def __init__(self, window_s: float = 5.0):
        self.window_s = window_s
        self.ts: Deque[float] = deque()

    def tick(self, t: Optional[float] = None):
        now = time.monotonic() if t is None else t
        self.ts.append(now)
        self._trim(now)

    def _trim(self, now: float):
        limit = now - self.window_s
        while self.ts and self.ts[0] < limit:
            self.ts.popleft()

    def hz(self) -> float:
        if len(self.ts) < 2:
            return 0.0
        dt = self.ts[-1] - self.ts[0]
        return (len(self.ts) - 1) / dt if dt > 0 else 0.0

    def age(self) -> float:
        if not self.ts:
            return float('inf')
        return time.monotonic() - self.ts[-1]

    def jitter_minmax(self) -> Tuple[float, float]:
        if len(self.ts) < 3:
            return (0.0, 0.0)
        diffs = [self.ts[i] - self.ts[i-1] for i in range(1, len(self.ts))]
        if not diffs:
            return (0.0, 0.0)
        return (min(diffs), max(diffs))

class BoxyGoMoteusDiagnostic(Node):
    def __init__(self):
        super().__init__('boxygo_moteus_diagnostic')
        self.declare_parameter('servo_ids', [1, 2, 3, 4, 5, 6])
        self.declare_parameter('cmd_topic', '/diff_cont/cmd_vel_out')
        self.declare_parameter('cmd_prefix', '/boxygo_moteus/id_')
        self.declare_parameter('state_prefix', '/moteus/id_')
        self.declare_parameter('report_period_s', 1.0)
        self.declare_parameter('stats_window_s', 5.0)
        self.declare_parameter('warn_cmd_vel_hz_min', 20.0)
        self.declare_parameter('warn_can_hz_min', 20.0)
        self.declare_parameter('heartbeat_timeout_s', 0.3)
        self.declare_parameter('v_warn', 21.0)
        self.declare_parameter('v_error', 20.0)
        self.declare_parameter('motor_temp_warn', 80.0)
        self.declare_parameter('motor_temp_error', 95.0)
        self.declare_parameter('enabled', True)

        self.servo_ids: List[int] = list(self.get_parameter('servo_ids').value)
        self.cmd_topic: str = str(self.get_parameter('cmd_topic').value)
        self.cmd_prefix: str = str(self.get_parameter('cmd_prefix').value)
        self.state_prefix: str = str(self.get_parameter('state_prefix').value)
        self.report_period: float = float(self.get_parameter('report_period_s').value)
        self.window_s: float = float(self.get_parameter('stats_window_s').value)
        self.warn_cmd_vel_hz_min: float = float(self.get_parameter('warn_cmd_vel_hz_min').value)
        self.warn_can_hz_min: float = float(self.get_parameter('warn_can_hz_min').value)
        self.heartbeat_timeout_s: float = float(self.get_parameter('heartbeat_timeout_s').value)
        self.v_warn: float = float(self.get_parameter('v_warn').value)
        self.v_error: float = float(self.get_parameter('v_error').value)
        self.motor_temp_warn: float = float(self.get_parameter('motor_temp_warn').value)
        self.motor_temp_error: float = float(self.get_parameter('motor_temp_error').value)
        self.enabled: bool = bool(self.get_parameter('enabled').value)

        self.add_on_set_parameters_callback(self._on_set_params)

        self.cmd_vel_rm = RateMeter()
        self.cmd_vel_win = StatsWindow(self.window_s)
        self.can_rm_by_id: Dict[int, RateMeter] = {sid: RateMeter() for sid in self.servo_ids}
        self.can_win_by_id: Dict[int, StatsWindow] = {sid: StatsWindow(self.window_s) for sid in self.servo_ids}

        self.last_seen_cmd_vel: Optional[float] = None
        self.last_seen_can: Dict[int, Optional[float]] = {sid: None for sid in self.servo_ids}
        self.last_seen_state: Dict[int, Optional[float]] = {sid: None for sid in self.servo_ids}

        self.last_cmd_ts: Optional[float] = None
        self.lat_ema: float = 0.0
        self.lat_max: float = 0.0
        self.lat_alpha: float = 0.2

        self.latest_fault: Dict[int, Optional[int]] = {sid: None for sid in self.servo_ids}
        self.prev_fault: Dict[int, Optional[int]] = {sid: None for sid in self.servo_ids}
        self.latest_mode: Dict[int, Optional[str]] = {sid: None for sid in self.servo_ids}
        self.latest_voltage: Dict[int, Optional[float]] = {sid: None for sid in self.servo_ids}
        self.latest_motor_temp: Dict[int, Optional[float]] = {sid: None for sid in self.servo_ids}

        self.create_subscription(TwistStamped, self.cmd_topic, self._on_cmd_vel, 50)

        if HAVE_MOTEUS and PositionCommand is not None:
            for sid in self.servo_ids:
                topic = f"{self.cmd_prefix}{sid}/cmd_position"
                self.create_subscription(PositionCommand, topic, self._on_cmd_can(sid), 10)
        else:
            self.get_logger().error('moteus_msgs PositionCommand not available – CAN cmd rate monitoring disabled.')

        if HAVE_MOTEUS and ControllerState is not None:
            for sid in self.servo_ids:
                topic = f"{self.state_prefix}{sid}/state"
                self.create_subscription(ControllerState, topic, self._on_state(sid), 50)
        else:
            self.get_logger().error('moteus_msgs ControllerState not available – fault/state monitoring disabled.')

        self.create_timer(self.report_period, self._report)
        self.get_logger().info('boxygo_moteus_diagnostic started.')

    def _on_set_params(self, params):
        for p in params:
            if p.name == 'enabled':
                self.enabled = bool(p.value)
                self.get_logger().warn(f"[diagnostic] enabled={self.enabled}")
        return SetParametersResult(successful=True)

    def _on_cmd_vel(self, _msg: TwistStamped):
        if not self.enabled:
            return
        t = time.monotonic()
        self.cmd_vel_rm.tick()
        self.cmd_vel_win.tick(t)
        self.last_seen_cmd_vel = t
        self.last_cmd_ts = t

    def _on_cmd_can(self, sid: int):
        def _cb(_msg):
            if not self.enabled:
                return
            t = time.monotonic()
            rm = self.can_rm_by_id.get(sid)
            win = self.can_win_by_id.get(sid)
            if rm:
                rm.tick()
            if win:
                win.tick(t)
            self.last_seen_can[sid] = t
        return _cb

    def _on_state(self, sid: int):
        def _cb(msg: ControllerState):
            if not self.enabled:
                return
            t = time.monotonic()
            self.last_seen_state[sid] = t

            fault = getattr(msg, 'fault_code', None)
            mode = getattr(msg, 'mode', None)
            voltage = getattr(msg, 'voltage', None)
            mtemp = getattr(msg, 'temperature', None)

            # zapamiętaj poprzedni i aktualny kod
            self.prev_fault[sid] = self.latest_fault.get(sid)
            if fault is not None:
                try:
                    self.latest_fault[sid] = int(fault)
                except Exception:
                    # jeśli przyjdzie nie-liczba, oznacz jako unknown
                    self.latest_fault[sid] = None

            # mapowanie trybu
            if mode is not None:
                try:
                    mode_int = int(mode)
                    mode_name = MODE_NAMES.get(mode_int, "UNKNOWN")
                    self.latest_mode[sid] = f"{mode_int}:{mode_name}"
                    if mode_int == 1:
                        self.get_logger().error(f"id {sid}: MODE=1 (fault)")
                except Exception:
                    self.latest_mode[sid] = str(mode)

            if voltage is not None:
                self.latest_voltage[sid] = float(voltage)
            if mtemp is not None:
                self.latest_motor_temp[sid] = float(mtemp)

            # latency
            if self.last_cmd_ts is not None:
                lat = t - self.last_cmd_ts
                self.lat_ema = self.lat_alpha * lat + (1 - self.lat_alpha) * self.lat_ema
                if lat > self.lat_max:
                    self.lat_max = lat
        return _cb

    # ---- Topic existence helpers ----
    def _topic_exists(self, name: str, typ: Optional[str] = None) -> bool:
        topics = dict(self.get_topic_names_and_types())
        if name not in topics:
            return False
        if typ is None:
            return True
        return typ in topics.get(name, [])

    # ---- Reporting ----
    def _report(self):
        if not self.enabled:
            return
        now = time.monotonic()

        # Check presence of key topics
        if not self._topic_exists(self.cmd_topic, 'geometry_msgs/msg/TwistStamped'):
            self.get_logger().error(f"topic missing: {self.cmd_topic} (geometry_msgs/msg/TwistStamped)")
        for sid in self.servo_ids:
            cmd_t = f"{self.cmd_prefix}{sid}/cmd_position"
            state_t = f"{self.state_prefix}{sid}/state"
            if not self._topic_exists(cmd_t, 'moteus_msgs/msg/PositionCommand'):
                self.get_logger().error(f"topic missing: {cmd_t} (moteus_msgs/msg/PositionCommand)")
            if not self._topic_exists(state_t, 'moteus_msgs/msg/ControllerState'):
                self.get_logger().error(f"topic missing: {state_t} (moteus_msgs/msg/ControllerState)")

        # cmd_vel stats (EXPICIT if/else — bez dynamicznej metody)
        cmd_hz_win = self.cmd_vel_win.hz()
        cmd_age = float('inf') if self.last_seen_cmd_vel is None else (now - self.last_seen_cmd_vel)
        jmin, jmax = self.cmd_vel_win.jitter_minmax()
        if self.last_seen_cmd_vel is None:
            self.get_logger().error(f"no messages received on {self.cmd_topic} since start")
        else:
            msg = f"cmd_vel_out: {cmd_hz_win:.1f} Hz (age {cmd_age:.3f}s, jitter {jmin:.3f}-{jmax:.3f}s)"
            if cmd_hz_win < self.warn_cmd_vel_hz_min or cmd_age > 0.5:
                self.get_logger().warn(msg)
            else:
                self.get_logger().info(msg)

        stale_count = 0

        for sid in self.servo_ids:
            hz = self.can_rm_by_id[sid].value()
            hzw = self.can_win_by_id[sid].hz()
            age = float('inf') if self.last_seen_can[sid] is None else (now - self.last_seen_can[sid])
            state_age = float('inf') if self.last_seen_state[sid] is None else (now - self.last_seen_state[sid])

            if self.last_seen_can[sid] is None:
                self.get_logger().error(f"id {sid}: no messages received on {self.cmd_prefix}{sid}/cmd_position since start")
            if self.last_seen_state[sid] is None:
                self.get_logger().error(f"id {sid}: no messages received on {self.state_prefix}{sid}/state since start")

            if state_age > self.heartbeat_timeout_s:
                stale_count += 1

            fault_code = self.latest_fault.get(sid)
            fault_name = decode_fault_code(fault_code)
            mode = self.latest_mode.get(sid)
            v = self.latest_voltage.get(sid)
            mt = self.latest_motor_temp.get(sid)

            # Log zmiany kodu fault
            pf = self.prev_fault.get(sid)
            if pf != fault_code:
                if fault_code not in (None, 0):
                    self.get_logger().error(f"id {sid}: FAULT set {fault_code} ({fault_name})")
                elif pf not in (None, 0):
                    self.get_logger().info(f"id {sid}: fault cleared (prev {pf})")

            # Progi napięcia i temperatury
            if v is not None and (v < self.v_error):
                self.get_logger().error(f"id {sid}: LOW VOLTAGE {v:.2f}V < {self.v_error:.2f}V")
            elif v is not None and (v < self.v_warn):
                self.get_logger().warn(f"id {sid}: Low voltage {v:.2f}V < {self.v_warn:.2f}V")

            if mt is not None and (mt > self.motor_temp_error):
                self.get_logger().error(f"id {sid}: MOTOR HOT {mt:.1f}C > {self.motor_temp_error:.1f}C")
            elif mt is not None and (mt > self.motor_temp_warn):
                self.get_logger().warn(f"id {sid}: motor warm {mt:.1f}C > {self.motor_temp_warn:.1f}C")

            # Linia zbiorcza (EXPICIT if/else — bez dynamicznej metody)
            line = (
                f"id {sid}: {hz:.1f}Hz (win {hzw:.1f}Hz) age {age:.3f}s state_age {state_age:.3f}s "
                f"fault={fault_code} ({fault_name}) mode={mode} V={v if v is not None else 'NA'} Tm={mt if mt is not None else 'NA'}"
            )
            warn_line = (self.can_win_by_id[sid].hz() < self.warn_can_hz_min) or \
                        (self.last_seen_can[sid] is not None and (now - self.last_seen_can[sid]) > 0.5)
            if warn_line:
                self.get_logger().warn(line)
            else:
                self.get_logger().info(line)

        if stale_count >= max(2, len(self.servo_ids)//2):
            self.get_logger().error(
                f"Suspected bus problem: {stale_count}/{len(self.servo_ids)} state heartbeats stale > {self.heartbeat_timeout_s}s"
            )

        self.get_logger().info(f"cmd→state latency: ema {self.lat_ema*1000:.1f} ms, max {self.lat_max*1000:.1f} ms")

def main(args=None):
    rclpy.init(args=args)
    node = BoxyGoMoteusDiagnostic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
