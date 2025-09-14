#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from moteus_msgs.msg import PositionCommand
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

class BoxyGoStopMoteus(Node):
    """
    Tryby:
      - 'stop'  : ustaw remote_node.enabled=false, wyślij JEDNORAZOWO /<prefix>/id_X/cmd_stop, zakończ
      - 'brake' : jw., ale cmd_brake
      - 'zero-velocity': jw., ale PositionCommand (position=[nan], velocity=[0.0], opcjonalnie watchdog_timeout=[nan])
    """
    def __init__(self):
        super().__init__('boxygo_stop_moteus')

        # --- Parametry ---
        self.declare_parameter('servo_ids', [1, 2, 3, 4, 5, 6])
        self.declare_parameter('method', 'stop')                  # 'stop' | 'brake' | 'zero-velocity'
        self.declare_parameter('topic_prefix', '/boxygo_moteus')  # np. '/moteus'
        # zero-velocity
        self.declare_parameter('max_torque', 0.0)
        self.declare_parameter('watchdog_timeout', float('nan'))
        # zdalne wyłączenie nadawcy komend
        self.declare_parameter('remote_node', '/boxygo_moteus')   # ścieżka do noda publikującego
        self.declare_parameter('disable_remote', True)            # czy wyłączyć zdalnie publikację
        self.declare_parameter('disable_timeout_s', 1.5)          # timeout na usługę

        self.servo_ids = list(self.get_parameter('servo_ids').value)
        self.method = str(self.get_parameter('method').value)
        self.prefix = str(self.get_parameter('topic_prefix').value)
        self.max_torque = float(self.get_parameter('max_torque').value)
        self.watchdog_override = float(self.get_parameter('watchdog_timeout').value)
        self.remote_node = str(self.get_parameter('remote_node').value)
        self.disable_remote = bool(self.get_parameter('disable_remote').value)
        self.disable_timeout_s = float(self.get_parameter('disable_timeout_s').value)

        if self.method not in ('stop', 'brake', 'zero-velocity'):
            raise ValueError("Parametr 'method' musi być jednym z: 'stop' | 'brake' | 'zero-velocity'")

        # Publishery do odpowiednich tematów
        self.pub_cmd_stop = {}
        self.pub_cmd_brake = {}
        self.pub_cmd_position = {}
        for sid in self.servo_ids:
            if self.method == 'stop':
                self.pub_cmd_stop[sid] = self.create_publisher(Empty, f'{self.prefix}/id_{sid}/cmd_stop', 10)
            elif self.method == 'brake':
                self.pub_cmd_brake[sid] = self.create_publisher(Empty, f'{self.prefix}/id_{sid}/cmd_brake', 10)
            else:
                self.pub_cmd_position[sid] = self.create_publisher(PositionCommand, f'{self.prefix}/id_{sid}/cmd_position', 10)

        # Timery (one-shot przez cancel w callbackach)
        self._timer_disable = self.create_timer(0.10, self._disable_remote_once)
        self._timer_publish = self.create_timer(0.35, self._publish_once)
        self._timer_shutdown = self.create_timer(0.80, self._shutdown_once)

        self.get_logger().info(f"mode={self.method}, prefix={self.prefix}, remote='{self.remote_node}', disable_remote={self.disable_remote}")

    # --- 1) Wyłącz zdalnego publishera przez parametr enabled=false ---
    def _disable_remote_once(self):
        self._timer_disable.cancel()
        if not self.disable_remote or not self.remote_node:
            return
        try:
            cli = self.create_client(SetParameters, f'{self.remote_node}/set_parameters')
            if not cli.wait_for_service(timeout_sec=self.disable_timeout_s):
                self.get_logger().warn(f"Brak serwisu parametrów: {self.remote_node}/set_parameters")
                return
            req = SetParameters.Request()
            p = Parameter()
            p.name = 'enabled'
            p.value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=False)
            req.parameters = [p]
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=self.disable_timeout_s)
            if fut.result() is not None:
                self.get_logger().info(f"Ustawiono {self.remote_node}.enabled = false")
            else:
                self.get_logger().warn("Ustawienie parametru: brak odpowiedzi (timeout).")
        except Exception as e:
            self.get_logger().error(f'Błąd ustawiania parametru: {e}')

    # --- 2) Jednorazowa publikacja komendy zatrzymania ---
    def _publish_once(self):
        self._timer_publish.cancel()
        if self.method == 'stop':
            msg = Empty()
            for pub in self.pub_cmd_stop.values():
                pub.publish(msg)
        elif self.method == 'brake':
            msg = Empty()
            for pub in self.pub_cmd_brake.values():
                pub.publish(msg)
        else:  # zero-velocity
            for pub in self.pub_cmd_position.values():
                cmd = PositionCommand()
                cmd.position = [math.nan]
                cmd.velocity = [0.0]
                if self.max_torque > 0.0:
                    cmd.maximum_torque = [self.max_torque]
                # dla bezpieczeństwa: zatrzymaj watchdog motusa, aby nie „wypuścił” ruchu
                cmd.watchdog_timeout = [float('nan') if math.isnan(self.watchdog_override)
                                        else float(self.watchdog_override)]
                pub.publish(cmd)

    # --- 3) Zakończenie noda ---
    def _shutdown_once(self):
        self._timer_shutdown.cancel()
        self.get_logger().info('Stop wykonany — zamykam node.')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = BoxyGoStopMoteus()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
