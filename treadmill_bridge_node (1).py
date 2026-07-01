#!/usr/bin/env python3
"""
Treadmill ROS2 Bridge Node
---------------------------------------
Bridges the ESP32 serial protocol to ROS2.

--- HOW THE BIDIRECTIONAL LOOP WORKS ---

  Robot locomotion
       │ publishes target walking speed
       ▼
  /vel_cmd (geometry_msgs/Twist)
       │ linear.x → belt m/s
       ▼
  [this node] ──serial──► ESP32 ──PWM──► treadmill belt
                                              │
                                         roller sensor
                                              │
  [this node] ◄──serial── ESP32 ◄──────── pulse count
       │
       ▼
  /platform/measured_velocity (std_msgs/Float32)   ← robot subscribes to THIS
       │                                              and adjusts gait to match
       ▼
  Robot locomotion controller closes the loop

The robot's locomotion stack must:
  1. Subscribe to /platform/measured_velocity and use it as ground-truth belt speed.
  2. Publish to /robot/heartbeat (std_msgs/Bool, any value, ~10 Hz) so this node
     knows the robot is alive.  If the heartbeat topic goes stale this node
     commands the belt to stop.
  3. Optionally wire a robot-computer GPIO to the ESP32 HEARTBEAT_PIN (pin 33)
     toggling at ~10 Hz for a firmware-level safety net that bypasses ROS entirely.

Services:
    /platform/start   (std_srvs/Trigger)  -> firmware 'S'
    /platform/stop    (std_srvs/Trigger)  -> firmware 'X'

Subscriber (commands):
    /vel_cmd          (geometry_msgs/Twist)  linear.x -> belt m/s
    /robot/heartbeat  (std_msgs/Bool)        any value, just needs to arrive

Publishers (feedback):
    /platform/state              (std_msgs/String)   IDLE / RUNNING / ESTOP
    /platform/measured_velocity  (std_msgs/Float32)  m/s  ← robot should subscribe
    /platform/tracking_error     (std_msgs/Float32)  m/s  (target − measured)
    /platform/estop              (std_msgs/Bool)

Run:
    python3 treadmill_bridge_node.py --ros-args -p port:=/dev/ttyUSB0
"""

import threading

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String, Float32, Bool
from geometry_msgs.msg import Twist

import serial


class TreadmillBridge(Node):
    def __init__(self):
        super().__init__('treadmill_bridge')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('v_max', 3.0)
        self.declare_parameter('vel_cmd_timeout_s', 0.5)    # freshness of /vel_cmd
        self.declare_parameter('heartbeat_timeout_s', 1.0)  # freshness of /robot/heartbeat
        self.declare_parameter('resend_hz', 20.0)

        port      = self.get_parameter('port').value
        baud      = int(self.get_parameter('baud').value)
        self.v_max            = float(self.get_parameter('v_max').value)
        self.vel_cmd_timeout  = float(self.get_parameter('vel_cmd_timeout_s').value)
        self.hb_timeout       = float(self.get_parameter('heartbeat_timeout_s').value)
        resend_hz             = float(self.get_parameter('resend_hz').value)

        # --- serial ---
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
        except serial.SerialException as e:
            self.get_logger().error(f'Could not open {port}: {e}')
            raise
        self._tx_lock = threading.Lock()
        self.get_logger().info(f'Connected to ESP32 on {port} @ {baud}')

        # --- command state ---
        self._last_v          = 0.0
        self._last_cmd_time   = self.get_clock().now()
        self._last_hb_time    = self.get_clock().now()  # heartbeat from robot
        self._platform_state  = 'IDLE'
        self._hb_ever_received = False  # don't warn before robot has started

        # --- publishers ---
        self.pub_state  = self.create_publisher(String,  'platform/state', 10)
        self.pub_meas   = self.create_publisher(Float32, 'platform/measured_velocity', 10)
        self.pub_err    = self.create_publisher(Float32, 'platform/tracking_error', 10)
        self.pub_estop  = self.create_publisher(Bool,    'platform/estop', 10)

        # --- subscribers ---
        self.create_subscription(Twist, 'vel_cmd',         self.on_vel_cmd,   10)
        self.create_subscription(Bool,  'robot/heartbeat', self.on_heartbeat, 10)

        # --- services ---
        self.create_service(Trigger, 'platform/start', self.on_start)
        self.create_service(Trigger, 'platform/stop',  self.on_stop)

        # --- resend / watchdog timer ---
        self.create_timer(1.0 / resend_hz, self.resend_velocity)

        # --- serial reader thread ---
        self._running = True
        self._reader  = threading.Thread(target=self._read_serial, daemon=True)
        self._reader.start()

    # ------------------------------------------------------------------ tx
    def _send(self, line: str):
        with self._tx_lock:
            try:
                self.ser.write((line + '\n').encode('ascii'))
            except serial.SerialException as e:
                self.get_logger().error(f'serial write failed: {e}')

    # ------------------------------------------------------------------ services
    def on_start(self, request, response):
        self._last_hb_time = self.get_clock().now()  # grace period
        self._send('S')
        response.success = True
        response.message = 'platform start requested'
        self.get_logger().info('START')
        return response

    def on_stop(self, request, response):
        self._last_v = 0.0
        self._send('X')
        response.success = True
        response.message = 'platform stop requested'
        self.get_logger().info('STOP')
        return response

    # ------------------------------------------------------------------ subscribers
    def on_vel_cmd(self, msg: Twist):
        v = max(0.0, min(self.v_max, float(msg.linear.x)))
        self._last_v        = v
        self._last_cmd_time = self.get_clock().now()

    def on_heartbeat(self, msg: Bool):
        """
        Robot publishes any Bool to /robot/heartbeat at ~10 Hz.
        We just care that messages keep arriving — the value is irrelevant.
        """
        self._last_hb_time     = self.get_clock().now()
        self._hb_ever_received = True

    # ------------------------------------------------------------------ resend timer
    def resend_velocity(self):
        now = self.get_clock().now()

        vel_age = (now - self._last_cmd_time).nanoseconds * 1e-9
        hb_age  = (now - self._last_hb_time).nanoseconds  * 1e-9

        # /robot/heartbeat gone stale while platform is RUNNING -> stop the belt
        if (self._hb_ever_received
                and self._platform_state == 'RUNNING'
                and hb_age > self.hb_timeout):
            self.get_logger().warn(
                f'Robot heartbeat stale ({hb_age:.1f}s) -> commanding belt stop',
                throttle_duration_sec=2.0)
            self._last_v = 0.0
            self._send('X')
            return

        # /vel_cmd stale -> send 0 but keep state (firmware has its own watchdog too)
        if vel_age > self.vel_cmd_timeout and self._last_v != 0.0:
            self.get_logger().warn('vel_cmd stale -> commanding 0',
                                   throttle_duration_sec=2.0)
            self._last_v = 0.0

        self._send(f'V{self._last_v:.3f}')

    # ------------------------------------------------------------------ serial rx
    def _read_serial(self):
        buf = b''
        while self._running and rclpy.ok():
            try:
                data = self.ser.read(128)
            except serial.SerialException:
                continue
            if not data:
                continue
            buf += data
            while b'\n' in buf:
                raw, buf = buf.split(b'\n', 1)
                self._handle_line(raw.decode('ascii', errors='ignore').strip())

    def _handle_line(self, line: str):
        if not line:
            return
        if line.startswith('T,'):
            # T,state,target,measured,duty,estop,err
            p = line.split(',')
            if len(p) != 7:
                return
            try:
                state    = p[1]
                measured = float(p[3])
                estop    = int(p[5])
                err      = float(p[6])
            except ValueError:
                return
            self._platform_state = state
            self.pub_state.publish(String(data=state))
            self.pub_meas.publish(Float32(data=measured))
            self.pub_err.publish(Float32(data=err))
            self.pub_estop.publish(Bool(data=bool(estop)))
        elif line.startswith('#'):
            self.get_logger().info(f'[esp32] {line[1:].strip()}')
        elif line.startswith('CAL,'):
            self.get_logger().info(f'[calib] {line}')

    # ------------------------------------------------------------------ cleanup
    def destroy_node(self):
        self._running = False
        try:
            self._send('X')
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TreadmillBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
