#!/usr/bin/env python3
"""
Keyboard Teleop for PX4 Drone
==============================
Sterowanie dronem z klawiatury w trybie Offboard (velocity setpoints).

Sterowanie:
    W / S     — przód / tył
    A / D     — lewo / prawo
    ↑ / ↓     — w górę / w dół
    E / R     — obrót lewo / prawo (yaw)

    SPACE     — arm + offboard + takeoff
    L         — land
    +/-       — zmień prędkość (domyślnie 0.5 m/s)
    Q         — wyjście

Uruchomienie:
    source /opt/ros/jazzy/setup.bash
    source ~/Desktop/PROJEKTY/SIiUM/Projekt-zespolowy/Dron/ros2_ws/install/setup.bash
    python3 keyboard_teleop.py
"""

import sys
import tty
import termios
import select
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)

HELP_TEXT = """
╔═══════════════════════════════════════════╗
║        PX4 Keyboard Teleop                ║
╠═══════════════════════════════════════════╣
║                                           ║
║   W/S      — przód / tył                  ║
║   A/D      — lewo / prawo                 ║
║   ↑/↓      — w górę / w dół               ║
║   E/R      — obrót lewo / prawo (yaw)     ║
║                                           ║
║   SPACE    — arm + takeoff                ║
║   L        — land                         ║
║   +/-      — zmień prędkość               ║
║   Q        — wyjście                      ║
║                                           ║
╚═══════════════════════════════════════════╝
"""


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__("keyboard_teleop")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Publishery
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos
        )
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos
        )
        self.command_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos
        )

        # Subscribery
        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self.on_position,
            qos,
        )
        self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status_v4",
            self.on_status,
            qos,
        )

        # Stan
        self.position = None
        self.vehicle_status = None
        self.armed = False
        self.preflight_counter = 0

        # Sterowanie
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.target_yaw = 0.0      # docelowy heading (rad)
        self.yaw_initialized = False
        self.speed = 0.5
        self.yaw_step = 0.15       # ~8.6° na kliknięcie

        # Timer 20 Hz
        self.timer = self.create_timer(0.05, self.timer_callback)

        # Terminal raw mode
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        print(HELP_TEXT)
        self.print_status()

    def on_position(self, msg):
        self.position = msg
        if not self.yaw_initialized and msg.heading_good_for_control:
            self.target_yaw = msg.heading
            self.yaw_initialized = True

    def on_status(self, msg):
        self.vehicle_status = msg
        self.armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def timer_callback(self):
        self.process_key()
        self.publish_offboard_mode()
        self.publish_velocity_setpoint()
        self.preflight_counter += 1

    def process_key(self):
        if select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)

            # Strzałki (escape sequences)
            if key == '\x1b':
                if select.select([sys.stdin], [], [], 0.05)[0]:
                    key2 = sys.stdin.read(1)
                    if key2 == '[':
                        key3 = sys.stdin.read(1)
                        if key3 == 'A':    # ↑
                            self.vz = -self.speed
                            self.print_status("↑ W GÓRĘ")
                        elif key3 == 'B':  # ↓
                            self.vz = self.speed
                            self.print_status("↓ W DÓŁ")
                        elif key3 == 'C':  # → (prawo = to samo co D)
                            self.vy = self.speed
                            self.print_status("→ PRAWO")
                        elif key3 == 'D':  # ← (lewo = to samo co A)
                            self.vy = -self.speed
                            self.print_status("← LEWO")
                return

            key = key.lower()

            if key == 'w':
                self.vx = self.speed
                self.print_status("W → PRZÓD")
            elif key == 's':
                self.vx = -self.speed
                self.print_status("S → TYŁ")
            elif key == 'a':
                self.vy = -self.speed
                self.print_status("A → LEWO")
            elif key == 'd':
                self.vy = self.speed
                self.print_status("D → PRAWO")
            elif key == 'e':
                self.target_yaw += self.yaw_step
                # Normalizuj do [-pi, pi]
                self.target_yaw = math.atan2(
                    math.sin(self.target_yaw), math.cos(self.target_yaw)
                )
                yaw_deg = math.degrees(self.target_yaw)
                self.print_status(f"E → YAW LEWO ({yaw_deg:.0f}°)")
            elif key == 'r':
                self.target_yaw -= self.yaw_step
                self.target_yaw = math.atan2(
                    math.sin(self.target_yaw), math.cos(self.target_yaw)
                )
                yaw_deg = math.degrees(self.target_yaw)
                self.print_status(f"R → YAW PRAWO ({yaw_deg:.0f}°)")
            elif key == ' ':
                self.do_arm_and_takeoff()
            elif key == 'l':
                self.do_land()
            elif key == '+' or key == '=':
                self.speed = min(self.speed + 0.1, 3.0)
                self.print_status(f"Prędkość: {self.speed:.1f} m/s")
            elif key == '-':
                self.speed = max(self.speed - 0.1, 0.1)
                self.print_status(f"Prędkość: {self.speed:.1f} m/s")
            elif key == 'q':
                self.shutdown()
        else:
            # Brak klawisza — hamuj liniowe prędkości
            self.vx *= 0.8
            self.vy *= 0.8
            self.vz *= 0.8
            if abs(self.vx) < 0.01:
                self.vx = 0.0
            if abs(self.vy) < 0.01:
                self.vy = 0.0
            if abs(self.vz) < 0.01:
                self.vz = 0.0
            # yaw NIE hamuje — zostaje na ustawionym kącie

    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(msg)

    def publish_velocity_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position = [float('nan'), float('nan'), float('nan')]
        msg.velocity = [self.vx, self.vy, self.vz]
        msg.yaw = self.target_yaw  # yaw jako pozycja, nie rate
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_pub.publish(msg)

    def do_arm_and_takeoff(self):
        if self.armed:
            self.print_status("Już uzbrojony — leci!")
            return

        self.print_status("Armowanie + Offboard + Takeoff...")

        self.send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0,
        )
        self.send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0,
        )
        self.vz = -self.speed

    def do_land(self):
        self.print_status("Lądowanie...")
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

    def send_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.param7 = param7
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(msg)

    def print_status(self, action=""):
        alt = -self.position.z if self.position else 0.0
        yaw_deg = math.degrees(self.target_yaw)
        armed_str = "ARMED" if self.armed else "DISARMED"

        status = (
            f"\r[{armed_str}] alt: {alt:.1f}m | "
            f"vel: ({self.vx:.1f}, {self.vy:.1f}, {self.vz:.1f}) | "
            f"yaw: {yaw_deg:.0f}° | "
            f"speed: {self.speed:.1f} m/s | {action}          "
        )
        sys.stdout.write(status)
        sys.stdout.flush()

    def shutdown(self):
        self.print_status("Zamykanie...")
        self.do_land()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        print("\nBye!")
        self.timer.cancel()
        raise SystemExit


def main():
    rclpy.init()
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.old_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()