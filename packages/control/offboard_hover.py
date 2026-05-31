#!/usr/bin/env python3
"""
Offboard Takeoff + Hover + Land
================================
Pierwszy node ROS2 sterujący dronem PX4 w trybie Offboard.

Sekwencja:
1. Publikuj setpointy przez 2s (PX4 wymaga strumienia przed przełączeniem w Offboard)
2. Armuj drona
3. Przełącz w tryb Offboard
4. Leć do pozycji (0, 0, -2) NED = 2m w górę
5. Po 10s hoveru — ląduj

Uruchomienie:
    source /opt/ros/jazzy/setup.bash
    source ~/Desktop/PROJEKTY/SIiUM/Projekt-zespolowy/Dron/ros2_ws/install/setup.bash
    python3 offboard_hover.py

Wymagania:
    - MicroXRCEAgent udp4 -p 8888 (Terminal 1)
    - PX4 SITL gz_x500 (Terminal 2)
    - QGroundControl (Terminal 3)
    - px4_msgs zbudowane w ros2_ws

Dostosowane do PX4 v1.17-alpha (topiki z sufiksami _v1, _v4).
"""

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


class OffboardHover(Node):
    """Node realizujący sekwencję: arm → offboard → hover → land."""

    def __init__(self):
        super().__init__("offboard_hover")

        # QoS profil kompatybilny z PX4 uXRCE-DDS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # === Publishery (komendy DO drona) ===
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos
        )
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos
        )
        self.command_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos
        )

        # === Subscribery (dane Z drona) ===
        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",  # v1.17-alpha suffix
            self.on_position,
            qos,
        )
        self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status_v4",  # v1.17-alpha suffix
            self.on_status,
            qos,
        )

        # === Stan wewnętrzny ===
        self.position = None  # ostatnia znana pozycja
        self.vehicle_status = None  # ostatni znany status
        self.offboard_setpoint_counter = 0  # ile setpointów wysłaliśmy
        self.start_time = self.get_clock().now()
        self.hover_start_time = None  # kiedy zaczęliśmy hover
        self.state = "PREFLIGHT"  # maszyna stanów

        # Parametry misji
        self.TARGET_ALTITUDE = -2.0  # NED: -2 = 2m w górę
        self.HOVER_DURATION = 10.0  # sekundy hoveru
        self.PREFLIGHT_SETPOINTS = 20  # ile setpointów przed armowaniem (2s @ 10Hz)

        # Timer główny — 10 Hz (co 100ms)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("=== Offboard Hover Node started ===")
        self.get_logger().info(f"Plan: hover at {-self.TARGET_ALTITUDE}m for {self.HOVER_DURATION}s, then land")

    # ─── Callbacki subskrypcji ───────────────────────────────────

    def on_position(self, msg: VehicleLocalPosition):
        """Odbiera pozycję drona z EKF2."""
        self.position = msg

    def on_status(self, msg: VehicleStatus):
        """Odbiera status drona (arming state, nav state)."""
        self.vehicle_status = msg

    # ─── Główna pętla (10 Hz) ────────────────────────────────────

    def timer_callback(self):
        """Maszyna stanów wywoływana co 100ms."""

        # ZAWSZE publikuj offboard mode + setpoint (PX4 wymaga ciągłego strumienia)
        self.publish_offboard_mode()
        self.publish_setpoint()
        self.offboard_setpoint_counter += 1

        # Maszyna stanów
        if self.state == "PREFLIGHT":
            # Czekaj na wystarczającą liczbę setpointów przed armowaniem
            if self.offboard_setpoint_counter >= self.PREFLIGHT_SETPOINTS:
                self.get_logger().info("Preflight complete. Arming...")
                self.send_arm_command()
                self.send_offboard_command()
                self.state = "ARMING"

        elif self.state == "ARMING":
            # Czekaj na potwierdzenie armowania
            if self.is_armed():
                self.get_logger().info("Armed! Switching to HOVER state.")
                self.hover_start_time = self.get_clock().now()
                self.state = "HOVER"
            else:
                # Ponawiaj komendy co sekundę (na wypadek, gdyby pierwsza nie doszła)
                if self.offboard_setpoint_counter % 10 == 0:
                    self.send_arm_command()
                    self.send_offboard_command()

        elif self.state == "HOVER":
            # Loguj pozycję co sekundę
            elapsed = (self.get_clock().now() - self.hover_start_time).nanoseconds / 1e9
            if self.offboard_setpoint_counter % 10 == 0:
                z = self.position.z if self.position else float("nan")
                self.get_logger().info(
                    f"Hovering: {elapsed:.1f}s / {self.HOVER_DURATION}s | "
                    f"altitude: {-z:.2f}m (target: {-self.TARGET_ALTITUDE}m)"
                )

            # Po HOVER_DURATION sekund — ląduj
            if elapsed >= self.HOVER_DURATION:
                self.get_logger().info("Hover complete. Landing...")
                self.send_land_command()
                self.state = "LANDING"

        elif self.state == "LANDING":
            # Czekaj na disarmowanie (= dron wylądował)
            if not self.is_armed():
                self.get_logger().info("Landed and disarmed. Mission complete!")
                self.state = "DONE"

        elif self.state == "DONE":
            self.get_logger().info("Mission finished. Shutting down.")
            self.timer.cancel()
            raise SystemExit  # czyste wyjście

    # ─── Publishery ──────────────────────────────────────────────

    def publish_offboard_mode(self):
        """Informuje PX4, że sterujemy pozycją (nie prędkością/przyspieszeniem)."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(msg)

    def publish_setpoint(self):
        """Publikuje docelową pozycję: (0, 0, TARGET_ALTITUDE) w NED."""
        msg = TrajectorySetpoint()
        msg.position = [0.0, 0.0, self.TARGET_ALTITUDE]  # NED: x=N, y=E, z=Down
        msg.yaw = -3.14  # heading (radiany), -pi = South-facing
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_pub.publish(msg)

    # ─── Komendy do PX4 ─────────────────────────────────────────

    def send_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        """Wysyła VehicleCommand do PX4."""
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

    def send_arm_command(self):
        """Armuj silniki."""
        self.send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0,  # 1.0 = arm, 0.0 = disarm
        )
        self.get_logger().info("  → ARM command sent")

    def send_offboard_command(self):
        """Przełącz w tryb Offboard."""
        self.send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,  # custom mode
            param2=6.0,  # PX4 mode: 6 = Offboard
        )
        self.get_logger().info("  → OFFBOARD mode command sent")

    def send_land_command(self):
        """Przełącz w tryb Land."""
        self.send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_LAND,
        )
        self.get_logger().info("  → LAND command sent")

    # ─── Helpers ─────────────────────────────────────────────────

    def is_armed(self) -> bool:
        """Sprawdza, czy dron jest uzbrojony."""
        if self.vehicle_status is None:
            return False
        return self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED


def main():
    rclpy.init()
    node = OffboardHover()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
