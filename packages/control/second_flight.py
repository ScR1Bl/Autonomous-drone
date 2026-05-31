#!/usr/bin/env python3
"""
Drugi node ROS2 sterujący dronem PX4 w trybie Offboard.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
    SensorCombined, # IMU
)


class OffboardHover(Node):

    def __init__(self):
        super().__init__("offboard_hover")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos
        )
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos
        )
        self.command_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos
        )

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

        # IMU
        self.imu_data = None
        self.create_subscription(
            SensorCombined,
            "/fmu/out/sensor_combined",
            self.on_imu,
            qos,
        )

        # Kamera
        self.bridge = CvBridge()
        self.frame_count = 0
        self.create_subscription(
            Image,
            "/camera_image",
            self.on_camera,
            10,
        )

        self.position = None
        self.vehicle_status = None
        self.offboard_setpoint_counter = 0
        self.start_time = self.get_clock().now()
        self.hover_start_time = None
        self.state = "PREFLIGHT"

        self.TARGET_ALTITUDE = -2.0
        self.HOVER_DURATION = 10.0
        self.PREFLIGHT_SETPOINTS = 20

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("=== Offboard Hover Node started ===")
        self.get_logger().info(f"Plan: hover at {-self.TARGET_ALTITUDE}m for {self.HOVER_DURATION}s, then land")

    def on_position(self, msg: VehicleLocalPosition):
        self.position = msg

    def on_status(self, msg: VehicleStatus):
        self.vehicle_status = msg

    def on_imu(self, msg: SensorCombined):
        """ BMI270 na Stamp Fly

        Odbiera surowe dane IMU (akcelerometr + żyroskop).

        # Akcelerometr — przyspieszenie liniowe [m/s²], 3 osie
        ax = msg.accelerometer_m_s2[0]   # X (przód)
        ay = msg.accelerometer_m_s2[1]   # Y (prawo)
        az = msg.accelerometer_m_s2[2]   # Z (dół) — na ziemi ≈ 9.81

        # Żyroskop — prędkość kątowa [rad/s], 3 osie
        gx = msg.gyro_rad[0]   # roll rate
        gy = msg.gyro_rad[1]   # pitch rate
        gz = msg.gyro_rad[2]   # yaw rate

        """
        self.imu_data = msg

    # def on_camera(self, msg: Image):
    #     """Odbiera obraz z kamery i zapisuje co 5-tą klatkę."""
    #     self.frame_count += 1
    #     frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    #
    #     # Zapisz co 5-tą klatkę (przy 2 FPS = co 2.5s)
    #     if self.frame_count % 5 == 0:
    #         filename = f"frame_{self.frame_count:04d}.jpg"
    #         cv2.imwrite(filename, frame)
    #         self.get_logger().info(
    #             f"Frame {self.frame_count} saved: {filename} "
    #             f"({frame.shape[1]}x{frame.shape[0]})"
    #         )

    def on_camera(self, msg: Image):
        """Wyświetla obraz z kamery na żywo."""
        self.frame_count += 1
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # print(type(frame))
        # print(frame)  # (240, 320, 3)
        print(frame.shape)
        # print(frame.dtype)  # uint8

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame_float = frame.astype(np.float32) / 255.0
        gray_float = gray.astype(np.float32) / 255.0

        cv2.imshow("Drone Camera (2 FPS)", frame)
        cv2.waitKey(1)

    def timer_callback(self):

        self.publish_offboard_mode()
        self.publish_setpoint()
        self.offboard_setpoint_counter += 1

        if self.state == "PREFLIGHT":
            if self.offboard_setpoint_counter >= self.PREFLIGHT_SETPOINTS:
                self.get_logger().info("Preflight complete. Arming...")
                self.send_arm_command()
                self.send_offboard_command()
                self.state = "ARMING"

        elif self.state == "ARMING":
            if self.is_armed():
                self.get_logger().info("Armed! Switching to HOVER state.")
                self.hover_start_time = self.get_clock().now()
                self.state = "HOVER"
            else:
                if self.offboard_setpoint_counter % 10 == 0:
                    self.send_arm_command()
                    self.send_offboard_command()

        elif self.state == "HOVER":
            elapsed = (self.get_clock().now() - self.hover_start_time).nanoseconds / 1e9
            if self.offboard_setpoint_counter % 10 == 0:
                z = self.position.z if self.position else float("nan")
                imu_str = ''
                if self.imu_data:
                    acc = self.imu_data.accelerometer_m_s2
                    gyr = self.imu_data.gyro_rad
                    imu_str = (
                        f" | acc: ({acc[0]:.2f}, {acc[1]:.2f}, {acc[2]:.2f})"
                        f" | gyr: ({gyr[0]:.3f}, {gyr[1]:.3f}, {gyr[2]:.3f})"
                    )
                self.get_logger().info(
                    f"Hovering: {elapsed:.1f}s / {self.HOVER_DURATION}s | "
                    f"altitude: {-z:.2f}m (target: {-self.TARGET_ALTITUDE}m)"
                    f"{imu_str}"
                )

            if elapsed >= self.HOVER_DURATION:
                self.get_logger().info("Hover complete. Landing...")
                self.send_land_command()
                self.state = "LANDING"

        elif self.state == "LANDING":
            if not self.is_armed():
                self.get_logger().info("Landed and disarmed. Mission complete!")
                self.state = "DONE"

        elif self.state == "DONE":
            self.get_logger().info("Mission finished. Shutting down.")
            self.timer.cancel()
            raise SystemExit  # czyste wyjście

    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(msg)

    def publish_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position = [0.0, 0.0, self.TARGET_ALTITUDE]
        msg.yaw = -3.14
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_pub.publish(msg)

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

    def send_arm_command(self):
        self.send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0,
        )
        self.get_logger().info("ARM command sent")

    def send_offboard_command(self):
        self.send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0,
        )
        self.get_logger().info("OFFBOARD mode command sent")

    def send_land_command(self):
        self.send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_LAND,
        )
        self.get_logger().info("LAND command sent")

    def is_armed(self) -> bool:
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
    cv2.destroyAllWindows()