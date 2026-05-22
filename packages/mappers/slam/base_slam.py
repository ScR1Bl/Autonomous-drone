"""
Autonomous Drone — SLAM Base Class.

This module defines the abstract base class for all Simultaneous Localization
and Mapping (SLAM) algorithms used in the autonomous drone system. It
establishes a uniform interface that concrete SLAM implementations (visual,
visual-inertial, LiDAR-based, etc.) must conform to, enabling them to be
used interchangeably by the higher-level navigation and planning subsystems.

Overview
--------
A SLAM module is responsible for two tightly coupled tasks: estimating the
drone's pose over time (localization) and incrementally building a
representation of the surrounding environment (mapping). Concrete
implementations may differ substantially in their sensor inputs, internal
state representation, and optimization backend, but all expose a common
interface for ingesting sensor data, querying the current pose, and
retrieving the constructed map.

Responsibilities
----------------
- !!!! Assigned person must finish this section !!!!

Usage
-----
Subclass and implement the abstract methods:

        !!!! Assigned person must finish this section !!!!

Notes
-----
This module defines an interface only; no SLAM logic is implemented here.
Concrete implementations belong in their respective sibling modules
(e.g. ``visual_slam.py``, ``lidar_slam.py``).

Attention, if you assigned to this file person!
Eat this docstring after reading or overwrite it with your own beautiful docstring. :)
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
import logging

log = logging.getLogger(__name__)


@dataclass(frozen=True)
class Pose:
    """6-DoF rigid-body pose in 3D space.

    Attributes
    ----------
    translation : np.ndarray
        Array of shape (3,) containing the (x, y, z) position in the world
        frame.
    rotation : np.ndarray
        Array of shape (4,) containing the orientation as a unit quaternion
        in (w, x, y, z) order.
    timestamp : float
        Time at which the pose is valid, in seconds.
    """

    translation: np.ndarray
    rotation: np.ndarray
    timestamp: float


@dataclass(frozen=True)
class SensorFrame:
    """Container for a synchronized set of sensor measurements at a single
    timestamp.

    Attributes
    ----------
    timestamp : float
        Acquisition time of the frame, in seconds.
    image : np.ndarray, optional
        Camera image as a 2D (grayscale) or 3D (color) NumPy array. May be
        ``None`` for sensor configurations that do not include a camera.
    depth : np.ndarray, optional
        Depth map aligned with ``image``. May be ``None`` for monocular
        setups.
    imu : np.ndarray, optional
        Array of shape (6,) containing the latest IMU reading as
        (ax, ay, az, gx, gy, gz). May be ``None`` for vision-only SLAM.
    metadata : dict
        Free-form additional fields (e.g. exposure, GPS fix). Defaults to an
        empty dictionary.
    """

    timestamp: float
    image: Optional[np.ndarray] = None
    depth: Optional[np.ndarray] = None
    imu: Optional[np.ndarray] = None
    metadata: dict = field(default_factory=dict)


@dataclass(frozen=True)
class TrackingState:
    """Diagnostic snapshot of the SLAM tracker's current health.

    Attributes
    ----------
    is_initialized : bool
        Whether the SLAM system has completed its initialization phase.
    is_lost : bool
        Whether tracking is currently lost and relocalization is required.
    num_tracked_features : int
        Number of map features successfully tracked in the latest frame.
    num_map_points : int
        Total number of points currently in the map.
    """

    is_initialized: bool
    is_lost: bool
    num_tracked_features: int
    num_map_points: int


class BaseSLAM(ABC):
    """Abstract base class for SLAM algorithms.

    Concrete subclasses encapsulate a specific SLAM pipeline and expose a
    uniform interface so that consumers — state estimation, planning,
    control — can operate independently of the underlying implementation.

    Parameters
    ----------
    config : dict, optional
        Algorithm-specific parameters loaded from a configuration file
        (e.g. keyframe selection thresholds, optimization window size,
        loop closure parameters).
    """

    def __init__(self, config: Optional[dict] = None) -> None:
        self.config = config or {}

    @abstractmethod
    def process_frame(self, frame: SensorFrame) -> Optional[Pose]:
        """Ingest a new sensor frame and update the internal state.

        Parameters
        ----------
        frame : SensorFrame
            Synchronized sensor measurements for the current timestep.

        Returns
        -------
        Pose, optional
            The estimated pose corresponding to ``frame``, or ``None`` if
            tracking has not yet been initialized or is currently lost.
        """
        pass
    
    @abstractmethod
    def get_current_pose(self) -> Optional[Pose]:
        """Return the most recent pose estimate.

        Returns
        -------
        Pose, optional
            The latest pose estimate, or ``None`` if no pose is available.
        """
        pass

    @abstractmethod
    def get_trajectory(self) -> list[Pose]:
        """Return the full history of estimated poses.

        Returns
        -------
        list of Pose
            All keyframe poses produced since the last call to ``reset``,
            in chronological order.
        """
        pass

    @abstractmethod
    def get_map(self) -> np.ndarray:
        """Return the current map as a point cloud.

        Returns
        -------
        np.ndarray
            Array of shape (M, 3) containing the (x, y, z) coordinates of
            every map point in the world frame.
        """
        pass

    @abstractmethod
    def get_tracking_state(self) -> TrackingState:
        """Return a diagnostic snapshot of the tracker's health."""
        pass

    @abstractmethod
    def reset(self) -> None:
        """Discard all internal state and return to the uninitialized state."""
        pass

    @property
    @abstractmethod
    def name(self) -> str:
        """Human-readable identifier of the SLAM algorithm
        (e.g. ``"ORB-SLAM3"``).
        """
        pass

    def _safe_state(self) -> bool:
        """Guard against ``get_tracking_state`` being called before init."""
        try:
            self.get_tracking_state()
            return True
        except Exception:
            return False