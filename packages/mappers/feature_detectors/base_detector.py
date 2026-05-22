"""
Autonomous Drone — Feature Detector Base Class.

This module defines the abstract base class for all feature detection
algorithms used in the perception and mapping subsystems. It establishes
a uniform interface that concrete detectors (like ORB) must implement, 
enabling them to be used interchangeably throughout the pipeline.

Overview
--------
Feature detectors are responsible for identifying salient, repeatable points
in an input image — corners, blobs, or learned interest points — and
producing associated descriptors that allow these points to be matched
across frames. By enforcing a common interface, the rest of the system
(SLAM, visual odometry, loop closure) remains decoupled from the specific
algorithm in use, and detectors can be swapped via configuration alone.

Responsibilities
----------------
- !!!! Assigned person must finish this section !!!!

Usage
-----
Subclass and implement the abstract methods:

        !!!! Assigned person must finish this section !!!!

Notes
-----
This module defines an interface only; no detection logic is implemented
here. Concrete implementations belong in their respective sibling modules
(like ``orb_detector.py``).

Attention, if you assigned to this file person!
Eat this docstring after reading or overwrite it with your own beautiful docstring. :)
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional

import numpy as np

import logging

log = logging.getLogger(__name__)


@dataclass(frozen=True)
class DetectionResult:
    """Container for the output of a feature detection step.

    Attributes
    ----------
    keypoints : np.ndarray
        Array of shape (N, 2) containing pixel coordinates (x, y) of detected
        keypoints.
    descriptors : np.ndarray
        Array of shape (N, D) containing the descriptor vector for each
        keypoint. The dimensionality D depends on the concrete detector.
    scores : np.ndarray, optional
        Array of shape (N,) containing a per-keypoint confidence or response
        score. May be ``None`` for detectors that do not provide one.
    """

    keypoints: np.ndarray
    descriptors: np.ndarray
    scores: Optional[np.ndarray] = None


class BaseFeatureDetector(ABC):
    """Abstract base class for feature detection algorithms.

    Concrete subclasses encapsulate a specific detection algorithm (classical
    or learned) and expose a uniform interface so that consumers — SLAM,
    visual odometry, place recognition — can operate independently of the
    underlying implementation.

    Parameters
    ----------
    max_keypoints : int, optional
        Upper bound on the number of keypoints returned per image. If
        ``None``, the detector returns all keypoints it finds.
    config : dict, optional
        Algorithm-specific parameters loaded from a configuration file.
    """

    def __init__(
        self,
        max_keypoints: Optional[int] = None,
        config: Optional[dict] = None,
    ) -> None:
        self.max_keypoints = max_keypoints
        self.config = config or {}

    @abstractmethod
    def detect(self, image: np.ndarray) -> DetectionResult:
        """Detect keypoints and compute descriptors for a single image.

        Parameters
        ----------
        image : np.ndarray
            Input image as a 2D (grayscale) or 3D (color) NumPy array.

        Returns
        -------
        DetectionResult
            Detected keypoints, descriptors, and optional scores.
        """
        pass
    
    @property
    @abstractmethod
    def descriptor_dim(self) -> int:
        """Dimensionality of the descriptor vector produced by this detector."""
        pass

    @property
    @abstractmethod
    def name(self) -> str:
        """Human-readable identifier of the detector (e.g. ``"SIFT"``)."""
        pass
