import cv2
import numpy as np

from .base_detector import BaseFeatureDetector, DetectionResult


class ORBDetector(BaseFeatureDetector):
    """ORB-style feature detector scaffold.

    This class currently implements the detector side of ORB:
    FAST keypoint detection, corner scoring, per-level non-maximum
    suppression, and an image pyramid. Descriptor extraction is intentionally
    left as an empty placeholder so BRIEF can be added next.
    """

    name = "ORB"
    descriptor_dim = 0

    def __init__(
        self,
        max_keypoints=None,
        config=None,
        threshold=40,
        successive_threshold=9,
        dist_threshold=10,
        levels=8,
        scale_factor=1.2,
        min_blur=1.0,
        max_blur=1.7,
    ):
        super().__init__(max_keypoints=max_keypoints, config=config)
        self.threshold = self.config.get("threshold", threshold)
        self.successive_threshold = self.config.get("successive_threshold", successive_threshold)
        self.dist_threshold = self.config.get("dist_threshold", dist_threshold)
        self.levels = self.config.get("levels", levels)
        self.scale_factor = self.config.get("scale_factor", scale_factor)
        self.min_blur = self.config.get("min_blur", min_blur)
        self.max_blur = self.config.get("max_blur", max_blur)
        self.circle = [
            (0, 3), (0, 4), (1, 5), (2, 6),
            (3, 6), (4, 6), (5, 5), (6, 4),
            (6, 3), (6, 2), (5, 1), (4, 0),
            (3, 0), (2, 0), (1, 1), (0, 2),
        ]

    def detect(self, image: np.ndarray) -> DetectionResult:
        gray = self._to_gray(image)
        pyramid = self._build_pyramid(gray)
        features = []

        for level, pyramid_img in enumerate(pyramid):
            scale = self.scale_factor ** level
            level_dist_threshold = max(3, self.dist_threshold / scale)
            keypoints, scores = self._fast_corner_detection(pyramid_img)
            keypoints, scores = self._nms(keypoints, scores, level_dist_threshold)

            for (x_level, y_level), score in zip(keypoints, scores):
                features.append((
                    int(x_level * scale),
                    int(y_level * scale),
                    float(score),
                ))

        features = self._limit_features(features)
        keypoints = np.array([(x, y) for x, y, _ in features], dtype=np.float32)
        scores = np.array([score for _, _, score in features], dtype=np.float32)
        descriptors = np.empty((len(features), self.descriptor_dim), dtype=np.uint8)

        return DetectionResult(keypoints=keypoints, descriptors=descriptors, scores=scores)

    def _to_gray(self, image: np.ndarray) -> np.ndarray:
        if image.ndim == 2:
            return image
        return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    def _fast_corner_detection(self, image: np.ndarray):
        keypoints = []
        scores = []
        s = self.successive_threshold
        t = self.threshold

        for n in range(image.shape[1] - 7):
            for m in range(image.shape[0] - 7):
                brighter_count = 0
                darker_count = 0
                brighter_differences = []
                darker_differences = []
                is_keypoint = False
                best_score = 0
                image_window = image[m:m + 7, n:n + 7]
                center = int(image_window[3, 3])

                for i in range(len(self.circle) + s - 1):
                    x, y = self.circle[i % len(self.circle)]
                    pixel = int(image_window[x, y])

                    if pixel >= center + t:
                        brighter_count += 1
                        darker_count = 0
                        difference = pixel - center
                        brighter_differences.append(difference)
                        darker_differences = []
                    elif pixel <= center - t:
                        darker_count += 1
                        brighter_count = 0
                        difference = center - pixel
                        darker_differences.append(difference)
                        brighter_differences = []
                    else:
                        brighter_count = 0
                        darker_count = 0
                        brighter_differences = []
                        darker_differences = []

                    if brighter_count >= s:
                        is_keypoint = True
                        best_score = max(best_score, sum(brighter_differences[-s:]))
                    elif darker_count >= s:
                        is_keypoint = True
                        best_score = max(best_score, sum(darker_differences[-s:]))

                if is_keypoint:
                    keypoints.append((n + 3, m + 3))
                    scores.append(best_score)

        return keypoints, scores

    def _nms(self, keypoints, scores, dist_threshold):
        if len(keypoints) == 0:
            return [], []

        order = np.argsort(scores)[::-1]
        keypoints = np.array(keypoints)[order]
        scores = np.array(scores)[order]
        selected_keypoints = []
        selected_scores = []

        for i, (x, y) in enumerate(keypoints):
            if len(selected_keypoints) == 0:
                selected_keypoints.append((int(x), int(y)))
                selected_scores.append(scores[i])
                continue

            distances = np.linalg.norm(np.array(selected_keypoints) - np.array([x, y]), axis=1)
            if distances.min() > dist_threshold:
                selected_keypoints.append((int(x), int(y)))
                selected_scores.append(scores[i])

        return selected_keypoints, selected_scores

    def _build_pyramid(self, image: np.ndarray):
        pyramid = []

        for level in range(self.levels):
            if self.levels == 1:
                blur_power = self.min_blur
            else:
                blur_power = self.min_blur + (self.max_blur - self.min_blur) * level / (self.levels - 1)

            scale = self.scale_factor ** level
            blurred = cv2.GaussianBlur(image, (5, 5), blur_power)
            resized = cv2.resize(
                blurred,
                dsize=None,
                fx=1 / scale,
                fy=1 / scale,
                interpolation=cv2.INTER_AREA,
            )
            pyramid.append(resized)

        return pyramid

    def _limit_features(self, features):
        if self.max_keypoints is None or len(features) <= self.max_keypoints:
            return features
        return sorted(features, key=lambda feature: feature[2], reverse=True)[:self.max_keypoints]


ORB = ORBDetector
