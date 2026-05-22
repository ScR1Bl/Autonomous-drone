import cv2
import numpy as np

try:
    from .base_detector import Detector
except ImportError:
    from base_detector import Detector


class ORB(Detector):
    def __init__(
        self,
        img,
        threshold=40,
        successive_threshold=9,
        dist_threshold=10,
        levels=8,
        scale_factor=1.2,
        min_blur=1.0,
        max_blur=1.7,
    ):
        super().__init__(img)
        self.threshold = threshold
        self.successive_threshold = successive_threshold
        self.dist_threshold = dist_threshold
        self.levels = levels
        self.scale_factor = scale_factor
        self.min_blur = min_blur
        self.max_blur = max_blur
        self.pyramid = []
        self.features = []
        self.circle = [
            (0, 3), (0, 4), (1, 5), (2, 6),
            (3, 6), (4, 6), (5, 5), (6, 4),
            (6, 3), (6, 2), (5, 1), (4, 0),
            (3, 0), (2, 0), (1, 1), (0, 2),
        ]

    def fast_corner_detection(self, img):
        keypoints = []
        scores = []
        s = self.successive_threshold
        t = self.threshold

        for n in range(img.shape[1] - 7):
            for m in range(img.shape[0] - 7):
                brighter_count = 0
                darker_count = 0
                brighter_differences = []
                darker_differences = []
                is_keypoint = False
                best_score = 0
                img_window = img[m:m + 7, n:n + 7]
                center = int(img_window[3, 3])

                for i in range(len(self.circle) + s - 1):
                    x, y = self.circle[i % len(self.circle)]
                    pixel = int(img_window[x, y])

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

    def nms(self, keypoints, scores, dist_threshold=None):
        if len(keypoints) == 0:
            return [], []

        if dist_threshold is None:
            dist_threshold = self.dist_threshold

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

    def build_pyramid(self):
        self.pyramid = []

        for level in range(self.levels):
            if self.levels == 1:
                blur_power = self.min_blur
            else:
                blur_power = self.min_blur + (self.max_blur - self.min_blur) * level / (self.levels - 1)

            scale = self.scale_factor ** level
            img_blurred = cv2.GaussianBlur(self.img, (5, 5), blur_power)
            img_rescaled = cv2.resize(
                img_blurred,
                dsize=None,
                fx=1 / scale,
                fy=1 / scale,
                interpolation=cv2.INTER_AREA,
            )
            self.pyramid.append(img_rescaled)

        return self.pyramid

    def detect(self):
        if len(self.pyramid) == 0:
            self.build_pyramid()

        self.features = []

        for level, pyramid_img in enumerate(self.pyramid):
            scale = self.scale_factor ** level
            level_dist_threshold = max(3, self.dist_threshold / scale)
            keypoints, scores = self.fast_corner_detection(pyramid_img)
            keypoints, scores = self.nms(keypoints, scores, level_dist_threshold)

            for (x_level, y_level), score in zip(keypoints, scores):
                x_original = int(x_level * scale)
                y_original = int(y_level * scale)
                self.features.append({
                    "pt": (x_original, y_original),
                    "pt_level": (x_level, y_level),
                    "level": level,
                    "scale": scale,
                    "score": score,
                })

        return self.features
