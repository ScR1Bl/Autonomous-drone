import cv2
import numpy as np
from base_detector import Detector
import random

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
        patch_size = 9,
        disc_len = 256,
        seed = 42
    ):
        super().__init__(img)
        self.threshold = threshold
        self.successive_threshold = successive_threshold
        self.dist_threshold = dist_threshold
        self.levels = levels
        self.scale_factor = scale_factor
        self.min_blur = min_blur
        self.max_blur = max_blur
        self.patch_size = patch_size
        self.disc_len = disc_len
        self.seed = seed
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

    def orientation(self, img, patch_size, keypoints):
        angles = []
        for x, y in keypoints:
            r = patch_size // 2
            m10 = 0
            m01 = 0
            patch = img[y-r:y+r+1, x-r:x+r+1]
            if patch.shape != (patch_size, patch_size):
                angles.append(None)
                continue
            for dx in range(patch_size):
                for dy in range(patch_size):
                    I = float(patch[dy, dx])
                    m10 += (dx - r) * I
                    m01 += (dy - r) * I
            angle = np.arctan2(m01, m10)
            angle = np.rad2deg(angle) % np.rad2deg(2*np.pi)
            angles.append(angle)

        return angles

    def patterns_search(self, disc_len, patch_size, seed):
        random.seed(seed)
        pattern = []
        while len(pattern) < disc_len:
            p1 = (random.choice(range(patch_size)) - int(patch_size//2), random.choice(range(patch_size)) - int(patch_size//2))
            p2 = (random.choice(range(patch_size)) - int(patch_size//2), random.choice(range(patch_size)) - int(patch_size//2))
            if p1 != p2 and (p1, p2) not in pattern and (p2, p1) not in pattern:
                pattern.append((p1, p2))
        
        return pattern

    def detect(self):
        if len(self.pyramid) == 0:
            self.build_pyramid()
        pattern = self.patterns_search(self.disc_len, self.patch_size, self.seed)
        self.features = []

        for level, pyramid_img in enumerate(self.pyramid):
            scale = self.scale_factor ** level
            level_dist_threshold = max(3, self.dist_threshold / scale)
            keypoints, scores = self.fast_corner_detection(pyramid_img)
            keypoints, scores = self.nms(keypoints, scores, level_dist_threshold)
            angles = self.orientation(pyramid_img, self.patch_size, keypoints)

            for (x_level, y_level), score, angle in zip(keypoints, scores, angles):
                x_original = int(x_level * scale)
                y_original = int(y_level * scale)
                if angle is None:
                    continue
                angle_rad = np.deg2rad(angle)        
                descriptor = []  
                valid = True  
                h, w = pyramid_img.shape[:2]
                for ((dx1,dy1),(dx2,dy2)) in pattern:
                    x1 = dx1*np.cos(angle_rad) - dy1*np.sin(angle_rad)
                    y1 = dx1*np.sin(angle_rad) + dy1*np.cos(angle_rad)
                    x2 = dx2*np.cos(angle_rad) - dy2*np.sin(angle_rad)
                    y2 = dx2*np.sin(angle_rad) + dy2*np.cos(angle_rad)

                    p1 = (x_level + x1, y_level + y1)
                    p2 = (x_level + x2, y_level + y2)
                    x1, y1 = p1
                    x2, y2 = p2

                    if not (0 <= x1 < w and 0 <= y1 < h and 0 <= x2 < w and 0 <= y2 < h):
                        valid = False
                        break
                    else:
                        if pyramid_img[int(y1), int(x1)] < pyramid_img[int(y2), int(x2)]:
                            descriptor.append(1)
                        else:
                            descriptor.append(0)

                if self.disc_len == len(descriptor) and valid:
                    self.features.append({
                        "pt": (x_original, y_original),
                        "pt_level": (x_level, y_level),
                        "level": level,
                        "scale": scale,
                        "score": score,
                        "angle":angle,
                        "descriptor": descriptor
                    })


        return self.features
    
    def draw_keypoints(self, img=None):
        if img is None:
            img = self.img.copy()
        else:
            img = img.copy()

        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        for feature in self.features:
            x, y = feature["pt"]
            cv2.circle(img, (x, y), 3, (0, 0, 255), -1)

        return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
