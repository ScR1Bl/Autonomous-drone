import cv2


def draw_keypoints(img, features, color=(255, 0, 0), radius=3, thickness=1):
    img_with_keypoints = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)

    for feature in features:
        x, y = feature["pt"]
        cv2.circle(img_with_keypoints, (int(x), int(y)), radius, color, thickness)

    return img_with_keypoints
