import cv2


def draw_keypoints(image, detection_result, color=(255, 0, 0), radius=3, thickness=1):
    if image.ndim == 2:
        output = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
    else:
        output = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    for x, y in detection_result.keypoints:
        cv2.circle(output, (int(x), int(y)), radius, color, thickness)

    return output
