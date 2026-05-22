from abc import ABC


class Detector(ABC):
    def __init__(self, img):
        self.img = img
