"""Lightweight appearance re-identification.

The original ccf_person_identification package trains an online-boosted
classifier over Convolutional Channel Features (dlib) to tell the target
apart from everyone else. That's a large, fragile dependency to port.

Here we use a much simpler (and much weaker, but dependency-free) stand-in:
an HSV color histogram of the person's bounding-box crop, compared with
histogram correlation. It's enough to survive brief occlusions/dropouts
and tell "same shirt" from "different shirt" in a small room, which is
the actual job re-id needs to do in person_follower_node's REACQUIRE state.

Pure python / numpy / OpenCV, no ROS or rclpy dependency, so it's
unit-testable in isolation (see test/test_reid.py).
"""
import cv2
import numpy as np

_H_BINS = 30
_S_BINS = 32


def extract_appearance(bgr_crop):
    """Compute a normalized HSV (H,S) histogram for a person crop.

    Returns None if the crop is empty/too small to be meaningful.
    """
    if bgr_crop is None or bgr_crop.size == 0:
        return None
    if bgr_crop.shape[0] < 8 or bgr_crop.shape[1] < 8:
        return None

    # Focus on the torso (middle 60% vertically) to reduce background
    # bleed from a loose bounding box and avoid head/feet clutter.
    h = bgr_crop.shape[0]
    top = int(h * 0.15)
    bottom = int(h * 0.75)
    torso = bgr_crop[top:bottom, :]
    if torso.size == 0:
        torso = bgr_crop

    hsv = cv2.cvtColor(torso, cv2.COLOR_BGR2HSV)
    hist = cv2.calcHist([hsv], [0, 1], None, [_H_BINS, _S_BINS], [0, 180, 0, 256])
    cv2.normalize(hist, hist, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX)
    return hist


def similarity(hist_a, hist_b):
    """Correlation between two histograms, in [-1, 1] (1 = identical).

    Returns -1.0 (worst score) if either histogram is missing.
    """
    if hist_a is None or hist_b is None:
        return -1.0
    return float(cv2.compareHist(hist_a, hist_b, cv2.HISTCMP_CORREL))


def average_histograms(histograms):
    """Average a list of histograms into a single reference appearance."""
    valid = [h for h in histograms if h is not None]
    if not valid:
        return None
    stacked = np.stack(valid, axis=0)
    mean = np.mean(stacked, axis=0).astype(np.float32)
    cv2.normalize(mean, mean, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX)
    return mean
