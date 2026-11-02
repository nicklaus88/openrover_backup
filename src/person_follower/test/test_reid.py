import numpy as np

from person_follower.reid import average_histograms, extract_appearance, similarity


def _solid_crop(color_bgr, size=64):
    crop = np.zeros((size, size, 3), dtype=np.uint8)
    crop[:, :] = color_bgr
    return crop


def test_identical_crops_score_near_one():
    crop = _solid_crop((40, 80, 180))
    hist_a = extract_appearance(crop)
    hist_b = extract_appearance(crop.copy())
    assert similarity(hist_a, hist_b) > 0.99


def test_different_colors_score_lower_than_identical():
    red_hist = extract_appearance(_solid_crop((20, 20, 200)))
    blue_hist = extract_appearance(_solid_crop((200, 20, 20)))
    same_hist = extract_appearance(_solid_crop((20, 20, 200)))

    assert similarity(red_hist, same_hist) > similarity(red_hist, blue_hist)


def test_missing_histograms_score_worst():
    hist = extract_appearance(_solid_crop((10, 10, 10)))
    assert similarity(hist, None) == -1.0
    assert similarity(None, None) == -1.0


def test_tiny_crop_returns_none():
    assert extract_appearance(np.zeros((2, 2, 3), dtype=np.uint8)) is None
    assert extract_appearance(None) is None


def test_average_histograms_ignores_none_entries():
    hist = extract_appearance(_solid_crop((50, 60, 70)))
    avg = average_histograms([hist, None, hist])
    assert avg is not None
    assert similarity(avg, hist) > 0.99

    assert average_histograms([None, None]) is None
