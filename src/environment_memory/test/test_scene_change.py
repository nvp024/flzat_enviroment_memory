import numpy as np

from environment_memory.perception.scene_change import histogram_distance, hsv_histogram


def test_identical_images_have_zero_scene_distance():
    image = np.full((32, 32, 3), (0, 0, 255), dtype=np.uint8)
    histogram = hsv_histogram(image)

    assert histogram_distance(histogram, histogram.copy()) == 0.0


def test_different_hues_exceed_default_scene_threshold():
    red = np.full((32, 32, 3), (0, 0, 255), dtype=np.uint8)
    green = np.full((32, 32, 3), (0, 255, 0), dtype=np.uint8)

    distance = histogram_distance(hsv_histogram(red), hsv_histogram(green))

    assert distance >= 0.35
