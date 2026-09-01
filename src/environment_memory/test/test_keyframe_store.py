import pytest

from environment_memory.storage.keyframe_store import KeyframeError, KeyframeStore


JPEG = b"\xff\xd8keyframe-fixture\xff\xd9"


def test_atomic_write_reuses_identical_observation_image(tmp_path):
    store = KeyframeStore(tmp_path)

    first_ref, first_created = store.write_jpeg("observation-1", JPEG)
    second_ref, second_created = store.write_jpeg("observation-1", JPEG)

    assert first_ref == "images/observation_observation-1.jpg"
    assert second_ref == first_ref
    assert first_created is True
    assert second_created is False
    assert (tmp_path / first_ref).read_bytes() == JPEG
    assert list((tmp_path / "images").glob(".keyframe-*.tmp")) == []


def test_rejects_bad_jpeg_unsafe_id_and_conflicting_bytes(tmp_path):
    store = KeyframeStore(tmp_path)

    with pytest.raises(KeyframeError, match="JPEG"):
        store.write_jpeg("observation-1", b"not-jpeg")
    with pytest.raises(KeyframeError, match="safe"):
        store.write_jpeg("../escape", JPEG)
    store.write_jpeg("observation-1", JPEG)
    with pytest.raises(KeyframeError, match="other bytes"):
        store.write_jpeg("observation-1", b"\xff\xd8different\xff\xd9")
