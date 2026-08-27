import hashlib

import pytest

from environment_memory.model_asset import ModelAssetError, resolve_verified_model


def test_resolves_only_a_matching_model(tmp_path):
    model = tmp_path / "model.pt"
    model.write_bytes(b"pinned model fixture")
    digest = hashlib.sha256(model.read_bytes()).hexdigest()

    assert resolve_verified_model(str(model), digest) == model.resolve()


def test_rejects_missing_or_changed_model(tmp_path):
    missing = tmp_path / "missing.pt"
    with pytest.raises(ModelAssetError, match="not found"):
        resolve_verified_model(str(missing), "0" * 64)

    model = tmp_path / "model.pt"
    model.write_bytes(b"wrong")
    with pytest.raises(ModelAssetError, match="mismatch"):
        resolve_verified_model(str(model), "0" * 64)
