import sys
from types import SimpleNamespace

import numpy as np
import pytest

from environment_memory.embedding import SentenceTransformerEmbedder


class FakeSentenceTransformer:
    instance = None

    def __init__(self, model_name, **kwargs):
        self.model_name = model_name
        self.kwargs = kwargs
        self.encode_kwargs = None
        FakeSentenceTransformer.instance = self

    def get_sentence_embedding_dimension(self):
        return 384

    def encode(self, texts, **kwargs):
        self.encode_kwargs = kwargs
        result = np.zeros((1, 384), dtype=np.float32)
        result[0, 0] = 1.0
        return result


def test_sentence_transformer_adapter_pins_revision_and_normalization(monkeypatch):
    monkeypatch.setitem(
        sys.modules,
        "sentence_transformers",
        SimpleNamespace(SentenceTransformer=FakeSentenceTransformer),
    )
    embedder = SentenceTransformerEmbedder(
        "model-name", "revision-1", "cpu", local_files_only=True
    )

    vector = embedder.encode("semantic text")

    instance = FakeSentenceTransformer.instance
    assert instance.model_name == "model-name"
    assert instance.kwargs == {
        "revision": "revision-1",
        "device": "cpu",
        "trust_remote_code": False,
        "local_files_only": True,
    }
    assert instance.encode_kwargs["normalize_embeddings"] is True
    assert len(vector) == 384
    assert vector[0] == 1.0


def test_embedding_adapter_rejects_unexpected_dimension(monkeypatch):
    class WrongDimension(FakeSentenceTransformer):
        def get_sentence_embedding_dimension(self):
            return 128

    monkeypatch.setitem(
        sys.modules,
        "sentence_transformers",
        SimpleNamespace(SentenceTransformer=WrongDimension),
    )
    with pytest.raises(RuntimeError, match="dimension mismatch"):
        SentenceTransformerEmbedder("model", "revision")
