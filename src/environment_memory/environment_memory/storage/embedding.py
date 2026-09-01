"""Provide the pinned multilingual sentence-embedding boundary."""

from __future__ import annotations

import math
from typing import Protocol, Sequence

from environment_memory.storage.memory_record import EMBEDDING_DIMENSION


class Embedder(Protocol):
    model_name: str
    revision: str
    dimension: int

    def encode(self, text: str) -> tuple[float, ...]: ...


class SentenceTransformerEmbedder:
    def __init__(
        self,
        model_name: str,
        revision: str,
        device: str = "cpu",
        local_files_only: bool = False,
    ) -> None:
        try:
            from sentence_transformers import SentenceTransformer
        except ImportError as exc:
            raise RuntimeError(
                "sentence-transformers is unavailable; install requirements-memory.txt"
            ) from exc
        self.model_name = model_name
        self.revision = revision
        self.dimension = EMBEDDING_DIMENSION
        self._model = SentenceTransformer(
            model_name,
            revision=revision,
            device=device,
            trust_remote_code=False,
            local_files_only=local_files_only,
        )
        dimension = self._model.get_sentence_embedding_dimension()
        if dimension != self.dimension:
            raise RuntimeError(
                f"embedding dimension mismatch: expected {self.dimension}, got {dimension}"
            )

    def encode(self, text: str) -> tuple[float, ...]:
        vector = self._model.encode(
            [text],
            normalize_embeddings=True,
            convert_to_numpy=True,
            show_progress_bar=False,
        )[0]
        result = tuple(float(value) for value in vector)
        _validate_embedding(result, self.dimension)
        return result


def _validate_embedding(values: Sequence[float], dimension: int) -> None:
    if len(values) != dimension or not all(math.isfinite(value) for value in values):
        raise ValueError(f"embedding must contain {dimension} finite values")
    norm = math.sqrt(sum(value * value for value in values))
    if not math.isclose(norm, 1.0, rel_tol=1e-4, abs_tol=1e-4):
        raise ValueError("embedding must be L2-normalized")
