import importlib.util

import pytest

from environment_memory.storage.memory_record import embedding_text, new_record
from environment_memory.storage.memory_store import COLLECTION_NAME, ChromaMemoryStore
from environment_memory.retrieval.search import ReadOnlyChromaMemoryStore
from test_memory_record import observation


pytestmark = pytest.mark.skipif(
    importlib.util.find_spec("chromadb") is None,
    reason="chromadb is not installed on this development host",
)


def test_chroma_upsert_and_restart_recovery(tmp_path):
    vector = (1.0,) + (0.0,) * 383
    record = new_record(observation(), "object-1", "images/a.jpg")
    store = ChromaMemoryStore(tmp_path / "chroma")

    store.upsert(record, vector, embedding_text(record))
    restarted = ChromaMemoryStore(tmp_path / "chroma")

    assert COLLECTION_NAME == "environment_objects_v1"
    assert restarted.count() == 1
    recovered = restarted.all()[0]
    assert recovered.record == record
    assert recovered.embedding == vector
    assert restarted.same_class("hotel_demo", "map-session", "bottle") == [
        recovered
    ]

    read_only = ReadOnlyChromaMemoryStore(tmp_path / "chroma")
    hits = read_only.search(vector, "hotel_demo", "map-session")
    assert len(hits) == 1
    assert hits[0].stored.record == record
    assert hits[0].cosine_score == pytest.approx(1.0)
    assert not hasattr(read_only, "upsert")
