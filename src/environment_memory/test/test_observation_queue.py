from environment_memory.perception.observation_queue import LatestObservationQueue


def test_queue_keeps_one_active_and_latest_pending():
    queue = LatestObservationQueue[str]()
    queue.submit("first", 10)
    assert queue.begin_next() == "first"

    queue.submit("old", 20)
    result = queue.submit("new", 20)

    assert result.accepted
    assert result.replaced == "old"
    queue.complete()
    assert queue.begin_next() == "new"


def test_lower_priority_does_not_replace_pending():
    queue = LatestObservationQueue[str]()
    queue.submit("high", 50)

    result = queue.submit("low", 10)

    assert not result.accepted
    assert queue.begin_next() == "high"
