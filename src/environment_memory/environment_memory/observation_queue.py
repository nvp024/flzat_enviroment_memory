from __future__ import annotations

from dataclasses import dataclass
from typing import Generic, Optional, TypeVar


T = TypeVar("T")


@dataclass(frozen=True)
class QueueResult(Generic[T]):
    accepted: bool
    replaced: Optional[T] = None


class LatestObservationQueue(Generic[T]):
    """Bounded one-active/one-latest-pending observation queue."""

    def __init__(self) -> None:
        self._active = False
        self._pending: Optional[tuple[int, T]] = None

    @property
    def active(self) -> bool:
        return self._active

    @property
    def has_pending(self) -> bool:
        return self._pending is not None

    def submit(self, observation: T, priority: int) -> QueueResult[T]:
        if self._pending is not None and self._pending[0] > priority:
            return QueueResult(accepted=False)
        replaced = None if self._pending is None else self._pending[1]
        self._pending = (priority, observation)
        return QueueResult(accepted=True, replaced=replaced)

    def begin_next(self) -> Optional[T]:
        if self._active or self._pending is None:
            return None
        _, observation = self._pending
        self._pending = None
        self._active = True
        return observation

    def complete(self) -> None:
        if not self._active:
            raise RuntimeError("Cannot complete without an active observation")
        self._active = False
