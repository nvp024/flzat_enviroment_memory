"""Parse deterministic memory-assistant command intents."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import re
import unicodedata
from typing import Sequence


class Intent(str, Enum):
    QUERY_MEMORY = "QUERY_MEMORY"
    NAVIGATE_TO_MEMORY = "NAVIGATE_TO_MEMORY"


class CommandError(ValueError):
    pass


@dataclass(frozen=True)
class ParsedCommand:
    intent: Intent
    query_text: str


@dataclass(frozen=True)
class CommandMatch:
    object_id: str
    label: str
    description: str
    scene: str
    score: float
    x: float
    y: float
    z: float


_NAVIGATION_PREFIXES = (
    "navigate to",
    "go to",
    "take me to",
    "bring me to",
    "drive to",
    "move to",
    "đi đến",
    "đến",
    "dẫn tôi đến",
    "di den",
    "dan toi den",
)

_QUERY_PREFIXES = (
    "where is",
    "where are",
    "find",
    "show me",
    "what room is",
    "which room is",
    "ở đâu",
    "tìm",
    "tim",
)


def parse_command(transcript: str) -> ParsedCommand:
    text = _clean(transcript)
    if not text:
        raise CommandError("speech transcript is empty")
    normalized = _normalized_for_matching(text)
    for prefix in _NAVIGATION_PREFIXES:
        target = _strip_prefix(text, normalized, prefix)
        if target is not None:
            if not target:
                raise CommandError("navigation command does not name an object")
            return ParsedCommand(Intent.NAVIGATE_TO_MEMORY, target)
    for prefix in _QUERY_PREFIXES:
        target = _strip_prefix(text, normalized, prefix)
        if target:
            return ParsedCommand(Intent.QUERY_MEMORY, target)
    return ParsedCommand(Intent.QUERY_MEMORY, text)


def ambiguous_navigation_matches(
    matches: Sequence[CommandMatch], score_margin: float = 0.05
) -> tuple[CommandMatch | None, tuple[CommandMatch, ...]]:
    if not matches:
        return None, ()
    if score_margin < 0.0:
        raise ValueError("ambiguity score margin cannot be negative")
    ordered = sorted(matches, key=lambda item: (-item.score, item.object_id))
    close = tuple(
        item for item in ordered if ordered[0].score - item.score <= score_margin
    )
    if len(close) > 1:
        return None, close
    return ordered[0], ()


def clarification_prompt(matches: Sequence[CommandMatch]) -> str:
    options = []
    for index, item in enumerate(matches[:3], start=1):
        detail = item.description.strip() or item.label.replace("_", " ")
        options.append(f"option {index}, {detail} in {item.scene.replace('_', ' ')}")
    return "I found several possible objects. Please choose " + "; ".join(options) + "."


def resolve_clarification(
    transcript: str, matches: Sequence[CommandMatch]
) -> CommandMatch | None:
    text = _normalized_for_matching(_clean(transcript))
    explicit_ordinals = {
        "1": 0,
        "first": 0,
        "mot": 0,
        "2": 1,
        "second": 1,
        "hai": 1,
        "3": 2,
        "third": 2,
        "ba": 2,
    }
    tokens = set(re.findall(r"[a-z0-9]+", text))
    selected_indexes = {
        index for word, index in explicit_ordinals.items() if word in tokens
    }
    if not selected_indexes:
        cardinal_ordinals = {"one": 0, "two": 1, "three": 2}
        selected_indexes = {
            index for word, index in cardinal_ordinals.items() if word in tokens
        }
    if len(selected_indexes) == 1:
        index = selected_indexes.pop()
        return matches[index] if index < len(matches) else None

    selected = []
    for item in matches:
        searchable = _normalized_for_matching(
            " ".join((item.label, item.description, item.scene, item.object_id))
        )
        if text and text in searchable:
            selected.append(item)
    return selected[0] if len(selected) == 1 else None


def format_query_answer(matches: Sequence[CommandMatch]) -> str:
    if not matches:
        return "I could not find a matching object in this environment memory."
    if len(matches) == 1:
        item = matches[0]
        return (
            f"I found {item.label.replace('_', ' ')} in "
            f"{item.scene.replace('_', ' ')}. {item.description}"
        ).strip()
    summaries = [
        f"{item.label.replace('_', ' ')} in {item.scene.replace('_', ' ')}"
        for item in matches[:3]
    ]
    return f"I found {len(matches)} matches. " + "; ".join(summaries) + "."


def _clean(value: str) -> str:
    return re.sub(r"\s+", " ", value.strip()).strip(" ?!.,")


def _normalized_for_matching(value: str) -> str:
    decomposed = unicodedata.normalize("NFKD", value.casefold())
    return "".join(char for char in decomposed if not unicodedata.combining(char))


def _strip_prefix(original: str, normalized: str, prefix: str) -> str | None:
    normalized_prefix = _normalized_for_matching(prefix)
    if normalized == normalized_prefix:
        return ""
    if not normalized.startswith(normalized_prefix + " "):
        return None
    # Prefix normalization only removes combining marks, so token counts remain stable.
    prefix_tokens = len(normalized_prefix.split())
    return " ".join(original.split()[prefix_tokens:]).strip(" ?!.,")
