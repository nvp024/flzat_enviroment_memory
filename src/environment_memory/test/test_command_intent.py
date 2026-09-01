import pytest

from environment_memory.assistant.command_intent import (
    CommandError,
    CommandMatch,
    Intent,
    ambiguous_navigation_matches,
    clarification_prompt,
    format_query_answer,
    parse_command,
    resolve_clarification,
)


def match(object_id, score, scene="kitchen"):
    return CommandMatch(
        object_id=object_id,
        label="water_bottle",
        description=f"Bottle {object_id}",
        scene=scene,
        score=score,
        x=1.0,
        y=2.0,
        z=0.8,
    )


def test_query_and_explicit_navigation_intents():
    assert parse_command("Where is the blue bottle?") == (
        parse_command("find the blue bottle")
    )
    assert parse_command("Where is the blue bottle?").intent == Intent.QUERY_MEMORY
    navigation = parse_command("Go to the blue bottle")
    assert navigation.intent == Intent.NAVIGATE_TO_MEMORY
    assert navigation.query_text == "the blue bottle"

    vietnamese = parse_command("Đi đến chai nước màu xanh")
    assert vietnamese.intent == Intent.NAVIGATE_TO_MEMORY
    assert vietnamese.query_text == "chai nước màu xanh"


def test_empty_navigation_target_is_rejected():
    with pytest.raises(CommandError, match="does not name"):
        parse_command("navigate to")


def test_ambiguity_requires_clarification_and_resolves_ordinal():
    matches = (match("a", 0.92), match("b", 0.90), match("c", 0.70))

    selected, ambiguous = ambiguous_navigation_matches(matches, 0.05)

    assert selected is None
    assert [item.object_id for item in ambiguous] == ["a", "b"]
    assert "option 1" in clarification_prompt(ambiguous)
    assert resolve_clarification("the second one", ambiguous).object_id == "b"
    assert resolve_clarification("not sure", ambiguous) is None


def test_query_response_has_no_numeric_navigation_pose():
    answer = format_query_answer((match("a", 0.9),))
    assert "kitchen" in answer
    assert "1.0" not in answer
    assert "2.0" not in answer
