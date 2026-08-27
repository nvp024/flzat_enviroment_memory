import math

from environment_memory.trigger_policy import (
    ObservationTriggerPolicy,
    Pose2D,
    TriggerReason,
)


def test_first_valid_bundle_is_immediately_eligible():
    policy = ObservationTriggerPolicy()
    pose = Pose2D(0.0, 0.0, 0.0)

    decision = policy.evaluate(0.0, pose, None, 0.0, 0.0)

    assert decision.eligible
    assert decision.reason == TriggerReason.FIRST_VALID


def test_translation_waits_for_cooldown_and_stable_frame():
    policy = ObservationTriggerPolicy()
    origin = Pose2D(0.0, 0.0, 0.0)
    policy.accept(0.0, origin, None)

    cooldown = policy.evaluate(5.0, Pose2D(2.0, 0.0, 0.0), None, 0.0, 0.0)
    moving = policy.evaluate(9.0, Pose2D(2.0, 0.0, 0.0), None, 0.2, 0.0)
    stable = policy.evaluate(9.1, Pose2D(2.0, 0.0, 0.0), None, 0.0, 0.0)

    assert not cooldown.eligible
    assert not moving.eligible
    assert moving.reason == TriggerReason.TRANSLATION
    assert stable.reason == TriggerReason.TRANSLATION
    assert stable.eligible


def test_accumulated_rotation_handles_wrapped_angles():
    policy = ObservationTriggerPolicy()
    policy.accept(0.0, Pose2D(0.0, 0.0, math.radians(170)), None)
    policy.evaluate(8.0, Pose2D(0.0, 0.0, math.radians(-170)), None, 0.0, 0.0)
    decision = policy.evaluate(
        8.1, Pose2D(0.0, 0.0, math.radians(-120)), None, 0.0, 0.0
    )

    assert decision.eligible
    assert decision.reason == TriggerReason.ROTATION


def test_waypoint_has_priority_after_settle_delay():
    policy = ObservationTriggerPolicy()
    pose = Pose2D(0.0, 0.0, 0.0)
    policy.accept(0.0, pose, None)
    policy.mark_waypoint_completed(8.0)

    too_early = policy.evaluate(8.5, pose, 0.5, 0.0, 0.0)
    ready = policy.evaluate(8.8, pose, 0.5, 0.0, 0.0)

    assert too_early.reason == TriggerReason.SCENE_CHANGE
    assert ready.reason == TriggerReason.WAYPOINT


def test_timed_refresh_can_capture_while_robot_is_moving():
    policy = ObservationTriggerPolicy()
    pose = Pose2D(0.0, 0.0, 0.0)
    policy.accept(0.0, pose, None)

    decision = policy.evaluate(20.0, pose, None, 0.5, 0.5)

    assert decision.eligible
    assert decision.reason == TriggerReason.TIMED_REFRESH
