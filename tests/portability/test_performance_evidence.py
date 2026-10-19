"""Claim classification for measured realtime runs (T29).

``steady_state_evidence`` consumes a ``measure_run`` result and decides
whether the recorded prefix can support a same-hardware steady-state
throughput claim. A short, invalid, interrupted or identity-drifted prefix
remains diagnostic evidence — it must never be presented as a
completed-window measurement. These tests pin the classification contract
on the existing ``measure_run`` schema: ``measured_ok``, ``valid_prefix``,
``identity_unchanged``, ``errors``, ``physics_steps``,
``simulated_time_s``, ``wall_clock_s`` and
``prefix{steps, requested_steps, reason, model_valid, numerically_valid}``.
"""
from __future__ import annotations

from copy import deepcopy

import pytest

from bike_sim.sim.realtime_measurement import steady_state_evidence


def measured_run(*, duration=30.0, steps=24000, valid=True,
                 reason="duration_reached", identity=True):
    return {"measured_ok": True, "valid_prefix": valid,
            "identity_unchanged": identity, "errors": [],
            "physics_steps": steps, "simulated_time_s": duration,
            "wall_clock_s": 10.0,
            "prefix": {"steps": steps, "requested_steps": 24000,
                       "reason": reason, "model_valid": valid,
                       "numerically_valid": True}}


def test_short_invalid_prefix_is_diagnostic_not_steady_state():
    run = measured_run(duration=.005, steps=4, valid=False,
                       reason="invalid_controller")
    result = steady_state_evidence(run, 30.0)
    assert result["status"] == "diagnostic"
    assert "invalid_prefix" in result["reasons"]
    assert "incomplete_requested_prefix" in result["reasons"]
    assert "insufficient_simulated_window" in result["reasons"]


def test_completed_valid_window_is_eligible():
    assert steady_state_evidence(measured_run(), 30.0) == {
        "status": "eligible", "reasons": []}


def test_changed_identity_is_not_a_valid_performance_claim():
    result = steady_state_evidence(measured_run(identity=False), 30.0)
    assert result["status"] == "diagnostic"
    assert "identity_changed" in result["reasons"]


def test_non_duration_terminal_reason_is_not_steady_state():
    # A complete step count that ended on a non-duration terminal condition
    # is still not a completed requested window.
    result = steady_state_evidence(measured_run(reason="finished"), 30.0)
    assert result == {"status": "diagnostic",
                      "reasons": ["non_duration_terminal"]}


@pytest.mark.parametrize("wall", [0.0, -1.0, float("nan"), float("inf")])
def test_invalid_wall_clock_blocks_steady_state(wall):
    run = measured_run()
    run["wall_clock_s"] = wall
    assert steady_state_evidence(run, 30.0) == {
        "status": "diagnostic", "reasons": ["invalid_wall_clock"]}


@pytest.mark.parametrize("measured_ok,errors", [
    (False, []),
    (True, [{"stage": "advance", "type": "RuntimeError", "message": "x"}]),
    (False, [{"stage": "flush", "type": "RuntimeError", "message": "x"}]),
])
def test_measurement_failure_is_diagnostic(measured_ok, errors):
    run = measured_run()
    run["measured_ok"] = measured_ok
    run["errors"] = errors
    assert steady_state_evidence(run, 30.0) == {
        "status": "diagnostic", "reasons": ["measurement_failed"]}


@pytest.mark.parametrize("minimum", [float("nan"), float("inf"),
                                     float("-inf"), 0.0, -1.0])
def test_non_finite_or_non_positive_minimum_is_rejected(minimum):
    with pytest.raises(ValueError, match="finite and positive"):
        steady_state_evidence(measured_run(), minimum)


@pytest.mark.parametrize("minimum", [True, False])
def test_bool_minimum_is_rejected(minimum):
    with pytest.raises(ValueError, match="finite and positive"):
        steady_state_evidence(measured_run(), minimum)


def test_partial_step_prefix_is_incomplete():
    # Half of the requested steps committed; the terminal reason still says
    # duration, but the prefix is not the requested window.
    result = steady_state_evidence(
        measured_run(steps=12000, reason="duration_reached"), 30.0)
    assert result == {"status": "diagnostic",
                      "reasons": ["incomplete_requested_prefix"]}


def test_committed_steps_must_match_prefix_record():
    # physics_steps counts the committed boundary; if it disagrees with the
    # prefix record the window is not trustworthy even at full requested
    # steps.
    run = measured_run()
    run["physics_steps"] = 23999
    assert steady_state_evidence(run, 30.0) == {
        "status": "diagnostic", "reasons": ["incomplete_requested_prefix"]}


def test_zero_requested_steps_is_incomplete():
    run = measured_run()
    run["prefix"]["requested_steps"] = 0
    assert steady_state_evidence(run, 30.0) == {
        "status": "diagnostic", "reasons": ["incomplete_requested_prefix"]}


def test_numerically_invalid_prefix_is_diagnostic():
    # The run-level flag and the prefix model/numerical flags are checked
    # independently: a run that reports valid_prefix but a numerically
    # invalid prefix is still diagnostic.
    run = measured_run()
    run["prefix"]["numerically_valid"] = False
    assert steady_state_evidence(run, 30.0) == {
        "status": "diagnostic", "reasons": ["invalid_prefix"]}


def test_non_finite_simulated_window_is_insufficient():
    run = measured_run()
    run["simulated_time_s"] = float("nan")
    assert steady_state_evidence(run, 30.0) == {
        "status": "diagnostic", "reasons": ["insufficient_simulated_window"]}


def test_window_minimum_has_a_narrow_equality_tolerance():
    # The window check uses a tight isclose band so a last-step float
    # remainder does not flip a complete run, while any material shortfall
    # remains insufficient.
    assert steady_state_evidence(
        measured_run(duration=30.0 - 1e-10), 30.0) == {
            "status": "eligible", "reasons": []}
    result = steady_state_evidence(measured_run(duration=30.0 - 1e-6), 30.0)
    assert result["status"] == "diagnostic"
    assert "insufficient_simulated_window" in result["reasons"]


def test_reasons_have_stable_documented_order():
    # The reasons list is part of the report contract: order is fixed so
    # audits can diff classifications without fuzzy matching.
    run = measured_run(duration=.005, steps=4, valid=False,
                       reason="invalid_controller", identity=False)
    run["measured_ok"] = False
    run["errors"] = [{"stage": "advance", "type": "RuntimeError",
                      "message": "x"}]
    run["wall_clock_s"] = float("nan")
    assert steady_state_evidence(run, 30.0) == {
        "status": "diagnostic",
        "reasons": ["measurement_failed", "invalid_prefix",
                    "identity_changed", "incomplete_requested_prefix",
                    "non_duration_terminal", "insufficient_simulated_window",
                    "invalid_wall_clock"]}


def test_classification_does_not_mutate_the_run():
    run = measured_run(duration=.005, steps=4, valid=False,
                       reason="invalid_controller")
    before = deepcopy(run)
    steady_state_evidence(run, 30.0)
    assert run == before
