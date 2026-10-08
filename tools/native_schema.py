"""Compatibility shim — the schema implementation lives in bike_sim.native.schema.

Kept so existing ``tools.native_schema`` imports (tests, sweep tooling and
out-of-tree scripts under output/) keep working unchanged.
"""
from bike_sim.native.schema import *  # noqa: F401,F403
from bike_sim.native.schema import (  # noqa: F401  -- explicit for checkers
    DAMPER, SCHEMAS, BOOLEAN, INTEGER, STRING, WIDTHS,
    exact_keys, integer, boolean, sequence, validate_mapping,
    validate_policy_modes, validate_drive_policies, validate_rider_contacts,
    validate_config, plain,
)
