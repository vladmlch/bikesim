"""Compatibility shim — the projection implementation lives in bike_sim.native.config.

Kept so existing ``tools.native_config`` imports (tests, sweep tooling and
out-of-tree scripts under output/) keep working unchanged. The schema
helpers and leading-underscore names mirror the original module surface
exactly: ``from tools.native_config import validate_config`` kept working
because the original imported it from the schema module.
"""
from bike_sim.native.config import *  # noqa: F401,F403
from bike_sim.native.config import (  # noqa: F401  -- mirrored re-exports
    validate_config, validate_drive_policies, plain, sequence,
    _joint_name, _damper_core, _surface_spec, _tire_side, _tire_section,
    _project, _project_drive, _project_drive_policies, _project_rider_contacts,
)
