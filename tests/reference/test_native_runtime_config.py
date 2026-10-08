"""Native runtime capability matrix: structural predicates, not filenames.

The native backend supports exactly the design's section-1 capability set.
Rejection names the full configuration field path and precedes any output
creation or extension construction.
"""

from dataclasses import replace

import pytest

from native_loader import load_native
from bike_sim.physics.resolution import load_physics_config
from bike_sim.physics.rider import RiderSpecs
from bike_sim.physics.tire_curve import TabulatedTireSpec
from bike_sim.native.setup import validate_supported

WELDED = "examples/research/viewer_physics_welded.toml"


def _supported():
    return (load_physics_config(WELDED),
            RiderSpecs(variant="articulated_planar"))


def _replaced(cfg, path, value):
    """Return cfg with one dotted field replaced (leaf dataclasses are frozen)."""
    parts = path.split(".")
    node, parents = cfg, []
    for part in parts[:-1]:
        parents.append((node, part))
        node = getattr(node, part)
    node = replace(node, **{parts[-1]: value})
    for parent, part in reversed(parents):
        node = replace(parent, **{part: node})
    return node


def test_native_capability_uses_fields_not_filename():
    cfg = load_physics_config(WELDED)
    rider = RiderSpecs(variant="articulated_planar")
    validate_supported(cfg, rider)
    different_attachment = replace(
        cfg, articulated=replace(cfg.articulated, pedal_attachment="weld"))
    with pytest.raises(ValueError, match=r"articulated\.pedal_attachment"):
        validate_supported(different_attachment, rider)


@pytest.mark.parametrize("path, value", [
    ("articulated.pedal_attachment", "flat"),
    ("articulated.pedal_attachment", "weld"),
    ("articulated.saddle_attachment", "flat"),
    ("articulated.saddle_attachment", "weld"),
    ("articulated.grip_attachment", "spring"),
    ("tires.backend", "distributed_2d_reference"),
    ("tires.surface_mode", "configured"),
    ("drive.motor_clutch", True),
    ("drive.rotor_inertia_kgm2", 0.001),
])
def test_native_capability_rejects_unsupported_structure(path, value):
    cfg, rider = _supported()
    with pytest.raises(ValueError, match=path.replace(".", r"\.")):
        validate_supported(_replaced(cfg, path, value), rider)


def _mutated(cfg, path, value):
    """Write one leaf past __post_init__ so the validator itself is tested.

    physics_mode/drive_mode/transmission_model combinations are already
    rejected at dataclass construction; the capability matrix still has to
    refuse them by full field path for any resolved config it is handed.
    """
    node = cfg
    for part in path.split(".")[:-1]:
        node = getattr(node, part)
    object.__setattr__(node, path.split(".")[-1], value)
    return cfg


@pytest.mark.parametrize("value", ["coast", "crank_effort",
                                   "ideal_speed_control"])
def test_native_capability_rejects_other_drive_modes(value):
    cfg, rider = _supported()
    with pytest.raises(ValueError, match=r"drive_mode"):
        validate_supported(_mutated(cfg, "drive_mode", value), rider)


@pytest.mark.parametrize("value", ["elastic_chain",
                                   "geometric_ideal_mid_drive"])
def test_native_capability_rejects_other_transmissions(value):
    cfg, rider = _supported()
    with pytest.raises(ValueError, match=r"drive\.transmission_model"):
        validate_supported(
            _mutated(cfg, "drive.transmission_model", value), rider)


def test_native_capability_rejects_legacy_physics_mode():
    cfg, rider = _supported()
    with pytest.raises(ValueError, match=r"physics_mode"):
        validate_supported(_mutated(cfg, "physics_mode", "legacy"), rider)


def test_native_capability_rejects_unported_tire_backend():
    """native_reference+configured is a valid config the port does not implement."""
    cfg, rider = _supported()
    tires = replace(cfg.tires, backend="native_reference",
                    surface_mode="configured")
    with pytest.raises(ValueError, match=r"tires\.backend"):
        validate_supported(replace(cfg, tires=tires), rider)


@pytest.mark.parametrize("variant", ["none", "lumped", "seated"])
def test_native_capability_rejects_other_riders(variant):
    cfg, _ = _supported()
    with pytest.raises(ValueError, match=r"rider\.variant"):
        validate_supported(cfg, RiderSpecs(variant=variant))


def test_native_capability_rejects_tabulated_tire_material():
    cfg, rider = _supported()
    table = TabulatedTireSpec(
        deflection_m=(0.0, 0.01),
        force_n=(0.0, 1000.0),
        radial_c_ns_m=cfg.tires.front.material.radial_c_ns_m,
        pressure_pa_gauge=cfg.tires.front.material.pressure_pa_gauge,
        provenance="test",
        valid_load_range_n=(100.0, 500.0))
    tires = replace(cfg.tires, front=replace(cfg.tires.front, material=table))
    with pytest.raises(ValueError, match=r"tires\.(front|material)"):
        validate_supported(replace(cfg, tires=tires), rider)


def test_native_capability_accepts_scalar_tuning():
    """Numerical tuning stays inside the supported topology."""
    cfg, rider = _supported()
    tuned = replace(cfg, timestep_s=0.000625, control_period_s=0.005,
                    initial_speed_mps=0.0, initial_front_brake=0.0,
                    initial_rear_brake=1.0)
    tuned = replace(tuned, drive=replace(tuned.drive, human_torque_nm=0.0,
                                         torque_ripple=0.0))
    validate_supported(tuned, rider)


def test_native_capability_accepts_disabled_features():
    """Disabling a supported controller feature stays supported."""
    cfg, rider = _supported()
    drive = replace(cfg.drive,
                    assist=replace(cfg.drive.assist, gain=0.0),
                    battery=replace(cfg.drive.battery, enabled=False),
                    shifting=replace(cfg.drive.shifting, enabled=False))
    validate_supported(
        replace(cfg, drive=drive,
                seated_climb=replace(cfg.seated_climb, enabled=False)),
        rider)


def test_native_extension_exposes_native_ride_runtime():
    """The selected artifact exposes the runtime boundary type."""
    module = load_native()
    assert hasattr(module, 'NativeRideRuntime')


def test_native_dependency_version_mismatch_rejected():
    """The runtime envelope pins its native dependency versions."""
    from bike_sim.native.artifact import check_dependencies
    check_dependencies({"mujoco": "3.12.0"}, installed={"mujoco": "3.12.0"})
    with pytest.raises(ValueError, match="mujoco"):
        check_dependencies({"mujoco": "3.12.0"},
                           installed={"mujoco": "3.11.0"})
