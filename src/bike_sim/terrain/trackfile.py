"""
TOML Track Files.

A track file is the user's way to sketch a road: a list of hand-placed obstacles, an
optional procedural ``[generator]`` block that fills the free stretches, or both.
Loading yields an ordinary validated :class:`TrackSpec`; dumping writes the materialised
layout back out, so a generated road can be frozen, inspected and hand-edited.

Units follow how people describe road defects: heights, depths and amplitudes in
**millimetres** (``*_mm`` keys), positions and lengths in **metres**. The loader converts
to the metres the terrain package works in. Every obstacle type also accepts its
dataclass field names verbatim in metres, so a dumped file round-trips exactly.

Format::

    name = "my_road"
    length_m = 150
    description = "optional"

    [[obstacles]]
    type = "pothole"          # sharp | sloped | bowl via `edge`
    start_m = 20.0
    depth_mm = 80
    length_m = 0.5
    edge = "sharp"

    [[obstacles]]
    type = "bump"             # cosine | trapezoid via `shape`
    start_m = 35.0
    height_mm = 50
    length_m = 0.4
    shape = "cosine"

    [generator]
    seed = 0
    potholes_per_100m = 4
    pothole_depth_mm = [40, 120]
    ...
"""

from dataclasses import fields as dataclass_fields
from pathlib import Path
import tomllib
from typing import Any, Dict, List, Mapping, Tuple, Type, Union

from bike_sim.terrain.obstacles import (
    BowlPothole,
    Bump,
    Drop,
    GOut,
    Kicker,
    Obstacle,
    Pothole,
    RoadRoughness,
    RockGarden,
    Roots,
    SlopedPothole,
    SquareEdge,
    TrapezoidBump,
    Washboard,
)
from bike_sim.terrain.profile import TrackSpec
from bike_sim.terrain.road import (
    BUMP_SHAPES,
    POTHOLE_EDGES,
    RoadGeneratorSpec,
    generate_road,
)

FILE_SUFFIX = ".toml"

# Type name in the file -> obstacle class. Potholes and bumps are resolved through their
# `edge` / `shape` key; the remaining names map one to one.
_PLAIN_TYPES: Dict[str, Type[Obstacle]] = {
    "square_edge": SquareEdge,
    "washboard": Washboard,
    "g_out": GOut,
    "drop": Drop,
    "kicker": Kicker,
    "roots": Roots,
    "rock_garden": RockGarden,
    "roughness": RoadRoughness,
}
_POTHOLE_CLASSES: Dict[str, Type[Obstacle]] = {
    "sharp": Pothole,
    "sloped": SlopedPothole,
    "bowl": BowlPothole,
}
_BUMP_CLASSES: Dict[str, Type[Obstacle]] = {
    "cosine": Bump,
    "trapezoid": TrapezoidBump,
}

# The file's generic `length_m` maps onto each class's own length field.
_LENGTH_FIELD: Dict[Type[Obstacle], str] = {
    Pothole: "hole_length_m",
    SlopedPothole: "hole_length_m",
    BowlPothole: "hole_length_m",
    Bump: "bump_length_m",
    SquareEdge: "ledge_length_m",
    GOut: "dip_length_m",
    Roots: "section_length_m",
    RockGarden: "section_length_m",
    RoadRoughness: "section_length_m",
}

# Class -> (type name, shape key, shape value) for dumping.
_CLASS_TAGS: Dict[Type[Obstacle], Tuple[str, str, str]] = {
    **{cls: (name, "", "") for name, cls in _PLAIN_TYPES.items()},
    **{cls: ("pothole", "edge", edge) for edge, cls in _POTHOLE_CLASSES.items()},
    **{cls: ("bump", "shape", shape) for shape, cls in _BUMP_CLASSES.items()},
}

_GENERATOR_RANGE_FIELDS = ("pothole_depth_m", "pothole_length_m", "bump_height_m", "bump_length_m")


class TrackFileError(ValueError):
    """Raised when a track file is malformed."""


# --------------------------------------------------------------------------------------
# Loading
# --------------------------------------------------------------------------------------


def _field_names(cls: Type[Obstacle]) -> List[str]:
    return [f.name for f in dataclass_fields(cls)]


def _resolve_class(entry: Mapping[str, Any], index: int) -> Type[Obstacle]:
    kind = entry.get("type")
    if not isinstance(kind, str):
        raise TrackFileError(f"obstacles[{index}]: missing 'type'")
    if kind == "pothole":
        edge = entry.get("edge", "sharp")
        if edge not in _POTHOLE_CLASSES:
            raise TrackFileError(f"obstacles[{index}]: unknown pothole edge '{edge}'; allowed: {list(POTHOLE_EDGES)}")
        return _POTHOLE_CLASSES[edge]
    if kind == "bump":
        shape = entry.get("shape", "cosine")
        if shape not in _BUMP_CLASSES:
            raise TrackFileError(f"obstacles[{index}]: unknown bump shape '{shape}'; allowed: {list(BUMP_SHAPES)}")
        return _BUMP_CLASSES[shape]
    if kind in _PLAIN_TYPES:
        return _PLAIN_TYPES[kind]
    allowed = ["pothole", "bump", *_PLAIN_TYPES]
    raise TrackFileError(f"obstacles[{index}]: unknown type '{kind}'; allowed: {allowed}")


def _convert_keys(raw: Mapping[str, Any], names: List[str], length_field: str, where: str) -> Dict[str, Any]:
    """Maps file keys onto dataclass fields, converting ``*_mm`` to metres."""
    out: Dict[str, Any] = {}
    for key, value in raw.items():
        if key == "length_m" and length_field and "length_m" not in names:
            target = length_field
        elif key.endswith("_mm") and key[:-3] + "_m" in names:
            target = key[:-3] + "_m"
            value = _scale(value, 1e-3)
        else:
            target = key
        if target not in names:
            raise TrackFileError(f"{where}: unknown key '{key}'; allowed: {sorted(names)}")
        if target in out:
            raise TrackFileError(f"{where}: '{target}' given twice (metres and millimetres?)")
        if target in ("seed", "n_bumps", "n_waves") and not (isinstance(value, int) and not isinstance(value, bool)):
            raise TrackFileError(f"{where}: '{key}' must be an integer, got {value!r}")
        out[target] = value
    return out


def _scale(value: Any, factor: float) -> Any:
    if isinstance(value, (list, tuple)):
        return [float(v) * factor for v in value]
    return float(value) * factor


def obstacle_from_dict(entry: Mapping[str, Any], index: int = 0) -> Obstacle:
    """
    Builds one obstacle from a track-file table.

    Args:
        entry: The ``[[obstacles]]`` table.
        index: Position in the file, for error messages.

    Returns:
        The obstacle.

    Raises:
        TrackFileError: On unknown type, shape or key.
    """
    cls = _resolve_class(entry, index)
    selector = {"pothole": "edge", "bump": "shape"}.get(entry["type"])
    raw = {k: v for k, v in entry.items() if k != "type" and k != selector}
    where = f"obstacles[{index}] ({entry.get('type')})"
    kwargs = _convert_keys(raw, _field_names(cls), _LENGTH_FIELD.get(cls, ""), where)
    if "start_m" not in kwargs:
        raise TrackFileError(f"{where}: missing 'start_m'")
    return cls(**kwargs)


def generator_from_dict(entry: Mapping[str, Any]) -> RoadGeneratorSpec:
    """
    Builds a generator spec from the ``[generator]`` table.

    Args:
        entry: The table; ``*_mm`` keys are converted, two-element lists become ranges,
            scalars become fixed ranges.

    Returns:
        The validated spec.

    Raises:
        TrackFileError: On unknown keys or malformed ranges.
    """
    names = _field_names(RoadGeneratorSpec)
    kwargs = _convert_keys(entry, names, "", "generator")
    for name in _GENERATOR_RANGE_FIELDS:
        if name in kwargs:
            value = kwargs[name]
            if isinstance(value, (int, float)):
                kwargs[name] = (float(value), float(value))
            elif isinstance(value, (list, tuple)) and len(value) == 2:
                kwargs[name] = (float(value[0]), float(value[1]))
            else:
                raise TrackFileError(f"generator: '{name}' must be a number or a [low, high] pair")
    for name in ("pothole_edge", "bump_shape"):
        if name in kwargs and isinstance(kwargs[name], str):
            kwargs[name] = {kwargs[name]: 1.0}
    spec = RoadGeneratorSpec(**kwargs)
    try:
        spec.validate()
    except ValueError as exc:
        raise TrackFileError(f"generator: {exc}") from exc
    return spec


ROUGHNESS_SPLIT_SEED_STEP = 1_000_003
"""Seed offset given to the right-hand remainder when a roughness segment is split."""

MIN_ROUGHNESS_REMAINDER_M = 0.5
"""Roughness remainders shorter than this are dropped rather than kept as slivers."""


def carve_roughness(obstacles: List[Obstacle]) -> List[Obstacle]:
    """
    Makes background roughness yield to everything placed on top of it.

    A dumped generated road lists its roughness as explicit segments spanning the free
    stretches, so the natural edit -- add a pothole to a dumped file -- would collide with
    one. Roughness is texture, not a defect, so instead of rejecting the file each segment
    is split around any other obstacle it overlaps. The left remainder keeps the segment's
    seed; the right one gets ``seed + ROUGHNESS_SPLIT_SEED_STEP`` so it is deterministic
    but not a repeat. Remainders shorter than ``MIN_ROUGHNESS_REMAINDER_M`` are dropped.

    Two non-roughness obstacles that overlap are left for ``TrackSpec.validate`` to reject.

    Args:
        obstacles: Obstacles as listed in the file.

    Returns:
        The same obstacles with roughness segments split, sorted by start position.
    """
    defects = [o for o in obstacles if not isinstance(o, RoadRoughness)]
    roughness = [o for o in obstacles if isinstance(o, RoadRoughness)]
    if not roughness or not defects:
        return sorted(obstacles, key=lambda o: o.start_m)

    carved: List[RoadRoughness] = []
    for seg in roughness:
        pieces: List[Tuple[float, float, int]] = [(seg.start_m, seg.end_m, seg.seed)]
        for d in sorted(defects, key=lambda o: o.start_m):
            next_pieces: List[Tuple[float, float, int]] = []
            for a, b, sd in pieces:
                if d.end_m <= a or d.start_m >= b:
                    next_pieces.append((a, b, sd))
                    continue
                if d.start_m > a:
                    next_pieces.append((a, d.start_m, sd))
                if d.end_m < b:
                    next_pieces.append((d.end_m, b, sd + ROUGHNESS_SPLIT_SEED_STEP))
            pieces = next_pieces
        for a, b, sd in pieces:
            if b - a >= MIN_ROUGHNESS_REMAINDER_M:
                # Trim float noise without ever crossing into the neighbour (validate()
                # rejects a one-ulp overlap).
                seg_len = round(b - a, 6)
                while a + seg_len > b:
                    seg_len = round(seg_len - 1e-6, 6)
                carved.append(
                    RoadRoughness(
                        start_m=a,
                        section_length_m=seg_len,
                        amplitude_m=seg.amplitude_m,
                        correlation_length_m=seg.correlation_length_m,
                        seed=sd,
                    )
                )
    return sorted(defects + carved, key=lambda o: o.start_m)


def track_from_dict(data: Mapping[str, Any], *, seed: Union[int, None] = None,
                    length_m: Union[float, None] = None) -> TrackSpec:
    """
    Builds a validated track from parsed file contents.

    Args:
        data: Parsed TOML document.
        seed: Overrides ``generator.seed`` when given.
        length_m: Overrides the file's ``length_m`` when given.

    Returns:
        The track, hand-placed obstacles first, generated ones after.

    Raises:
        TrackFileError: On a malformed document or an invalid layout.
    """
    name = data.get("name")
    if not isinstance(name, str) or not name:
        raise TrackFileError("track file needs a non-empty 'name'")
    track_length = float(length_m if length_m is not None else data.get("length_m", 0.0))
    if track_length <= 0.0:
        raise TrackFileError("track file needs a positive 'length_m'")

    entries = data.get("obstacles", [])
    if not isinstance(entries, list):
        raise TrackFileError("'obstacles' must be an array of tables ([[obstacles]])")
    hand_placed = carve_roughness([obstacle_from_dict(e, i) for i, e in enumerate(entries)])

    generated: List[Obstacle] = []
    if seed is not None and "generator" not in data:
        raise TrackFileError("a seed override was given but the track file has no [generator] block")
    if "generator" in data:
        gen = dict(data["generator"])
        if seed is not None:
            gen["seed"] = int(seed)
        spec = generator_from_dict(gen)
        try:
            generated = generate_road(spec, track_length, [(o.start_m, o.end_m) for o in hand_placed])
        except ValueError as exc:
            raise TrackFileError(f"generator: {exc}") from exc

    track = TrackSpec(
        name=name,
        length_m=track_length,
        description=str(data.get("description", "")),
        obstacles=hand_placed + generated,
    )
    try:
        track.validate()
    except ValueError as exc:
        raise TrackFileError(str(exc)) from exc
    return track


def load_track(path: Union[str, Path], *, seed: Union[int, None] = None,
               length_m: Union[float, None] = None) -> TrackSpec:
    """
    Loads a track file.

    Args:
        path: Path to the TOML file.
        seed: Overrides the generator seed.
        length_m: Overrides the track length.

    Returns:
        The validated track.

    Raises:
        TrackFileError: On a malformed file.
        FileNotFoundError: If the path does not exist.
    """
    with open(path, "rb") as handle:
        try:
            data = tomllib.load(handle)
        except tomllib.TOMLDecodeError as exc:
            raise TrackFileError(f"{path}: {exc}") from exc
    return track_from_dict(data, seed=seed, length_m=length_m)


# --------------------------------------------------------------------------------------
# Dumping
# --------------------------------------------------------------------------------------


def obstacle_to_dict(obstacle: Obstacle) -> Dict[str, Any]:
    """
    Serialises one obstacle into a track-file table.

    Args:
        obstacle: Obstacle to serialise.

    Returns:
        Ordered mapping with ``type`` first, then ``start_m``, then the class's own
        fields in metres (round-trip exact).
    """
    cls = type(obstacle)
    if cls not in _CLASS_TAGS:
        raise TrackFileError(f"obstacle type {cls.__name__} has no track-file representation")
    kind, shape_key, shape_value = _CLASS_TAGS[cls]
    out: Dict[str, Any] = {"type": kind}
    if shape_key:
        out[shape_key] = shape_value
    for f in dataclass_fields(cls):
        out[f.name] = getattr(obstacle, f.name)
    return out


def track_to_dict(track: TrackSpec) -> Dict[str, Any]:
    """
    Serialises a track into the document structure of a track file.

    The layout is materialised: generated obstacles are written out individually and no
    ``[generator]`` block is emitted, so the file pins exactly what was simulated.
    """
    out: Dict[str, Any] = {"name": track.name, "length_m": track.length_m}
    if track.description:
        out["description"] = track.description
    out["obstacles"] = [obstacle_to_dict(o) for o in track.sorted_obstacles]
    return out


def _toml_value(value: Any) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        return repr(float(value))
    if isinstance(value, str):
        escaped = value.replace("\\", "\\\\").replace('"', '\\"')
        return f'"{escaped}"'
    if isinstance(value, (list, tuple)):
        return "[" + ", ".join(_toml_value(v) for v in value) + "]"
    if isinstance(value, dict):
        return "{ " + ", ".join(f"{k} = {_toml_value(v)}" for k, v in value.items()) + " }"
    raise TrackFileError(f"cannot write value of type {type(value).__name__} to TOML")


def dump_track(track: TrackSpec) -> str:
    """
    Renders a track as TOML text.

    Args:
        track: Track to write.

    Returns:
        TOML document; loading it back yields an equal TrackSpec.
    """
    doc = track_to_dict(track)
    lines = [
        "# Track file for bike-ride. Heights/depths in the classes' own units (metres);",
        "# `*_mm` keys are also accepted when editing by hand. Obstacles may not overlap.",
        "",
    ]
    for key in ("name", "length_m", "description"):
        if key in doc:
            lines.append(f"{key} = {_toml_value(doc[key])}")
    for entry in doc["obstacles"]:
        lines.append("")
        lines.append("[[obstacles]]")
        for key, value in entry.items():
            lines.append(f"{key} = {_toml_value(value)}")
    lines.append("")
    return "\n".join(lines)


def save_track(track: TrackSpec, path: Union[str, Path]) -> Path:
    """
    Writes a track file.

    Args:
        track: Track to write.
        path: Destination path.

    Returns:
        The path written.
    """
    destination = Path(path)
    destination.write_text(dump_track(track), encoding="utf-8")
    return destination


__all__ = [
    "FILE_SUFFIX",
    "TrackFileError",
    "carve_roughness",
    "obstacle_from_dict",
    "generator_from_dict",
    "track_from_dict",
    "load_track",
    "obstacle_to_dict",
    "track_to_dict",
    "dump_track",
    "save_track",
]
