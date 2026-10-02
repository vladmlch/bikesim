"""
XML Formatting & Element Creation Utilities for MuJoCo MJCF Serializers.

Provides compact coordinate/vector formatting and declarative MJCF element builder helpers.
"""

from typing import Optional, Sequence, Tuple, Union
import xml.etree.ElementTree as ET
import numpy as np


def _format_vec(vec: Union[np.ndarray, Sequence[float]], precision: int = 6) -> str:
    """
    Formats a 3D numeric vector into a space-separated string.

    Args:
        vec: 3D vector or sequence of floats.
        precision: Number of decimal places.

    Returns:
        Space-separated float string (e.g. '0.123456 0.000000 0.654321').
    """
    return " ".join(f"{val:.{precision}f}" for val in vec)


def _format_fromto(
    p1: Union[np.ndarray, Sequence[float]],
    p2: Union[np.ndarray, Sequence[float]],
    precision: int = 6,
) -> str:
    """
    Formats two 3D points into a MuJoCo 'fromto' space-separated string.

    Args:
        p1: Start point (3D).
        p2: End point (3D).
        precision: Number of decimal places.

    Returns:
        Space-separated 6-float string (e.g. '0 0 0 1 0 0').
    """
    return f"{_format_vec(p1, precision)} {_format_vec(p2, precision)}"


def add_geom(
    parent: ET.Element,
    name: str,
    geom_type: str,
    *,
    fromto: Optional[str] = None,
    pos: Optional[Union[str, np.ndarray, Sequence[float]]] = None,
    size: Optional[str] = None,
    euler: Optional[str] = None,
    mass: Optional[Union[str, float]] = None,
    material: Optional[str] = None,
    rgba: Optional[str] = None,
    contype: Optional[str] = None,
    conaffinity: Optional[str] = None,
    friction: Optional[str] = None,
    condim: Optional[str] = None,
    solref: Optional[str] = None,
) -> ET.Element:
    """Creates and appends a <geom> element to parent with exact attribute preservation."""
    attrib = {"name": name, "type": geom_type}
    if fromto is not None:
        attrib["fromto"] = fromto
    if pos is not None:
        attrib["pos"] = pos if isinstance(pos, str) else _format_vec(pos)
    if size is not None:
        attrib["size"] = str(size)
    if euler is not None:
        attrib["euler"] = euler
    if mass is not None:
        attrib["mass"] = str(mass)
    if material is not None:
        attrib["material"] = material
    if rgba is not None:
        attrib["rgba"] = rgba
    if contype is not None:
        attrib["contype"] = str(contype)
    if conaffinity is not None:
        attrib["conaffinity"] = str(conaffinity)
    if friction is not None:
        attrib["friction"] = friction
    if condim is not None:
        attrib["condim"] = str(condim)
    if solref is not None:
        attrib["solref"] = solref
    return ET.SubElement(parent, "geom", attrib)



def add_joint(
    parent: ET.Element,
    name: str,
    joint_type: str = "hinge",
    *,
    pos: str = "0 0 0",
    axis: Optional[Union[str, np.ndarray, Sequence[float]]] = None,
    range: Optional[str] = None,
    stiffness: Optional[str] = None,
    damping: Optional[str] = None,
    springref: Optional[str] = None,
    solreflimit: Optional[str] = None,
) -> ET.Element:
    """Creates and appends a <joint> element to parent."""
    attrib = {"name": name, "type": joint_type, "pos": pos}
    if axis is not None:
        attrib["axis"] = axis if isinstance(axis, str) else _format_vec(axis)
    if range is not None:
        attrib["range"] = range
    if stiffness is not None:
        attrib["stiffness"] = str(stiffness)
    if damping is not None:
        attrib["damping"] = str(damping)
    if springref is not None:
        attrib["springref"] = str(springref)
    if solreflimit is not None:
        attrib["solreflimit"] = solreflimit
    return ET.SubElement(parent, "joint", attrib)


def add_site(
    parent: ET.Element,
    name: str,
    *,
    pos: Optional[Union[str, np.ndarray, Sequence[float]]] = None,
    size: Optional[str] = None,
    rgba: Optional[str] = None,
    site_type: Optional[str] = None,
) -> ET.Element:
    """Creates and appends a <site> element to parent."""
    attrib = {"name": name}
    if pos is not None:
        attrib["pos"] = pos if isinstance(pos, str) else _format_vec(pos)
    if size is not None:
        attrib["size"] = str(size)
    if rgba is not None:
        attrib["rgba"] = rgba
    if site_type is not None:
        attrib["type"] = site_type
    return ET.SubElement(parent, "site", attrib)


def add_marker(
    parent: ET.Element,
    name: str,
    pos: Union[str, np.ndarray, Sequence[float]],
    size: str = "0.013",
    material: str = "mat_pivot_yellow",
) -> ET.Element:
    """Creates and appends a debug visual marker sphere."""
    pos_str = pos if isinstance(pos, str) else _format_vec(pos)
    return ET.SubElement(
        parent,
        "geom",
        {
            "name": name,
            "type": "sphere",
            "pos": pos_str,
            "size": str(size),
            "material": material,
            "contype": "0",
            "conaffinity": "0",
            "mass": "0",
        },
    )


