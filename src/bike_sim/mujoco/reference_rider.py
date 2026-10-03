"""Reference seated-rider attachments that are neither pad nor weld models.

The reference plant pins the pelvis to the frame at the declared saddle site
and keeps each foot welded to its own pedal platform. A point pin carries no
couple, so the pelvis keeps its free pitch: a healthy saddle solve reports a
zero COP moment by construction, never by clipping a measured one.
"""
import xml.etree.ElementTree as ET
from numbers import Real


def reference_joint_names() -> tuple[str, ...]:
    """The 11 internal joints of the two-arm reference rider topology."""
    return ('rider_torso_hinge',) + tuple(
        f'rider_{joint}_{side}'
        for side in ('left', 'right') for joint in ('shoulder', 'elbow')) + tuple(
        f'rider_{joint}_{side}'
        for side in ('front', 'rear') for joint in ('hip', 'knee', 'ankle'))


def add_saddle_pin(root: ET.Element, solref_s: float) -> None:
    """Replace the legacy pelvis weld with a three-row `connect` equality.

    `anchor` is the declared saddle site in rider_pelvis-local coordinates;
    MuJoCo derives the coincident frame point from the initial configuration,
    so the pose solver must place the site on the physical saddle, not in air.
    """
    if not (isinstance(solref_s, Real) and solref_s > 0):
        raise ValueError('saddle pin requires a positive time constant')
    equality = root.find('equality')
    site = root.find(".//site[@name='site_rider_saddle']")
    if equality is None or site is None:
        raise ValueError('saddle pin requires an equality block and saddle site')
    old = equality.find("weld[@name='weld_saddle']")
    if old is not None:
        equality.remove(old)
    ET.SubElement(equality, 'connect', name='connect_saddle',
                  body1='rider_pelvis', body2='frame',
                  anchor=site.get('pos', '0 0 0'),
                  solref=f'{float(solref_s):.17g} 1')
