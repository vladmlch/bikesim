"""The Accelerate apple-system build_id: the Python spelling of the formula.

Standard library only. ``tools/native_dependencies.py`` loads this file by path
so its stdlib-only probe checker never imports the ``bike_sim`` package, whose
``__init__`` pulls in numpy and mujoco. The formula is spelled in two languages:
here, and in ``native/cmake/BikeProviderIdentity.cmake`` (``bike_accelerate_build_id``)
for the Python-free root graph, which cannot call Python. Both spellings are
pinned to ``tests/fixtures/portability/build_id_vectors.json``.
"""
from __future__ import annotations

import re

# Components are the tool-reported build strings and the LC_UUID. ``+`` is the
# separator, so a component containing it would make the composition
# ambiguous. Whitespace is rejected because tool output must be stripped first.
_COMPONENT = re.compile(r'[A-Za-z0-9.-]+')


def accelerate_build_id(macos_build: str, accelerate_uuid: str,
                        sdk_build: str) -> str:
    """``macos-<macOS build>+accelerate-<umbrella LC_UUID>+sdk-<SDK build>``.

    Raises ValueError for a component that is not a nonempty token of
    ``[A-Za-z0-9.-]``, so a malformed identity fails rather than being spelled.
    """
    components = (('macos_build', macos_build),
                  ('accelerate_uuid', accelerate_uuid),
                  ('sdk_build', sdk_build))
    for name, value in components:
        if not isinstance(value, str) or _COMPONENT.fullmatch(value) is None:
            raise ValueError(
                f'build_id component {name} must be a nonempty token of '
                f'[A-Za-z0-9.-], found {value!r}')
    return (f'macos-{macos_build}+accelerate-{accelerate_uuid}'
            f'+sdk-{sdk_build}')
