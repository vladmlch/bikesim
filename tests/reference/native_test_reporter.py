"""Report the native test items included in a native/full profile."""

from __future__ import annotations

import pytest


_NATIVE_TOOL_MODULES = {
    'test_native_check_tools.py',
    'test_native_contract_checks.py',
    'test_native_loader.py',
}


def pytest_collection_finish(session: object) -> None:
    items = getattr(session, 'items')
    native_count = sum(
        1
        for item in items
        if item.path.name.startswith('test_native_') and item.path.name not in _NATIVE_TOOL_MODULES
    )
    reporter = session.config.pluginmanager.get_plugin('terminalreporter')
    if reporter is not None:
        reporter.write_line(f'native extension test items collected: {native_count}')
    if native_count == 0:
        pytest.exit(
            'native/full profile collected zero native extension test items',
            returncode=pytest.ExitCode.NO_TESTS_COLLECTED,
        )
    from native_loader import record_native_provenance

    record_native_provenance(native_test_count=native_count)
