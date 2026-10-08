"""Shared fixtures for tests/reference.

Aligns application adapters with the selected test artifact without importing
an extension or changing production defaults. An explicit application selector
remains authoritative so mismatches remain detectable.
"""

import os

import pytest
from native_loader import selected_build


@pytest.fixture(autouse=True)
def select_application_artifact_for_tests(monkeypatch):
    if "BIKE_NATIVE_BUILD_PATH" not in os.environ:
        monkeypatch.setenv("BIKE_NATIVE_BUILD_PATH", str(selected_build(os.environ)))
