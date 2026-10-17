"""Packaging regressions; fixtures never compile or import the simulator."""

from __future__ import annotations

from copy import deepcopy
from hashlib import sha256
import json
import os
from pathlib import Path
import subprocess
import sys
from zipfile import ZipFile

import pytest

from tools.check_offline_sources import compare_sources
from tools.offline_bundle import BundleError, create_bundle, validate_bundle


LINUX = {"system": "linux", "machine": "x86_64", "python": "3.13",
         "distribution": "ubuntu", "release": "24.04"}
MACOS = {"system": "darwin", "machine": "arm64", "python": "3.13",
         "distribution": "macos", "release": "unspecified"}
MACOS314 = {**MACOS, "python": "3.14"}


def wheel(root: Path, *, name: str = "bike_sim", version: str = "0.3.0",
          tags: str = "py3-none-any", package: dict[str, bytes] | None = None) -> Path:
    root.mkdir(parents=True, exist_ok=True)
    path = root / f"{name}-{version}-{tags}.whl"
    with ZipFile(path, "w") as archive:
        archive.writestr(f"{name}-{version}.dist-info/METADATA",
                         f"Metadata-Version: 2.1\nName: {name}\nVersion: {version}\nRequires-Python: >=3.13\n")
        archive.writestr(f"{name}-{version}.dist-info/WHEEL",
                         f"Wheel-Version: 1.0\nRoot-Is-Purelib: true\nTag: {tags}\n")
        for member, data in (package or {}).items():
            archive.writestr(member, data)
    return path


@pytest.fixture
def inputs(tmp_path: Path) -> tuple[Path, Path]:
    source = tmp_path / "source"
    package = source / "src" / "bike_sim"
    package.mkdir(parents=True)
    (package / "__init__.py").write_bytes(b"VALUE = 1\n")
    (package / "settings.json").write_bytes(b'{"tire": "mtb"}\n')
    (source / "pyproject.toml").write_text(
        '[project]\nname = "bike-sim"\nversion = "0.3.0"\n'
        'requires-python = ">=3.13"\ndependencies = ["numpy>=2"]\n'
        '[build-system]\nrequires = ["setuptools>=61"]\n'
        '[dependency-groups]\nnative = ["nanobind>=3.1"]\n', encoding="utf-8")
    wheels = tmp_path / "wheels"
    wheel(wheels, package={
        "bike_sim/__init__.py": b"VALUE = 1\n",
        "bike_sim/settings.json": b'{"tire": "mtb"}\n',
    })
    return source, wheels


def make_bundle(tmp_path: Path, inputs: tuple[Path, Path], **options) -> tuple[Path, dict]:
    output = tmp_path / "bundle.zip"
    manifest = create_bundle(*inputs, output, target=LINUX, profile="slim",
                             preinstalled=["numpy==2.3.0"], **options)
    extracted = tmp_path / "extracted"
    with ZipFile(output) as archive:
        archive.extractall(extracted)
    return extracted, manifest


def write_manifest(bundle: Path, manifest: dict) -> None:
    (bundle / "bundle-manifest.json").write_text(json.dumps(manifest), encoding="utf-8")


def update_digest(manifest: dict, bundle: Path, path: str) -> None:
    data = (bundle / path).read_bytes()
    row = next(row for row in manifest["files"] if row["path"] == path)
    row.update(size=len(data), sha256=sha256(data).hexdigest())


@pytest.mark.parametrize("target", [LINUX, MACOS, MACOS314])
def test_profiles_describe_target_and_never_claim_qualification(tmp_path, inputs, target):
    output = tmp_path / "bundle.zip"
    manifest = create_bundle(*inputs, output, target=target, profile="full")
    assert manifest["target"] == target
    assert set(manifest["producer"]) == {"system", "machine", "python"}
    assert manifest["project"]["name"] == "bike-sim"
    assert manifest["project"]["version"] == "0.3.0"
    assert manifest["project"]["requires_python"] == ">=3.13"
    assert manifest["project"]["authority"] == "source"
    assert manifest["project"]["native_artifacts"] == "rebuild_on_target"
    assert manifest["requirements"]["runtime"] == ["numpy>=2"]
    assert manifest["requirements"]["build"] == ["setuptools>=61", "nanobind>=3.1"]
    assert manifest["requirements"]["preinstalled"] == []
    assert manifest["requirements"]["closure"] == "not_verified"
    assert manifest["requirements"]["wheel_dependency_metadata"] == "not_verified"
    assert set(manifest["qualification"].values()) == {"not_run"}
    assert "source_commit" not in manifest["project"]


def test_bundle_roundtrip_is_byte_stable_and_keeps_legacy_layout(tmp_path, inputs):
    bundle, manifest = make_bundle(tmp_path, inputs)
    second = tmp_path / "second.zip"
    create_bundle(*inputs, second, target=LINUX, profile="slim",
                  preinstalled=["numpy==2.3.0"])
    assert (tmp_path / "bundle.zip").read_bytes() == second.read_bytes()
    assert validate_bundle(bundle) == manifest
    assert (bundle / "project/src/bike_sim/settings.json").is_file()
    assert (bundle / "wheelhouse/bike_sim-0.3.0-py3-none-any.whl").is_file()
    assert (bundle / "requirements.txt").read_text().startswith("bike-sim==0.3.0 --hash=sha256:")


def test_source_changes_and_undeclared_files_are_rejected(tmp_path, inputs):
    bundle, _ = make_bundle(tmp_path, inputs)
    path = bundle / "project/src/bike_sim/__init__.py"
    original = path.read_bytes()
    path.write_bytes(b"VALUE = 2\n")
    with pytest.raises(BundleError, match="digest"):
        validate_bundle(bundle)
    path.write_bytes(original)
    (bundle / "injected.txt").write_text("undeclared")
    with pytest.raises(BundleError, match="inventory"):
        validate_bundle(bundle)


@pytest.mark.parametrize("bad_path", ["/absolute", "../outside", "project/../outside",
                                      "project//file", "project/./file", "C:/outside",
                                      "project\\file", "project/file\nname"])
def test_manifest_rejects_unsafe_paths(tmp_path, inputs, bad_path):
    bundle, manifest = make_bundle(tmp_path, inputs)
    manifest["files"][0]["path"] = bad_path
    write_manifest(bundle, manifest)
    with pytest.raises(BundleError, match="path"):
        validate_bundle(bundle)


@pytest.mark.parametrize("change", ["duplicate", "case_collision", "boolean_size", "negative_size",
                                    "bad_digest", "extra_field", "qualified", "missing_external"])
def test_manifest_rejects_malformed_metadata(tmp_path, inputs, change):
    bundle, manifest = make_bundle(tmp_path, inputs)
    if change == "duplicate":
        manifest["files"].append(deepcopy(manifest["files"][0]))
    elif change == "case_collision":
        row = deepcopy(manifest["files"][0])
        row["path"] = row["path"].upper()
        manifest["files"].append(row)
    elif change == "boolean_size":
        manifest["files"][0]["size"] = True
    elif change == "negative_size":
        manifest["files"][0]["size"] = -1
    elif change == "bad_digest":
        manifest["files"][0]["sha256"] = "A" * 64
    elif change == "extra_field":
        manifest["invented"] = True
    elif change == "qualified":
        manifest["qualification"]["native_build"] = "passed"
    else:
        manifest["requirements"]["preinstalled"] = []
    write_manifest(bundle, manifest)
    with pytest.raises(BundleError):
        validate_bundle(bundle)


def test_duplicate_json_keys_are_rejected(tmp_path, inputs):
    bundle, _ = make_bundle(tmp_path, inputs)
    (bundle / "bundle-manifest.json").write_text('{"schema_version": 1, "schema_version": 1}')
    with pytest.raises(BundleError, match="duplicate"):
        validate_bundle(bundle)


def test_bundle_and_source_symlinks_are_rejected(tmp_path, inputs):
    bundle, _ = make_bundle(tmp_path, inputs)
    path = bundle / "project/src/bike_sim/settings.json"
    path.unlink()
    path.symlink_to(inputs[0] / "src/bike_sim/settings.json")
    with pytest.raises(BundleError, match="symlink"):
        validate_bundle(bundle)
    (inputs[0] / "src/bike_sim/external.py").symlink_to(tmp_path / "outside.py")
    with pytest.raises(BundleError, match="symlink"):
        create_bundle(*inputs, tmp_path / "bad.zip", target=LINUX, profile="full")


@pytest.mark.parametrize("path", [".venv/site-packages/secret.py", "native/build-x86/cache.txt",
                                   "src/bike_sim/__pycache__/cached.pyc", "native/libbike.so.1",
                                   "native/libbike.dylib", "native/generated.o", "cached.whl"])
def test_producer_excludes_machine_state_and_binary_build_outputs(tmp_path, inputs, path):
    state = inputs[0] / path
    state.parent.mkdir(parents=True, exist_ok=True)
    state.write_text("not portable")
    bundle, manifest = make_bundle(tmp_path, inputs)
    assert f"project/{path}" not in {row["path"] for row in manifest["files"]}
    assert not (bundle / "project" / path).exists()


@pytest.mark.parametrize("path", ["native/.clangd", "native/.clang-tidy", "native/.clang-format",
                                   ".editorconfig", "migration/.gitattributes"])
def test_source_build_and_editor_dotfiles_are_preserved(tmp_path, inputs, path):
    config = inputs[0] / path
    config.parent.mkdir(parents=True, exist_ok=True)
    config.write_text("source configuration\n")
    bundle, manifest = make_bundle(tmp_path, inputs)
    assert f"project/{path}" in {row["path"] for row in manifest["files"]}
    assert (bundle / "project" / path).read_text() == "source configuration\n"


@pytest.mark.parametrize("requirement", [">=3.14", ">=3.13,<3.13", ">=3.13.1", "!=3.13.1"])
def test_source_python_floor_or_unsupported_patch_specifier_cannot_be_silently_ignored(tmp_path, inputs, requirement):
    path = inputs[0] / "pyproject.toml"
    path.write_text(path.read_text().replace('requires-python = ">=3.13"', f'requires-python = "{requirement}"'))
    with pytest.raises(BundleError, match="requires-python"):
        create_bundle(*inputs, tmp_path / "bad.zip", target=LINUX, profile="full")


@pytest.mark.parametrize("extra", ["bike_sim-0.3.0.data/purelib/bike_sim/__init__.py",
                                    "bike_sim-0.3.0.data/platlib/bike_sim/__init__.py", "bike_sim.py"])
def test_project_wheel_cannot_spread_payload_over_authoritative_source(tmp_path, inputs, extra):
    wheel(inputs[1], package={"bike_sim/__init__.py": b"VALUE = 1\n",
                             "bike_sim/settings.json": b'{"tire": "mtb"}\n', extra: b"VALUE = 99\n"})
    with pytest.raises(BundleError, match="outside authoritative"):
        create_bundle(*inputs, tmp_path / "bad.zip", target=LINUX, profile="full")


@pytest.mark.parametrize("source_table,wheel_group", [("scripts", "console_scripts"),
                                                      ("gui-scripts", "gui_scripts"),
                                                      ('entry-points."BikePlugin"', "BikePlugin")])
def test_entry_point_metadata_matches_source_and_preserves_case(tmp_path, inputs, source_table, wheel_group):
    config = inputs[0] / "pyproject.toml"
    config.write_text(config.read_text() + f'\n[project.{source_table}]\nRide = "bike_sim:main"\n')
    contents = {"bike_sim/__init__.py": b"VALUE = 1\n", "bike_sim/settings.json": b'{"tire": "mtb"}\n',
                "bike_sim-0.3.0.dist-info/entry_points.txt": f"[{wheel_group}]\nRide = bike_sim:main\n".encode()}
    wheel(inputs[1], package=contents)
    create_bundle(*inputs, tmp_path / "good.zip", target=LINUX, profile="full")
    contents["bike_sim-0.3.0.dist-info/entry_points.txt"] = f"[{wheel_group}]\nride = bike_sim:old_main\n".encode()
    wheel(inputs[1], package=contents)
    with pytest.raises(BundleError, match="entry points"):
        create_bundle(*inputs, tmp_path / "stale.zip", target=LINUX, profile="full")


@pytest.mark.parametrize("member,data", [("bike_sim/__init__.py", b"VALUE = 2\n"),
                                          ("bike_sim/settings.json", b"{}\n")])
def test_stale_project_wheel_rejected_for_code_and_resources(tmp_path, inputs, member, data):
    contents = {"bike_sim/__init__.py": b"VALUE = 1\n",
                "bike_sim/settings.json": b'{"tire": "mtb"}\n'}
    contents[member] = data
    wheel(inputs[1], package=contents)
    with pytest.raises(BundleError, match="source"):
        create_bundle(*inputs, tmp_path / "bad.zip", target=LINUX, profile="full")


def test_missing_project_wheel_and_missing_package_file_fail(tmp_path, inputs):
    (inputs[1] / "bike_sim-0.3.0-py3-none-any.whl").unlink()
    with pytest.raises(BundleError, match="exactly one"):
        create_bundle(*inputs, tmp_path / "missing.zip", target=LINUX, profile="full")
    wheel(inputs[1], package={"bike_sim/__init__.py": b"VALUE = 1\n"})
    with pytest.raises(BundleError, match="source"):
        create_bundle(*inputs, tmp_path / "missing-file.zip", target=LINUX, profile="full")


@pytest.mark.parametrize("tags", ["cp313-cp313-macosx_12_0_arm64", "cp313-cp313-manylinux_2_28_aarch64",
                                   "cp314-cp314-manylinux_2_28_x86_64", "cp313-cp313t-manylinux_2_28_x86_64"])
def test_obviously_incompatible_wheel_tags_fail(tmp_path, inputs, tags):
    wheel(inputs[1], name="dependency", version="1.0", tags=tags)
    with pytest.raises(BundleError, match="target"):
        create_bundle(*inputs, tmp_path / "bad.zip", target=LINUX, profile="full")


@pytest.mark.parametrize("target,tags", [(MACOS314, "cp313-abi3-macosx_12_0_arm64"),
                                        (MACOS314, "cp314-cp314-macosx_12_0_universal2"),
                                        (LINUX, "cp310-abi3-manylinux_2_28_x86_64")])
def test_target_drives_python_and_stable_abi_tag_eligibility(tmp_path, inputs, target, tags):
    wheel(inputs[1], name="dependency", version="1.0", tags=tags)
    manifest = create_bundle(*inputs, tmp_path / "good.zip", target=target, profile="full")
    assert manifest["target"] == target
    assert manifest["qualification"]["binary_compatibility"] == "not_run"


def test_standalone_checker_and_installer_do_not_create_bundle_bytecode(tmp_path, inputs):
    source_tools = Path(__file__).resolve().parents[1] / "tools"
    destination = inputs[0] / "tools"
    destination.mkdir()
    for name in ("check_offline_sources.py", "install_offline.py", "offline_bundle.py"):
        (destination / name).write_bytes((source_tools / name).read_bytes())
    bundle, _ = make_bundle(tmp_path, inputs)
    environment = {name: value for name, value in os.environ.items()
                   if name not in {"PYTHONDONTWRITEBYTECODE", "PYTHONPYCACHEPREFIX"}}
    checker = bundle / "project/tools/check_offline_sources.py"
    # Deliberately omit -B: the standalone script must prevent its own sibling
    # import from changing the exact inventory before content validation.
    result = subprocess.run([sys.executable, str(checker), "--bundle", str(bundle)],
                            capture_output=True, text=True, cwd=tmp_path, env=environment)
    assert result.returncode == 0, result.stderr
    installer = bundle / "project/tools/install_offline.py"
    help_result = subprocess.run([sys.executable, str(installer), "--help"],
                                 capture_output=True, text=True, cwd=tmp_path, env=environment)
    assert help_result.returncode == 0, help_result.stderr
    assert not list(bundle.rglob("__pycache__"))
    validate_bundle(bundle)


@pytest.mark.parametrize("member", ["../escape", "repeated-slash//"])
def test_unsafe_wheel_members_fail_before_publication(tmp_path, inputs, member):
    wheel(inputs[1], name="dependency", version="1.0", package={member: b"bad"})
    output = tmp_path / "bad.zip"
    with pytest.raises(BundleError, match="path"):
        create_bundle(*inputs, output, target=LINUX, profile="full")
    assert not output.exists()


def test_full_has_no_inherited_python_packages_and_output_is_outside_source(tmp_path, inputs):
    with pytest.raises(BundleError, match="full"):
        create_bundle(*inputs, tmp_path / "bad.zip", target=LINUX, profile="full",
                      preinstalled=["numpy==2.3.0"])
    with pytest.raises(BundleError, match="outside"):
        create_bundle(*inputs, inputs[0] / "self.zip", target=LINUX, profile="full")


def test_manifest_requirements_file_cannot_be_changed_even_with_updated_file_digest(tmp_path, inputs):
    bundle, manifest = make_bundle(tmp_path, inputs)
    (bundle / "requirements.txt").write_text("--index-url https://example.invalid\n")
    update_digest(manifest, bundle, "requirements.txt")
    write_manifest(bundle, manifest)
    with pytest.raises(BundleError, match="requirements.txt"):
        validate_bundle(bundle)


def test_legacy_comparator_keeps_its_python_only_report_shape(inputs):
    source, wheels = inputs
    report = compare_sources(source / "src", wheels / "bike_sim-0.3.0-py3-none-any.whl")
    assert report == {"missing_source": [], "missing_wheel": [], "different": []}
    (source / "src/bike_sim/__init__.py").write_bytes(b"VALUE = 2\n")
    assert compare_sources(source / "src", wheels / "bike_sim-0.3.0-py3-none-any.whl")["different"] == ["bike_sim/__init__.py"]


def test_installer_manifest_guard_preserves_no_manifest_and_rejects_unsupported_targets(tmp_path, monkeypatch):
    from tools import install_offline

    monkeypatch.setattr(install_offline, "BUNDLE", tmp_path)
    assert install_offline._check_bundle_manifest() is None
    manifest_path = tmp_path / "bundle-manifest.json"
    manifest_path.write_text("{}")
    manifest = {"profile": "slim", "target": LINUX}
    monkeypatch.setattr(install_offline, "validate_bundle", lambda _: manifest)
    assert install_offline._check_bundle_manifest() == manifest
    manifest["profile"] = "full"
    with pytest.raises(ValueError, match="T25"):
        install_offline._check_bundle_manifest()
    manifest["profile"] = "slim"
    manifest["target"] = MACOS
    with pytest.raises(ValueError, match="T25"):
        install_offline._check_bundle_manifest()
