#!/usr/bin/env python3
"""T25 recapture: freeze the welded-flat initial state document per platform.

capture_reference.py's RCP-01 recipe for
tests/fixtures/migration/initial_state/<platform>/welded_flat.json:

    --physics-config tests/fixtures/migration/config/viewer_physics_welded.toml
    --track tests/fixtures/migration/tracks/flat.toml
    --headless --duration 1 --no-plots

requests/welded_flat.toml restores this captured t=0 state through the
audited initial-state transfer. A second build is a guaranteed
equilibrium-cache hit (the first build saves the entry), so the captured
qacc_warmstart is the canonical zeroed restore value and the document bytes
are deterministic on cold and warm caches.

The document is schema_version 2: the contract binds joint envelope/strength
file CONTENT (envelope_sha256/strength_sha256) instead of the resolved
worktree-absolute path spellings, so the artifact validates from any
checkout. Compiled-model fields and the settled pose stay
platform-sensitive, so each platform owns its document under the golden
platform key (darwin-arm64, linux-aarch64) — the request reader prefers the
platform-keyed sibling over the shared spelling.
"""
from __future__ import annotations

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "src"))


def main() -> int:
    from bike_sim.cli import ride as ride_cli
    from bike_sim.sim.ride.initial_state import PhysicalInitialState

    fixture = REPO / "tests" / "fixtures" / "migration"
    args = ride_cli.parse_args([
        "--physics-config",
        str(fixture / "config" / "viewer_physics_welded.toml"),
        "--track", str(fixture / "tracks" / "flat.toml"),
        "--headless", "--duration", "1", "--no-plots"])
    # The first build settles and saves the derived-name cache entry; the
    # second is a guaranteed hit, so qacc_warmstart is the canonical zeroed
    # restore value on cold and warm caches alike.
    sim_ns = ride_cli.build_physical_simulation_from_args(args)
    del sim_ns
    sim_is = ride_cli.build_physical_simulation_from_args(args)
    document = PhysicalInitialState.capture(sim_is)
    import platform
    destination = (fixture / "initial_state" /
                   f"{platform.system().lower()}-{platform.machine()}" /
                   "welded_flat.json")
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(document.payload_json + "\n")
    print("wrote", destination, destination.stat().st_size, "bytes")
    print("schema_version =", document.payload["schema_version"])
    print("contract_sha256 =", document.payload["contract_sha256"])
    print("qpos[3] =", document.payload["qpos"][3])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
