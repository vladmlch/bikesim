# Python-free release qualification

This is the active Phase 3/4 qualification checklist. A row closes only
with evidence from the final source commit, a recorded command, its exit
code, date and concrete CI run or local evidence section. An implemented
test, preset or workflow is not a completed row. No row below was run or
closed during the 2026-10-21 source-only Phase 3 implementation.

Linux is **aarch64 in the qualified container**. Hosted Linux jobs use
`ubuntu-24.04-arm` and build/run inside `tools/linux/container.sh`; this
runner label is documented in the
[GitHub runner reference](https://docs.github.com/en/actions/reference/runners/github-hosted-runners).
Real-time results must additionally identify the host and CPU allocation.
`release` uses Clang 18; Linux `debug` uses GCC. macOS GUI acceptance remains
a manual user action on a real display. Linux Xvfb smoke never closes a
desktop GUI acceptance row.

| Required lane | Host | Preset | Command | CI run or local evidence | Exit | Date | Commit |
| --- | --- | --- | --- | --- | --- | --- | --- |
| macos-arm64-release | macOS arm64 (M4 Max, macOS 27.0.1) | release | `bash tools/ci_presets.sh release package` (superset of the `checks` mode: identical workflow + sweeps) | `output/ci/Darwin-arm64-release-package/`; 181/181 ordinary CTest, all 10 sweeps green; [task-403-report](../../.superpowers/sdd/2026-10-20-cpp-linux-completion/task-403-report.md) | 0 | 2026-11-01 | c5e114f9159a1b5fcdf074123c72a0bb653dab84 |
| macos-arm64-asan | macOS arm64 | asan | `bash tools/ci_presets.sh asan` | Not run — deferred; not qualified | — | — | — |
| macos-arm64-coverage | macOS arm64 | coverage | `bash tools/ci_presets.sh coverage` | Not run — deferred; not qualified | — | — | — |
| linux-aarch64-release | aarch64 host + container | release | `bash tools/ci_presets.sh release` | Not run — deferred to the linux wave; not qualified | — | — | — |
| linux-aarch64-debug | aarch64 host + container | debug | `bash tools/ci_presets.sh debug` | Not run — deferred to the linux wave; not qualified | — | — | — |
| linux-aarch64-asan | aarch64 host + container | asan | `bash tools/ci_presets.sh asan` | Not run — deferred to the linux wave; not qualified | — | — | — |
| linux-aarch64-coverage | aarch64 host + container | coverage | `bash tools/ci_presets.sh coverage` | Not run — deferred to the linux wave; not qualified | — | — | — |
| package-macos | macOS arm64 (M4 Max, macOS 27.0.1) | release | `bash tools/ci_presets.sh release package` | `output/ci/Darwin-arm64-release-package/` (`exit-code.txt`=0); `package_smoke` 76.77 s, `-L package` 3/3, `-L audit` 4/4; binary TGZ `de0f6870…826d7b`, source TGZ `c75d1978…b8d1` | 0 | 2026-11-01 | c5e114f9159a1b5fcdf074123c72a0bb653dab84 |
| package-linux | aarch64 host + container | release | `bash tools/ci_presets.sh release package` | Not run — deferred to the linux wave; not qualified | — | — | — |
| offline-rebuild-linux | fresh aarch64 container, network disabled | release | `bash tools/ci_presets.sh release offline` | Not run — linux-only lane, deferred; not qualified | — | — | — |
| macos-manual-gui | user's real macOS display | gui | `cmake --workflow --preset gui`; complete [viewer acceptance](../development/viewer-platform-acceptance.md) | User acceptance pending | — | — | — |
| linux-xvfb-smoke | aarch64 container + Xvfb | gui | `bash tools/ci_presets.sh gui gui-smoke` | Not run; smoke only — deferred to the linux wave | — | — | — |

For direct Linux invocation set a unique `BIKE_LINUX_CONTAINER` first.
CI sets a name from run ID, attempt and lane. No old container or worktree
is deleted. Build and analysis evidence is under `output/ci/<lane>/`;
CTest's `Testing/`, the check summaries/logs, compiler context, coverage
reports and CPack outputs are uploaded with the existing SHA-pinned
artifact action. Trusted self-hosted package/offline/RTSan lanes remain
dispatch-only behind the `release-qualification` environment.

## Explicit additional gates

- Run `ctest --test-dir build/native-release -L slow --no-tests=error --output-on-failure`
  on both actual performance hosts; preserve the measurements and machine identity.
  2026-11-01 darwin-arm64 leg (Mac16,5 M4 Max, c5e114f9): exit 8, 47/49 —
  the two reds are the documented `reference.golden_episode_tool` 3.03
  fixture debt and `reference.native_pneumatic_tire`
  `observations_declare_multi_contact` ~8e-7 seal-path parity gap
  (lane/h2-documented). The seven `planned` ledger exceptions
  (`ledger-reconciliation-2026-10-31.md` §7) stay `planned` by ruling —
  see the 4.03 report for the per-record runtime state. Linux leg unrun.
- Run the same final source against both platform golden captures, with
  every node/entry accounted for by the test-port and removal ledgers.
- Run the completeness and documentation audits at the Phase 4 points
  defined by the reconciled completion plan. Ordinary preset tests exclude
  `audit`; a passing preset therefore does not establish these gates.
- Complete `install_smoke` and `package_smoke`, then the fresh offline
  rebuild. Neither a source archive nor a successful packaging command
  alone establishes a usable distribution.

## Not qualified or advertised

Linux x86_64, macOS x86_64, Linux desktop GUI acceptance, Windows, musl,
cross-compilation and native Wayland are outside the required lanes.
RTSan has an explicit preset and clean/negative controls. Where the real
capability probe fails, record the compiler and diagnostic as unsupported;
do not replace it with a successful unsanitized job. The Clang 18 Linux
release lane is not an RTSan qualification.

`docs/development/toolchain-matrix.json` is retained as historical metadata.
The transition Python qualification code that once consumed it was deleted
at T63, so nothing machine-readable reads it now; its appended Phase 3
qualification section still identifies this checklist as authoritative
and marks the historical x86_64 lanes not qualified.
