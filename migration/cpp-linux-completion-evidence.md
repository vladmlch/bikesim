# C++ + Linux completion: evidence log

Append-only evidence for
docs/superpowers/plans/2026-10-20-cpp-linux-completion.md.
Historical measurements in the plan are not measurements of this patch.
Executed gates must record the source revision, host/image identity,
commands, exit codes, test counts and failing node IDs.

## 2026-10-21 — source-only Phase 1 implementation session

Input: repomix-output.xml(20261021-145241).zip.
Input SHA-256: 8226e739c82198a9ce9bdf34e1b3e4581c0e5bb34deac1ca1272dfc78738e0b9.
The Repomix-unmixer skill
restored 1,992 text files into an isolated working copy. Original extracted
files were retained separately for patch generation. Git objects, branch
refs, omitted binaries and dependency environments are not part of this
snapshot; its Git HEAD cannot be established from the archive.

The current user instruction prohibits compilation and execution.
Accordingly no project configure, build, test, static-analysis sweep,
interpreter import, container start, numerical probe, simulation, Git
merge or push is executed in this session. Filesystem extraction,
source inspection, editing and deliverable packaging are the evidence.
No L1/L2/L3 pass or current-platform qualification is claimed.

### Implementation work allocation

- Tasks 1.01–1.05: existing-fix inspection, Linux lane, migration policy,
  source-supported finding review and honest evidence bookkeeping.
- Tasks 1.06–1.09: checker schema/platform/tool/control/child-configure
  parity, plus the depth-2 script sweep and provider-binding regressions.
- Tasks 1.10–1.12, 1.17–1.18: numerics probe/face/callers, binary64
  feasibility, and measurement preparation for non-finite trigonometry.
- Tasks 1.13–1.16: recursive Value ownership, Linux source branches,
  explicit lock types and GCC source compatibility.
- Task 1.19 and checkpoints: static integration review here; runtime
  verification remains pending under the current user restriction.

The existing detailed plan is the implementation specification; its
gate-dependent checkboxes and historical plans remain unchanged.

### Task 1.01 — inherited fixes

The supplied BikeLinkage.cmake already guards optional ARGV1 with ARGC,
then falls back to _native_mujoco_library for one-argument callers.
The supplied io/value.hpp already includes <algorithm>.
These are input facts, not new fixes or newly verified build results.

### Task 1.03 — missing merge input

The archive has only ADR-0001, no ADR-0002, no tools/research/ payload
from feat/drop-oracle-parity and no Git objects for either branch.
No substitute merge, invented 22-file manifest or fabricated commit is
created. Task 1.03 remains pending in the real checkout. The known D12
transition is recorded directly in ADR-0003 and the active contracts;
ADR-0002's original text/status must be reconciled when its actual merge
lands, retaining ADR-0003's precedence.

### Task 1.05 — source-supported fixes, verification still open

The snapshot's PhysicalStep now calls cruise.compute() for the admitted
ideal-speed mode (F2), adds resistance from refreshed engine contacts
when no tire writer exists (F3), and passes the configured recovery
switch to the rider controller (F5).
The corresponding Python physical_runtime.py branches and test_t23.cpp
regressions support the intended changes by source inspection.
The completion plan attributes those corrections to 89cfcb1; that Git
object is not available in this archive.

No regression was run on either graph. The 73 python-removal rows and
four physics-research task records therefore retain their existing
implemented_tested_findings_open statuses and findings_open arrays.
Nothing is promoted to tested, verified, qualified or closed.

### Task 1.02 — acquisition provenance, not runtime qualification

The Dockerfile records metadata fetched read-only on 2026-10-21:

- Ubuntu 24.04 index digest:
  sha256:68ee22004a9109b44973313b02ff7b1de250f1de0f5ecb628d73f579aaff33ac.
  Source: https://hub.docker.com/layers/library/ubuntu/24.04/images/sha256-7f622ca8766bccb22f04242ecb6f19f770b2f08827dc4b8c707de5e78a6da7ab
  (the current page reports the index digest above).
- uv 0.13.1 aarch64 GNU/Linux archive SHA-256:
  9041c73c99fdc5892cf128140beb354eca9231d502068cc36e6cb53a8e59ad14.
  Source: https://releases.astral.sh/github/uv/releases/download/0.13.1/uv-aarch64-unknown-linux-gnu.tar.gz.sha256

No image or executable archive was run to obtain these values. The image
was not built or inspected with Docker. Its package versions, Python
runtime and numerical provider still require the planned lane checks.

### Source implementation review — Tasks 1.02, 1.04, 1.06–1.19

The following is a source-review status table. "Implemented" describes
the edited source and regression cases; it does not mean a task's
execution gate passed.

| Task | Source result | Outstanding requirement |
|---|---|---|
| 1.01 | Both fixes were already present in the supplied snapshot. | Rebuild the affected graphs when execution is authorized. |
| 1.02 | Added the pinned Linux image recipe and checkout-owned container driver; both script sweeps select depth 2 and reject an empty selection. | Build/provision the image, record actual versions and qualify the lane. |
| 1.03 | No merge was attempted; required branch objects, ADR-0002 and research payload are absent. | Perform the specified merge from the real branch and reconcile its actual files. |
| 1.04 | Added ADR-0003 and updated AGENTS.md, native engineering rules and CONTEXT.md to the agreed per-file golden transition. | Amend the actual ADR-0002 when Task 1.03 is possible. |
| 1.05 | Reviewed the existing F2/F3/F5 source fixes and regression definitions. | Run the required regressions on both graphs before changing any open finding/status. |
| 1.06 | Added schema-2 platform/SDK/sysroot/stdlib/policy/capability validation and context-only CLI validation. | Run T56 and the affected sweeps on the supported platforms. |
| 1.07 | Added optional sysroot validation/forwarding and the reference tool candidate chain, preserving executable symlink spelling. | Run tool/sysroot regression cases and sweeps. |
| 1.08 | Added shared platform control dispatch/listing and the five Linux context controls; Darwin-only controls remain gated. | Execute each platform's control set. |
| 1.09 | Child configure uses CMake directly and only the recorded binding interpreter; added Linux standard-library hardening controls. | Run the binding-context L1 command and hardened/unhardened controls. |
| 1.10 | Probe, record validators, CMake projection and fixtures now require eight operations with exact spellings and separate ABI classes. | Regenerate real Darwin and Linux lock entries; the generated lock is intentionally unchanged here. |
| 1.11 | Added typed raw ILP64 dot, unconjugated complex dot, dgesv and dsyevd adapters and analytic contract cases. | Measure the raw-dot zero bits and execute adapter/provider contracts. |
| 1.12 | Replaced all five direct caller bindings with the typed faces; added structural provider-boundary checks and necessary CMake usage requirements. | Build/link both graphs and run the affected bitwise and structural regressions. |
| 1.13 | Applied the plan's primary ObjectList solution and added aggregate/ownership/copy/move/lexeme guards. | Confirm the diagnosed incomplete-pair chain is resolved on both Linux compilers and preserve macOS behavior; no boxing fallback was selected without that evidence. |
| 1.14 | Corrected the glibc spawn-chdir gate, bounded executable-path construction and spawn lock type. | Compile the platform objects and run T06. |
| 1.15 | Supplied explicit mutex template arguments at 37 lock declarations, retaining the recursive mutex where required. | Run the affected compile/sweep/runtime gates. |
| 1.16 | Made the tyre-tier table constexpr with string_view names, removed the duplicate research-startup default argument and used a stateless FILE deleter. | Compile both graphs and run the covering tests. |
| 1.17 | Changed interval storage/arithmetic to binary64 while retaining every outward down/up/nextafter step; added boundary and one-ULP guards. | Run feasibility/native regression gates on both platforms. |
| 1.18 | Added analytic native sin/cos contracts, a native/libm/subtraction bit-table mode and scalar/contiguous-array/native NumPy comparison support. | Measure both platforms, freeze the actual tables, and only then decide the Linux nonfinite branch. The existing branch is retained. |
| 1.19 | Reviewed source integration, new interfaces, callers, CMake edges and regression fixtures. | Discover and resolve remaining compiler/runtime findings through the specified Linux/macOS gates. |

No task/checkpoint checkbox in the completion plan was marked complete.
Checkpoints 1-A/1-B and Phase 1's final acceptance remain pending. Linux
root-graph/C++ checker execution still belongs to Phase 2; Phase 1's
Linux binding sweeps use the Python commands specified by Task 1.19.

#### Integration decisions and independent review

- The numerics interface keeps ordinary `blas::ddot`'s existing `0. +`
  seed and Darwin legacy symbol. `blas::ddot_raw` forwards the original
  unseeded ILP64 fold; its platform-specific zero sign was not guessed.
- Probe/discovery distinguishes legacy BLAS, always-ILP64 BLAS and
  LAPACK. New LAPACK and dot bindings must belong to the corresponding
  NumPy import set as well as the provider's export set.
- Geometry's inline functions require `Bike::Numerics` as a PUBLIC
  dependency in both graphs. Model, kinematics and filter callers carry
  explicit adapter edges. The adapter exports the MuJoCo include
  requirement of its public CBLAS header; generated provider names stay
  private to the backend translation units.
- Checker validation/control code was compared directly with
  `tools/native_checks.py` and `tools/native_toolchain.py`. The fixtures
  use the actual Python messages where the plan's prose was stale:
  `missing configured sysroot` and `invalid <kind> override`.
- The standard-library control is deliberately stricter than the current
  Python implementation in one case: an unhardened probe that does not
  compile fails the C++ sweep. A failed compile cannot establish that
  hardening flags did not leak into the negative control.
- The contracts command's default repository-root traversal was one
  parent short. It now reaches the repository root; a T56 case covers
  invocation without `--repo-root`.
- Independent source reviews covered checker fixtures/CLI, typed
  numerical ABI signatures/callers/CMake dependencies, Value ownership
  and Linux scripts. No new ABI-width, argument-order or fixture blocker
  was identified by those reviews. This is not compilation evidence.
- The driver stages source selection/archive creation before touching
  container sources, checks checkout ownership, excludes host dependency
  and build directories, and rejects symlink parents. It retains Ninja's
  failure status and does not publish a zero count for an unreadable log.
  Rebuilding an image does not replace an existing container: an explicit
  `down`/`sync`/`venv` cycle is documented.

#### Added regression coverage (not executed)

- T56: schema-2 valid and malformed contexts, platform controls/CLI modes,
  Linux sysroot rejection, override precedence, provider binding boundary,
  script depth/empty selection, and default repository-root resolution.
- T05: aggregate Value shape, insertion order, large integer and signed
  zero lexemes, object/array deep copies, copy/move assignments,
  moved-from reassignment, range/base-vector construction, null vs object.
- T28: a one-ULP contradictory interval cannot yield a feasible witness,
  and a boundary-feasible interval cannot be certified empty.
- Platform BLAS/LAPACK: raw/seeded dot separation, strides, unconjugated
  complex products, linear solve/singularity and eigenvalue workspace/
  eigenvector contracts.
- Dependency portability: all eight operation records, import
  classification/membership, exact-name rejection and platform entries.
- Nonfinite trigonometry: native analytic contracts and explicit measured
  table output, plus NumPy scalar/contiguous-array/native agreement.

#### Local suppression records

These entries record source-level intent. No checker was executed to
claim a new observed diagnostic in this session.

| Checker / location | Origin, reason and reproducer | Removal condition |
|---|---|---|
| clang-tidy `modernize-use-equals-default`, `native/src/io/value.hpp` | Task 1.13's recorded libstdc++ 13 chain: defaulted constexpr destruction reaches incomplete `pair<string, Value>`. Reproducer is the plan's isolated `io/json.cpp` object compile. Non-constexpr out-of-line bodies are the prescribed fix. | Qualified Linux compilers accept the defaulted form and the Value/config/JSON copy/move guards pass. |
| clang-tidy `misc-redundant-expression`, `native/tests/test_platform_sincos.cpp` | Task 1.18 explicitly requires measuring `x-x` as a candidate nonfinite rule. The source expression in `dump_nonfinite` is intentional; its four inputs are the two infinities and signed quiet NaNs. | The candidate measurement is no longer required or the checker recognizes this intentional probe without an exception. |
| Clang `-Wunsafe-buffer-usage-in-container`, `native/tests/test_platform_sincos.cpp` | The CLI constructs a bounded span from C/C++ `main`'s `argv` and `argc`, then inspects only admitted indices. Reproducer is that single construction under the project's warning set; the local pragma matches the existing test CLI pattern. | The supported compiler accepts the bounded entry-point adapter, or an equally bounded adapter can express it without a local diagnostic exception. |
| ShellCheck `SC2016`, `tools/linux/container.sh` | The inner Bash snippets intentionally defer variables/positionals to the container process. The annotated sync/build/failures command strings are the reproducer; host interpolation would alter their meaning. | The snippets no longer cross a shell boundary, or ShellCheck can recognize the deferred interpretation. |

#### Qualification handoff — commands below were not executed

First obtain the missing branch payload for Task 1.03 and provision the
two exact dependency environments. The extended validator rejects the
old four-operation record even on Darwin. Do not add symbol names or
Linux provider facts by hand and do not configure against that stale
lock expecting a qualified build.

In the real macOS checkout, with its `.venv` and the Linux container
already provisioned, regenerate the record through the actual probe:

~~~sh
numerics_backup_dir="$(mktemp -d "${TMPDIR:-/tmp}/bike-numerics-lock.XXXXXXXX")"
mv native/numerics.lock.json "$numerics_backup_dir/numerics.lock.json"
uv run --frozen --group native python tools/native_dependencies.py \
  --python .venv/bin/python3 \
  --write-numerics-record native/numerics.lock.json
tools/linux/container.sh sync
tools/linux/container.sh exec uv run --frozen --group native python \
  tools/native_dependencies.py \
  --python /opt/bike-venv/bin/python3 \
  --write-numerics-record native/numerics.lock.json
tools/linux/container.sh pull native/numerics.lock.json
~~~

Moving the old lock aside is necessary because the writer validates an
existing file before preserving other platforms' entries. The first
successful probe writes the new Darwin record; the Linux probe then
preserves it while appending its measured record. Retain the backup
until both new records are reviewed.

After the affected graph can build, record raw-dot bits and the two
platforms' scalar/array/native/libm/subtraction sin/cos tables. The new
`native_platform_sincos_tests --dump-nonfinite` mode and
`test_sin_cos_nonfinite_scalar_array_agreement` reference test provide
measurement output; neither is a frozen per-platform acceptance table.
Then follow the existing plan's focused regressions, Checkpoint 1-A,
Checkpoint 1-B and final Phase 1 gates. Record exact source/image
identities, commands, exits and failing cases before closing findings.

## Phase 2 source implementation — 2026-10-21 (execution prohibited)

Source: the user-supplied Repomix ZIP, SHA-256
`db0b8be6eac8a6e7abea6d4a000f11023532af770ec19d5cb9426d83a14f7ddb`.
The archive restored 1,997 text files and included the Phase-1 source
work and its open prerequisites recorded above. It contains no usable
Git branch history for the prescribed merge/commit/push sequence.

The user explicitly required that nothing be compiled or run. No
project configure, build, CTest, pytest, checker, ShellCheck, container,
application, viewer or numerical probe was executed. Filesystem edits,
archive extraction, byte/hash inspection and source comparisons are the
only verification evidence from this session. No runtime exit codes,
test counts or platform qualification are added.

### Source coverage

| Tasks | Source result | Open evidence |
|---|---|---|
| 2.01–2.02 | Shared dependency/platform/compiler-policy layers now drive the root graph. Root consumers use resolved shared-library paths; the policy is applied directly. | Darwin provenance/load-name parity and effective compiler-flag comparison. |
| 2.03a–2.03b | One schema-2 emitter serves both graphs. Root-only metadata and null Python fields have strict validation. The two unused legacy modules are deleted. | T56, binding-context parity, sweeps and regenerated build identities. |
| 2.04–2.05 | Schema-2 platform selection, locked Linux aarch64 artifacts, explicit acquisition and a narrow Docker acquire stage. Container initialization preserves existing dependency trees. | Actual acquisition/image provisioning and platform configure gates. |
| 2.06–2.07 | Linux RECORD_ONLY maps measured records to locked providers with glibc/hash/containment checks. Root provider declarations avoid Darwin-only probes. Linux T03 loader checks and root integration are prepared. | Real record regeneration and the first Linux root configure/build/CTest. |
| 2.08 | Relative installed rpaths, installed-only Darwin load-name/signature handling, Linux provider/runtime-library installation, moved-prefix smoke and strict root installed-provider identity. | Both-platform install smoke, T03 partial installs and native runtime verification. |
| 2.09–2.10 | Complete GLFW tree and X11 selection; native-only `--close-on-finish` for physical/research viewers; T47 definitions and bounded Xvfb script. | GUI builds and smoke. Desktop viewer acceptance remains separate. |
| 2.11 | The existing common gate runner is exposed through the container with explicit Clang/environment selection and wrong-compiler cache refusal. | L3 Linux, actual counts/failing IDs, qualification and repository publication. |

The detailed decisions, source-level compiler-policy comparison,
regression inventory and remaining requirements are in
[`cpp-linux-phase2-source-review.md`](cpp-linux-phase2-source-review.md).

### Dependency byte evidence

All eight selected Linux archives and all 17 selected required-file
hashes were checked against their bytes. Exact source URLs and identities
are committed in `native/dependencies.lock.json`; acquisition details are
in `native/dependencies.md`. Original Darwin pins are preserved. The
GLFW archive hash is unchanged; its top-level and `src/` CMake files are
now pinned and the whole source tree is retained.

Static ELF inspection established that the selected OpenBLAS image
requires its wheel's renamed libgfortran companion. Both are pinned and
installed together; neither was executed. Archive inspection also led to
the RapidJSON `license.txt` to `LICENSE` mapping and deterministic DuckDB
MIT-license extraction. These byte checks do not establish ABI or runtime
qualification. The acquisition script and Dockerfile were not run.

### Review decisions and preservation

- The actual context validators require only a numerics object and a
  nonempty backend. No additional nullable numerics fields or invented
  probe evidence were needed. Binding validation remains strict.
- The root migration numerics helper now uses the shared target on every
  platform; it no longer adds a raw Darwin framework edge on Linux.
- Installed-only Darwin signature repair is defined defensively after
  verification; no launch failure or measured signing requirement is
  claimed. Full `@rpath` names use one CMake substitution field.
- Linux installed-provider fallback is root-only, uses exact adjacent
  files and recorded hashes, and then checks real loaded-image membership.
  It addresses the identity API rather than asserting an unobserved CLI
  failure. Binding and configure-probe identity rules are unchanged.
- Both actual viewer launch consumers receive the new option. Research
  retains its runtime-owned duration. Nonzero GUI diagnostic and timeout
  statuses are propagated without an unmeasured success exception.
- Stale Linux-refusal numerics tests and Darwin-only root-test host guards
  were updated. The lane explicitly includes Make for generator-unspecified
  CMake fixtures as well as Ninja. All regression definitions are unrun.
- `native/numerics.lock.json`, `migration/cli-contracts.json`, the original
  completion plan, captured oracle/source manifests, historical baselines
  and viewer acceptance records retain their supplied content.

The numerics lock still contains the old four-operation Darwin record
and no Linux entry. Its real Phase-1 regeneration is a prerequisite;
it was not hand-edited to make an unrun configure appear acceptable.
The missing Task 1.03 branch payload remains an inherited prerequisite.

No task/checkpoint checkbox was changed. **Checkpoint 2-A and L3 Linux
(end of Phase 2) were not run and remain open.** No commit, push or new
qualified lane is claimed. The source delivery is ready for review and
the existing plan's execution gates when execution is authorized.

## 2026-10-21 — Task 1.03 source merge from supplied snapshots

The user supplied both `orig.xml.zip` and `drop-oracle-parity.xml.zip` and
requested the merge required by the completion plan. Input ZIP SHA-256:

- orig: `d76f46a7745130aa8d3ad8c9c56b94acbaeecdf86b4657d4a531a05833f22037`.
- drop-oracle-parity: `d30493e3fbe353a16c6256a65346b12073e0470a317283934dd3dfefe0ea9092`.

Repomix-unmixer restored 2,004 and 1,010 text files respectively. The
source merge imports the 144 donor-only docs/research-tool files and
recovers one expected JSON from the donor's preserved audit output.
It reconciles ADR-0002 with the already adopted ADR-0003, retaining the
original Phase-1/Phase-2 implementation and every original path.

This resolves the missing text-payload portion of Task 1.03 recorded
above. It does not establish Git ancestry or a real branch merge commit:
the supplied snapshots contain no branch objects. Earlier missing-input
entries remain historical, and no checkpoint or task gate is marked green.

All original native code, root/binding build graphs, dependency locks,
Python implementation/tests, active agent contracts, oracle manifests,
and historical plans retain their input bytes. The donor's older source
and build definitions do not overwrite orig. `research/fp32-physics` is
outside this merge. Historical r2 policy text is subordinate to ADR-0003:
live parity stays a per-file gate until both-platform capture and the
C++ port land; Python is removed at T63.

The nine compressed alpine snapshots and lateral source images/PDFs are
not present in the supplied text archives. The expected statistics JSON
is recovered from existing input bytes, not recalculated. Its generated
output ignore rule is anchored so the expected reference is trackable.
One overlong report filename is shortened without altering its text.

Only source inspection, filesystem extraction/copying, hashing and patch
application to temporary source copies are performed. No project code,
configure, build, test, checker, container, numerical probe or application
is executed. No L1/L2/L3 result or platform qualification is claimed.
The source review, provenance, availability limits and application notes
are in [cpp-linux-task103-source-merge.md](cpp-linux-task103-source-merge.md);
the exact imported-path/hash inventory is in
[cpp-linux-task103-merge-manifest.json](cpp-linux-task103-merge-manifest.json).

## Task 1.03 — Python files added by the r2 merge (OD-1)

Exactly 22 new files under `tools/research/`, copied unchanged from the
supplied donor snapshot: 17 OSM/DEM scripts, three lateral-digitisation
scripts and two adjudication scripts. This is the real input inventory,
not a reconstructed list from the plan. OD-1 remains pending until the
Task-3.02 triage; this merge does not delete or relocate these files.

- `tools/research/adjudication/audit_golden_cases.py`
- `tools/research/adjudication/gemini_golden_cases.py`
- `tools/research/alpine_osm_dem/analyze_profiles.py`
- `tools/research/alpine_osm_dem/bandfrac.py`
- `tools/research/alpine_osm_dem/compare_stats.py`
- `tools/research/alpine_osm_dem/dem_resolution_check.py`
- `tools/research/alpine_osm_dem/dense_check.py`
- `tools/research/alpine_osm_dem/descfrac.py`
- `tools/research/alpine_osm_dem/download_tiles.py`
- `tools/research/alpine_osm_dem/fetch_dem_index.py`
- `tools/research/alpine_osm_dem/fetch_osm_paths.py`
- `tools/research/alpine_osm_dem/noise_check.py`
- `tools/research/alpine_osm_dem/prepare.py`
- `tools/research/alpine_osm_dem/scree_analyze.py`
- `tools/research/alpine_osm_dem/scree_collect.py`
- `tools/research/alpine_osm_dem/scree_elev.py`
- `tools/research/alpine_osm_dem/scree_stats.py`
- `tools/research/alpine_osm_dem/scree_tiles.py`
- `tools/research/alpine_osm_dem/stats2.py`
- `tools/research/lateral_digitise/analyse.py`
- `tools/research/lateral_digitise/digitise.py`
- `tools/research/lateral_digitise/finalise.py`
