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

## 2026-10-21 — Task 1.03 actual-checkout integration

Starting revision: `fc848de2d0d16389725b12337983bdf844e7e65c`.
Merged branch: `feat/drop-oracle-parity` at
`64c0a50043fd87a695e61a402572a63ee44fcfc4`.

The previously imported text files are now reconciled with the actual Git
branch. `git diff --name-status HEAD...feat/drop-oracle-parity -- native/
CMakeLists.txt` exited 0 with no changes. Initial `git merge-tree
--write-tree --name-only` exited 1 because the snapshot imports already
exist as separate additions. Repeating with `-Xignore-space-at-eol` reduced
the unresolved set to nine documentation/policy files. The actual merge
used `git merge --no-ff --no-commit -Xignore-space-at-eol
feat/drop-oracle-parity` (exit 1 pending those resolutions).

Conflict resolution preserves the current ADR-0003 transition, portable
reference-source lookup, native engineering rules, testing documentation,
anchored generated-output ignore rule and lateral-source availability
note. It restores all nine compressed alpine datasets from the branch.
Each restored file's `git hash-object` equals the branch's `git rev-parse
<branch>:<path>`; `gzip -t` exits 0 for all nine. The additional tyre/terrain
research report equals the user's pre-existing untracked file byte for
byte (blob `a0264f50f9d177903446891095b1538da84af456`); that file is now
tracked. The geomorphology report already exists under its shortened
portable name with identical text, so the duplicate 303-byte basename is
excluded. No existing container or worktree is removed.

Checks before the merge commit:

- no unresolved index entries;
- `git diff --cached --quiet HEAD -- native/ CMakeLists.txt`: exit 0;
- `git diff --cached --quiet HEAD -- docs/superpowers`: exit 0;
- nine blob comparisons and nine archive-integrity checks: exit 0;
- comparison with the preserved untracked report: exit 0;
- `git diff --check` and `git diff --cached --check`: exit 0.

No project build, pytest/CTest suite, research calculation or L1/L2/L3
gate is run for this source/history integration. Task 1.03's build/checker
verification and the phase checkpoints remain open. The 22 research Python
files and OD-1 are unchanged; no extra Python file is introduced here.

## 2026-10-21 — Task 1.10 measured numerics-record regeneration

Source revision: `cb057ef` (the actual Task-1.03 merge).
Generator `tools/native_dependencies.py` SHA-256:
`0204c0543ba6213224b7d84766e7d50e593c1cb665a3a5dcf97f22e69a6ff226`.
The user's follow-up request to perform items 1 and 2 authorized the
numerics probes needed for regeneration. No simulation, project test
suite, CMake configure or full gate was run.

### Measurement identities and outcomes

| Fact | macOS arm64 | Linux aarch64 |
|---|---|---|
| Interpreter | CPython 3.13.12, existing checkout `.venv` | CPython 3.13.16, image-owned `/opt/bike-venv` |
| NumPy / MuJoCo | 2.5.2 / 3.12.0 | 2.5.2 / 3.12.0 |
| Provider | Accelerate; macOS build 26A434, SDK build 26A425 | NumPy scipy-openblas ILP64; glibc 2.39 |
| ABI-control compiler | Apple clang 21.0.0 (clang-2100.3.34.2) | Ubuntu GCC 13.3.0 |
| ABI compile and link/run | pass / pass | pass / pass |
| NumPy interpolation | contracted, 1511 distinguishing queries / 4096 | separate, 1470 / 4096 |
| Broadcast NaN winner | scalar, 64 distinguishing lanes | element, 64 distinguishing lanes |
| SciPy interposition | checked, stable (SciPy 1.17.0) | checked, stable (SciPy 1.17.0) |

Both ABI controls reported:

```text
BIKE-ABI-OK ddot=32 dgemv=[10 26 42] dgemm=[4 5;10 11] ddot_ilp64=32 zdotu_sub=[-18 92] dgesv=[1 2] dsyevd=[1 3] dgelsd_rank=1 x=[0.6 1.2]
```

Docker Desktop was initially stopped. `open -a Docker` exited 1 (application
not found); opening the outer and inner application paths also exited 1
with `kLSNoExecutableErr`. `docker desktop start` subsequently exited 0
and the daemon became available. No installation/settings repair was made.
An existing image was reused for this measurement only:
`bike-linux-image:port`, image ID
`sha256:959091e9b453461be335eb31edadc63e36269b827755994f6b54d3796703ca69`,
architecture `arm64`, OS `linux`. A new container
`bike-dev-python-free-cpp-numerics` has no bind mounts; only the current
generator and generated macOS record were copied into it. Existing
containers and worktrees were retained. This is not qualification of the
new `tools/linux/Dockerfile` recipe.

### Commands and record checks

The first macOS attempt with `--python .venv/bin/python3` exited 1:
`selected interpreter must be an absolute path`. The successful commands
use the absolute interpreter. An initial container invocation with bare
`uv` exited 127 because image PATH lacks its directory; subsequent commands
use `/root/.local/bin/uv`.

All commands in this block exited 0:

```sh
uv run --frozen --no-sync --group native python tools/native_dependencies.py --python "$PWD/.venv/bin/python3" --output /private/tmp/bike-numerics-darwin-measured.json
uv run --frozen --no-sync --group native python tools/native_dependencies.py --python "$PWD/.venv/bin/python3" --write-numerics-record /private/tmp/bike-numerics-regenerated.lock.json
docker exec bike-dev-python-free-cpp-numerics /root/.local/bin/uv run --no-project --no-sync --python /opt/bike-venv/bin/python3 python tools/native_dependencies.py --python /opt/bike-venv/bin/python3 --output /src/linux-numerics-probe.json
docker exec bike-dev-python-free-cpp-numerics /root/.local/bin/uv run --no-project --python /opt/bike-venv/bin/python3 python tools/native_dependencies.py --python /opt/bike-venv/bin/python3 --write-numerics-record /src/native/numerics.lock.json
uv run --frozen --no-sync --group native python tools/native_dependencies.py --check-numerics-record native/numerics.lock.json --probe-document /private/tmp/bike-numerics-darwin-measured.json
docker exec bike-dev-python-free-cpp-numerics /root/.local/bin/uv run --no-project --python /opt/bike-venv/bin/python3 python tools/native_dependencies.py --check-numerics-record native/numerics.lock.json --probe-document /src/linux-numerics-probe.json
```

The existing four-operation lock was rejected against the real macOS probe
(exit 1, expected): `numerics.symbols must map exactly ('ddot', 'dgemv',
'dgemm', 'dgelsd', 'dgesv', 'dsyevd', 'ddot_ilp64', 'zdotu_sub')`.
The regenerated combined lock passes the same check, printing
`darwin-arm64` on macOS and `linux-aarch64` in the container.

The tool generated a fresh macOS file at the temporary path, then the
Linux writer preserved that entry and appended its measured record. The
combined generated file was copied back without hand-editing. JSON
comparison proves every pre-existing Darwin fact is unchanged after
removing only the four newly added symbol fields. The Linux record has
eight symbols, BLAS/LAPACK widths 64/64 and exactly the planned provider
SHA-256 `5dafca6786e1e366f29ff5e57a6b3beb66a4a02cadf899f51b38fa9fb22c5798`.

Generated `native/numerics.lock.json` SHA-256:
`19f542e9a581ef5b47379ab1420b6104f5bac72cf270860421c5ae5acd0539bc`.
Full machine-specific probe documents remain in ignored local evidence:

- `.work/evidence/numerics-2026-10-21/darwin-arm64.json`, SHA-256
  `b2124b4e70d181ed3499219d9e34f4e212e8d337be9fc5cd49c7f942f57ee517`;
- `.work/evidence/numerics-2026-10-21/linux-aarch64.json`, SHA-256
  `7b7913529dd7409012f9a1cd1291094e02b900a5b6d453c0eedf0943454ea358`.

Task 1.10's record-generation requirement is fulfilled. Its portability
pytest/build/configure checks, Task 1.03's L1 checks and all phase
checkpoints remain open; no task checkbox or qualification status is
advanced by these successful numerics controls alone.

## 2026-10-21 — Task 1.18 measured nonfinite sin/cos parity

Source revision: `36b0b0af5f386fa509179c6520d204f5fde89aad`, branch
`impl/python-free-cpp`. This is the explicitly authorized focused
measurement/regression slice for review item 3. No full project build,
native/full profile, simulation episode or phase checkpoint was run.

### Identities and measured decision

| Identity | macOS arm64 | Linux aarch64 |
|---|---|---|
| Host | macOS 27.0.1, build 26A434, Darwin 27.0.0 | Linux 7.0.14-linuxkit, aarch64 |
| Interpreter | CPython 3.13.12, checkout `.venv/bin/python3` | CPython 3.13.16, `/opt/bike-venv/bin/python3` |
| NumPy | 2.5.2, existing checkout venv | 2.5.2, image-owned venv |
| Native compiler | Apple Clang 21.0.0 (clang-2100.3.34.2) | Clang 18.1.3 (1ubuntu1); GCC 13.3.0-6ubuntu2~24.04.1 |
| Trig provider | libSystem.B.dylib 1359.0.0 | `/lib/aarch64-linux-gnu/libm.so.6`, Ubuntu glibc 2.39-0ubuntu8.9 |
| Focused linter | Homebrew clang-tidy/LLVM 23.1.2 | Ubuntu clang-tidy/LLVM 18.1.3 |

Reused running container `bike-dev-python-free-cpp-numerics`, image
`sha256:959091e9b453461be335eb31edadc63e36269b827755994f6b54d3796703ca69`.
Its `bike.checkout` label matches this worktree and `Mounts=[]`.
Only the probe, wrapper header, focused test and clang-tidy policy were
copied into `/tmp`; no host venv, bind mount, sync or image rebuild.

**Reproduced decision:** scalar and array NumPy results agree for all four
inputs and both operations on each platform (512 array output lanes per
platform). Linux libm already matches every oracle bit, so its existing
plain-libm rule stays unchanged. On macOS, libm sin preserves the negative
qNaN while libm cos canonicalizes it positive; the existing Apple
subtraction rule matches NumPy. No wrapper arithmetic, platform gate,
finite operation order or volatile anti-fusion load was changed.
No inference about other NaN payloads, signaling NaNs or other platforms
is made by this four-input measurement.

### Probe source and commands

The exact `/private/tmp/bike-sincos-probe.py` source (SHA-256
`1738b72ed70d45e53222d68c7d2a07cca5809973750b72b94432ebe9df83a8e1`):

```python
import platform
import struct
import sys

import numpy as np


def bits(value):
    return f"{struct.unpack('<Q', struct.pack('<d', value))[0]:016x}"


print(f"platform={platform.platform()} machine={platform.machine()}")
print(f"python={sys.version} executable={sys.executable}")
print(f"numpy={np.__version__} path={np.__file__}")
print("input scalar_sin scalar_cos array_sin array_cos lanes_equal")
for word in (0x7FF0000000000000, 0xFFF0000000000000,
             0x7FF8000000000000, 0xFFF8000000000000):
    value = struct.unpack('<d', struct.pack('<Q', word))[0]
    with np.errstate(invalid="ignore"):
        scalar = (np.sin(np.float64(value)), np.cos(np.float64(value)))
        array = (np.sin(np.full(64, value, dtype=np.float64)),
                 np.cos(np.full(64, value, dtype=np.float64)))
    equal = all(np.all(result.view(np.uint64) == result.view(np.uint64)[0])
                for result in array)
    print(f"{word:016x}", *(bits(item) for item in scalar),
          *(bits(result[0]) for result in array), equal)
```

These commands exited 0:

```sh
uv run --frozen --no-sync --group native python /private/tmp/bike-sincos-probe.py
docker inspect bike-dev-python-free-cpp-numerics --format '{{.Image}} {{.State.Status}} {{json .Mounts}} {{json .Config.Labels}}'
docker cp /private/tmp/bike-sincos-probe.py bike-dev-python-free-cpp-numerics:/tmp/bike-sincos-probe.py
docker exec bike-dev-python-free-cpp-numerics /root/.local/bin/uv run --no-project --python /opt/bike-venv/bin/python3 python /tmp/bike-sincos-probe.py
```

NumPy scalar/contiguous-array output on **both** measured platforms (each
`lanes_equal` covers all 64 lanes for both operations):

```text
input scalar_sin scalar_cos array_sin array_cos lanes_equal
7ff0000000000000 7ff8000000000000 7ff8000000000000 7ff8000000000000 7ff8000000000000 True
fff0000000000000 7ff8000000000000 7ff8000000000000 7ff8000000000000 7ff8000000000000 True
7ff8000000000000 7ff8000000000000 7ff8000000000000 7ff8000000000000 7ff8000000000000 True
fff8000000000000 fff8000000000000 fff8000000000000 fff8000000000000 fff8000000000000 True
```

macOS Apple Clang native dump (before and after the test/comment change):

```text
input native_sin native_cos libm_sin libm_cos subtract
7ff0000000000000 7ff8000000000000 7ff8000000000000 7ff8000000000000 7ff8000000000000 7ff8000000000000
fff0000000000000 7ff8000000000000 7ff8000000000000 7ff8000000000000 7ff8000000000000 7ff8000000000000
7ff8000000000000 7ff8000000000000 7ff8000000000000 7ff8000000000000 7ff8000000000000 7ff8000000000000
fff8000000000000 fff8000000000000 fff8000000000000 fff8000000000000 7ff8000000000000 fff8000000000000
```

Linux aarch64 Clang 18 native dump (before and after), and GCC 13 final
dump (identical):

```text
input native_sin native_cos libm_sin libm_cos subtract
7ff0000000000000 7ff8000000000000 7ff8000000000000 7ff8000000000000 7ff8000000000000 7ff8000000000000
fff0000000000000 7ff8000000000000 7ff8000000000000 7ff8000000000000 7ff8000000000000 7ff8000000000000
7ff8000000000000 7ff8000000000000 7ff8000000000000 7ff8000000000000 7ff8000000000000 7ff8000000000000
fff8000000000000 fff8000000000000 fff8000000000000 fff8000000000000 fff8000000000000 fff8000000000000
```

### Focused regression and negative control

The final standalone TU is compiled with C++23, `-O2`,
`-ffp-contract=off -Wall -Wextra -Wpedantic -Werror`, and the shared
required safety flags
`-fstack-protector-strong -ftrivial-auto-var-init=zero
-fvisibility=hidden -fvisibility-inlines-hidden -fstrict-flex-arrays=3`.
Apple Clang, Linux Clang 18 and Linux GCC 13 each compile and run with
exit 0 and no test output: **24 exact-bit assertions per binary** (eight
measured nonfinite results, four signed-zero controls, twelve nonzero
finite np/math-versus-separate-libm controls). Final strict commands and
all exit details are in
`.superpowers/sdd/2026-10-21-sincos-golden-pull/task-3-report.md`.

The temporary negative-control TU changes only the +infinity row's
expected sin from `7ff8000000000000` to `fff8000000000000` in both
platform tables. Apple Clang and Linux Clang 18 compile it with the same
strict flags (exit 0); both runs exit **1**, reporting:

```text
np_sin nonfinite input=0x7ff0000000000000 expected=0xfff8000000000000 actual=0x7ff8000000000000
```

The production tree always retains the correct table. The three correct
binaries were run again after the isolated negative control and exit 0.
Because the original Linux rule already matches NumPy, this is an
expectation negative control, not evidence of a production behavior fix.

Focused clang-tidy, one selected TU plus its first-party header, uses
`native/.clang-tidy` and the same strict flags. Both final runs exit 0
with zero first-party diagnostics. macOS prints 1496 generated warnings
and suppresses 1500 (1496 non-user, four existing NOLINT); Linux prints
and suppresses 56118 non-user warnings. Earlier local helper diagnostics
(one swappable-parameter finding, then ten designated-initializer
findings on macOS) were corrected and rechecked; no suppression was added.

Two transient sandbox access failures occurred: uv could not initialize
its user cache (exit 2), and a Docker identity query could not access the
socket (exit 1). Each was retried with approved sandbox escalation. An
initial sequential negative-control preparation batch continued after
uv's failure, producing missing-source compile (1), missing binary (127)
and missing Docker-copy source (1); its Linux compile/run used the prior
analytic TU and exited 0. Those results are **not** RED evidence. The
subsequent successful preparation and final isolated negative controls
above establish RED; no failure is hidden.

### Verification limits

The existing cached native Makefile/compile database has no sincos target
entry, so this task uses the expressly permitted standalone compilation.
`native/CMakeLists.txt` already registers the test in the binding graph;
it is unchanged. Root-graph registration remains upstream Task 3.02.
No CMake configure, full graph rebuild, CTest/pytest suite, orchestrated
`check_tidy` sweep, sanitizer, RTSan, coverage or native/full gate was
run. Direct one-TU clang-tidy is not a claim that the full sweep passed.
Review item 5 owns full-project/phase verification. No phase status or
plan checkbox advances here. `native/numerics.lock.json` is unchanged,
SHA-256 `19f542e9a581ef5b47379ab1420b6104f5bac72cf270860421c5ae5acd0539bc`.

## Review item 4 — safe golden directory pulls (2026-10-21)

Base `e78ccec`, branch `impl/python-free-cpp`. The shared capture procedure's
`tools/linux/container.sh pull tests/fixtures/golden/linux-aarch64` previously
failed because the driver required `test -f` and rejected directory destinations.
The focused wrapper regression reproduced the legitimate initial directory
rejection before implementation (`/bin/bash tools/linux/test_container_pull.sh`,
exit 1: `FAIL: legitimate golden directory rejected`).

Pull now accepts regular files and complete directory trees at the exact selected
checkout-relative path. Repeated directory pulls merge recursively and overwrite
matching files, retaining unrelated destination entries and siblings without an
extra basename level. Hidden files, names with spaces and executable file modes
survive. All operands transfer into host staging and validate before publication;
merged replacements retain originals until publication succeeds, with rollback on
publication failure. Exact duplicate operands remain accepted; parent/child
operands must be requested separately. File/directory conflicts are rejected.
The source and destination must remain unchanged during the pull: this is not a
crash-atomic snapshot. Disk space includes staged source, merged tree and original
backups. A rollback failure reports and retains the backup staging directory.

The source is checked before Docker transfer and the staged copy checked again:
source symlink parents, symlink/special entries, excluded environment/build
subtrees (including empty build directories) are rejected. Existing destination
counterparts and their parents must not be symlinks or incompatible types.
Unrelated destination symlinks are preserved by physical copying, never followed.
No container environment, physics, dependency version, numerics lock, golden
capture or simulator episode changed.

Final focused commands, all exit 0:

```bash
/bin/bash -n tools/linux/container.sh tools/linux/test_container_pull.sh
shellcheck tools/linux/container.sh tools/linux/test_container_pull.sh
/bin/bash tools/linux/test_container_pull.sh
git diff --check
/bin/bash -n /tmp/task-4-real-pull-smoke.sh
shellcheck /tmp/task-4-real-pull-smoke.sh
/bin/bash /tmp/task-4-real-pull-smoke.sh
```

Syntax/ShellCheck/diff checks emit no diagnostics. The committed shell regression
reports `PASS: 37 pull regression cases`: original directory behavior, recursive
and repeated merges, exact basename, hidden/spaced names, executable files,
duplicated file operands, empty directories, eight invalid path forms, six
excluded path examples, missing source, ownership, source root/parent/nested
symlinks, special source/destination entries, excluded subtrees including empty
build directories, destination root/parent/nested symlinks, partial copy failure,
post-copy validation, prevalidation of all operands, type conflicts, and two
publication rollback cases. The injected Docker copy exit 41 and publication exit
42 propagate unchanged while preserving original host data.

The real smoke script uses a unique `.pull-smoke-task4.XXXXXXXX` subtree under this
checkout and the same `/src/` subtree in the existing container. It exercises three
initial operands (directory, executable file, empty directory), repeats the
directory pull with an updated file, verifies local-only and sibling files,
then rejects missing source, source symlink and nested destination symlink while
checking existing data. Final output:

```text
container.sh: missing, symlink or special pull source: .pull-smoke-task4.LDxXLs0f/missing
container.sh: symlink or special pull entry: .pull-smoke-task4.LDxXLs0f/artifacts/link
container.sh: symlink destination: .pull-smoke-task4.LDxXLs0f/artifacts/nested dir
PASS: real Docker initial/repeat directory, file mode, empty directory, hidden/spaced entries, exact basename, sibling preservation; missing source, source symlink, nested destination symlink rejected
```

Both scratch subtrees were removed by that script. No image/up/sync was called.
Before and after smoke, `docker container inspect --format '{{.State.Status}}
{{.Image}} {{json .Mounts}} {{index .Config.Labels "bike.checkout"}}'
bike-dev-python-free-cpp-numerics` reports running, image
`sha256:959091e9b453461be335eb31edadc63e36269b827755994f6b54d3796703ca69`,
mounts `[]`, and ownership by this exact checkout. Host: Darwin arm64,
`/bin/bash` 3.2.57(1), ShellCheck 0.11.0. Docker client/server 29.8.2; container
Linux aarch64, Bash 5.2.21(1). No compiler was invoked. The first unprivileged
Docker inspect failed with socket permission denied (exit 1); the approved
escalated retry and all actual smoke transfers succeeded. An initial fixture
setup run also failed ownership because macOS temporary paths needed physical
canonicalization; it was corrected before the valid RED run and is not RED
evidence. Additional tests exposed an empty-build exclusion gap and duplicate
file-operand compatibility regression during implementation; both were fixed
before final verification.

Verification is deliberately limited to the focused shell checks, fixture and
actual transfers. Review item 5 owns project profiles/phase verification: no
native/full gate, broad sweep, CMake, build, CTest or pytest was run. No phase
checkbox advances. Detailed RED/GREEN and self-review are in
`.superpowers/sdd/2026-10-21-sincos-golden-pull/task-4-report.md`.

### 2026-10-21 final pull fix — protected names and filesystem aliases

Addressed both `final-review.md` Important/P1 findings (#1 protected exclusions,
#2 equivalent/ancestor operands and rename placement), based at
`e5ce714350ec41ce339ee735d4617fe834755946` on `impl/python-free-cpp`.
No numerical/native/physics behavior changed.

Pull-specific exclusion checks now fold ASCII case before applying the existing
exclusion policy, both in-container and to every staged tree entry. This is a
conservative policy on all hosts: `.VENV`, `.DEPS`, `BUILD*`, `.GIT` at the checkout
root, and corresponding previously excluded nested names cannot bypass checks.
Sync's existing matcher is unchanged. Ordinary non-ASCII/spaced names remain valid.
Private reservation directories under checkout-local staging use the actual
filesystem's name equivalence for absent as well as existing operand names.
Ancestor identity checks reject overlap in either order; existing host identity
checks also reject hardlink operands. Exact literal duplicates remain compatible.

Prepared and backup entries now retain the destination basename, and rename into
its parent. Thus an unexpectedly existing directory cannot turn `ready` into an
extra nested level. Publication rechecks destination/parent assumptions; rollback
tracks successful installs separately from attempted moves, preserving an
unexpected, unpublished destination while restoring earlier operands. All
reservation/staging directories are removed on ordinary success/rejection/failure;
rollback-failure retention and the documented quiescent-tree contract remain.

TDD RED commands, each exit 1 before production edits (full logs retained in /tmp):

```bash
PULL_TEST_FOCUS=excluded /bin/bash tools/linux/test_container_pull.sh
PULL_TEST_FOCUS=operands /bin/bash tools/linux/test_container_pull.sh
PULL_TEST_FOCUS=publication /bin/bash tools/linux/test_container_pull.sh
```

Each passed the original 37 cases, then respectively printed:

```text
FAIL: accepted unsafe/failed pull: .VENV
INFO: host case aliases share filesystem identity; alias cases execute
FAIL: accepted unsafe/failed pull: newdir NEWDIR
FAIL: accepted unsafe/failed pull: guard late
```

Logs: `/tmp/pull-fix-red-{excluded,operands,publication}.log`.
An intermediate combined syntax/ShellCheck/diff command returned 0 because the
last diff check succeeded, but ShellCheck emitted SC2034 for an unused loop
variable. That was not a clean static check. Replaced the loop with an arithmetic
counter; independently invoked ShellCheck subsequently exits 0 without diagnostics.
First GREEN: 56 cases; added normalization, successful-absent rollback and
publication-recheck coverage before final validation.

Final commands, each exit 0:

```bash
/bin/bash tools/linux/test_container_pull.sh > /tmp/pull-fix-green-final.log 2>&1
/bin/bash -n tools/linux/container.sh tools/linux/test_container_pull.sh
shellcheck tools/linux/container.sh tools/linux/test_container_pull.sh
git diff --check
/bin/bash -n /tmp/task-4-final-fix-smoke.sh
shellcheck /tmp/task-4-final-fix-smoke.sh
/bin/bash /tmp/task-4-final-fix-smoke.sh > /tmp/pull-fix-real-smoke.log 2>&1
```

Regression result: `PASS: 59 pull regression cases`, retaining all original 37.
The host reports case aliases and Unicode-normalization aliases share filesystem
identity, so all five conditional case groups and the normalization case execute;
there are no skipped cases on this lane. Other filesystems print explicit SKIP
messages for unsupported equivalences. Covers direct/nested protected variants,
absent/existing equivalent operands, existing/absent ancestor aliases in both
orders, hardlinks, aliases with publication failure armed, rollback after a new
operand was installed, destination invalidation before and after the publication
check, non-ASCII/spaced directories and no leftover staging directories.

Real Docker smoke reuses the existing owned numerics container and a unique
`.pull-smoke-task4.unCcXruR` subtree (removed in host/container on exit), and proves
actual initial/repeated merges, exact placement, file modes, hidden/spaced names,
empty directories and existing negative controls. It additionally creates Linux-
distinct `alias-dir`/`ALIAS-DIR`, rejects them together before macOS publication,
and preserves a disposable nested `.venv` sentinel against source `.VENV`.
Output (five expected rejection diagnostics, final exit 0):

```text
container.sh: missing, symlink or special pull source: .pull-smoke-task4.unCcXruR/missing
container.sh: symlink or special pull entry: .pull-smoke-task4.unCcXruR/artifacts/link
container.sh: symlink destination: .pull-smoke-task4.unCcXruR/artifacts/nested dir
container.sh: overlapping pull paths: .pull-smoke-task4.unCcXruR/alias-dir and .pull-smoke-task4.unCcXruR/ALIAS-DIR
container.sh: excluded pull entry: .pull-smoke-task4.unCcXruR/protected/.VENV
PASS: real Linux-distinct case operands rejected before macOS publication; nested protected case alias preserved
PASS: real Docker initial/repeat directory, file mode, empty directory, hidden/spaced entries, exact basename, sibling preservation; missing source, source symlink, nested destination symlink rejected
```

Before/after command (both exit 0 and identical output):

```bash
docker container inspect --format '{{.State.Status}} {{.Image}} {{json .Mounts}} {{index .Config.Labels "bike.checkout"}}' bike-dev-python-free-cpp-numerics
```

```text
running sha256:959091e9b453461be335eb31edadc63e36269b827755994f6b54d3796703ca69 [] /Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/python-free-cpp
```

Identity commands (`/bin/bash --version`, `shellcheck --version`, `uname -sm`,
`docker version --format 'Docker client={{.Client.Version}} server={{.Server.Version}}'`,
`docker exec bike-dev-python-free-cpp-numerics bash -c 'uname -sm; bash --version | head -1'`)
exit 0: host Darwin arm64, Bash 3.2.57(1), ShellCheck 0.11.0; Docker client/server
29.8.2; container Linux aarch64, Bash 5.2.21(1). Docker operations used approved
escalation. No compiler/provider was involved; no numerical qualification implied.

Only focused shell regressions, syntax, ShellCheck, diff checks and the narrow
existing-container smoke ran. No native/full profile, build, simulator, network,
download, image/up/sync, new container, container/worktree removal, push or merge.
All real protected-name repros used disposable fixture/smoke directories; actual
checkout `.git`/`.venv`, `/tmp/bike-sincos`, prior artifacts and the four unrelated
untracked XML/ZIP files were untouched. Concurrent tree mutation/crashes and
rollback filesystem failures retain the previously documented limits. Full
command/output and self-review: the appended final-fix section of
`.superpowers/sdd/2026-10-21-sincos-golden-pull/task-4-report.md`.

Configured-image query `docker container inspect --format '{{.Config.Image}}'
bike-dev-python-free-cpp-numerics` exits 0 with `bike-linux-image:port`.
Pre-commit `git diff --cached --check` exits 0; staged file enumeration contains
exactly the four task files listed in the report. Staging uses approved escalation
because this worktree's shared Git index is outside the writable root.

## 2026-10-21 — Phase 3 source implementation from the supplied Repomix snapshot

This entry records a new source-only session. The user explicitly prohibited
compilation and project execution. No compiler (including `-fsyntax-only`),
CMake configure, pytest collection/run, CTest, application, simulation, benchmark,
container, capture script, sanitizer, coverage or package command was invoked.
The preceding historical logs are inherited evidence, not runs from this session.
No L1/L2/L3 gate or Phase-3 task is closed by this entry.

Implemented source includes the opt-in Python golden hook and strict native
sidecar/payload readers; native episode save/load/capture/compare and recording
CLI; whole-boundary realtime measurement/report/CLI; root CTest successor
registrations; selected per-function test ports; native validation modules and
CLI; root presets, sanitizer/RTSan controls, coverage, offline archive extraction,
CPack recipes and preset-based CI. The validation work remains partial even at
the source level. Shared projection/codec seams reuse existing owned native
implementations; physical thresholds and FP policy were not relaxed.

The independently reconciled text inventory contains 2369 original test
functions/methods: the original map had 1611, leaving 758 missing. Twelve new
capture-hook test functions bring the current map to 2381 unique rows, including
16 class methods. The removal ledger now covers all 532 original `.py` paths plus
three new capture sources, fixing the original 73 missing rows. No Python file
was removed, and no row was promoted to qualified or `replacement_landed`.

Current function dispositions are 183 `port`, 58 `golden`, 35 future `drop`,
and 2105 `planned`. These are source decisions, not executed or parameterized
test counts. Named full-file source ports include pneumatic config/forces
(29 functions), drivetrain/policies/clone/topology (69), six drive-policy and
pedalling files (60), the three episode regression files (9 functions, 10 cases),
and the smaller reviewed realtime/clock/domain/kernel groups. Required platform
goldens are absent; none was invented or populated from native results.

Across the 53 Phase-3 tasks: 13 have source implementations, 11 have partial
source implementations, 23 test-port batches remain unimplemented, and six
historical failure diagnoses require execution. Source review does not establish
compilation, numerical parity, initialization validity, performance or relocation.
The phase is explicitly incomplete.

Detailed source evidence and unresolved work:

- `migration/cpp-linux-phase3-source-status.json`
- `migration/cpp-linux-phase3-source-review.md`
- `migration/cpp-linux-phase3-golden-review.md`
- `migration/cpp-linux-phase3-episode-ports.md`
- `migration/cpp-linux-phase3-pneumatic-ports.md`
- `migration/cpp-linux-phase3-drive-ports.md`
- `migration/cpp-linux-phase3-drive-dispositions.json`
- `migration/cpp-linux-phase3-realtime-review.md`
- `migration/cpp-linux-phase3-validation-review.md`
- `migration/cpp-linux-phase3-validation-map.json`
- `migration/cpp-linux-phase3-tooling-review.md`

The quoted earlier audit described `fc848de`; the supplied text already contains
the later two-platform/eight-operation numerics lock, exact canonical-nonfinite
sincos assertions and recursive safe container pull. Its later inherited evidence
also describes actual-checkout branch integration and restoration of nine binary
research files. Original Git objects and binary payloads were omitted by Repomix,
so this session cannot independently prove ancestry or those restored bytes.
The numerics lock and prior qualification results were left intact.

The delivery uses a normalized text patch and a reconstructed text-source archive.
File inspection, JSON/AST inventory and patch applicability are file operations;
they are not execution of the project. The export manifest records exactly those
checks and the limitations of reconstruction. Apply the patch to a matching real
checkout to retain its original history, binary artifacts and existing file modes.

## 2026-10-21 — Task 3.05: Rider allocation family (5 of 7 fixed, 2 stopped on physics rule)

Scope: macOS baseline nodes in `test_rider_allocation.py` (3),
`test_coupled_rider_task.py` (2), `test_closed_form_rider.py` (1),
`test_joint_strength.py` (1).

### Reproduction (macOS, before fix)

```
PYTHONPATH=tests/reference NATIVE_TEST_BUILD_PATH="$PWD/native/build" \
  uv run --frozen --group native python -m pytest -q \
  tests/reference/test_rider_allocation.py \
  tests/reference/test_coupled_rider_task.py \
  tests/reference/test_closed_form_rider.py \
  tests/reference/test_joint_strength.py
→ 7 failed, 66 passed (44.07 s), exit 1
```

Signatures identical to `migration/baseline-full.log` (stale-failure family,
not a fresh regression): `KeyError: 'feasible'`,
`AttributeError: '_alloc_attachments'`, `TOMLDecodeError` duplicate key,
motor-on fraction 0.5875 < 0.8, elbow strength violations.

### Root cause — one shared cause, two distinct fix classes

All seven trace to commit `835a48c` ("feat: closed-form seated rider control
on the pinned plant"), which repurposed `examples/research/viewer_physics_welded.toml`
from a fully welded rider (`pedal/saddle/grip = weld/weld/weld`) to the pinned
plant (`spindle/pin/connect`) and added the closed-form `compute()` path.
`ArticulatedRiderController.compute` dispatches on `self.spindle`
(`rider_control.py:758`); `_allocation_setup`/`_alloc_attachments` and the
`'feasible'`/`'solution_*'` diagnostics exist only on the welded LP path
(`rider_control.py:399`, `1240`, `1331`; `rider_response_allocation.py`).

**Class A — stale fixture profile (test-side fix, 5 nodes).** The allocation
and coupled-task tests assert contracts of the welded LP response allocator,
which remains live code reachable through the attachment enums
(`builder.py:345-388` still emits `weld_foot_*`/`weld_saddle`/`connect_grip_*`;
`grip 'weld'` migrates to `'connect'` at `physical_config.py:470`). The tests
referenced the shared profile after its semantics changed. Checks:

- [reproduced] Under a generated weld/weld/weld variant of the profile the
  full diagnostic contract returns: `feasible=True`,
  `invalid_controller=False`, all `solution_*` keys, populated
  `_alloc_attachments`; coupled run `checked=42` (bar ≥20) worst rel err
  2.95e-11 (bar 1e-3); tissue damping passive 170.1 W < 0.3×899.0 W;
  `test_command_does_not_read_solved_reactions` bitwise identical (0.0);
  `test_infeasible_intent` feasible with all torques inside bounds.
- [source-supported] `coupled = cfg.coupled_task_control and self.welded and
  self.welded_grip and cfg.saddle_attachment == 'weld'`
  (`rider_control.py:912`) — unreachable under the pinned profile.
- [source-supported] `test_coupled_rider_task._environment` inserted a second
  `joint_passive_damping_nms_rad` key (the TOML gained `= 0.0` in 835a48c) →
  `TOMLDecodeError`.
- [reproduced] `test_allocated_wrenches_close_the_rider_dynamics` then failed
  on its own reconstruction: it subtracted `joint_kd_nms_rad*qvel` while the
  profile's actual `dof_damping` is `passive_damping_nms_rad` (= 0.0 under
  the fixture; residual equalled the phantom term, max 32.35).

**Class B — physical-model behaviour on the shipped path (STOP RULE, 2 nodes).**

`test_flat_launch_delivers_crank_torque` (measured, 6 s flat launch, late
window t>2 s, 400 rows): `mean(human)=14.92 Nm` (bar ≥10 ✓),
`mean(motor>0)=0.5875` (bar ≥0.8 ✗), `vmax=5.37 m/s` (bar ≥4.17 ✓),
`allocation_invalid` all false ✓. The `connect_foot_*` torque sensor reads
delivered crank torque with dead zones below the 4 Nm `engage_torque_nm`
gate for ~40 % of late samples (clusters of ~0.2–0.3 s at ~103 rpm cadence);
delivery does not track the commanded `crank_task` (task 24.5→sensed 2.4;
task 3.4→sensed 46.4). The era audit
`docs/superpowers/audits/2026-10-04-realistic-pedelec-drive.md` records the
same signature as "an unresolved new delivery regression" — a known physical
gap of the closed-form leg law, not a test or sensor defect.

`test_stepped_effort_respects_strength_and_the_power_budget` (measured):
`rider_positive_power_w` ≤ 450 ✓, `rider_passive_power_w`=0 ✓, but
`rider_strength_violations=('rider_elbow_left','rider_elbow_right')` from env
step 28 (sim.steps 232). At control tick 228 the elbow velocity was
+2.74 rad/s *against* the −34.63 Nm held command (eccentric → cap 43.24,
`limit_torques` correctly unclipped, `saturated=False`); held across the
4-step period, velocity reversed to −1.62 (concentric → cap 22.64) and
−3.05 (cap 14.89), so `solved_effort` records delivered-over-capacity at the
later interval-start states. Under a welded variant the same 200-step
scenario produces zero violations (the LP allocates different torques). Any
fix is a model or contract decision: margin the closed-form law against
within-period capacity change, re-clip held torques per interval, re-scope
the diagnostic to the command state, or re-scope the test to the welded path.

### Fix applied (Class A, test files only; no assertions weakened)

- `tests/reference/test_rider_allocation.py`: added `_welded_config(tmp_path)`
  producing the weld/weld/weld variant (absolute asset paths) and defaulted
  `_environment` to it; corrected the dynamics reconstruction to
  `config.passive_damping_nms_rad` (the parameter that actually lands in
  `dof_damping`; `joint_kd` was only correct before the override key existed).
- `tests/reference/test_coupled_rider_task.py`: `_environment` now rewrites
  the existing `joint_passive_damping_nms_rad` row to 0.5 (was: duplicate
  key → TOMLDecodeError), injects `coupled_task_control = true`, and restores
  weld/weld/weld attachments so the coupled path engages.

Written argument for test-side fix: the tested subject (the welded LP
allocator contract) is unchanged live code; only the shared fixture profile
was repurposed. Pinning the welded configuration restores the intended code
path with every original assertion intact, and the `passive_damping_nms_rad`
correction makes the test's own dynamics reconstruction name the real
parameter — the assertion (residual ≈ 0) is unchanged.

### Verification

```
same pytest command after fix → 2 failed, 71 passed (30.59 s), exit 1
remaining: test_flat_launch_delivers_crank_torque (0.5875 < 0.8),
           test_stepped_effort… (rider_elbow_left/right)   [STOP RULE]
```

macOS baseline list in `docs/TESTING.md` updated 24 → 19 (anchor phrases
preserved). Linux container `bike-dev-python-free-cpp` does not exist on this
machine (only the unrelated `bike-dev-python-free-cpp-numerics` lane), so the
container verification named by the task could not run; the fixed nodes are
Python-oracle environment tests whose behaviour is platform-independent, and
the container lane is expected to reproduce the same result.

### Pending user decision (physics stop rule)

1. `test_flat_launch_delivers_crank_torque`: fix the closed-form law's
   crank-torque delivery (control-law change), re-express the ≥0.8 motor-on
   bar with a written physical argument, or triage `drop` — per the
   stopped-task states.
2. `test_stepped_effort_respects_strength_and_the_power_budget`: margin or
   re-clip inside the closed-form law / control clock, re-scope
   `rider_strength_violations` to the command state, or pin the test to the
   welded variant like Class A. Measured values above.

## 2026-10-22 — Task 3.06: Seated pedalling and posture (0 of 5 fixed, all 5 stopped on physics rule)

Nodes (macOS baseline):
- `tests/reference/test_seated_pedaling_cycle.py::test_strict_flat_cycle_delivers_and_stays_within_budgets[20.0]`, `[40.0]`, `[60.0]`
- `tests/reference/test_seated_pedaling_cycle.py::test_free_coast_is_an_intent_not_a_crank_lock`
- `tests/reference/test_seated_posture_program.py::test_pulse_redistributes_load_through_inertia_only`

### Reproduction (macOS)

```bash
PYTHONPATH=tests/reference NATIVE_TEST_BUILD_PATH="$PWD/native/build" \
uv run --frozen --group native python -m pytest -q \
  tests/reference/test_seated_pedaling_cycle.py \
  tests/reference/test_seated_posture_program.py
# → 5 failed, 20 passed in 10.32 s, exit 1
# 4 pedaling nodes: InvalidReferenceRun at t=0.005: ('energy.constraint_work',)
# posture node:     InvalidReferenceRun at t=0.003: ('rider_strength.rider_elbow_left',
#                                                   'rider_strength.rider_elbow_right')
```

The same five IDs fail in `migration/baseline-full.log` — baseline, not new.

### Dead-spot crank-freeze hint: checked, ruled out (reproduced + source-supported)

The plan hinted at `motor_clutch` opening near crank-vertical with legs
stalling and coast hysteresis toggling ~4/s. It cannot explain these nodes:
`viewer_physics_welded.toml` sets `motor_clutch = false` (line 32), so no
clutch exists in these runs, and every failure fires at t=0.003–0.005 —
the first strict period close — before the crank has turned a measurable
fraction of a revolution. The tests never reach their crank/coast
assertions; the failing item is the strict reference monitor at startup.

### Root cause — startup transient violates two per-step monitor contracts (reproduced)

Both failure classes reproduce inside the first control period(s) of a
*correctly settled* run (`sim.equilibrium['residual_qacc']` = 0.0217 flat /
0.035 on grade, `cache_hit` — i.e., the equilibrium solve converged).

**(A) `energy.constraint_work` (4 pedaling nodes).**
`period_buffer.publish_close` accumulates `rt.constraint_absolute_j` =
Σ|efc·Jv|·dt over all constraint rows and asserts
`absolute ≤ max(1e-8, 0.01·source_positive_j)` at every period close
(`energy_ledger.constraint_work_ok`, `period_buffer.py:250-259`).

Measured at the first close (t=0.005, flat, human 20 Nm):
- `constraint_absolute_j` = **4.478 mJ**; budget = 0.272 mJ
- `source_positive_j` = 27.15 mJ (all muscle; motor 0)
- Ratio ≈ **16.5 %** of source work — 16× over the 1 % budget.

The dominant rows are the soft equality constraints doing real spring work
during the settle→run transition: `connect_foot_front/rear` (equality 2/3),
`connect_saddle` (4), grip welds (5/6), suspension closure (0/1) and
`tendon_limit:0` (`ideal_mid_drive_freehub`). With `solref` time constants
~2.5 ms these are spring-dampers, not rigid joints; the episode's initial
state carries small residual constraint deflections and the episode command
(`human_torque_nm` 20–60 Nm, posture servo) re-loads the weld chain, so the
welds exchange ~1 W while `source_positive` is still ramping from zero. The
cumulative-integral form of the check is structurally strictest at t=0 —
absolute constraint work has a floor from the settle residual while the
source integral is ≈0. In diagnostic mode the run does not recover: the
posture servo oscillation grows (elbows reach ±3.1 rad/s,
qacc ≈ ±993 rad/s² at t=0.075) and the episode is stopped by
`numerical_quality` at t=0.075 with `residual_j` = 1.757 J,
`constraint_work_ratio` = 0.058 — the residual growth is partly a
f·v_in bookkeeping undercount (≈½·F·a·dt² per step uncounted during
intra-step acceleration), which is itself measured, not a proved
conservation defect.

**(B) `rider_strength.rider_elbow_*` (posture node).**
`compute()` (spindle path) clips each joint torque to the Hill
`directional_capacity` at the *tick instant* via `limit_torques`
(rider_control.py:823/827 + `finalize_effort` rider_effort.py:44-45), then
`control_clock.hold()` holds it for `steps_per_period` physics steps
(10 × 0.5 ms here). `_effort_observations` evaluates `strength_violations`
at *every* raw's recorded incoming state — a stricter contract than the
controller implements.

Measured (18 % grade env, dt=0.0005):
- tick-0 held elbow command = **−23.34 Nm**, legal at v=0 (cap 40.35)
- elbow accelerates ≈ −370 rad/s per step under the held torque;
  capacity derates concentrically: v=−1.78 → cap 21.62 < 23.34 →
  first violation at raw5 (t=0.003), growing to cap 14.78 at v=−3.10
- the command is a PD+bias posture servo term
  (`kp·err − kd·v + qfrc_bias`; kp=600, kd=15): the settle converged under
  `control=None` (default upright posture) while the episode's
  seated-climb program resolves `torso_lean_rad=0.01` at t=0, which
  shifts the arm-chain IK target (`_upper_targets`) by ≈ −0.036 rad at the
  elbow → the −23 Nm bracing kick.

Same mechanism previously stop-ruled for
`test_stepped_effort_respects_strength_and_the_power_budget` (Task 3.05
section above): a held command legal at its tick becomes illegal as the
joint speed evolves within the period.

### Checks (≥2 independent, labelled)

1. **reproduced**: strict-mode failures at t=0.005 / t=0.003 with the exact
   violation keys above; identical set in `migration/baseline-full.log`.
2. **reproduced**: instrumented per-raw capacity/torque/speed table —
   `rider_strength` fires only once |v| derates the concentric capacity
   below the held |torque|; the pedaling nodes' `constraint_absolute_j` is
   dominated by weld/connect rows during the settle transient.
3. **source-supported**: the monitor contract is per-raw
   (`_effort_observations`, period_buffer.py:170) while the enforcement is
   per-tick (`limit_torques` + `control_clock.hold`), rider_control.py:369.
4. **internal-precondition**: equilibrium solve converged
   (residual_qacc 0.022–0.035 ≤ 0.05 tol) under `steady_state` forces with
   the *default* `RideControl()` — not the episode's resolved posture/effort.
5. **unresolved**: whether the growing servo oscillation (±3 rad/s elbows,
   `numerical_quality` stop at 75 ms) reflects an intended underdamped
   posture servo or a missing rate/velocity limit — a model question; the
   whip is real dynamics, not a bookkeeping artifact (though ~2 J of the
   reported residual is f·v_in undercount during whip acceleration).

### Physics stop rule — invoked; no fix applied

Every remedy is a physical-model or contract/threshold decision, so per the
shared rule this task records measured values and asks the user. Candidate
decisions:

1. **Settle under the resolved initial control** — run
   `solve_physical_equilibrium` (or a post-settle re-equilibration) with the
   episode's actual first `RideControl` (posture program output, human
   effort ramp). Removes the settle→run command mismatch for both classes.
   *Model change* (physical initial condition).
2. **Per-interval envelope enforcement** — re-clip the held torque against
   `directional_capacity` at each physics step (or shorten the control
   period / rate-limit posture-induced joint torque). *Controller model
   change*; affects `rider_control.compute`/`finalize_effort` and is
   mirrored in the C++ port (check scope vs Task 3.05-owned files —
   `rider_response_allocation.py` is untouched here).
3. **Constraint-work accounting** — track weld/connect spring deflection as
   a `stored_terms` elastic channel and/or exempt it from
   `constraint_absolute_j`; or grow the 1 % integral budget /
   source-positive floor at startup. *Ledger/threshold change*.
4. **Test-scope change** — start the strict budgets after a declared
   settle window, or re-scope `rider_strength` to tick states. *Threshold
   change* — needs the written physical argument and user approval.
5. **drop** — triage in `test-port-map.json` if the startup transient is
   accepted as out-of-scope for the strict monitor.

### Verification status

- macOS: reproduction above; **no code or threshold changed**, so the
  targeted command still reports 5 failed / 20 passed.
- Linux lane: `tools/linux/container.sh sync` → `docker: Unable to find
  image 'bike-dev-linux:24.04'`, exit 125. No `bike-dev-python-free-cpp`
  container exists on this machine (only the unrelated
  `bike-dev-python-free-cpp-numerics` lane). Lane unavailable — recorded,
  not blocking.
- `docs/TESTING.md`: unchanged — the five nodes remain listed; the
  macOS baseline count stays at 19 under
  `failing node IDs, identical in the baseline`.

## 2026-10-22 — Task 3.08: Period buffer (both nodes fixed)

Nodes: `tests/reference/test_period_buffer.py::test_non_aligned_flush_keeps_each_intervals_actual_held_command_terms`
and `::test_runtime_batch_matches_preserved_scalar_step_and_flush`.

### Reproduction

```
PYTHONPATH=tests/reference NATIVE_TEST_BUILD_PATH="$PWD/native/build" \
  uv run --frozen --group native python -m pytest -q tests/reference/test_period_buffer.py
```
Before the fix: 2 failed / 8 passed — `AttributeError: 'NoneType' object has
no attribute 'copy'` at the `_last_solution` probe (test line 188), and
`0.44856197998153463 == 0.4486492160970741 ± 1e-9` on the energy channel
(bitwise-identical to `migration/baseline-full.log`).

### Root cause (single source commit `835a48c`, two stacked defects)

`835a48c` ("closed-form seated rider control on the pinned plant") changed
the pinned plant and production `_advance_physics` without updating either
the test's allocator probe or the preserved scalar oracle `_step_oracle.py`.

1. **`_last_solution` probe vs the spindle controller** — the commit flipped
   `examples/research/viewer_physics_welded.toml` `pedal_attachment` weld →
   spindle. Under `self.spindle`, `compute` takes the closed-form path
   (`rider_control.py:758`) and never reaches `allocate` /
   `_select_allocation_branch` — the only writers of `_last_branch` /
   `_last_solution` (rider_control.py:1556-1557). `reset_activation`
   (rider_control.py:624-626) therefore leaves both `None` permanently, and
   the test's unconditional `.copy()` raised. The probe's real contract —
   "both paths see identical settled controller memory" — survives: under
   the closed-form controller the persistent mutable state is the
   activation filter `active_state` (`reset_activation` clears it;
   `reset()` repopulates it from the last `effort_diagnostics`).

2. **Scalar oracle dropped the tire-load signal** — the same commit moved
   production's per-step intent-signal update to
   `update_rider_intent_signals(sensors, tires)` after `tire_channels`
   (physical_runtime.py:701-703), while the oracle kept the stale
   `signals_from_channels(sensors)` with no `tires` key
   (`_step_oracle.py:100-101`). `signals_from_channels` then yields
   `front_load_share=None` (rider_intent.py:14-16). Once the posture
   program's `trim_dead_time_s = 0.2 s` elapses, the batch path's
   `_delayed_load_share` stays numeric and `trim_rad` keeps integrating,
   while the oracle's samples deque drains to `None` entries and its
   `trim_rad` freezes (rider_program.py:109-116). The differing
   `RiderPosture.torso_lean_rad` produces different rider muscle torques.

3. **Oracle missing published channel keys** — `5374cc7` plus `835a48c`
   added `rider_balance`, `rider_joint_torques`, `rider_joint_capacity_nm`
   and `rider_lean_limit_rad` to `_advance_physics` raw channels
   (physical_runtime.py:742-747); the oracle's `channels` dict never
   gained them, so the `set(decimated[-1].channels) ==
   set(expected[-1].channels)` line could never pass.

### Checks (≥2 independent, labelled)

1. **reproduced**: instrumented probe — `compute` invoked 2× per reset
   (both `advance=False`), `allocate`/`_select_allocation_branch` invoked
   0×, `_last_branch`/`_last_solution` `None` after `make_environment` and
   after `env.reset()`; `c.spindle is True`.
2. **source-supported**: `git show 835a48c` carries the weld→spindle flip
   in the test's config and the
   `signals_from_channels(sensors)` → `update_rider_intent_signals(sensors,
   tires)` move in `_advance_physics`; `_step_oracle.py` untouched since
   `d93e06c`. `git log -S 'rider_joint_torques'` / `-S 'rider_balance'`
   date the channel keys to `835a48c`/`5374cc7`, again production-only.
3. **reproduced**: per-interval diff of the two paths — every energy field
   bitwise-equal through interval 419; first divergence at interval 420
   (t = 0.2105 s) with `qpos`/`qvel` still bitwise identical
   (`max|Δ| = 0.0`), `muscle_signed_j` off by +5.49e-3 J,
   `constraint_absolute_j` by −8.72e-5 J, `muscle_positive_j` equal.
   420 is exactly one intent tick after the 0.2 s trim dead-time boundary
   (step 400): the oracle's `_load_samples` head `(0.01, None)` pops at the
   k = 21 update and `_delayed_load_share` goes `None`, freezing
   `trim_rad`, while production integrates one more
   `lean_trim_gain_rad_s*(0.30 − share)*dt` increment.
4. **internal-precondition**: `active_state` after two consecutive
   `env.reset()` calls is bitwise identical (probe), and `rider_intent`'s
   `state_dict()` round-trips equal — the closed-form controller's
   settled-cache analogue is deterministic across resets.

### Fix

- `src/bike_sim/sim/ride/_step_oracle.py`: mirror production — call
  `self.update_rider_intent_signals(sensors, tires)` after the
  `tire_channels` row (dropping the stale no-tires `signals_from_channels`
  call and its dead `rider_intent` import); add `rider_balance` via
  `self._balance_channel(front, rear, braking=braking,
  rider_enabled=control.rider_enabled)` and the three
  `rider_joint_*`/`rider_lean_limit_rad` keys under the same
  `rider_control is not None` guard as `_advance_physics`. The oracle is a
  preserved-scalar reference for the period-buffer contract tests only;
  production arithmetic is untouched (no macOS numeric change).
- `tests/reference/test_period_buffer.py`: the warm-start probe now
  tolerates the allocator-less controller — `_last_solution` is captured
  `None`-safely, the post-reset assertion requires `None → None`
  (and array equality when a weld allocator populates it again), and
  `active_state` is pinned bitwise across the two resets so the
  "identical settled-cache reset" check stays non-vacuous under the
  closed-form controller.

### Verification

```
PYTHONPATH=tests/reference NATIVE_TEST_BUILD_PATH="$PWD/native/build" \
  uv run --frozen --group native python -m pytest -q tests/reference/test_period_buffer.py
→ 10 passed, 6 warnings in ~21 s   (exit 0, macOS arm64)
```

All 10 nodes pass, including the pre-existing energy.constraint_work
`RuntimeWarning` at t = 0.005 s that both paths report identically.

- `docs/TESTING.md`: the two node IDs removed from the macOS baseline
  block; count updated 19 → 17; anchor sentence
  `failing node IDs, identical in the baseline` and the following `## `
  heading kept verbatim.
- Linux lane: no `bike-dev-python-free-cpp` container exists on this
  machine — container verification skipped per the task instruction.
- Physics stop rule: not invoked — both causes are oracle/test
  instrumentation drift, not physical-model choices.
